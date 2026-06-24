import mujoco
import mujoco.viewer
import time
import os
import sys
import numpy as np
import scipy.io as sio

# Import the modular controller and gait generator
from wbc_controller import MinimalWBC
from gait_generator import GaitGenerator, GaitMode

# =====================================================================
#  CHANGE THIS LINE TO SWITCH GAITS:
#  GaitMode.STAND | GaitMode.SQUAT | GaitMode.WEIGHT_SHIFT | GaitMode.STEP_IN_PLACE | GaitMode.JUMP
# =====================================================================
ACTIVE_GAIT = GaitMode.JUMP

# Set paths
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, '..', 'controllers'))

print("==================================================================")
print("         KBOT LEGS-ONLY WHOLE-BODY OPERATIONAL SPACE CONTROL      ")
print("==================================================================")

# 1. Dynamic Model Compiler (Stripping Arms/IMU/Upper-Torso and Adding pelvis box)
def generate_legs_only_model():
    print("Generating legs-only model with physical collision geoms...")
    import xml.etree.ElementTree as ET
    
    original_robot_path = os.path.join(current_dir, "..", "kbot", "robot.mjcf")
    tree = ET.parse(original_robot_path)
    root = tree.getroot()
    
    # 1. Remove arm bodies from the torso body
    torso_body = root.find(".//body[@name='torso']")
    if torso_body is not None:
        arms_to_remove = []
        for b in torso_body.findall("body"):
            name = b.attrib.get("name", "")
            if "ShldYokeDrive" in name or name.startswith("KC_") or name.startswith("KD_C_"):
                arms_to_remove.append(b)
        for arm in arms_to_remove:
            torso_body.remove(arm)
            print(f"Removed arm body: {arm.attrib.get('name')}")
            
        # 2. Hide base_visual, torso_origin, and torso_visual geoms
        base_body = root.find(".//body[@name='base']")
        if base_body is not None:
            base_visual = base_body.find("./geom[@name='base_visual']")
            if base_visual is not None:
                base_body.remove(base_visual)
        
        torso_origin = torso_body.find("./geom[@name='torso_origin']")
        if torso_origin is not None:
            torso_body.remove(torso_origin)
            
        torso_visual = torso_body.find("./geom[@name='torso_visual']")
        if torso_visual is not None:
            torso_body.remove(torso_visual)
            
        # 3. Add pelvis primitive visual box (representing only the pelvis, no torso/neck)
        pelvis_visual = ET.Element("geom", {
            "name": "pelvis_visual",
            "type": "box",
            "size": "0.06 0.12 0.04",
            "pos": "-0.025 0 0.0",
            "material": "torso_material",
            "class": "visual"
        })
        torso_body.append(pelvis_visual)
        print("Added pelvis_visual primitive box")
        
        # 4. Add torso_collision box (keeps physical collision bounds)
        torso_collision = ET.Element("geom", {
            "name": "torso_collision",
            "type": "box",
            "size": "0.12 0.15 0.08",
            "pos": "-0.025 0 0.1",
            "class": "collision"
        })
        torso_body.append(torso_collision)
        
        # 5. Remove the entire IMU body (floating mass above the pelvis)
        imu_body = torso_body.find("body[@name='imu']")
        if imu_body is not None:
            torso_body.remove(imu_body)
            print("Removed IMU body")
                
        # 6. Inject capsule collisions for femurs and shins
        left_femur = torso_body.find(".//body[@name='KD_D_301L_L_Femur_Lower_Drive']")
        if left_femur is not None:
            left_femur_col = ET.Element("geom", {
                "name": "L_femur_collision",
                "type": "capsule",
                "fromto": "0 0 0 0 0 -0.2",
                "size": "0.045",
                "class": "collision"
            })
            left_femur.append(left_femur_col)
            
        right_femur = torso_body.find(".//body[@name='KD_D_301R']")
        if right_femur is not None:
            right_femur_col = ET.Element("geom", {
                "name": "R_femur_collision",
                "type": "capsule",
                "fromto": "0 0 0 0 0 -0.2",
                "size": "0.045",
                "class": "collision"
            })
            right_femur.append(right_femur_col)
            
        left_shin = torso_body.find(".//body[@name='KD_D_401L_L_Shin_Drive']")
        if left_shin is not None:
            left_shin_col = ET.Element("geom", {
                "name": "L_shin_collision",
                "type": "capsule",
                "fromto": "0 0 0 0 -0.29 0",
                "size": "0.035",
                "class": "collision"
            })
            left_shin.append(left_shin_col)
            
        right_shin = torso_body.find(".//body[@name='KD_D_401R']")
        if right_shin is not None:
            right_shin_col = ET.Element("geom", {
                "name": "R_shin_collision",
                "type": "capsule",
                "fromto": "0 0 0 0 -0.29 0",
                "size": "0.035",
                "class": "collision"
            })
            right_shin.append(right_shin_col)
            
    # 7. Remove arm actuators
    actuator = root.find("actuator")
    if actuator is not None:
        motors_to_remove = []
        for motor in actuator.findall("motor"):
            name = motor.attrib.get("name", "")
            if "shoulder" in name or "elbow" in name or "wrist" in name:
                motors_to_remove.append(motor)
        for motor in motors_to_remove:
            actuator.remove(motor)
    
    # 8. Remove IMU sensors (imu_acc, imu_gyro) that reference the deleted imu body/site
    sensor = root.find("sensor")
    if sensor is not None:
        sensors_to_remove = [s for s in sensor if "imu" in s.attrib.get("name", "") or "imu" in s.attrib.get("site", "")]
        for s in sensors_to_remove:
            sensor.remove(s)
            print(f"Removed sensor: {s.attrib.get('name', s.tag)}")
            
    # Write to robot_legs.mjcf
    robot_legs_path = os.path.join(current_dir, "..", "kbot", "robot_legs.mjcf")
    tree.write(robot_legs_path, encoding="utf-8", xml_declaration=True)
    
    # Read original kbot_in_scene.xml and modify include
    original_scene_path = os.path.join(current_dir, "..", "scene", "kbot_in_scene.xml")
    with open(original_scene_path, "r") as f:
        scene_content = f.read()
    scene_content = scene_content.replace(
        '<include file="kbot/robot.mjcf"/>',
        '<include file="kbot/robot_legs.mjcf"/>'
    )
    legs_scene_path = os.path.join(current_dir, "..", "scene", "kbot_legs_in_scene.xml")
    with open(legs_scene_path, "w") as f:
        f.write(scene_content)
        
    return legs_scene_path

# Generate legs-only scene
legs_scene_path = generate_legs_only_model()

try:
    # 1. Instantiate the WBC from the separate file with lightly crouched knee_bend
    wbc = MinimalWBC(legs_scene_path, knee_bend=0.45)
    gait_gen = GaitGenerator(mode=ACTIVE_GAIT)
    
    print(f"Active gait mode: {ACTIVE_GAIT.name}")
    print(f"Zero-impact spawn height determined: z_base = {wbc.data.qpos[2]:.4f} m (exact flat tangent contact)")

    # Define joints of interest (hips, knees, and ankles for left and right legs)
    joint_names = {
        'left_hip_pitch': 'dof_left_hip_pitch_04',
        'left_hip_roll': 'dof_left_hip_roll_03',
        'left_hip_yaw': 'dof_left_hip_yaw_03',
        'left_knee': 'dof_left_knee_04',
        'left_ankle': 'dof_left_ankle_02',
        'right_hip_pitch': 'dof_right_hip_pitch_04',
        'right_hip_roll': 'dof_right_hip_roll_03',
        'right_hip_yaw': 'dof_right_hip_yaw_03',
        'right_knee': 'dof_right_knee_04',
        'right_ankle': 'dof_right_ankle_02'
    }
    
    # Look up joint indices in qpos (positions) and qvel (velocities/dofs)
    joint_info = {}
    for short_name, fullname in joint_names.items():
        joint_id = mujoco.mj_name2id(wbc.model, mujoco.mjtObj.mjOBJ_JOINT, fullname)
        if joint_id == -1:
            raise ValueError(f"Joint '{fullname}' not found in the model!")
        joint_info[short_name] = {
            'qpos_adr': wbc.model.jnt_qposadr[joint_id],
            'dof_adr': wbc.model.jnt_dofadr[joint_id]
        }
        
    # Initialize data logging lists
    logged_data = {
        'time': [],
        'mode_id': [],
        'mode_name': []
    }
    for short_name in joint_names.keys():
        logged_data[f'{short_name}_angle'] = []
        logged_data[f'{short_name}_velocity'] = []
        logged_data[f'{short_name}_acceleration'] = []
        logged_data[f'{short_name}_net_torque'] = []
        logged_data[f'{short_name}_contact_torque'] = []
        
    saved_gait_data = False

    def save_mat_data():
        global saved_gait_data
        if saved_gait_data or len(logged_data['time']) == 0:
            return
        # Structure into nested structs
        mat_data = {
            'gait_data': {
                'time': np.array(logged_data['time']),
                'mode_id': np.array(logged_data['mode_id']),
                'mode_name': np.array(logged_data['mode_name'], dtype=object),
                'joints': {}
            }
        }
        for short_name in joint_names.keys():
            mat_data['gait_data']['joints'][short_name] = {
                'angle': np.array(logged_data[f'{short_name}_angle']),
                'velocity': np.array(logged_data[f'{short_name}_velocity']),
                'acceleration': np.array(logged_data[f'{short_name}_acceleration']),
                'net_torque': np.array(logged_data[f'{short_name}_net_torque']),
                'contact_torque': np.array(logged_data[f'{short_name}_contact_torque'])
            }
        
        gait_data_dir = os.path.abspath(os.path.join(current_dir, "..", "phase0", "gait_data"))
        os.makedirs(gait_data_dir, exist_ok=True)
        mat_path = os.path.join(gait_data_dir, f"{ACTIVE_GAIT.name.lower()}_gait_data_legs.mat")
        csv_path = os.path.join(gait_data_dir, f"{ACTIVE_GAIT.name.lower()}_gait_data_legs.csv")
        
        # Save MAT file
        sio.savemat(mat_path, mat_data)
        
        # Save CSV file
        import csv
        with open(csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            headers = ['time', 'mode_id']
            for short_name in joint_names.keys():
                headers.extend([
                    f"{short_name}_angle",
                    f"{short_name}_velocity",
                    f"{short_name}_acceleration",
                    f"{short_name}_net_torque",
                    f"{short_name}_contact_torque"
                ])
            writer.writerow(headers)
            
            num_rows = len(logged_data['time'])
            for idx in range(num_rows):
                row = [logged_data['time'][idx], logged_data['mode_id'][idx]]
                for short_name in joint_names.keys():
                    row.extend([
                        logged_data[f'{short_name}_angle'][idx],
                        logged_data[f'{short_name}_velocity'][idx],
                        logged_data[f'{short_name}_acceleration'][idx],
                        logged_data[f'{short_name}_net_torque'][idx],
                        logged_data[f'{short_name}_contact_torque'][idx]
                    ])
                writer.writerow(row)
                
        print(f"\n=======================================================")
        print(f" >>> GAIT DATA EXPORTED SUCCESSFULLY! <<< ")
        print(f" Saved MAT to: {mat_path}")
        print(f" Saved CSV to: {csv_path}")
        print(f" Total recorded steps: {len(logged_data['time'])}")
        print(f"=======================================================\n")
        saved_gait_data = True

    # 4. Launch Passive Viewer Loop
    print("\nLaunching MuJoCo viewer (passive)...")
    with mujoco.viewer.launch_passive(wbc.model, wbc.data) as viewer:
        viewer.cam.distance = 1.8
        viewer.cam.elevation = -12
        viewer.cam.azimuth = 90
        
        step_count = 0
        while viewer.is_running():
            step_start = time.time()
            
            current_time = wbc.data.time
            
            # Get trajectory targets from the gait generator
            targets = gait_gen.get_targets(current_time)
            
            # Apply programmatic manual pushes (perturbations) to show absorption
            # Apply a 30 N push in +X direction for 0.15s starting at t=2.5s
            if 2.5 <= current_time <= 2.65:
                wbc.data.xfrc_applied[wbc.torso_id, :3] = [30.0, 0.0, 0.0]
                if step_count % 10 == 0:
                    print(f" >>> [PUSH] Applying 30N forward perturbation! Time: {current_time:.3f} s")
            # Apply a 30 N push in -X direction for 0.15s starting at t=6.0s
            elif 6.0 <= current_time <= 6.15:
                wbc.data.xfrc_applied[wbc.torso_id, :3] = [-30.0, 0.0, 0.0]
                if step_count % 10 == 0:
                    print(f" >>> [PUSH] Applying -30N backward perturbation! Time: {current_time:.3f} s")
            else:
                wbc.data.xfrc_applied[wbc.torso_id, :3] = [0.0, 0.0, 0.0]
                
            # Step the WBC controller and simulator with the current trajectory targets
            wbc.step(targets)
            
            # Determine gait mode/segment from the trajectory targets
            mode_name = ACTIVE_GAIT.name.lower()
            mode_id = ACTIVE_GAIT.value
                    
            # Record current step data
            logged_data['time'].append(current_time)
            logged_data['mode_id'].append(mode_id)
            logged_data['mode_name'].append(mode_name)
            
            for short_name, info in joint_info.items():
                qp = info['qpos_adr']
                qv = info['dof_adr']
                logged_data[f'{short_name}_angle'].append(wbc.data.qpos[qp])
                logged_data[f'{short_name}_velocity'].append(wbc.data.qvel[qv])
                logged_data[f'{short_name}_acceleration'].append(wbc.data.qacc[qv])
                logged_data[f'{short_name}_net_torque'].append(wbc.data.qfrc_actuator[qv])
                logged_data[f'{short_name}_contact_torque'].append(wbc.data.qfrc_constraint[qv])
                
            # Save gait data to .mat file at t = 10.0 seconds
            if current_time >= 10.0 and not saved_gait_data:
                save_mat_data()
            
            step_count += 1
            if step_count == 1 or step_count % 50 == 0:
                z_curr = wbc.data.xpos[wbc.torso_id][2] - 0.5 * (wbc.data.xpos[wbc.left_foot_body_id][2] + wbc.data.xpos[wbc.right_foot_body_id][2])
                print(f"\n================ WBC STEP {step_count} DIAGNOSTICS ==================")
                print(f"Height target: {wbc.target_height:.4f} m, Current height: {z_curr:.4f} m")
                print(f"COM x: {wbc.data.subtree_com[0][0]:.6f}")
                print(f"Left foot x: {wbc.data.xpos[wbc.left_foot_body_id][0]:.6f}, Right foot x: {wbc.data.xpos[wbc.right_foot_body_id][0]:.6f}")
                print(f"Torso pitch (rad): {wbc.get_pitch_from_quat(wbc.data.qpos[3:7]):.6f}")
                print(f"Task torque sum: {np.sum(wbc.tau_task):.6f}")
                # Print pitch task joint torques for Left Hip (6), Left Ankle (10), Right Hip (11), Right Ankle (15)
                # Note: In both models, leg joint DoFs start after the 6 floating base coordinates (indices 6 to 15)
                print(f"Hips/Ankles task torques: L_Hip={wbc.tau_task[6]:.4f}, L_Ankle={wbc.tau_task[10]:.4f}, R_Hip={wbc.tau_task[11]:.4f}, R_Ankle={wbc.tau_task[15]:.4f}")
                print(f"Motor commands (ctrl):    L_Hip={wbc.data.ctrl[0]:.2f}, L_Ankle={wbc.data.ctrl[4]:.2f}, R_Hip={wbc.data.ctrl[5]:.2f}, R_Ankle={wbc.data.ctrl[9]:.2f}")
                print("===================================================\n")
            
            # Sync the viewer
            viewer.sync()
 
            # Maintain real-time simulation speed
            time_until_next_step = wbc.model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    # Save any remaining data upon viewer window closure
    save_mat_data()

except Exception as e:
    print(f"\n[ERROR] An exception occurred: {e}")
    import traceback
    traceback.print_exc()
