import mujoco
import mujoco.viewer
import time
import os
import sys
import numpy as np
import scipy.io as sio

# Import the modular controller
from wbc_controller import MinimalWBC

# Set paths
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, '..', 'controllers'))
robot_path = os.path.join(current_dir, "..", "scene", "kbot_in_scene.xml")

print("==================================================================")
print("             KBOT WHOLE-BODY OPERATIONAL SPACE CONTROL            ")
print("==================================================================")
print(f"Loading full-robot scene: {robot_path}")

try:
    # 1. Instantiate the WBC from the separate file with lightly crouched knee_bend
    wbc = MinimalWBC(robot_path, knee_bend=0.45)
    
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
        mat_path = os.path.join(gait_data_dir, "squat_gait_data.mat")
        
        sio.savemat(mat_path, mat_data)
        print(f"\n=======================================================")
        print(f" >>> GAIT DATA EXPORTED SUCCESSFULLY! <<< ")
        print(f" Saved to: {mat_path}")
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
            
            # Squatting Gait Generation:
            # Vary height target sinusoidally between 0.52m (deep squat) and 0.72m (standing) at 0.5 Hz
            # Only start squatting after t=1.5s to allow base settling
            current_time = wbc.data.time
            if current_time > 1.5:
                freq = 0.5
                omega = 2.0 * np.pi * freq
                wbc.target_height = 0.62 + 0.10 * np.cos(omega * (current_time - 1.5))
                
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
                
            # Step the WBC controller and simulator
            wbc.step()
            
            # Determine gait mode/segment
            if current_time <= 1.5:
                mode_id = 0
                mode_name = "settling"
            else:
                omega = 2.0 * np.pi * 0.5
                phase = omega * (current_time - 1.5)
                # target height derivative is proportional to -sin(phase)
                # sin(phase) > 0 means target height is decreasing (squat_downwards)
                if np.sin(phase) > 0:
                    mode_id = 1
                    mode_name = "squat_downwards"
                else:
                    mode_id = 2
                    mode_name = "squat_upwards"
                    
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
