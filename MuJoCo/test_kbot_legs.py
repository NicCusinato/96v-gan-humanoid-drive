import mujoco
import mujoco.viewer
import time
import os
import numpy as np
import json

# Set paths
current_dir = os.path.dirname(os.path.abspath(__file__))
robot_path = os.path.join(current_dir, "kbot_in_scene.xml")
metadata_path = os.path.join(current_dir, "kbot", "metadata.json")

print("==================================================================")
print("         KBOT INTERACTIVE KINEMATIC POSE CAPTURE TOOL            ")
print("==================================================================")

# 1. Dynamic Model Compiler (Welding Arms and Injecting Collision Shapes)
def generate_legs_only_model():
    print("Generating legs-only model with physical collision geoms...")
    
    original_robot_path = os.path.join(current_dir, "kbot", "robot.mjcf")
    with open(original_robot_path, "r") as f:
        content = f.read()
        
    # List of arm joints to weld (remove from degrees of freedom)
    arm_joints = [
        "dof_right_shoulder_pitch_03", "dof_right_shoulder_roll_03", "dof_right_shoulder_yaw_02", 
        "dof_right_elbow_02", "dof_right_wrist_00", "dof_left_shoulder_pitch_03", 
        "dof_left_shoulder_roll_03", "dof_left_shoulder_yaw_02", "dof_left_elbow_02", "dof_left_wrist_00"
    ]
    
    # Filter out arm joints and actuators
    lines = content.splitlines()
    filtered_lines = []
    for line in lines:
        skip = False
        for joint in arm_joints:
            if joint in line:
                skip = True
                break
        if not skip:
            filtered_lines.append(line)
            
    filtered_content = "\n".join(filtered_lines)
    
    # Inject physical collision geometries so the robot body cannot swing/clip through the floor
    # 1. Box collision for the pelvis (torso body)
    filtered_content = filtered_content.replace(
        '<geom name="torso_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="torso_material" type="mesh" mesh="torso.stl" class="visual" />',
        '<geom name="torso_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="torso_material" type="mesh" mesh="torso.stl" class="visual" />\n          <geom name="torso_collision" type="box" size="0.12 0.15 0.08" pos="-0.025 0 0.1" class="collision" />'
    )
    # 2. Capsule collisions for the thighs (femurs)
    filtered_content = filtered_content.replace(
        '<geom name="KD_D_301L_L_Femur_Lower_Drive_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="torso_material" type="mesh" mesh="KD_D_301L_L_Femur_Lower_Drive.stl" class="visual" />',
        '<geom name="KD_D_301L_L_Femur_Lower_Drive_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="torso_material" type="mesh" mesh="KD_D_301L_L_Femur_Lower_Drive.stl" class="visual" />\n              <geom name="L_femur_collision" type="capsule" fromto="0 0 0 0 0 -0.2" size="0.045" class="collision" />'
    )
    filtered_content = filtered_content.replace(
        '<geom name="KD_D_301R_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="torso_material" type="mesh" mesh="KD_D_301R.stl" class="visual" />',
        '<geom name="KD_D_301R_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="torso_material" type="mesh" mesh="KD_D_301R.stl" class="visual" />\n              <geom name="R_femur_collision" type="capsule" fromto="0 0 0 0 0 -0.2" size="0.045" class="collision" />'
    )
    # 3. Capsule collisions for the shins
    filtered_content = filtered_content.replace(
        '<geom name="KD_D_401L_L_Shin_Drive_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="torso_material" type="mesh" mesh="KD_D_401L_L_Shin_Drive.stl" class="visual" />',
        '<geom name="KD_D_401L_L_Shin_Drive_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="torso_material" type="mesh" mesh="KD_D_401L_L_Shin_Drive.stl" class="visual" />\n                <geom name="L_shin_collision" type="capsule" fromto="0 0 0 0 -0.29 0" size="0.035" class="collision" />'
    )
    filtered_content = filtered_content.replace(
        '<geom name="KD_D_401R_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="torso_material" type="mesh" mesh="KD_D_401R.stl" class="visual" />',
        '<geom name="KD_D_401R_visual" pos="0 0 0" quat="1.0 0.0 0.0 0.0" material="torso_material" type="mesh" mesh="KD_D_401R.stl" class="visual" />\n                <geom name="R_shin_collision" type="capsule" fromto="0 0 0 0 -0.29 0" size="0.035" class="collision" />'
    )
    
    # Write temporary robot_legs.mjcf
    robot_legs_path = os.path.join(current_dir, "kbot", "robot_legs.mjcf")
    with open(robot_legs_path, "w") as f:
        f.write(filtered_content)
        
    # Read original kbot_in_scene.xml and modify the include tag
    original_scene_path = os.path.join(current_dir, "kbot_in_scene.xml")
    with open(original_scene_path, "r") as f:
        scene_content = f.read()
        
    scene_content = scene_content.replace(
        '<include file="kbot/robot.mjcf"/>',
        '<include file="kbot/robot_legs.mjcf"/>'
    )
    
    legs_scene_path = os.path.join(current_dir, "kbot_legs_in_scene.xml")
    with open(legs_scene_path, "w") as f:
        f.write(scene_content)
        
    return legs_scene_path

# Compile model
legs_scene_path = generate_legs_only_model()

try:
    # 2. Load the compiled model
    model = mujoco.MjModel.from_xml_path(legs_scene_path)
    data = mujoco.MjData(model)
    
    # Create isolated MjData for kinematic sweep solver
    kin_data = mujoco.MjData(model)
    
    with open(metadata_path, 'r') as f:
        metadata = json.load(f)

    # Leg joint names
    leg_joint_names = [
        "dof_left_hip_pitch_04",
        "dof_left_hip_roll_03",
        "dof_left_hip_yaw_03",
        "dof_left_knee_04",
        "dof_left_ankle_02",
        "dof_right_hip_pitch_04",
        "dof_right_hip_roll_03",
        "dof_right_hip_yaw_03",
        "dof_right_knee_04",
        "dof_right_ankle_02",
    ]

    # Dynamic joint index mapping
    leg_joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in leg_joint_names]
    leg_qpos_idx = [model.jnt_qposadr[jid] for jid in leg_joint_ids]
    leg_qvel_idx = [model.jnt_dofadr[jid] for jid in leg_joint_ids]

    leg_actuator_idx = []
    for name in leg_joint_names:
        actuator_id = -1
        for act_id in range(model.nu):
            joint_id = model.actuator_trnid[act_id, 0]
            act_joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            if act_joint_name == name:
                actuator_id = act_id
                break
        leg_actuator_idx.append(actuator_id)

    # 3. Dynamic Massless Pelvis Setup
    torso_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso")
    # Restore actual torso mass for realistic gravity physics
    model.body_mass[torso_body_id] = 12.606875  # Set to actual torso mass
    model.body_ipos[torso_body_id] = [0.0, 0.0, 0.0]  # Center of mass perfectly centered
    model.body_inertia[torso_body_id] = [0.1, 0.1, 0.1]

    # 4. Crouched Target Configuration
    knee_bend = 0.6  # Crouch lowers center of gravity for maximum standing stability
    
    # 5. Kinematic flat-foot solver (runs ONLY on kin_data, leaving main simulation untouched!)
    left_foot_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "LFootBushing_GPF_1517_12")
    right_foot_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "RFootBushing_GPF_1517_12")

    def solve_flat_foot_ankles(hip_pitch, knee_bend_val):
        ankle_l = 0.0
        ankle_r = 0.0
        
        # Retrieve physical joint limits for the ankles
        jnt_range_l = model.jnt_range[leg_joint_ids[4]]
        jnt_range_r = model.jnt_range[leg_joint_ids[9]]
        
        # Left Leg
        for _ in range(10):
            kin_data.qpos[leg_qpos_idx[0]] = hip_pitch
            kin_data.qpos[leg_qpos_idx[3]] = knee_bend_val
            kin_data.qpos[leg_qpos_idx[4]] = ankle_l
            mujoco.mj_forward(model, kin_data)
            
            # Read orientation of left foot relative to the world
            mat_l = kin_data.xmat[left_foot_body_id].reshape(3, 3)
            pitch_l = np.arcsin(mat_l[2, 0])  # Pitch angle relative to ground
            
            # Ankle pitch Y-axis is 0 0 -1. Adjust ankle to make foot level
            ankle_l = np.clip(ankle_l - pitch_l, jnt_range_l[0], jnt_range_l[1])
            
        # Right Leg
        for _ in range(10):
            kin_data.qpos[leg_qpos_idx[5]] = -hip_pitch
            kin_data.qpos[leg_qpos_idx[8]] = -knee_bend_val
            kin_data.qpos[leg_qpos_idx[9]] = ankle_r
            mujoco.mj_forward(model, kin_data)
            
            mat_r = kin_data.xmat[right_foot_body_id].reshape(3, 3)
            pitch_r = np.arcsin(mat_r[2, 0])
            # Mirrored ankle axis requires inverting correction feedback direction to converge
            ankle_r = np.clip(ankle_r + pitch_r, jnt_range_r[0], jnt_range_r[1])
            
        return ankle_l, ankle_r

    # 6. Kinematic Standing Balanced Pose Solver
    # Midpoint of the physical foot capsule in local coords (capsule spans [-0.06, 0.12], center is 0.03)
    local_foot_center = np.array([0.03, 0.0, 0.0])

    print("\nSolving for perfectly balanced initial standing pose...")
    best_theta = 0.0
    min_com_err = 100.0
    
    for theta in np.linspace(-1.0, 1.0, 500):
        # Get perfectly flat ankle angles for this hip angle
        ankle_l, ankle_r = solve_flat_foot_ankles(theta, knee_bend)
        
        kin_data.qpos[leg_qpos_idx[0]] = theta
        kin_data.qpos[leg_qpos_idx[3]] = knee_bend
        kin_data.qpos[leg_qpos_idx[4]] = ankle_l
        
        kin_data.qpos[leg_qpos_idx[5]] = -theta
        kin_data.qpos[leg_qpos_idx[8]] = -knee_bend
        kin_data.qpos[leg_qpos_idx[9]] = ankle_r
        
        mujoco.mj_forward(model, kin_data)
        
        com_x = kin_data.subtree_com[0][0]
        
        # Calculate support_x at the true physical center of the feet
        left_foot_glob = kin_data.xpos[left_foot_body_id] + kin_data.xmat[left_foot_body_id].reshape(3, 3).dot(local_foot_center)
        right_foot_glob = kin_data.xpos[right_foot_body_id] + kin_data.xmat[right_foot_body_id].reshape(3, 3).dot(local_foot_center)
        support_x = 0.5 * (left_foot_glob[0] + right_foot_glob[0])
        
        com_err = abs(com_x - support_x)
        if com_err < min_com_err:
            min_com_err = com_err
            best_theta = theta

    print(f"  Balanced hip pitch found: {best_theta:.4f} rad (Initial CoM error: {min_com_err:.6f} m)")

    # Solve flat-foot ankle angles for the best balanced hip pitch
    best_ankle_l, best_ankle_r = solve_flat_foot_ankles(best_theta, knee_bend)
    print(f"  Perfect flat ankle pitches: Left = {best_ankle_l:.4f} rad, Right = {best_ankle_r:.4f} rad")

    # Define target standing joint positions
    leg_stand_targets = [
        best_theta, 0.0, 0.0, knee_bend, best_ankle_l,  # Left leg
        -best_theta, 0.0, 0.0, -knee_bend, best_ankle_r  # Right leg
    ]

    # Initialize joint positions in the simulator
    for idx, act_idx in enumerate(leg_actuator_idx):
        joint_id = model.actuator_trnid[act_idx, 0]
        qpos_idx = model.jnt_qposadr[joint_id]
        data.qpos[qpos_idx] = leg_stand_targets[idx]

    # 7. Endpoint-Aware Zero-Impact Spawn Height Solver
    data.qpos[2] = 1.0  # Reset base height
    mujoco.mj_forward(model, data)
    
    local_foot_points = [
        np.array([-0.06, 0.0, 0.0]),  # Heel
        np.array([0.12, 0.0, 0.0]),   # Toe
        np.array([0.0, -0.03, 0.0]),  # Left lateral edge
        np.array([0.0, 0.03, 0.0])    # Right lateral edge
    ]
    
    lowest_z = 100.0
    
    # Check left foot points
    pos_l = data.xpos[left_foot_body_id]
    mat_l = data.xmat[left_foot_body_id].reshape(3, 3)
    for p in local_foot_points:
        p_glob = pos_l + mat_l.dot(p)
        z_bottom = p_glob[2] - 0.02  # Capsule radius is 0.02
        if z_bottom < lowest_z:
            lowest_z = z_bottom
            
    # Check right foot points
    pos_r = data.xpos[right_foot_body_id]
    mat_r = data.xmat[right_foot_body_id].reshape(3, 3)
    for p in local_foot_points:
        p_glob = pos_r + mat_r.dot(p)
        z_bottom = p_glob[2] - 0.02  # Capsule radius is 0.02
        if z_bottom < lowest_z:
            lowest_z = z_bottom
            
    # Shift base position so the exact lowest capsule endpoint touches the ground at z = 0.0
    data.qpos[2] += -lowest_z
    mujoco.mj_forward(model, data)
    print(f"Zero-impact spawn height determined: z_base = {data.qpos[2]:.4f} m (exact flat tangent contact)")

    # 8. Load safety limits for actuators
    tau_max_leg = np.zeros(10)
    for idx, name in enumerate(leg_joint_names):
        meta = metadata["joint_name_to_metadata"].get(name, {"soft_torque_limit": 40})
        limit_val = float(meta.get("soft_torque_limit", 40.0))
        tau_max_leg[idx] = limit_val

    # 9. Launch Passive Viewer Loop
    print("\nLaunching MuJoCo viewer (passive)...")
    print("------------------------------------------------------------------")
    print("INSTRUCTIONS FOR POSE CAPTURE:")
    print("1. Set the camera view to your liking.")
    print("2. Hold Ctrl + LMB (Left Mouse Button) on the robot pelvis or links to drag them.")
    print("3. Check this console window: it prints the exact joint angles (qpos) every 0.5s.")
    print("4. Copy-paste the printed target array when you find a stable posture!")
    print("------------------------------------------------------------------")

    last_print_time = 0.0

    # Configure soft, compliant posturing springs on ALL 10 leg joints to hold the stance stable,
    # preventing collapses or spins while remaining fully poseable.
    Kp_posture = np.zeros(10)
    Kd_posture = np.zeros(10)
    
    # Hip and Ankle Pitch (Strong orientation support)
    Kp_posture[0] = 80.0  # Left hip pitch
    Kd_posture[0] = 8.0
    Kp_posture[4] = 80.0  # Left ankle pitch
    Kd_posture[4] = 8.0
    Kp_posture[5] = 80.0  # Right hip pitch
    Kd_posture[5] = 8.0
    Kp_posture[9] = 80.0  # Right ankle pitch
    Kd_posture[9] = 8.0

    # Knee Pitch (Actively supports height and prevents collapse under torso weight!)
    Kp_posture[3] = 80.0  # Left knee pitch
    Kd_posture[3] = 8.0
    Kp_posture[8] = 80.0  # Right knee pitch
    Kd_posture[8] = 8.0

    # Hip Roll (Stabilizes side-to-side lean)
    Kp_posture[1] = 60.0  # Left hip roll
    Kd_posture[1] = 6.0
    Kp_posture[6] = 60.0  # Right hip roll
    Kd_posture[6] = 6.0

    # Hip Yaw (Stabilizes rotational twist / yaw)
    Kp_posture[2] = 40.0  # Left hip yaw
    Kd_posture[2] = 4.0
    Kp_posture[7] = 40.0  # Right hip yaw
    Kd_posture[7] = 4.0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 1.8
        viewer.cam.elevation = -12
        viewer.cam.azimuth = 90
        
        while viewer.is_running():
            step_start = time.time()
            
            # --- 1. Compute Joint-Space Gravity Compensation for leg links ---
            mujoco.mj_forward(model, data)
            tau_bias = data.qfrc_bias.copy()
            tau_g_legs = tau_bias[leg_qvel_idx]
            
            # --- 2. Compute Torso Weight & Orientation Moment Compensation ---
            m_torso = model.body_mass[torso_body_id]
            f_torso_gravity = m_torso * 9.81
            f_leg_share = 0.5 * f_torso_gravity
            
            tau_g_torso = np.zeros(10)
            
            # Read orientation for lever arm calculations
            quat = data.qpos[3:7]
            mat = np.zeros(9)
            mujoco.mju_quat2Mat(mat, quat)
            mat = mat.reshape(3, 3)
            up_vector = mat[:, 2]
            pitch_error = up_vector[0]  # sin(pitch_angle)
            
            sin_phi = pitch_error
            cos_phi = np.sqrt(max(0.0, 1.0 - sin_phi**2))
            
            # Torso CoM relative to hip joint axis
            z_com = 0.17  # Height
            x_com = -0.026  # Backward offset
            
            # Torso gravity moment (tilt moment)
            tau_hip_moment = f_leg_share * (z_com * sin_phi + x_com * cos_phi)
            
            # Extract current joint states for knee lever arms
            q_knee_l = data.qpos[leg_qpos_idx[3]]
            q_knee_r = data.qpos[leg_qpos_idx[8]]
            
            # A. Knee Extension Torques (to push up against torso weight)
            l_lever = 0.15
            tau_g_torso[3] = -f_leg_share * l_lever * np.sin(abs(q_knee_l))
            tau_g_torso[8] = f_leg_share * l_lever * np.sin(abs(q_knee_r))
            
            # B. Hip Pitch Torques (to perfectly balance the physical torso tilt moment!)
            tau_g_torso[0] = tau_hip_moment
            tau_g_torso[5] = -tau_hip_moment
            
            # C. Ankle Pitch Torques (to cancel transmission moments and keep foot flat)
            tau_g_torso[4] = -tau_hip_moment
            tau_g_torso[9] = tau_hip_moment
            
            # --- 3. Compute Soft Compliant Springs for Hips, Knees, Rolls, & Yaws ---
            q = data.qpos.copy()
            v = data.qvel.copy()
            q_j = q[leg_qpos_idx]
            v_j = v[leg_qvel_idx]
            
            # Constant targets (baseline standing coordinates)
            q_des = np.array(leg_stand_targets)
            qd_des = np.zeros(10)
            
            # Soft compliant orientation spring force
            tau_pd = Kp_posture * (q_des - q_j) + Kd_posture * (qd_des - v_j)
            
            # --- 4. Combine Gravity, Torso Moment, and Posture Springs ---
            tau_cmd = tau_g_legs + tau_g_torso + tau_pd
            tau_cmd = np.clip(tau_cmd, -tau_max_leg, tau_max_leg)
            
            data.ctrl[:] = tau_cmd
            
            # Step physics
            mujoco.mj_step(model, data)
            
            # --- 5. Real-Time Pose Capture Printing ---
            current_time = time.time()
            if current_time - last_print_time >= 0.5:
                last_print_time = current_time
                print("\n" + "="*60)
                print("           KBOT KINEMATIC POSE CAPTURE TOOL")
                print("="*60)
                print(f"Base Height (z): {data.qpos[2]:.4f} m")
                print(f"Base Orientation (quat): [{data.qpos[3]:.4f}, {data.qpos[4]:.4f}, {data.qpos[5]:.4f}, {data.qpos[6]:.4f}]")
                print("\nJoint Angles (qpos):")
                for name, idx in zip(leg_joint_names, leg_qpos_idx):
                    print(f"  {name:28s} : {data.qpos[idx]:.6f} rad ({np.degrees(data.qpos[idx]):.2f} deg)")
                
                # Provide a clean copy-pasteable Python list
                raw_list = [round(float(data.qpos[idx]), 6) for idx in leg_qpos_idx]
                print("\nCopy-Pasteable Joint Target Array:")
                print(f"leg_targets = {raw_list}")
            
            # Sync the viewer
            viewer.sync()

            # Maintain real-time simulation speed
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

except Exception as e:
    print(f"\n[ERROR] An exception occurred: {e}")
    import traceback
    traceback.print_exc()
