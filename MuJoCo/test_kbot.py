import mujoco
import mujoco.viewer
import time
import os
import numpy as np

# Import the modular controller
from wbc_controller import MinimalWBC

# Set paths
current_dir = os.path.dirname(os.path.abspath(__file__))
robot_path = os.path.join(current_dir, "kbot_in_scene.xml")

print("==================================================================")
print("             KBOT WHOLE-BODY OPERATIONAL SPACE CONTROL            ")
print("==================================================================")
print(f"Loading full-robot scene: {robot_path}")

try:
    # 1. Instantiate the WBC from the separate file
    wbc = MinimalWBC(robot_path)
    
    # 2. Pre-set joint position states in qpos
    for i in range(wbc.model.nu):
        joint_id = wbc.model.actuator_trnid[i, 0]
        qpos_idx = wbc.model.jnt_qposadr[joint_id]
        wbc.data.qpos[qpos_idx] = wbc.stand_target[i]
        
    # Pre-set initial base orientation to match target pitch
    wbc.data.qpos[3:7] = [np.cos(wbc.target_pitch/2), 0, np.sin(wbc.target_pitch/2), 0]
        
    # 3. Endpoint-Aware Zero-Impact Spawn Height Solver
    left_foot_body_id = mujoco.mj_name2id(wbc.model, mujoco.mjtObj.mjOBJ_BODY, "LFootBushing_GPF_1517_12")
    right_foot_body_id = mujoco.mj_name2id(wbc.model, mujoco.mjtObj.mjOBJ_BODY, "RFootBushing_GPF_1517_12")
    
    wbc.data.qpos[2] = 1.0  # Reset base height for calculation
    mujoco.mj_forward(wbc.model, wbc.data)
    
    local_foot_points = [
        np.array([-0.06, 0.0, 0.0]),  # Heel
        np.array([0.12, 0.0, 0.0]),   # Toe
        np.array([0.0, -0.03, 0.0]),  # Left lateral edge
        np.array([0.0, 0.03, 0.0])    # Right lateral edge
    ]
    
    lowest_z = 100.0
    
    # Find lowest point of left foot capsule
    pos_l = wbc.data.xpos[left_foot_body_id]
    mat_l = wbc.data.xmat[left_foot_body_id].reshape(3, 3)
    for p in local_foot_points:
        p_glob = pos_l + mat_l.dot(p)
        z_bottom = p_glob[2] - 0.02  # Capsule radius is 0.02
        if z_bottom < lowest_z:
            lowest_z = z_bottom
            
    # Find lowest point of right foot capsule
    pos_r = wbc.data.xpos[right_foot_body_id]
    mat_r = wbc.data.xmat[right_foot_body_id].reshape(3, 3)
    for p in local_foot_points:
        p_glob = pos_r + mat_r.dot(p)
        z_bottom = p_glob[2] - 0.02  # Capsule radius is 0.02
        if z_bottom < lowest_z:
            lowest_z = z_bottom
            
    # Position base tangent to the ground
    wbc.data.qpos[2] += -lowest_z
    mujoco.mj_forward(wbc.model, wbc.data)
    print(f"Zero-impact spawn height determined: z_base = {wbc.data.qpos[2]:.4f} m (exact flat tangent contact)")

    # 4. Launch Passive Viewer Loop
    print("\nLaunching MuJoCo viewer (passive)...")
    with mujoco.viewer.launch_passive(wbc.model, wbc.data) as viewer:
        viewer.cam.distance = 1.8
        viewer.cam.elevation = -12
        viewer.cam.azimuth = 90
        
        step_count = 0
        while viewer.is_running():
            step_start = time.time()
            
            # Step the WBC controller and simulator
            wbc.step()
            
            step_count += 1
            if step_count == 1 or step_count % 50 == 0:
                print(f"\n================ WBC STEP {step_count} DIAGNOSTICS ==================")
                print(f"COM x: {wbc.data.subtree_com[0][0]:.6f}")
                print(f"Left foot x: {wbc.data.xpos[wbc.left_foot_body_id][0]:.6f}, Right foot x: {wbc.data.xpos[wbc.right_foot_body_id][0]:.6f}")
                print(f"Torso pitch (rad): {wbc.get_pitch_from_quat(wbc.data.qpos[3:7]):.6f}")
                print(f"Task torque sum: {np.sum(wbc.tau_task):.6f}")
                # Print pitch task joint torques for Left Hip (6), Left Ankle (10), Right Hip (11), Right Ankle (15)
                print(f"Hips/Ankles task torques: L_Hip={wbc.tau_task[6]:.4f}, L_Ankle={wbc.tau_task[10]:.4f}, R_Hip={wbc.tau_task[11]:.4f}, R_Ankle={wbc.tau_task[15]:.4f}")
                print("===================================================\n")
            
            # Sync the viewer
            viewer.sync()

            # Maintain real-time simulation speed
            time_until_next_step = wbc.model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

except Exception as e:
    print(f"\n[ERROR] An exception occurred: {e}")
    import traceback
    traceback.print_exc()
