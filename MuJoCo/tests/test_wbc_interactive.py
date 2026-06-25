import mujoco
import mujoco.viewer
import time
import os
import sys
import numpy as np
import scipy.io as sio

# Set paths before importing controllers
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, '..', 'controllers'))

# Import the modular controller and gait generator
from wbc_controller import MinimalWBC

print("==================================================================")
print("         KBOT LEGS-ONLY WHOLE-BODY OPERATIONAL SPACE CONTROL      ")
print("==================================================================")

# 1. Path to the full 20-DoF model from loco-mujoco
full_scene_path = os.path.join(current_dir, "..", "loco-mujoco", "loco_mujoco", "models", "kbot_v2", "kbot_v2.xml")

# Global state for interactive mode
current_mode = "WALK"
walk_start_time = 0.0

try:
    # 1. Instantiate the WBC from the separate file with lightly crouched knee_bend
    wbc = MinimalWBC(full_scene_path, knee_bend=0.45)
    
    # 2. Load the LocoMuJoCo Gait Replay (using the 07_01 walk clip)
    from gait_replay import GaitReplay
    GAIT_DATA_DIR = os.path.join(current_dir, "..", "loco-mujoco", "gait_data")
    walk_clip = os.path.join(GAIT_DATA_DIR, "walk", "07_01_poses.npz")
    gait_replay = GaitReplay(walk_clip, wbc.model, loop=True)
    
    print(f"Zero-impact spawn height determined: z_base = {wbc.data.qpos[2]:.4f} m (exact flat tangent contact)")

    # Define joints of interest
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
    
    # 5. Set up key callback for viewer to toggle modes if desired
    def key_callback(keycode):
        global current_mode, walk_start_time
        if keycode == 32:  # Spacebar
            if current_mode == "STAND":
                current_mode = "WALK"
                walk_start_time = time.time() - getattr(wbc.data, 'time', 0)
                print(f"\n[MODE] Switched to WALK (tracking trajectory from t=0)")
            else:
                current_mode = "STAND"
                print("\n[MODE] Switched to STAND (balanced kinematic posture)")

    # 6. Main Simulation Loop
    with mujoco.viewer.launch_passive(wbc.model, wbc.data, key_callback=key_callback) as viewer:
        print("\n==================================================================")
        print(" INTERACTIVE MODE:")
        print(" Press [SPACEBAR] in the viewer to toggle between STAND and WALK.")
        print(" (Defaulting to WALK to demonstrate the trajectory)")
        print("==================================================================\n")
        
        step_count = 0
        while viewer.is_running():
            step_start = time.time()
            
            # --- Controller Update ---
            if current_mode == "STAND":
                wbc.set_reference_trajectory(None, None)
                wbc.step()
            elif current_mode == "WALK":
                # Get the current elapsed walking time
                elapsed_walk = time.time() - walk_start_time
                
                # Fetch reference joint states (these are already just the 20 actuator joints)
                qpos_ref, qvel_ref = gait_replay.get_targets(elapsed_walk)
                
                # Fetch base targets to extract height and forward trajectory
                base_pos_ref, base_quat_ref, base_vel_ref = gait_replay.get_base_target(elapsed_walk)
                target_height = base_pos_ref[2]
                
                # Pass full dynamic cartesian targets to the Walking WBC
                wbc.set_reference_trajectory(
                    qpos_ref=qpos_ref, 
                    qvel_ref=qvel_ref, 
                    base_pos=base_pos_ref, 
                    base_vel=base_vel_ref
                )
                
                # We could also feed the com_offset and pitch to the WBC, but for now
                # the WBC will balance over the feet with the default pitch while tracking legs.
                wbc.step({'torso_pitch': -0.05, 'com_offset': [0, 0, target_height - wbc.target_height]})
            else:
                # In STAND mode, it defaults to the posture PD and CoM stabilization
                wbc.step()
            
            step_count += 1
            if step_count % 5 == 0:
                viewer.sync()
 
            # Maintain real-time simulation speed
            time_until_next_step = wbc.model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

except Exception as e:
    print(f"\n[ERROR] An exception occurred: {e}")
    import traceback
    traceback.print_exc()
