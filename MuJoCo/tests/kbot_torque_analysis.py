import numpy as np
import mujoco
from loco_mujoco.trajectory import Trajectory
from loco_mujoco.environments.humanoids.kbot import KBot

def analyze_torques(traj_path):
    print(f"Loading trajectory from {traj_path}...")
    traj = Trajectory.load(traj_path)
    
    # Instantiate the KBot environment to get the model
    env = KBot()
    model = env.model
    data = env.data
    
    # Extract joint indices mapping from trajectory
    qpos_indices = traj.info.joint_name2ind_qpos
    qvel_indices = traj.info.joint_name2ind_qvel
    
    n_samples = traj.data.qpos.shape[0]
    dt = env.dt
    
    # Store maximums
    max_rpm = {}
    max_torque = {}
    
    # Joints to analyze (we ignore the floating base)
    joint_names = [j for j in traj.info.joint_names if j != "floating_base"]
    for j in joint_names:
        max_rpm[j] = 0.0
        max_torque[j] = 0.0
        
    print(f"Analyzing {n_samples} frames...")
    
    # We need qacc for inverse dynamics. We can approximate qacc using finite differences of qvel.
    qvel_data = traj.data.qvel
    qacc_data = np.zeros_like(qvel_data)
    qacc_data[:-1] = (qvel_data[1:] - qvel_data[:-1]) / dt
    
    for i in range(n_samples):
        # Set kinematics
        data.qpos[:] = traj.data.qpos[i]
        data.qvel[:] = qvel_data[i]
        data.qacc[:] = qacc_data[i]
        
        # Run inverse dynamics
        mujoco.mj_inverse(model, data)
        
        # Parse results for each joint
        for j in joint_names:
            # Velocity in rad/s -> RPM: (rad/s) * (60 / 2pi)
            vel_rads = np.abs(qvel_data[i, qvel_indices[j][0]])
            rpm = vel_rads * (60.0 / (2 * np.pi))
            
            # Torque is stored in qfrc_inverse
            qfrc_idx = qvel_indices[j][0]
            torque = np.abs(data.qfrc_inverse[qfrc_idx])
            
            if rpm > max_rpm[j]: max_rpm[j] = rpm
            if torque > max_torque[j]: max_torque[j] = torque

    print("\n" + "="*50)
    print("KBOT MAXIMUM HARDWARE REQUIREMENTS (WALKING)")
    print("="*50)
    print(f"{'Joint Name':<25} | {'Max Speed (RPM)':<15} | {'Max Torque (Nm)':<15}")
    print("-" * 50)
    for j in joint_names:
        print(f"{j:<25} | {max_rpm[j]:<15.2f} | {max_torque[j]:<15.2f}")
    print("="*50)

if __name__ == "__main__":
    analyze_torques("loco-mujoco/kbot_walk_retargeted.npz")
