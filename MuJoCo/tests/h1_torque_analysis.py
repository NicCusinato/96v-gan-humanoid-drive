import numpy as np
import mujoco
from loco_mujoco.task_factories import ImitationFactory, DefaultDatasetConf
from loco_mujoco.environments.humanoids.unitreeH1 import UnitreeH1

def analyze_torques():
    print("Initializing UnitreeH1 environment...")
    env = UnitreeH1()
    model = env.model
    data = env.data
    
    print("Loading perfectly-fitted UnitreeH1 walk trajectory...")
    traj = ImitationFactory.get_default_traj(env, DefaultDatasetConf("walk"))
    
    # Extract joint indices mapping from trajectory
    qpos_indices = traj.info.joint_name2ind_qpos
    qvel_indices = traj.info.joint_name2ind_qvel
    
    n_samples = traj.data.qpos.shape[0]
    dt = env.dt
    
    # Store maximums
    max_rpm = {}
    max_torque = {}
    
    # Joints to analyze (we ignore the floating base)
    joint_names = [j for j in traj.info.joint_names if j != "root"]
    for j in joint_names:
        max_rpm[j] = 0.0
        max_torque[j] = 0.0
        
    print(f"Analyzing {n_samples} frames...")
    
    qvel_data = traj.data.qvel
    qacc_data = np.zeros_like(qvel_data)
    qacc_data[:-1] = (qvel_data[1:] - qvel_data[:-1]) / dt
    
    step = 10
    print(f"Analyzing {n_samples//step} frames (sampled every {step} frames)...")
    for i in range(0, n_samples, step):
        # Set kinematics
        data.qpos[:] = traj.data.qpos[i]
        data.qvel[:] = qvel_data[i]
        data.qacc[:] = qacc_data[i]
        
        # Run inverse dynamics
        mujoco.mj_inverse(model, data)
        
        # Parse results for each joint
        for j in joint_names:
            vel_rads = np.abs(qvel_data[i, qvel_indices[j][0]])
            rpm = vel_rads * (60.0 / (2 * np.pi))
            
            qfrc_idx = qvel_indices[j][0]
            torque = np.abs(data.qfrc_inverse[qfrc_idx])
            
            if rpm > max_rpm[j]: max_rpm[j] = rpm
            if torque > max_torque[j]: max_torque[j] = torque

    print("\n" + "="*50)
    print("UNITREE H1 MAXIMUM HARDWARE REQUIREMENTS (WALKING)")
    print("="*50)
    print(f"{'Joint Name':<25} | {'Max Speed (RPM)':<15} | {'Max Torque (Nm)':<15}")
    print("-" * 50)
    for j in joint_names:
        print(f"{j:<25} | {max_rpm[j]:<15.2f} | {max_torque[j]:<15.2f}")
    print("="*50)

if __name__ == "__main__":
    analyze_torques()
