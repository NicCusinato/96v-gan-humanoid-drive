"""
Test Gait Replay — Step 1: Full-Body Joint-Space PD Tracking

Loads a loco-mujoco walking clip and replays it on the KBot V2 model
(from loco-mujoco, matching the gait data source) using PD + gravity
compensation. Base is kinematically driven from reference (puppet mode).

Usage:
    python tests/test_gait_replay.py
"""

import mujoco
import mujoco.viewer
import numpy as np
import os
import sys
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, '..', 'controllers'))

from gait_replay import GaitReplay

# === Configuration ===
GAIT_DATA_DIR = os.path.join(current_dir, "..", "loco-mujoco", "gait_data")
WALK_CLIP = os.path.join(GAIT_DATA_DIR, "walk", "07_01_poses.npz")

# Use the loco-mujoco kbot_v2 model — matches the gait data source exactly
MODEL_PATH = os.path.join(current_dir, "..", "loco-mujoco", "loco_mujoco",
                          "models", "kbot_v2", "kbot_v2.xml")

# === Main ===
print("=" * 66)
print("  STEP 1: FULL-BODY GAIT REPLAY (PD + GRAVITY COMP)")
print("=" * 66)

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

gait = GaitReplay(WALK_CLIP, model, loop=True)

# Set initial pose from the first frame
qpos_init, _ = gait.get_targets(0.0)
base_pos_init, base_quat_init, _ = gait.get_base_target(0.0)

data.qpos[0:3] = base_pos_init
data.qpos[3:7] = base_quat_init
for i in range(model.nu):
    jid = model.actuator_trnid[i, 0]
    data.qpos[model.jnt_qposadr[jid]] = qpos_init[i]
mujoco.mj_forward(model, data)

# === PD Gains ===
KP = np.zeros(model.nu)
KD = np.zeros(model.nu)
for i in range(model.nu):
    jid = model.actuator_trnid[i, 0]
    jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
    if "hip_pitch" in jname or "knee" in jname:
        KP[i], KD[i] = 300.0, 15.0   # RS04 — big motors
    elif "hip_roll" in jname or "hip_yaw" in jname or "ankle" in jname:
        KP[i], KD[i] = 200.0, 10.0   # RS03/RS02
    elif "shoulder" in jname or "elbow" in jname:
        KP[i], KD[i] = 100.0, 5.0    # RS03 arms
    else:
        KP[i], KD[i] = 50.0, 3.0     # RS00 wrists

# === Tracking metrics ===
max_error = np.zeros(model.nu)
joint_names = []
for i in range(model.nu):
    jid = model.actuator_trnid[i, 0]
    joint_names.append(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid))

print(f"\n  {model.nu} actuators mapped. Running simulation...\n")

# === Simulation loop ===
sim_time = 0.0
step_count = 0

with mujoco.viewer.launch_passive(model, data) as viewer:
    print("  Viewer open. Close window to stop and see results.")
    print("  Base = puppet mode | Joints = PD + gravity comp\n")

    while viewer.is_running():
        qpos_ref, qvel_ref = gait.get_targets(sim_time)
        base_pos_ref, base_quat_ref, base_vel_ref = gait.get_base_target(sim_time)

        # Kinematically drive the base
        data.qpos[0:3] = base_pos_ref
        data.qpos[3:7] = base_quat_ref
        data.qvel[0:3] = base_vel_ref[0:3]
        data.qvel[3:6] = base_vel_ref[3:6]

        mujoco.mj_forward(model, data)

        # Gravity + Coriolis compensation (all actuated DoFs)
        gravity_comp = data.qfrc_bias[6:]  # skip 6 floating-base DoFs

        # PD torques
        torques = np.zeros(model.nu)
        for i in range(model.nu):
            jid = model.actuator_trnid[i, 0]
            qidx = model.jnt_qposadr[jid]
            vidx = model.jnt_dofadr[jid]

            pos_err = qpos_ref[i] - data.qpos[qidx]
            vel_err = qvel_ref[i] - data.qvel[vidx]
            torques[i] = KP[i] * pos_err + KD[i] * vel_err
            max_error[i] = max(max_error[i], abs(pos_err))

        torques += gravity_comp

        # Clamp to actuator limits
        for i in range(model.nu):
            torques[i] = np.clip(torques[i],
                                 model.actuator_ctrlrange[i, 0],
                                 model.actuator_ctrlrange[i, 1])

        data.ctrl[:] = torques
        mujoco.mj_step(model, data)
        sim_time = data.time
        step_count += 1

        # Sum up vertical contact forces (Z-axis) to verify feet are hitting the ground
        total_fz = 0.0
        for j in range(data.ncon):
            contact = data.contact[j]
            # Extract contact force (6D vector in contact frame)
            c_force = np.zeros(6, dtype=np.float64)
            mujoco.mj_contactForce(model, data, j, c_force)
            total_fz += c_force[0] # The normal force is the first element in contact frame

        if step_count % 5 == 0:
            viewer.sync()
            
        if step_count % 100 == 0:
            print(f"  [Time {sim_time:.2f}s] Ground Force (Fz): {total_fz:.1f} N | Max Torque: {np.max(np.abs(torques)):.1f} Nm")

# === Results ===
print("\n" + "=" * 66)
print("  TRACKING RESULTS")
print("=" * 66)
print(f"  Simulated {sim_time:.2f}s ({step_count} steps)\n")
print(f"  {'Joint':<35} {'Max Error (rad)':>15} {'Max Error (deg)':>15}")
print(f"  {'-'*35} {'-'*15} {'-'*15}")
for i, name in enumerate(joint_names):
    print(f"  {name:<35} {max_error[i]:>15.4f} {np.degrees(max_error[i]):>15.2f}")

avg_err = np.mean(max_error)
print(f"\n  Average max tracking error: {avg_err:.4f} rad ({np.degrees(avg_err):.2f} deg)")
if avg_err < 0.1:
    print("  ✓ PASS")
else:
    print("  ✗ FAIL — check gains or joint mapping")
