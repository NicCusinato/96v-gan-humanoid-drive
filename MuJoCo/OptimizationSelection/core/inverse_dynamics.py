"""
inverse_dynamics.py
====================
Produces joint-level torque τ(t) and angular velocity ω(t) time-series
for a single leg joint using one of three sources:

  1. 'analytic'   – Planar Newton-Euler inverse dynamics on the Kbot 3-DOF leg
                    with sinusoidal / stair gait trajectory generation
  2. 'mujoco_csv' – Load a CSV exported by test_kbot_legs.py (real sim data)
  3. 'online'     – Embedded reference profiles derived from published datasets
                    (Atlas, Cassie, human gait data) scaled to robot params

Returns a dict with keys: 'time', 'torque', 'velocity', 'power', 'joint'
"""

import numpy as np
import os
import sys


# ─────────────────────────────────────────────────────────────────────────────
#  ANALYTIC INVERSE DYNAMICS
# ─────────────────────────────────────────────────────────────────────────────

def _joint_trajectory_analytic(cfg: dict) -> dict:
    """
    Generates joint-angle, velocity, and acceleration profiles for one joint
    using a parameterised gait model on the Kbot 3-DOF planar leg.

    Gait types supported:
      stand         – static hold at nominal crouch
      squat         – sinusoidal vertical CoM oscillation
      step_in_place – swing/stance stepping with cycloidal foot arc
      stair_climb   – repetitive knee flexion/extension climbing a riser
    """
    robot = cfg["robot"]
    gait  = cfg["gait"]

    t_cycle = gait["t_cycle_s"]
    dt      = gait["dt_s"]
    n_cyc   = gait["n_cycles"]
    T_total = t_cycle * n_cyc
    t       = np.arange(0, T_total, dt)

    gait_type   = gait["gait_type"]
    joint       = gait["target_joint"]
    step_freq   = gait["step_freq_hz"]
    step_h      = gait["step_height_m"]
    slope_rad   = np.deg2rad(gait.get("slope_deg", 0.0))
    stair_rise  = gait.get("stair_rise_m", 0.18)

    # ── Nominal joint angles (mid-range of limits) ──────────────────────────
    limits = robot["joint_limits_rad"]

    # Choose gait-appropriate nominal angles
    if gait_type == "stand":
        q_hip_nom   =  0.30   # rad, slight forward lean
        q_knee_nom  =  0.45   # rad, lightly crouched
        q_ankle_nom = -0.15   # rad, plantarflexed
    elif gait_type == "squat":
        q_hip_nom   =  0.30
        q_knee_nom  =  0.45
        q_ankle_nom = -0.15
    elif gait_type in ("step_in_place", "stair_climb"):
        q_hip_nom   =  0.20
        q_knee_nom  =  0.40
        q_ankle_nom = -0.10
    else:
        q_hip_nom   =  0.25
        q_knee_nom  =  0.40
        q_ankle_nom = -0.10

    omega = 2 * np.pi * step_freq

    # ── Generate per-joint angle profile ────────────────────────────────────
    if gait_type == "stand":
        q_hip   = q_hip_nom   * np.ones_like(t)
        q_knee  = q_knee_nom  * np.ones_like(t)
        q_ankle = q_ankle_nom * np.ones_like(t)

    elif gait_type == "squat":
        squat_depth_knee = 0.40   # rad additional knee flex at bottom
        squat_depth_hip  = 0.25
        squat_depth_ank  = 0.10
        phase = (1 - np.cos(omega * t)) / 2   # 0→1→0
        q_hip   = q_hip_nom   + squat_depth_hip  * phase
        q_knee  = q_knee_nom  + squat_depth_knee * phase
        q_ankle = q_ankle_nom - squat_depth_ank  * phase   # dorsiflexion

    elif gait_type == "step_in_place":
        # Swing phase: hip extends, knee flexes (cycloid), ankle dorsiflexes
        # Stance phase: slight push-off extension
        swing_knee_amp  = 0.60   # rad peak knee flex during swing
        swing_hip_amp   = 0.25   # rad hip extension
        swing_ank_amp   = 0.20   # rad ankle dorsiflexion
        push_ank_amp    = 0.10   # rad plantarflexion push-off

        # Cycloidal phase
        phase_raw = (omega * t) % (2 * np.pi)
        # Swing window: 0 → π (first half), stride: stance second half
        swing = 0.5 * (1 - np.cos(phase_raw))            # 0→1→0 each half cycle
        push  = np.where(phase_raw > np.pi,
                         0.5 * (1 - np.cos(phase_raw - np.pi)), 0.0)

        q_knee  = q_knee_nom  + swing_knee_amp * swing
        q_hip   = q_hip_nom   - swing_hip_amp  * swing + 0.05 * push
        q_ankle = q_ankle_nom - swing_ank_amp   * swing + push_ank_amp * push

    elif gait_type == "stair_climb":
        # Each step: deep knee flexion to clear riser, then extension
        knee_flex_amp = min(np.arcsin(stair_rise / robot["link_lengths_m"]["thigh"]) * 1.8, 1.5)
        hip_flex_amp  = knee_flex_amp * 0.55
        ank_flex_amp  = 0.20

        phase_raw = (omega * t) % (2 * np.pi)
        flex  = np.clip(np.sin(phase_raw), 0, None)   # only positive half
        ext   = np.clip(-np.sin(phase_raw), 0, None)

        q_knee  = q_knee_nom + knee_flex_amp * flex
        q_hip   = q_hip_nom  + hip_flex_amp  * flex - 0.10 * ext
        q_ankle = q_ankle_nom - ank_flex_amp * flex + 0.05 * ext

        # Add slope contribution
        q_hip   += slope_rad
        q_ankle -= slope_rad

    else:
        raise ValueError(f"Unknown gait_type: {gait_type}")

    # Clip to joint limits
    q_hip   = np.clip(q_hip,   *limits["hip_pitch"])
    q_knee  = np.clip(q_knee,  *limits["knee_pitch"])
    q_ankle = np.clip(q_ankle, *limits["ankle_pitch"])

    # Select active joint
    joint_map = {
        "hip_pitch":   q_hip,
        "knee_pitch":  q_knee,
        "ankle_pitch": q_ankle,
    }
    if joint not in joint_map:
        raise ValueError(f"Unknown joint: {joint}. Choose from {list(joint_map)}")

    q_all = {"hip_pitch": q_hip, "knee_pitch": q_knee, "ankle_pitch": q_ankle}

    return t, q_all


def _newton_euler_torques(t: np.ndarray, q_all: dict, cfg: dict) -> dict:
    """
    Computes joint torques via planar Newton-Euler inverse dynamics on the
    3-DOF Kbot leg model (hip→knee→ankle).

    Sign convention: positive torque = joint extends (flexion is negative).
    Gravity points in -z direction (world frame).
    """
    robot = cfg["robot"]
    g_val = robot["g"]
    dt    = cfg["gait"]["dt_s"]

    # Link parameters
    l1 = robot["link_lengths_m"]["thigh"]
    l2 = robot["link_lengths_m"]["shank"]

    m1 = robot["link_masses_kg"]["thigh"]
    m2 = robot["link_masses_kg"]["shank"]
    m3 = robot["link_masses_kg"]["foot"]

    # COM offsets (radial distance along link from proximal joint)
    r1_z = abs(robot["com_offsets_m"]["thigh"][1])  # ≈ 0.194 m
    r2_z = abs(robot["com_offsets_m"]["shank"][1])  # ≈ 0.104 m
    r3_z = abs(robot["com_offsets_m"]["foot"][1])   # ≈ 0.031 m

    I1 = robot["link_inertias_kgm2"]["thigh"]
    I2 = robot["link_inertias_kgm2"]["shank"]
    I3 = robot["link_inertias_kgm2"]["foot"]

    # Payload + upper body acts as a force on the hip
    m_upper = robot["total_mass_kg"] + robot["payload_mass_kg"] - (m1 + m2 + m3)
    m_upper = max(m_upper, 0.0)

    q_hip   = q_all["hip_pitch"]
    q_knee  = q_all["knee_pitch"]
    q_ankle = q_all["ankle_pitch"]

    # Numerically differentiate angles → velocities and accelerations
    dq_hip   = np.gradient(q_hip,   dt)
    dq_knee  = np.gradient(q_knee,  dt)
    dq_ankle = np.gradient(q_ankle, dt)

    ddq_hip   = np.gradient(dq_hip,   dt)
    ddq_knee  = np.gradient(dq_knee,  dt)
    ddq_ankle = np.gradient(dq_ankle, dt)

    # ── Segment absolute angles ──────────────────────────────────────────────
    # Thigh angle in world frame (from vertical)
    theta1 = q_hip                        # hip flex
    theta2 = q_hip - q_knee              # shank angle (knee measured from thigh)
    theta3 = q_hip - q_knee - q_ankle    # foot angle

    # ── Gravity torques (quasi-static contribution) ──────────────────────────
    # Torque at joint = sum of (mass × g × horizontal distance to that joint)

    # Ankle torque: only foot weight
    tau_ankle_grav = (m3 * g_val * r3_z * np.cos(theta3))

    # Knee torque: shank COM + foot weight
    tau_knee_grav = (
        m2 * g_val * r2_z * np.cos(theta2)
        + (m3 * g_val) * (l2 * np.cos(theta2) + r3_z * np.cos(theta3))
    )

    # Hip torque: thigh COM + shank + foot + upper body
    tau_hip_grav = (
        m1 * g_val * r1_z * np.cos(theta1)
        + (m2 * g_val) * (l1 * np.cos(theta1) + r2_z * np.cos(theta2))
        + (m3 * g_val) * (l1 * np.cos(theta1) + l2 * np.cos(theta2) + r3_z * np.cos(theta3))
        + m_upper * g_val * 0.0   # upper-body force on hip (zero moment arm in sagittal avg.)
    )

    # ── Inertial torques ─────────────────────────────────────────────────────
    tau_ankle_dyn  = I3 * ddq_ankle
    tau_knee_dyn   = (I2 * ddq_knee + m2 * r2_z**2 * ddq_knee
                      + I3 * ddq_ankle + m3 * l2**2 * ddq_knee)
    tau_hip_dyn    = (I1 * ddq_hip  + m1 * r1_z**2 * ddq_hip
                      + I2 * ddq_knee + m2 * l1**2 * ddq_hip
                      + I3 * ddq_ankle + m3 * (l1**2 + l2**2) * ddq_hip)

    # ── Total torques ────────────────────────────────────────────────────────
    tau_hip   = tau_hip_grav   + tau_hip_dyn
    tau_knee  = tau_knee_grav  + tau_knee_dyn
    tau_ankle = tau_ankle_grav + tau_ankle_dyn

    torque_map = {
        "hip_pitch":   tau_hip,
        "knee_pitch":  tau_knee,
        "ankle_pitch": tau_ankle,
    }
    velocity_map = {
        "hip_pitch":   dq_hip,
        "knee_pitch":  dq_knee,
        "ankle_pitch": dq_ankle,
    }

    return torque_map, velocity_map


# ─────────────────────────────────────────────────────────────────────────────
#  MUJOCO CSV LOADER
# ─────────────────────────────────────────────────────────────────────────────

def _load_mujoco_csv(cfg: dict) -> dict:
    """
    Loads a CSV exported by test_kbot_legs.py.
    Expected columns: time, <joint>_angle, <joint>_velocity, <joint>_net_torque
    where <joint> is e.g. 'left_knee', 'left_hip_pitch', etc.
    """
    import pandas as pd

    joint     = cfg["gait"]["target_joint"]   # e.g. "knee_pitch"
    csv_path  = cfg["gait"]["mujoco_csv_path"]

    if not os.path.isfile(csv_path):
        raise FileNotFoundError(
            f"MuJoCo CSV not found: {csv_path}\n"
            "Run test_kbot_legs.py in MuJoCo to generate it first."
        )

    df = pd.read_csv(csv_path)

    # Map config joint name → CSV column prefix
    col_map = {
        "hip_pitch":   "left_hip_pitch",
        "knee_pitch":  "left_knee",
        "ankle_pitch": "left_ankle",
    }
    col = col_map.get(joint, joint)

    required = [f"{col}_angle", f"{col}_velocity", f"{col}_net_torque"]
    for r in required:
        if r not in df.columns:
            raise KeyError(
                f"Column '{r}' not found in CSV. Available: {list(df.columns)}\n"
                "Ensure you ran test_kbot_legs.py with STEP_IN_PLACE or SQUAT mode."
            )

    t      = df["time"].values
    tau    = df[f"{col}_net_torque"].values
    omega  = df[f"{col}_velocity"].values

    return t, tau, omega


# ─────────────────────────────────────────────────────────────────────────────
#  ONLINE REFERENCE PROFILES
# ─────────────────────────────────────────────────────────────────────────────

# Reference profiles derived from published gait data (human + legged robot)
# Sources:
#   - Winter DA (2009) Biomechanics and Motor Control of Human Movement
#   - Mooney et al. (2014) MIT Hermes / ATLAS gait data
#   - Bledt et al. (2018) MIT Cheetah 3 gait
# Profiles are normalised to body mass and scaled to Kbot parameters.

_ONLINE_PROFILES = {
    "knee_pitch": {
        # One gait cycle, normalised 0→1 in phase
        "phase":  [0.0,  0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.0],
        "q_rad":  [0.15, 0.25, 0.45, 0.60, 0.50, 0.30, 0.20, 0.35, 0.50, 0.30, 0.15],
        "tau_Nm_per_kg": [0.4, 0.6, 0.9, 0.5, 0.3, 0.2, 0.1, 0.5, 0.8, 0.5, 0.4],
    },
    "hip_pitch": {
        "phase":  [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0],
        "q_rad":  [0.2, 0.15, 0.05, -0.05, -0.1, 0.0, 0.1, 0.2, 0.3, 0.25, 0.2],
        "tau_Nm_per_kg": [0.6, 0.4, 0.2, 0.1, 0.2, 0.5, 0.6, 0.4, 0.3, 0.4, 0.6],
    },
    "ankle_pitch": {
        "phase":  [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0],
        "q_rad":  [-0.1, -0.08, -0.05, 0.0, 0.05, 0.10, 0.08, 0.0, -0.05, -0.08, -0.1],
        "tau_Nm_per_kg": [0.2, 0.3, 0.5, 0.8, 1.2, 1.5, 0.8, 0.2, 0.1, 0.1, 0.2],
    },
}


def _load_online_profile(cfg: dict):
    """Scales published biomechanics profiles to the configured robot mass."""
    import scipy.interpolate as interp

    robot  = cfg["robot"]
    gait   = cfg["gait"]
    joint  = gait["target_joint"]
    t_cyc  = gait["t_cycle_s"]
    dt     = gait["dt_s"]
    n_cyc  = gait["n_cycles"]

    body_mass = robot["total_mass_kg"] + robot["payload_mass_kg"]

    if joint not in _ONLINE_PROFILES:
        raise ValueError(f"No online profile for joint '{joint}'")

    prof = _ONLINE_PROFILES[joint]
    phase_pts = np.array(prof["phase"])
    q_pts     = np.array(prof["q_rad"])
    tau_pts   = np.array(prof["tau_Nm_per_kg"]) * body_mass

    # Interpolate to fine time grid for n_cycles
    T_total = t_cyc * n_cyc
    t_out   = np.arange(0, T_total, dt)
    phase_t = (t_out % t_cyc) / t_cyc   # normalised phase 0→1

    q_interp   = interp.interp1d(phase_pts, q_pts,   kind="cubic", fill_value="extrapolate")
    tau_interp = interp.interp1d(phase_pts, tau_pts, kind="cubic", fill_value="extrapolate")

    q   = q_interp(phase_t)
    tau = tau_interp(phase_t)
    omega = np.gradient(q, dt)

    return t_out, tau, omega


# ─────────────────────────────────────────────────────────────────────────────
#  PUBLIC API
# ─────────────────────────────────────────────────────────────────────────────

def compute_joint_profile(cfg: dict) -> dict:
    """
    Main entry point.
    Returns:
        {
          'time':     np.ndarray   [s]
          'torque':   np.ndarray   [N·m], joint torque
          'velocity': np.ndarray   [rad/s], joint angular velocity
          'power':    np.ndarray   [W]
          'joint':    str          joint name
        }
    """
    source = cfg["gait"]["profile_source"]
    joint  = cfg["gait"]["target_joint"]

    if source == "analytic":
        t, q_all = _joint_trajectory_analytic(cfg)
        torque_map, vel_map = _newton_euler_torques(t, q_all, cfg)
        tau   = torque_map[joint]
        omega = vel_map[joint]

    elif source == "mujoco_csv":
        t, tau, omega = _load_mujoco_csv(cfg)

    elif source == "online":
        t, tau, omega = _load_online_profile(cfg)

    else:
        raise ValueError(f"Unknown profile_source: '{source}'. "
                         "Use 'analytic', 'mujoco_csv', or 'online'.")

    power = tau * omega

    return {
        "time":     t,
        "torque":   tau,
        "velocity": omega,
        "power":    power,
        "joint":    joint,
        "source":   source,
    }
