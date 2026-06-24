"""
requirements.py
================
Extracts joint-level requirements from a torque/velocity time-series:
  - T_cont      : RMS torque (continuous rating requirement) [N·m]
  - T_peak      : Maximum absolute torque [N·m]
  - omega_max   : Maximum absolute angular velocity [rad/s]
  - P_mech_rms  : RMS mechanical power [W]
  - E_negative  : Total negative mechanical work per cycle (regen energy) [J]
"""

import numpy as np


def extract_requirements(profile: dict) -> dict:
    """
    Parameters
    ----------
    profile : dict
        Output of inverse_dynamics.compute_joint_profile()
        Keys: 'time', 'torque', 'velocity', 'power', 'joint'

    Returns
    -------
    dict with joint requirements and derived quantities
    """
    t     = profile["time"]
    tau   = profile["torque"]
    omega = profile["velocity"]
    power = profile["power"]
    dt    = float(t[1] - t[0]) if len(t) > 1 else 0.002

    # ── Core requirements ────────────────────────────────────────────────────
    T_cont    = float(np.sqrt(np.mean(tau**2)))
    T_peak    = float(np.max(np.abs(tau)))
    omega_max = float(np.max(np.abs(omega)))

    # ── Power metrics ────────────────────────────────────────────────────────
    P_mech_rms = float(np.sqrt(np.mean(power**2)))
    P_peak_pos = float(np.max(power))
    P_peak_neg = float(np.min(power))

    # ── Regen energy: integrate negative power over each gait cycle ──────────
    neg_power  = np.where(power < 0, power, 0.0)
    E_negative = float(-np.trapz(neg_power, t))       # J (positive value)

    # Cycle period estimate (assumes uniform cycling)
    T_total  = float(t[-1] - t[0])
    E_negative_per_cycle = E_negative  # already integrated over n_cycles × t_cycle

    # Time fraction in negative power (regen opportunity fraction)
    n_neg     = np.sum(power < 0)
    n_total   = len(power)
    regen_frac = float(n_neg / n_total) if n_total > 0 else 0.0

    # Mean regen power (when regenerating)
    regen_mask = power < 0
    P_regen_mean = float(-np.mean(power[regen_mask])) if np.any(regen_mask) else 0.0
    P_regen_peak = float(-P_peak_neg)

    return {
        # Hard constraints
        "T_cont_Nm":     T_cont,
        "T_peak_Nm":     T_peak,
        "omega_max_rads": omega_max,
        # Power
        "P_mech_rms_W":  P_mech_rms,
        "P_peak_pos_W":  P_peak_pos,
        "P_peak_neg_W":  P_peak_neg,
        # Regen characterisation
        "E_regen_J":     E_negative_per_cycle,
        "P_regen_mean_W": P_regen_mean,
        "P_regen_peak_W": P_regen_peak,
        "regen_fraction": regen_frac,
        # Meta
        "joint":         profile["joint"],
        "T_total_s":     T_total,
    }
