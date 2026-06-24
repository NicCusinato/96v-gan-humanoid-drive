"""
regen.py
=========
Supercapacitor / regenerative braking model for a single joint with
a decentralised local supercap bank.

Physics model:
  - Regen energy comes from negative mechanical work at the joint
  - Converted to electrical via motor + gear efficiency in reverse
  - Current injects into the supercap through its ESR
  - Voltage rise must stay within allowed cap voltage range

Constraints applied here:
  A. Minimum regen power: P_regen > regen_min_power_W
     (if regen is negligible, mark as poor fit for supercap storage)
  B. Over-voltage: peak cap voltage < V_max × safety_margin
     (if regen current saturates the cap too fast, reject)

Also computes:
  - Regen energy per cycle (after losses)
  - Estimated cap voltage swing
  - Regen power match score (0–1)
"""

import numpy as np


def compute_regen(candidate,      # CandidateResult (pre-populated by evaluator.py)
                  profile: dict,  # output of inverse_dynamics.compute_joint_profile()
                  reqs: dict,
                  cfg: dict) -> dict:
    """
    Adds regen metrics to a CandidateResult.

    Returns a dict of regen metrics:
      regen_energy_J        : Net regen energy delivered to cap per total sim period [J]
      regen_power_mean_W    : Mean regen power into cap [W]
      regen_power_peak_W    : Peak regen power into cap [W]
      I_regen_mean_A        : Mean regen current [A]
      I_regen_peak_A        : Peak regen current [A]
      V_cap_delta_V         : Estimated cap voltage rise from one regen event [V]
      V_cap_peak_V          : Estimated peak cap voltage after regen [V]
      cap_power_match_score : 0–1 score of how well regen matches cap capability
      regen_ok              : bool — passes both regen constraints
      fail_reasons          : list of strings
    """
    scap  = cfg["supercap"]
    gait  = cfg["gait"]
    elec  = cfg["electrical"]

    C       = scap["capacitance_F"]
    ESR     = scap["esr_ohm"]
    V_min   = scap["V_min"]
    V_max   = scap["V_max"]
    V_nom   = scap["V_nominal"]
    P_regen_min = scap["regen_min_power_W"]
    V_margin    = scap["regen_overvoltage_margin"]
    V_bus       = elec["V_bus"]

    GR       = candidate.GR
    # Use gear-level efficiency stored on candidate (set by evaluator from gear dict)
    # eta_gear_fwd is gear.eta * eta_drive; regen is ≈ same (bidirectional motor)
    # candidate stores it in .regen_eta if set, else use a reasonable default
    eta_regen = getattr(candidate, "regen_eta", 0.75)
    # Floor at 0.50 (worst-case lossy regen path)
    eta_regen = max(0.50, float(eta_regen))

    time    = profile["time"]
    power   = profile["power"]   # τ × ω at joint [W]
    dt      = float(time[1] - time[0]) if len(time) > 1 else 0.002

    # ── Regen power: mechanical negative power × efficiency into electrical ─
    neg_power_mech = np.where(power < 0, -power, 0.0)   # positive W when regenerating
    P_elec_regen   = neg_power_mech * eta_regen           # electrical power into cap [W]

    # Convert to cap current: I = P / V_bus
    # (simplified: DC bus voltage is the reference; cap charges from bus)
    I_regen = P_elec_regen / V_bus   # [A]

    # ── Metrics ─────────────────────────────────────────────────────────────
    regen_mask = neg_power_mech > 0
    has_regen  = np.any(regen_mask)

    if has_regen:
        P_regen_mean = float(np.mean(P_elec_regen[regen_mask]))
        P_regen_peak = float(np.max(P_elec_regen))
        I_regen_mean = float(np.mean(I_regen[regen_mask]))
        I_regen_peak = float(np.max(I_regen))
    else:
        P_regen_mean = 0.0
        P_regen_peak = 0.0
        I_regen_mean = 0.0
        I_regen_peak = 0.0

    # Total regen energy over full simulation
    E_regen_J = float(np.trapz(P_elec_regen, time))

    # ── Cap voltage rise estimate ────────────────────────────────────────────
    # ΔV_cap from energy: ΔV = sqrt(V_nom² + 2E/C) - V_nom
    V_cap_delta = float(np.sqrt(max(V_nom**2 + 2 * E_regen_J / C, 0)) - V_nom)
    V_cap_peak  = V_nom + V_cap_delta

    # ESR voltage drop: V_ESR_peak = I_peak × ESR
    V_ESR_peak = I_regen_peak * ESR
    V_cap_total_peak = V_cap_peak + V_ESR_peak

    # ── Cap power capability ──────────────────────────────────────────────────
    # Max safe regen current into cap: limited by ESR heating and overvoltage
    I_cap_max_ov   = (V_max - V_nom) / (ESR + 1e-9)   # overvoltage limit
    P_cap_max_W    = V_nom * I_cap_max_ov               # cap power capability [W]

    # ── Constraint A: minimum regen power ───────────────────────────────────
    fail_reasons = []
    if P_regen_mean < P_regen_min:
        fail_reasons.append(
            f"Regen too low: P_regen_mean={P_regen_mean:.2f}W < min={P_regen_min:.1f}W "
            f"(GR={GR} makes joint velocity too low for meaningful electrical regen)"
        )

    # ── Constraint B: over-voltage ───────────────────────────────────────────
    if V_cap_total_peak > V_max * V_margin:
        fail_reasons.append(
            f"Regen over-voltage: V_cap_peak={V_cap_total_peak:.1f}V > "
            f"{V_margin}×V_max={V_max * V_margin:.1f}V "
            f"(regen current {I_regen_peak:.1f}A saturates {C}F cap)"
        )

    # ── Power match score ─────────────────────────────────────────────────────
    # Score = 1 when P_regen_mean is exactly at the "sweet spot" of cap capability
    # (between 5% and 80% of P_cap_max; outside these bounds score declines)
    if P_cap_max_W > 0:
        ratio = P_regen_mean / P_cap_max_W
        if ratio < 0.05:
            match_score = ratio / 0.05   # linearly ramps up from 0
        elif ratio <= 0.80:
            match_score = 1.0
        else:
            match_score = max(0, 1.0 - (ratio - 0.80) / 0.20)
    else:
        match_score = 0.0

    regen_ok = len(fail_reasons) == 0

    return {
        "regen_energy_J":        E_regen_J,
        "regen_power_mean_W":    P_regen_mean,
        "regen_power_peak_W":    P_regen_peak,
        "I_regen_mean_A":        I_regen_mean,
        "I_regen_peak_A":        I_regen_peak,
        "V_cap_delta_V":         V_cap_delta,
        "V_cap_peak_V":          V_cap_total_peak,
        "cap_power_match_score": match_score,
        "P_cap_max_W":           P_cap_max_W,
        "regen_ok":              regen_ok,
        "fail_reasons":          fail_reasons,
    }


def simulate_cap_voltage(profile: dict, cfg: dict,
                         eta_regen: float = 0.70) -> dict:
    """
    Simulates the supercap voltage trajectory for a given joint power profile.
    Used for UI plotting (regen analysis tab).

    Returns dict with:
      'time'       : time array [s]
      'V_cap'      : cap voltage trajectory [V]
      'I_cap'      : cap current trajectory [A]
      'P_regen'    : electrical regen power [W]
    """
    scap  = cfg["supercap"]
    C     = scap["capacitance_F"]
    ESR   = scap["esr_ohm"]
    V_nom = scap["V_nominal"]
    V_max = scap["V_max"]
    V_min = scap["V_min"]
    V_bus = cfg["electrical"]["V_bus"]

    time  = profile["time"]
    power = profile["power"]
    dt    = float(time[1] - time[0]) if len(time) > 1 else 0.002

    V_cap = np.zeros_like(time)
    I_cap = np.zeros_like(time)
    P_regen_arr = np.zeros_like(time)
    V_cap[0] = V_nom

    for i in range(1, len(time)):
        P_mech = power[i]
        if P_mech < 0:
            # Regen phase: negative power → charge cap
            P_elec = -P_mech * eta_regen
            I = P_elec / max(V_cap[i-1], V_min)   # avoid division by zero
        else:
            # Motoring phase: draw from cap (discharge)
            P_elec = P_mech
            I = -P_elec / max(V_cap[i-1], V_min)

        # dV/dt = I/C (simplified, ignoring ESR voltage for integration)
        dV = I * dt / C
        V_new = V_cap[i-1] + dV
        V_cap[i] = float(np.clip(V_new, V_min, V_max))
        I_cap[i] = I
        P_regen_arr[i] = P_elec if P_mech < 0 else 0.0

    return {
        "time":     time,
        "V_cap":    V_cap,
        "I_cap":    I_cap,
        "P_regen":  P_regen_arr,
    }
