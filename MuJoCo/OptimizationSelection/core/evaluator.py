"""
evaluator.py
=============
Evaluates each (motor, gearbox) candidate against hard physical constraints
derived from joint requirements.

Hard constraint checks:
  1. Motor speed feasibility  (omega_motor <= omega_rated)
  2. Voltage budget            (V_emf + V_resistive <= V_margin × V_bus)
  3. Peak torque               (T_motor_peak <= motor T_peak)
  4. Continuous torque         (T_motor_cont <= motor T_cont)
  5. Phase current             (I_rms <= I_phase_max)
  6. Thermal / copper loss     (P_cu = I_rms² × R <= P_cu_limit)

For passing candidates, also computes:
  - Reflected inertia J_ref
  - Bus voltage utilisation @ omega_max
  - Efficiency estimate
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class CandidateResult:
    motor_id: str
    gear_id: str
    GR: float
    # Derived motor quantities
    omega_motor_max: float = 0.0
    V_emf: float = 0.0
    V_resistive: float = 0.0
    V_total: float = 0.0
    V_utilisation: float = 0.0
    I_peak: float = 0.0
    I_rms: float = 0.0
    P_cu_W: float = 0.0
    T_motor_peak: float = 0.0
    T_motor_cont: float = 0.0
    J_ref: float = 0.0
    bandwidth_score: float = 0.0
    eta_total: float = 0.0
    regen_eta: float = 0.75    # gear+drive efficiency used for regen calc
    total_mass_kg: float = 0.0
    # Regen (filled by regen.py)
    regen_ok: bool = False
    regen_metrics: dict = field(default_factory=dict)
    # Pass/fail
    passed: bool = False
    fail_reasons: list = field(default_factory=list)


def evaluate_candidate(motor: dict,
                       gear: dict,
                       reqs: dict,
                       cfg: dict) -> CandidateResult:
    """
    Evaluates a single (motor, gear) pair against joint requirements.

    Parameters
    ----------
    motor : dict  — motor library entry
    gear  : dict  — gearbox library entry
    reqs  : dict  — output of requirements.extract_requirements()
    cfg   : dict  — full configuration

    Returns
    -------
    CandidateResult
    """
    elec  = cfg["electrical"]
    therm = cfg["thermal"]

    V_bus     = elec["V_bus"]
    I_max     = elec["I_phase_max_A"]
    V_margin  = elec["voltage_margin"]
    eta_drive = elec["eta_drive"]
    P_cu_lim  = therm["P_cu_limit_W"]

    GR  = gear["GR"]
    eta = gear["eta"] * eta_drive     # combined gear + drive efficiency

    res = CandidateResult(
        motor_id=motor["id"],
        gear_id=gear["id"],
        GR=GR,
    )

    fail = []

    # ── 1. Motor speed feasibility ───────────────────────────────────────────
    omega_motor_max = GR * reqs["omega_max_rads"]
    res.omega_motor_max = omega_motor_max

    if omega_motor_max > motor["omega_rated"]:
        fail.append(
            f"Speed: ω_motor={omega_motor_max:.1f} > ω_rated={motor['omega_rated']:.1f} rad/s"
        )

    # ── 2. Voltage budget ────────────────────────────────────────────────────
    ke       = motor["ke"]   # V·s/rad  (= kt for SI ideal motor)
    R_phase  = motor["R_phase"]   # Ω

    V_emf = ke * omega_motor_max
    # Peak current for peak torque check (used for voltage at rated operating point)
    I_peak_est = reqs["T_peak_Nm"] / (motor["kt"] * eta * GR)
    V_resistive = I_peak_est * R_phase
    V_total     = V_emf + V_resistive

    res.V_emf        = V_emf
    res.V_resistive  = V_resistive
    res.V_total      = V_total
    res.V_utilisation = V_emf / V_bus   # back-EMF utilisation fraction

    if V_total > V_margin * V_bus:
        fail.append(
            f"Voltage: V_emf+IR={V_total:.1f}V > {V_margin}×{V_bus}={V_margin*V_bus:.1f}V"
        )

    # ── 3. Peak torque ───────────────────────────────────────────────────────
    T_motor_peak_req = reqs["T_peak_Nm"] / (eta * GR)
    res.T_motor_peak = T_motor_peak_req

    if T_motor_peak_req > motor["T_peak"]:
        fail.append(
            f"Peak torque: T_m_peak={T_motor_peak_req:.2f} > T_motor_peak={motor['T_peak']:.2f} N·m"
        )

    # ── 4. Continuous (RMS) torque ───────────────────────────────────────────
    T_motor_cont_req = reqs["T_cont_Nm"] / (eta * GR)
    res.T_motor_cont = T_motor_cont_req

    if T_motor_cont_req > motor["T_cont"]:
        fail.append(
            f"Cont torque: T_m_cont={T_motor_cont_req:.2f} > T_motor_cont={motor['T_cont']:.2f} N·m"
        )

    # ── 5. Phase current ─────────────────────────────────────────────────────
    kt = motor["kt"]
    I_peak  = T_motor_peak_req / kt
    I_rms   = T_motor_cont_req / kt
    res.I_peak = I_peak
    res.I_rms  = I_rms

    if I_peak > I_max:
        fail.append(f"Peak current: I_peak={I_peak:.1f}A > I_max={I_max:.1f}A")

    # ── 6. Thermal / copper loss ─────────────────────────────────────────────
    P_cu = I_rms**2 * R_phase
    res.P_cu_W = P_cu

    if P_cu > P_cu_lim:
        fail.append(f"Copper loss: P_cu={P_cu:.1f}W > limit={P_cu_lim:.1f}W")

    # ── Derived metrics (computed regardless of pass/fail for diagnostics) ────
    J_rotor   = motor["J_rotor"]
    J_gear    = gear["J_gear"]
    J_ref     = (J_rotor + J_gear) * GR**2
    res.J_ref = J_ref

    # Bandwidth score: inversely proportional to reflected inertia and GR
    # Higher score = better dynamic responsiveness
    J_ref_norm = J_ref / 0.05   # normalise to a typical "bad" value of 0.05 kg·m²
    res.bandwidth_score = float(np.clip(1.0 / (1.0 + J_ref_norm * GR**0.5), 0, 1))

    # Total efficiency at RMS operating point
    P_input  = V_bus * I_rms * eta_drive
    P_useful = reqs["T_cont_Nm"] * reqs["omega_max_rads"]   # conservative estimate
    res.eta_total = float(np.clip(P_useful / (P_input + 1e-9), 0, 1))

    # Regen efficiency: gear.eta × drive efficiency (bidirectional)
    res.regen_eta = float(gear["eta"] * eta_drive)

    # Total mass
    res.total_mass_kg = motor["mass"] + gear["mass"]

    # ── Pass/fail ────────────────────────────────────────────────────────────
    res.fail_reasons = fail
    res.passed = (len(fail) == 0)

    return res


def evaluate_all(motors: list,
                 gearboxes: list,
                 reqs: dict,
                 cfg: dict) -> list:
    """
    Sweeps all (motor × gearbox) pairs.
    Returns list of CandidateResult (all, including failed ones).
    """
    results = []
    for motor in motors:
        for gear in gearboxes:
            r = evaluate_candidate(motor, gear, reqs, cfg)
            results.append(r)
    return results
