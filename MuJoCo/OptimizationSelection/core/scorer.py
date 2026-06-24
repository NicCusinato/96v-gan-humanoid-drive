"""
scorer.py
==========
Computes a multi-objective weighted score for each passing candidate
and ranks them.

Score components (all normalised 0–1, higher = better):

  1. mass_score           : reward low total actuator mass
  2. v_util_score         : reward V_emf near 70–80% of V_bus
  3. efficiency_score     : reward high combined efficiency (low copper loss)
  4. regen_score          : reward well-matched regen power to supercap
  5. inertia_score        : reward low reflected inertia J_ref
  6. gr_score             : penalise very high or very low GR

Final score = weighted sum of all components.
"""

import numpy as np
from dataclasses import asdict


def _normalise(values: np.ndarray,
               low_is_good: bool = True) -> np.ndarray:
    """Min-max normalisation. If low_is_good, flips so lower → higher score."""
    vmin = values.min()
    vmax = values.max()
    if vmax == vmin:
        return np.ones_like(values, dtype=float) * 0.5
    normed = (values - vmin) / (vmax - vmin)
    return (1.0 - normed) if low_is_good else normed


def score_candidates(candidates: list, cfg: dict) -> list:
    """
    Parameters
    ----------
    candidates : list of CandidateResult  (only 'passed' ones, with regen_metrics filled)
    cfg        : full configuration dict

    Returns
    -------
    List of dicts, sorted by score descending, each containing:
      all CandidateResult fields + score breakdown + final score
    """
    if not candidates:
        return []

    sw = cfg["scoring"]["weights"]
    gr_low  = cfg["scoring"]["gr_penalty_low"]
    gr_high = cfg["scoring"]["gr_penalty_high"]
    V_bus   = cfg["electrical"]["V_bus"]
    V_target = cfg["electrical"]["V_bus_utilization_target"]

    # Convert to list of dicts for easier manipulation
    rows = []
    for c in candidates:
        d = {}
        # Core metrics
        d["motor_id"]       = c.motor_id
        d["gear_id"]        = c.gear_id
        d["GR"]             = c.GR
        d["total_mass_kg"]  = c.total_mass_kg
        d["V_utilisation"]  = c.V_utilisation
        d["P_cu_W"]         = c.P_cu_W
        d["J_ref"]          = c.J_ref
        d["eta_total"]      = c.eta_total
        d["omega_motor_max"]= c.omega_motor_max
        d["V_emf"]          = c.V_emf
        d["V_total"]        = c.V_total
        d["I_rms"]          = c.I_rms
        d["I_peak"]         = c.I_peak
        d["T_motor_cont"]   = c.T_motor_cont
        d["T_motor_peak"]   = c.T_motor_peak
        d["bandwidth_score"]= c.bandwidth_score
        # Regen metrics
        rm = c.regen_metrics
        d["regen_energy_J"]    = rm.get("regen_energy_J", 0)
        d["regen_power_mean_W"]= rm.get("regen_power_mean_W", 0)
        d["regen_power_peak_W"]= rm.get("regen_power_peak_W", 0)
        d["cap_power_match_score"] = rm.get("cap_power_match_score", 0)
        d["V_cap_peak_V"]      = rm.get("V_cap_peak_V", 0)
        d["regen_ok"]          = rm.get("regen_ok", False)
        rows.append(d)

    n = len(rows)

    # ── 1. Mass score ────────────────────────────────────────────────────────
    masses = np.array([r["total_mass_kg"] for r in rows])
    mass_scores = _normalise(masses, low_is_good=True)

    # ── 2. Voltage utilisation score ─────────────────────────────────────────
    # Peak score at target (0.75); decays symmetrically outside [0.60, 0.85]
    v_utils = np.array([r["V_utilisation"] for r in rows])
    v_util_scores = np.where(
        (v_utils >= 0.60) & (v_utils <= 0.85),
        1.0 - np.abs(v_utils - V_target) / 0.15,
        np.maximum(0.0, 1.0 - np.abs(v_utils - V_target) / 0.30)
    )
    v_util_scores = np.clip(v_util_scores, 0, 1)

    # ── 3. Efficiency score (inverse of copper loss) ──────────────────────────
    pcu = np.array([r["P_cu_W"] for r in rows])
    efficiency_scores = _normalise(pcu, low_is_good=True)

    # ── 4. Regen match score ─────────────────────────────────────────────────
    regen_scores = np.array([r["cap_power_match_score"] for r in rows])

    # ── 5. Reflected inertia score ───────────────────────────────────────────
    jref = np.array([r["J_ref"] for r in rows])
    inertia_scores = _normalise(jref, low_is_good=True)

    # ── 6. GR penalty score ──────────────────────────────────────────────────
    grs = np.array([r["GR"] for r in rows])
    # Score = 1 in [gr_low, gr_high], linearly decays outside
    gr_scores = np.where(
        grs < gr_low,
        grs / gr_low,
        np.where(
            grs > gr_high,
            np.maximum(0.0, 1.0 - (grs - gr_high) / gr_high),
            1.0
        )
    )
    gr_scores = np.clip(gr_scores, 0, 1)

    # ── Final weighted score ─────────────────────────────────────────────────
    final = (
        sw["mass"]               * mass_scores
        + sw["voltage_utilization"] * v_util_scores
        + sw["efficiency"]          * efficiency_scores
        + sw["regen_match"]         * regen_scores
        + sw["reflected_inertia"]   * inertia_scores
        + sw["gr_penalty"]          * gr_scores
    )

    # Attach scores to rows
    for i, r in enumerate(rows):
        r["score_mass"]       = float(mass_scores[i])
        r["score_v_util"]     = float(v_util_scores[i])
        r["score_efficiency"] = float(efficiency_scores[i])
        r["score_regen"]      = float(regen_scores[i])
        r["score_inertia"]    = float(inertia_scores[i])
        r["score_gr"]         = float(gr_scores[i])
        r["score_total"]      = float(final[i])

    # Sort descending by total score
    rows.sort(key=lambda r: r["score_total"], reverse=True)

    return rows
