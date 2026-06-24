"""
main.py
========
CLI entry point for the Motor + GR Co-Design Optimizer.

Usage:
    python main.py [--config config/robot_params.yaml] [--joint knee_pitch]
                   [--profile analytic|mujoco_csv|online] [--top 10]

Steps:
  1. Load config + component libraries
  2. Compute joint torque/velocity profile
  3. Extract requirements (T_cont, T_peak, omega_max)
  4. Evaluate all (motor × gear) candidates against hard constraints
  5. Apply supercap regen constraints
  6. Score and rank passing candidates
  7. Export results to CSV and print summary
"""

import argparse
import os
import sys
import yaml
import json
import numpy as np

# Ensure the OptimizationSelection root is on the Python path
_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _DIR)

from core.inverse_dynamics import compute_joint_profile
from core.requirements import extract_requirements
from core.evaluator import evaluate_all
from core.regen import compute_regen
from core.scorer import score_candidates


# ─────────────────────────────────────────────────────────────────────────────
#  YAML LOADERS
# ─────────────────────────────────────────────────────────────────────────────

def load_yaml(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def load_config(cfg_path: str) -> dict:
    return load_yaml(cfg_path)


def load_motors(path: str) -> list:
    data = load_yaml(path)
    return data["motors"]


def load_gearboxes(path: str) -> list:
    data = load_yaml(path)
    return data["gearboxes"]


# ─────────────────────────────────────────────────────────────────────────────
#  RESULTS EXPORT
# ─────────────────────────────────────────────────────────────────────────────

def export_results(ranked: list, cfg: dict, out_dir: str):
    """Saves top results to CSV and full results to JSON."""
    os.makedirs(out_dir, exist_ok=True)
    joint = cfg["gait"]["target_joint"]

    # ── CSV ──────────────────────────────────────────────────────────────────
    import csv
    csv_path = os.path.join(out_dir, f"results_{joint}.csv")
    if ranked:
        fieldnames = list(ranked[0].keys())
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(ranked)
        print(f"  CSV saved → {csv_path}")

    # ── JSON ─────────────────────────────────────────────────────────────────
    json_path = os.path.join(out_dir, f"results_{joint}.json")
    with open(json_path, "w") as f:
        json.dump(ranked, f, indent=2, default=float)
    print(f"  JSON saved → {json_path}")

    return csv_path, json_path


# ─────────────────────────────────────────────────────────────────────────────
#  PRETTY PRINT
# ─────────────────────────────────────────────────────────────────────────────

def print_summary(ranked: list, reqs: dict, top_n: int = 10):
    joint = reqs["joint"]
    print("\n" + "═" * 78)
    print(f"  MOTOR + GEAR CO-DESIGN RESULTS  │  Joint: {joint.upper()}")
    print("═" * 78)
    print(f"  Joint Requirements:")
    print(f"    T_cont  = {reqs['T_cont_Nm']:.2f} N·m   │  "
          f"T_peak   = {reqs['T_peak_Nm']:.2f} N·m   │  "
          f"ω_max    = {reqs['omega_max_rads']:.2f} rad/s")
    print(f"    P_regen_mean = {reqs['P_regen_mean_W']:.1f} W  │  "
          f"E_regen  = {reqs['E_regen_J']:.2f} J  │  "
          f"Regen fraction = {reqs['regen_fraction']*100:.1f}%")
    print("─" * 78)
    print(f"  TOP {min(top_n, len(ranked))} CANDIDATES\n")

    header = (
        f"{'#':>2}  {'Motor':<22} {'Gear':<20} {'GR':>5}  "
        f"{'Score':>6}  {'Mass':>5}  {'V_util':>6}  "
        f"{'P_cu':>6}  {'J_ref':>8}  {'Regen':>8}"
    )
    print(header)
    print("─" * 95)

    for i, r in enumerate(ranked[:top_n]):
        print(
            f"{i+1:>2}  {r['motor_id']:<22} {r['gear_id']:<20} {r['GR']:>5.1f}  "
            f"{r['score_total']:>6.3f}  {r['total_mass_kg']:>5.2f}  "
            f"{r['V_utilisation']*100:>5.1f}%  "
            f"{r['P_cu_W']:>5.1f}W  "
            f"{r['J_ref']*1000:>7.3f}e-3  "
            f"{r['regen_power_mean_W']:>7.1f}W"
        )

    print("─" * 95)
    if ranked:
        best = ranked[0]
        print(f"  BEST: {best['motor_id']} + {best['gear_id']}")
        print(f"    GR={best['GR']:.1f}  Score={best['score_total']:.4f}  "
              f"Mass={best['total_mass_kg']:.2f}kg  "
              f"V_util={best['V_utilisation']*100:.1f}%  "
              f"J_ref={best['J_ref']*1000:.3f}e-3 kg.m2")
        print(f"    Regen: {best['regen_power_mean_W']:.1f}W mean  "
              f"V_cap_peak={best['V_cap_peak_V']:.1f}V  "
              f"E_regen={best['regen_energy_J']:.2f}J")
    print("═" * 78 + "\n")


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN PIPELINE
# ─────────────────────────────────────────────────────────────────────────────

def run_pipeline(cfg_path: str,
                 motors_path: str,
                 gearboxes_path: str,
                 joint_override: str = None,
                 profile_override: str = None,
                 top_n: int = None) -> dict:
    """
    Runs the full co-design pipeline.
    Returns dict with 'ranked', 'reqs', 'profile', 'all_candidates'.
    """
    print("\n[1/6] Loading configuration…")
    cfg      = load_config(cfg_path)
    motors   = load_motors(motors_path)
    gearboxes = load_gearboxes(gearboxes_path)

    if joint_override:
        cfg["gait"]["target_joint"] = joint_override
    if profile_override:
        cfg["gait"]["profile_source"] = profile_override
    if top_n:
        cfg["output"]["top_n"] = top_n

    joint = cfg["gait"]["target_joint"]
    print(f"    Target joint   : {joint}")
    print(f"    Profile source : {cfg['gait']['profile_source']}")
    print(f"    Motors in lib  : {len(motors)}")
    print(f"    Gearboxes      : {len(gearboxes)}")
    print(f"    Total candidates: {len(motors) * len(gearboxes)}")

    print("\n[2/6] Computing joint torque/velocity profile…")
    profile = compute_joint_profile(cfg)
    print(f"    Profile length: {len(profile['time'])} steps  "
          f"({profile['time'][-1]:.2f} s)")

    print("\n[3/6] Extracting joint requirements…")
    reqs = extract_requirements(profile)
    print(f"    T_cont   = {reqs['T_cont_Nm']:.2f} N.m")
    print(f"    T_peak   = {reqs['T_peak_Nm']:.2f} N.m")
    print(f"    w_max    = {reqs['omega_max_rads']:.3f} rad/s")
    print(f"    E_regen  = {reqs['E_regen_J']:.2f} J  ({reqs['regen_fraction']*100:.1f}% of cycle)")

    print("\n[4/6] Evaluating (motor × gear) candidates against hard constraints…")
    all_candidates = evaluate_all(motors, gearboxes, reqs, cfg)
    passed_elec = [c for c in all_candidates if c.passed]
    print(f"    Passed electrical constraints: {len(passed_elec)} / {len(all_candidates)}")

    print("\n[5/6] Applying supercap regen constraints…")
    passed_all = []
    for c in passed_elec:
        rm = compute_regen(c, profile, reqs, cfg)
        c.regen_metrics = rm
        c.regen_ok = rm["regen_ok"]
        if rm["regen_ok"]:
            passed_all.append(c)
        else:
            c.passed = False
            c.fail_reasons.extend(rm["fail_reasons"])

    print(f"    Passed regen constraints: {len(passed_all)} / {len(passed_elec)}")

    if not passed_all:
        print("\n  ⚠ No candidates passed all constraints!")
        print("    Try relaxing: I_phase_max, P_cu_limit, regen_min_power, "
              "or supercap V_max/C settings.")
        return {
            "ranked": [],
            "reqs": reqs,
            "profile": profile,
            "all_candidates": all_candidates,
        }

    print("\n[6/6] Scoring and ranking candidates…")
    ranked = score_candidates(passed_all, cfg)

    # Print summary
    top_n_cfg = cfg["output"]["top_n"]
    print_summary(ranked, reqs, top_n=top_n_cfg)

    # Export
    out_dir = os.path.join(_DIR, cfg["output"]["results_dir"])
    csv_path, json_path = export_results(ranked[:top_n_cfg], cfg, out_dir)

    return {
        "ranked": ranked,
        "reqs": reqs,
        "profile": profile,
        "all_candidates": all_candidates,
        "csv_path": csv_path,
        "json_path": json_path,
    }


# ─────────────────────────────────────────────────────────────────────────────
#  CLI
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Motor + Gear Ratio Co-Design Tool for Legged Robots"
    )
    parser.add_argument(
        "--config", default=os.path.join(_DIR, "config", "robot_params.yaml"),
        help="Path to robot_params.yaml"
    )
    parser.add_argument(
        "--motors", default=os.path.join(_DIR, "config", "motors.yaml"),
        help="Path to motors.yaml"
    )
    parser.add_argument(
        "--gearboxes", default=os.path.join(_DIR, "config", "gearboxes.yaml"),
        help="Path to gearboxes.yaml"
    )
    parser.add_argument(
        "--joint", default=None,
        choices=["hip_pitch", "knee_pitch", "ankle_pitch"],
        help="Override target joint"
    )
    parser.add_argument(
        "--profile", default=None,
        choices=["analytic", "mujoco_csv", "online"],
        help="Override profile source"
    )
    parser.add_argument(
        "--top", type=int, default=None,
        help="Number of top candidates to display"
    )

    args = parser.parse_args()
    run_pipeline(
        cfg_path=args.config,
        motors_path=args.motors,
        gearboxes_path=args.gearboxes,
        joint_override=args.joint,
        profile_override=args.profile,
        top_n=args.top,
    )


if __name__ == "__main__":
    main()
