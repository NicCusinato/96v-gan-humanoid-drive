import os
import shutil

# Baseline trial definitions: name -> (CMU path, total_timesteps, random_start)
# random_start=True  -> good for cyclic motions (walk/run/squat): robot must be robust at any phase
# random_start=False -> required for one-shot motions (jump/backflip): robot must see full sequence
TRIALS = {
    "walk_02_02":     ("CMU/02/02_02_poses.npz",  150_000_000, True),
    "walk_07_12":     ("CMU/07/07_12_poses.npz",  150_000_000, True),
    "walk_07_08":     ("CMU/07/07_08_poses.npz",  150_000_000, True),
    "run_09_04":      ("CMU/09/09_04_poses.npz",  250_000_000, True),
    "run_38_03":      ("CMU/38/38_03_poses.npz",  250_000_000, True),
    "run_16_35":      ("CMU/16/16_35_poses.npz",  250_000_000, True),
    "jump_13_13":     ("CMU/13/13_13_poses.npz",  400_000_000, False),
    "jump_13_11":     ("CMU/13/13_11_poses.npz",  400_000_000, False),
    "jump_75_01":     ("CMU/75/75_01_poses.npz",  400_000_000, False),
    "jump_75_03":     ("CMU/75/75_03_poses.npz",  400_000_000, False),
    "backflip_87_01": ("CMU/87/87_01_poses.npz",  600_000_000, False),
    "squat_22_14":    ("CMU/22/22_14_poses.npz",  250_000_000, True),
    "squat_23_14":    ("CMU/23/23_14_poses.npz",  250_000_000, True),
}

BASE_CONFIG = "tests/conf_kbot_amass.yaml"
OUT_DIR = "tests/baseline"

os.makedirs(OUT_DIR, exist_ok=True)

with open(BASE_CONFIG, 'r') as f:
    base_str = f.read()

for name, (path, steps, random_start) in TRIALS.items():
    lines = base_str.split('\n')
    out_lines = []
    skip_dataset = False

    for line in lines:
        # Strip energy penalty lines entirely
        if "energy_coeff:" in line or "regen_efficiency:" in line:
            continue

        # Replace dataset path block
        if "rel_dataset_path:" in line:
            out_lines.append(line)
            out_lines.append(f'          - "{path}"')
            skip_dataset = True
            continue

        if skip_dataset and line.strip().startswith('- "CMU/'):
            continue
        else:
            skip_dataset = False

        # Replace total_timesteps
        if "total_timesteps:" in line:
            out_lines.append(f"  total_timesteps: {steps}")
            continue

        # Force debug: false for cleaner logs during long runs
        if "debug: true" in line:
            out_lines.append("  debug: false")
            continue

        # Bump num_envs to use more VRAM
        if line.startswith("  num_envs:"):
            out_lines.append("  num_envs: 2560")
            continue

        out_lines.append(line)

    # Inject th_params for non-random-start trials (jump/backflip).
    # This forces each episode to always start from frame 0 so the robot
    # experiences the full motion arc (e.g. run-up -> jump -> landing).
    if not random_start:
        out_lines.append("")
        out_lines.append("  # Force episode to always start from frame 0 for one-shot motions")
        out_lines.append("  th_params:")
        out_lines.append("    random_start: false")

    out_file = os.path.join(OUT_DIR, f"conf_{name}.yaml")
    with open(out_file, 'w') as f:
        f.write('\n'.join(out_lines))
    rs_label = "random_start" if random_start else "fixed_start"
    print(f"Generated {out_file}  ({steps // 1_000_000}M steps, {path}, {rs_label})")

print(f"\nAll {len(TRIALS)} baseline configs written to {OUT_DIR}/")
