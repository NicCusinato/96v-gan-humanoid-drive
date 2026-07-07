import os
import shutil

# Baseline trial definitions: name -> (CMU path, total_timesteps)
TRIALS = {
    "walk_02_02":     ("CMU/02/02_02_poses.npz",  150_000_000),
    "walk_07_12":     ("CMU/07/07_12_poses.npz",  150_000_000),
    "walk_07_08":     ("CMU/07/07_08_poses.npz",  150_000_000),
    "run_09_04":      ("CMU/09/09_04_poses.npz",  250_000_000),
    "run_38_03":      ("CMU/38/38_03_poses.npz",  250_000_000),
    "run_16_35":      ("CMU/16/16_35_poses.npz",  250_000_000),
    "jump_13_13":     ("CMU/13/13_13_poses.npz",  400_000_000),
    "jump_13_11":     ("CMU/13/13_11_poses.npz",  400_000_000),
    "jump_75_01":     ("CMU/75/75_01_poses.npz",  400_000_000),
    "jump_75_03":     ("CMU/75/75_03_poses.npz",  400_000_000),
    "backflip_87_01": ("CMU/87/87_01_poses.npz",  600_000_000),
}

BASE_CONFIG = "tests/conf_kbot_amass.yaml"
OUT_DIR = "tests/baseline"

os.makedirs(OUT_DIR, exist_ok=True)

with open(BASE_CONFIG, 'r') as f:
    base_str = f.read()

for name, (path, steps) in TRIALS.items():
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

    out_file = os.path.join(OUT_DIR, f"conf_{name}.yaml")
    with open(out_file, 'w') as f:
        f.write('\n'.join(out_lines))
    print(f"Generated {out_file}  ({steps // 1_000_000}M steps, {path})")

print(f"\nAll {len(TRIALS)} baseline configs written to {OUT_DIR}/")
