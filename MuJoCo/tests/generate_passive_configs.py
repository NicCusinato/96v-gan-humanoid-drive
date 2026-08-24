import os

# Stage 1 Passive Policy trials: walk + run only (stable motions first)
TRIALS = {
    "walk_02_02": ("CMU/02/02_02_poses.npz", 50_000_000, True),
    "walk_07_12": ("CMU/07/07_12_poses.npz", 50_000_000, True),
    "walk_07_08": ("CMU/07/07_08_poses.npz", 50_000_000, True),
    "run_09_04":  ("CMU/09/09_04_poses.npz", 50_000_000, True),
    "run_38_03":  ("CMU/38/38_03_poses.npz", 50_000_000, True),
    "run_16_35":  ("CMU/16/16_35_poses.npz", 50_000_000, True),
}

BASE_CONFIG = "tests/conf_kbot_amass_passive.yaml"
OUT_DIR = "tests/passive"
os.makedirs(OUT_DIR, exist_ok=True)

with open(BASE_CONFIG) as f:
    base_str = f.read()

for name, (path, steps, random_start) in TRIALS.items():
    lines = base_str.split("\n")
    out_lines = []
    skip_dataset = False
    for line in lines:
        if "rel_dataset_path:" in line:
            out_lines.append(line)
            out_lines.append(f'          - "{path}"')
            skip_dataset = True
            continue
        if skip_dataset and line.strip().startswith('- "CMU/'):
            continue
        else:
            skip_dataset = False
        if "total_timesteps:" in line:
            out_lines.append(f"  total_timesteps: {steps}")
            continue
        if "debug: true" in line:
            out_lines.append("  debug: false")
            continue
        if line.startswith("  num_envs:"):
            out_lines.append("  num_envs: 2048")
            continue
        out_lines.append(line)
    if not random_start:
        out_lines += ["", "  th_params:", "    random_start: false"]
    out_file = os.path.join(OUT_DIR, f"conf_{name}.yaml")
    with open(out_file, "w") as f:
        f.write("\n".join(out_lines))
    print(f"Generated {out_file}  ({steps//1_000_000}M steps, {path})")

print(f"All {len(TRIALS)} passive Stage 1 configs written to {OUT_DIR}/")
