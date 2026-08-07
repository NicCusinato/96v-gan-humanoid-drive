import os
import yaml
from copy import deepcopy

# The list of trials
trials = {
    "jump_13_11": ("CMU/13/13_11_poses.npz", 200000000),
    "jump_75_01": ("CMU/75/75_01_poses.npz", 200000000),
    "jump_75_03": ("CMU/75/75_03_poses.npz", 200000000),
    "run_16_35": ("CMU/16/16_35_poses.npz", 100000000),
}

base_config_path = "tests/conf_kbot_amass.yaml"

with open(base_config_path, 'r') as f:
    base_yaml_str = f.read()

for name, (path, steps) in trials.items():
    lines = base_yaml_str.split('\n')
    out_lines = []
    skip = False
    for line in lines:
        if "rel_dataset_path:" in line:
            out_lines.append(line)
            out_lines.append(f'          - "{path}"')
            skip = True
            continue
        
        if skip and line.strip().startswith('- "CMU/'):
            continue
        elif skip:
            skip = False
            
        if "total_timesteps:" in line:
            out_lines.append(f"  total_timesteps: {steps}")
            continue
            
        out_lines.append(line)
        
    out_file = f"tests/conf_{name}.yaml"
    with open(out_file, 'w') as f:
        f.write('\n'.join(out_lines))
    print(f"Generated {out_file} for {path}")
