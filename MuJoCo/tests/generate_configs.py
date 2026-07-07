import os
import yaml
from copy import deepcopy

# The list of trials
trials = {
    "walk_02_02": ("CMU/02/02_02_poses.npz", 50000000),
    "walk_07_12": ("CMU/07/07_12_poses.npz", 50000000),
    "walk_07_08": ("CMU/07/07_08_poses.npz", 50000000),
    "run_09_04": ("CMU/09/09_04_poses.npz", 100000000),
    "run_38_03": ("CMU/38/38_03_poses.npz", 100000000),
    "jump_13_13": ("CMU/13/13_13_poses.npz", 200000000),
    "jump_16_03": ("CMU/16/16_03_poses.npz", 200000000),
    "jump_16_09": ("CMU/16/16_09_poses.npz", 200000000),
    "backflip_87_04": ("CMU/87/87_04_poses.npz", 250000000),
}

base_config_path = "tests/conf_kbot_amass.yaml"

with open(base_config_path, 'r') as f:
    base_yaml_str = f.read()

for name, (path, steps) in trials.items():
    # We will do a simple string replacement because yaml round-tripping 
    # can mess up Hydra's special interpolations like ${control_config...}
    
    # 1. Replace the dataset path
    # In base config, it looks like:
    #         rel_dataset_path:
    #           - "CMU/07/07_01_poses.npz"
    #           - "CMU/07/07_02_poses.npz"
    
    # Let's just find "rel_dataset_path:" and replace the next lines
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
