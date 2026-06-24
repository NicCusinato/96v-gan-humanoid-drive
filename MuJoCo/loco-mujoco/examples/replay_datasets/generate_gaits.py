import os
import pandas as pd
import numpy as np
from loco_mujoco import LocoEnv
from loco_mujoco.task_factories import ImitationFactory, AMASSDatasetConf

def generate_gait(gait_category, amass_path, base_save_dir="gait_data"):
    # Create the subfolder for the specific action (e.g. gait_data/hop)
    save_dir = os.path.join(base_save_dir, gait_category)
    os.makedirs(save_dir, exist_ok=True)
    
    # Extract the base file name from the AMASS path (e.g., '83_40_poses')
    base_name = amass_path.split('/')[-1]
    filename = os.path.join(save_dir, f"{base_name}.npz")
    
    # Skip if we already generated it
    if os.path.exists(filename):
        print(f"⏩ Skipping {amass_path} (already exists at {filename})")
        return
        
    print(f"\n======================================")
    print(f"Generating {gait_category} trajectory: {amass_path}")
    print(f"======================================")
    
    try:
        # Configure the factory to use the specific AMASS sequence
        dataset_conf = AMASSDatasetConf([amass_path])
        
        # Build the environment and generate trajectory
        env = ImitationFactory.make("KBotV2",
                                    amass_dataset_conf=dataset_conf,
                                    n_substeps=20)
        
        # Get the generated trajectory and save it
        traj = env.th.traj
        traj.save(filename)
        print(f"✅ Saved to {filename}\n")
    except Exception as e:
        print(f"❌ Failed to generate {amass_path}: {e}")

def main():
    base_dir = "gait_data"
    
    # --- 1. Generate specific hardcoded manual gaits ---
    manual_gaits = {
        "walk": [
            "CMU/07/07_01_poses",
            "CMU/07/07_02_poses",
            "CMU/35/35_01_poses",
            "CMU/35/35_02_poses",
            "CMU/39/39_01_poses"
        ],
        "run": [
            "CMU/16/16_35_poses",
            "CMU/16/16_36_poses",
            "CMU/35/35_17_poses",
            "CMU/35/35_18_poses",
            "CMU/35/35_19_poses"
        ],
        "jump": [
            "CMU/13/13_11_poses",
            "CMU/16/16_01_poses",
            "CMU/16/16_02_poses",
            "CMU/49/49_02_poses",
            "CMU/49/49_03_poses"
        ],
        "squat": [
            "CMU/13/13_02_poses",
            "CMU/13/13_14_poses",
            "CMU/23/23_14_poses",
            "CMU/86/86_02_poses",
            "CMU/86/86_05_poses"
        ]
    }
    
    for category, paths in manual_gaits.items():
        for path in paths:
            generate_gait(category, path, base_dir)
        
    # --- 2. Batch generate from an Excel index (e.g. cmu_hop_index.xlsx) ---
    index_file = "cmu_hop_index.xlsx"
    if os.path.exists(index_file):
        print(f"\n📄 Found index file: {index_file}. Processing batch...")
        df = pd.read_excel(index_file)
        
        for index, row in df.iterrows():
            subject = int(row['subject'])
            motion_id = row['motion_id']
            # Some excel indexes have a category/keyword column. If not, fallback to 'unknown'
            category = row['search_keyword'] if 'search_keyword' in df.columns else 'imported'
            
            # Format subject folder (CMU uses 2-digit zero-padding for < 100, e.g. '09', '83')
            subject_folder = f"{subject:02d}"
            
            # Construct the AMASS path expected by loco_mujoco
            amass_path = f"CMU/{subject_folder}/{motion_id}_poses"
            
            generate_gait(category, amass_path, base_dir)

if __name__ == '__main__':
    main()
