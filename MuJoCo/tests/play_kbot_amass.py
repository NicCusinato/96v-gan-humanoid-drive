import os
import argparse
from pathlib import Path

# Important: set AMASS_DIR so LocoMuJoCo knows where to find the raw datasets
os.environ["AMASS_DIR"] = "/mnt/c/AMASS"

from loco_mujoco import TaskFactory
from loco_mujoco.algorithms import PPOJax
from omegaconf import OmegaConf

def main():
    parser = argparse.ArgumentParser(description='Run evaluation with PPOJax.')
    parser.add_argument('--path', type=str, required=True, help='Path to the agent pkl file')
    args = parser.parse_args()

    # Use the path from command line arguments
    path = Path(args.path)
    print(f"Loading agent from {path}...")
    agent_conf, agent_state = PPOJax.load_agent(path)
    config = agent_conf.config

    # get task factory
    factory = TaskFactory.get_factory_cls(config.experiment.task_factory.name)

    # create env
    OmegaConf.set_struct(config, False)  # Allow modifications
    config.experiment.env_params["headless"] = False
    
    # Disable the "ghost" target robot rendering to prevent the MuJoCo geom indexing crash
    if "goal_params" in config.experiment.env_params:
        config.experiment.env_params["goal_params"]["visualize_goal"] = False
    
    print("Initializing environment in CPU mode for rendering...")
    env = factory.make(**config.experiment.env_params, **config.experiment.task_factory.params)

    print("Launching viewer...")
    # run eval mujoco (uses CPU MuJoCo for real-time 3D rendering)
    PPOJax.play_policy_mujoco(
        env, 
        agent_conf, 
        agent_state, 
        deterministic=False, 
        n_steps=10000, 
        record=False,
        train_state_seed=0
    )

if __name__ == "__main__":
    main()
