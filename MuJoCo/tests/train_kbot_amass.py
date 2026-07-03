import os

# Important: set AMASS_DIR so LocoMuJoCo knows where to find the raw datasets
os.environ["AMASS_DIR"] = "/mnt/c/AMASS"
os.environ["HYDRA_FULL_ERROR"] = "1"
os.environ["XLA_PYTHON_CLIENT_PREALLOCATE"] = "false"
os.environ["XLA_PYTHON_CLIENT_MEM_FRACTION"] = "0.5"

import hydra
import jax
from omegaconf import DictConfig, open_dict

from loco_mujoco.algorithms import PPOJax
from loco_mujoco.task_factories import ImitationFactory


@hydra.main(version_base=None, config_path=".", config_name="conf_kbot_amass")
def main(config: DictConfig):
    # Initialize the RNG key for JAX
    rng = jax.random.PRNGKey(0)

    print("Initializing environment...")
    # Create the environment using ImitationFactory which automatically handles AMASS datasets
    env = ImitationFactory.make(
        **config.experiment.task_factory.params,
        **config.experiment.env_params
    )

    print("Initializing PPOJax Agent...")
    # Create the agent configuration
    agent_conf = PPOJax.init_agent_conf(env, config)



    # Build the JIT-compiled train function
    train_fn = PPOJax.build_train_fn(env, agent_conf, mh=None)
    train_fn = jax.jit(train_fn)

    print("Starting Training...")
    # Run the training loop
    try:
        out = train_fn(rng)
        print("Training finished normally!")
    except KeyboardInterrupt:
        print("\nTraining interrupted by user!")
        # On interrupt, we can't easily extract the partial state from the JIT function 
        # unless it was periodically saved to disk. But let's notify the user.
        print("Note: Partial training inside a JIT loop is difficult to extract.")
        raise
    
    # Save the trained agent weights to the Hydra output directory
    from hydra.core.hydra_config import HydraConfig
    output_dir = HydraConfig.get().runtime.output_dir
    saved_path = PPOJax.save_agent(output_dir, agent_conf, out["agent_state"])
    print(f"Model successfully saved to {saved_path}")
    
    # Auto-generate info.md
    config_name = HydraConfig.get().job.config_name
    # Try to parse the action from the config name (e.g. conf_walk_02_02 -> walk)
    action = "Unknown"
    if "_" in config_name:
        parts = config_name.split("_")
        if len(parts) >= 2:
            action = parts[1].capitalize() # e.g. "Walk"
            
    datasets = config.experiment.task_factory.params.amass_dataset_conf.rel_dataset_path
    dataset_str = ", ".join([d.split("/")[-1].replace("_poses.npz", "") for d in datasets])
    
    info_path = os.path.join(output_dir, "info.md")
    with open(info_path, "w") as f:
        f.write(f"# {action}\n")
        f.write(f"CMU Dataset: {dataset_str}\n")
        f.write(f"Action: {action}ing\n")
    print(f"Generated {info_path}")
    
    return out


if __name__ == '__main__':
    main()
