import os
import argparse
from pathlib import Path
import numpy as np
import imageio
import jax
import mujoco

# Important: set AMASS_DIR so LocoMuJoCo knows where to find the raw datasets
os.environ["AMASS_DIR"] = "C:\\AMASS"

from loco_mujoco import TaskFactory
from loco_mujoco.algorithms import PPOJax
from omegaconf import OmegaConf

def main():
    parser = argparse.ArgumentParser(description='Save MP4 video of PPOJax policy.')
    parser.add_argument('--path', type=str, required=True, help='Path to the agent pkl file')
    args = parser.parse_args()

    path = Path(args.path)
    print(f"Loading agent from {path}...")
    agent_conf, agent_state = PPOJax.load_agent(path)
    config = agent_conf.config

    # get task factory
    factory = TaskFactory.get_factory_cls(config.experiment.task_factory.name)

    # create env
    OmegaConf.set_struct(config, False)
    config.experiment.env_params["headless"] = True
    if "goal_params" in config.experiment.env_params:
        config.experiment.env_params["goal_params"]["visualize_goal"] = False
    
    print("Initializing environment...")
    env = factory.make(**config.experiment.env_params, **config.experiment.task_factory.params)

    # The actual mujoco environment is inside the LocoMuJoCo wrapper
    # we need the model and data for rendering
    mj_model = env.model
    
    # Initialize off-screen renderer
    renderer = mujoco.Renderer(mj_model, height=480, width=640)
    
    # Set up policy function
    train_state = agent_state.train_state
    
    # Make deterministic
    train_state.params["log_std"] = np.ones_like(train_state.params["log_std"]) * -np.inf
    
    def sample_actions(ts, obs, _rng):
        y, updates = agent_conf.network.apply({'params': ts.params,
                                               'run_stats': ts.run_stats},
                                              obs, mutable=["run_stats"])
        ts = ts.replace(run_stats=updates['run_stats'])
        pi, _ = y
        a = pi.sample(seed=_rng)
        return a, ts

    plcy_call = jax.jit(sample_actions)
    rng = jax.random.key(0)

    obs = env.reset()
    
    frames = []
    print("Simulating and rendering frames (this may take a minute)...")
    for step in range(500):
        # Action
        rng, _rng = jax.random.split(rng)
        action, train_state = plcy_call(train_state, obs, _rng)
        action = np.atleast_2d(action)

        # Step
        obs, reward, absorbing, done, info = env.step(action)
        
        # Render
        renderer.update_scene(env.data)
        pixels = renderer.render()
        frames.append(pixels)
        
        if done:
            obs = env.reset()

    # Save to mp4
    out_file = path.parent / "kbot_walk.mp4"
    print(f"Saving video to {out_file}...")
    imageio.mimsave(str(out_file), frames, fps=60)
    print("Done!")

if __name__ == "__main__":
    main()
