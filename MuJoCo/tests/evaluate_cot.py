import os
import argparse
from pathlib import Path
import numpy as np
import jax

os.environ["AMASS_DIR"] = "/mnt/c/AMASS"

from loco_mujoco import TaskFactory
from loco_mujoco.algorithms import PPOJax
from omegaconf import OmegaConf

def evaluate_cot(env, agent_conf, agent_state, train_state_seed=0, num_episodes=5):
    # Setup policy sampling
    def sample_actions(ts, obs, _rng):
        y, updates = agent_conf.network.apply({'params': ts.params, 'run_stats': ts.run_stats}, obs, mutable=["run_stats"])
        pi, _ = y
        return pi.mean(), ts  # Use mean for deterministic evaluation

    plcy_call = jax.jit(sample_actions)
    train_state = agent_state.train_state
    
    # Handle multiple seeds if present
    if agent_conf.config.experiment.n_seeds > 1:
        train_state = jax.tree.map(lambda x: x[train_state_seed], train_state)

    rng = jax.random.PRNGKey(42)
    
    mass = env._model.body_mass.sum()
    g = 9.81
    dt = env.dt
    
    print(f"Robot Mass: {mass:.2f} kg")
    
    cots = []
    
    for ep in range(num_episodes):
        obs = env.reset()
        
        total_energy = 0.0
        start_pos = env._data.qpos[0:2].copy()
        
        done = False
        step = 0
        
        while not done and step < 2000:
            rng, _rng = jax.random.split(rng)
            action, train_state = plcy_call(train_state, obs, _rng)
            action = np.atleast_2d(action)
            
            obs, reward, absorbing, done, info = env.step(action)
            
            # P = sum(|tau * qdot|)
            # qfrc_actuator is zero for the 6 unactuated root DoFs
            power = np.sum(np.abs(env._data.qfrc_actuator * env._data.qvel))
            total_energy += power * dt
            step += 1
            
        end_pos = env._data.qpos[0:2].copy()
        distance = np.linalg.norm(end_pos - start_pos)
        
        if distance > 0.1:
            cot = total_energy / (mass * g * distance)
            cots.append(cot)
            print(f"Episode {ep+1} | Steps: {step} | Distance: {distance:.2f}m | Energy: {total_energy:.2f}J | CoT: {cot:.3f}")
        else:
            print(f"Episode {ep+1} | Robot fell or barely moved (distance: {distance:.2f}m). Ignoring CoT.")
            
    if len(cots) > 0:
        mean_cot = np.mean(cots)
        std_cot = np.std(cots)
        print("-" * 50)
        print(f"Final CoT: {mean_cot:.3f} ± {std_cot:.3f}")
        print("-" * 50)
    else:
        print("Could not evaluate CoT (robot failed all episodes).")

def main():
    parser = argparse.ArgumentParser(description='Evaluate Cost of Transport (CoT) for a trained PPOJax policy.')
    parser.add_argument('--path', type=str, required=True, help='Path to the agent directory (e.g. policies/Baseline_Active_Policy/walk_02_02)')
    parser.add_argument('--episodes', type=int, default=10, help='Number of episodes to average')
    args = parser.parse_args()

    path = Path(args.path)
    print(f"Loading agent from {path}...")
    agent_conf, agent_state = PPOJax.load_agent(path)
    config = agent_conf.config

    # get task factory
    factory = TaskFactory.get_factory_cls(config.experiment.task_factory.name)

    # create env (CPU MuJoCo for accurate metric extraction)
    OmegaConf.set_struct(config, False)
    config.experiment.env_params["headless"] = True
    
    if "goal_params" in config.experiment.env_params:
        config.experiment.env_params["goal_params"]["visualize_goal"] = False

    env = factory.make(**config.experiment.env_params, **config.experiment.task_factory.params)
    
    evaluate_cot(env, agent_conf, agent_state, num_episodes=args.episodes)

if __name__ == "__main__":
    main()
