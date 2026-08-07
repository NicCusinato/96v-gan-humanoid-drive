import os
import sys
import numpy as np
import jax
import jax.numpy as jnp
from dataclasses import dataclass

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, "..", "loco-mujoco"))

from loco_mujoco import TaskFactory
from loco_mujoco.algorithms import PPOJax
from loco_mujoco.trajectory import Trajectory, TrajectoryInfo, TrajectoryModel, TrajectoryData
from loco_mujoco.task_factories.dataset_confs import CustomDatasetConf
import mujoco

# We have to parse the 07_01_poses.npz
def load_gait_trajectory(npz_path):
    data = np.load(npz_path)
    qpos = data['qpos']
    qvel = data['qvel']
    N_steps = qpos.shape[0]

    current_dir = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(current_dir, "..", "loco-mujoco", "loco_mujoco", "models", "kbot_v2", "kbot_v2.xml")
    model = mujoco.MjModel.from_xml_path(xml_path)
    
    njnt = model.njnt
    jnt_type = model.jnt_type
    jnt_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(njnt)]
    
    traj_info = TrajectoryInfo(jnt_names, model=TrajectoryModel(njnt, jnp.array(jnt_type)), frequency=50.0)
    traj_data = TrajectoryData(jnp.array(qpos), jnp.array(qvel), split_points=jnp.array([0, N_steps]))
    
    traj = Trajectory(traj_info, traj_data)
    return traj

def main():
    os.environ['XLA_FLAGS'] = '--xla_gpu_triton_gemm_any=True '
    
    # 1. Initialize environment
    env_name = "MjxKBotV2"
    
    # We need to construct a configuration object similar to what hydra does.
    # But since we are bypassing hydra, we can manually create the dictionaries.
    
    # Alternatively, we can use the environment directly and PPOJax.
    factory = TaskFactory.get_factory_cls("ImitationFactory")
    
    env_params = {
        "env_name": env_name,
        "headless": True,
        "disable_arms": False,
        "horizon": 1000,
        "goal_type": "GoalTrajMimic",
        "goal_params": {"visualize_goal": True},
        "reward_type": "MimicReward",
        "reward_params": {
            "qpos_w_sum": 0.4,
            "qvel_w_sum": 0.2,
            "rpos_w_sum": 0.5,
            "rquat_w_sum": 0.3,
            "rvel_w_sum": 0.1,
            "sites_for_mimic": [
                "upper_body_mimic",
                "pelvis_mimic",
                "left_foot_mimic",
                "right_foot_mimic"
            ]
        }
    }
    
    # Load dataset
    current_dir = os.path.dirname(os.path.abspath(__file__))
    npz_path = os.path.join(current_dir, "..", "loco-mujoco", "gait_data", "walk", "07_01_poses.npz")
    
    # Need to instantiate the environment FIRST to parse the trajectory properly (or we can just pass the traj)
    # Wait, get_custom_dataset inside ImitationFactory expects `env` as argument to do FK to get rpos.
    
    from omegaconf import OmegaConf
    
    config = OmegaConf.create({
        "experiment": {
            "env_params": env_params,
            "task_factory": {
                "name": "ImitationFactory",
                "params": {}
            },
            "validation": {
                "active": False,
                "num": 10
            },
            "n_seeds": 1,
            "debug": True,
            "num_steps": 200,
            "total_timesteps": int(300e6),
            "update_epochs": 4,
            "proportion_env_reward": 0.0,
            "num_minibatches": 32,
            "gamma": 0.99,
            "gae_lambda": 0.95,
            "clip_eps": 0.2,
            "init_std": 0.2,
            "learnable_std": False,
            "ent_coef": 0.0,
            "vf_coef": 0.5,
            "max_grad_norm": 0.5,
            "activation": "tanh",
            "anneal_lr": False,
            "weight_decay": 0.0,
            "normalize_env": True,
            "lr": 1e-4,
            "num_envs": 2048,
            "hidden_layers": [512, 256]
        }
    })

    print("Loading trajectory...")
    traj = load_gait_trajectory(npz_path)
    custom_dataset_conf = CustomDatasetConf(traj=traj)

    print(f"Creating {env_name} environment...")
    env = factory.make(**env_params, custom_dataset_conf=custom_dataset_conf)

    print("Initializing agent...")
    agent_conf = PPOJax.init_agent_conf(env, config)

    print("Building train function...")
    train_fn = PPOJax.build_train_fn(env, agent_conf, mh=None)
    train_fn = jax.jit(train_fn)

    print("Starting training...")
    rng = jax.random.PRNGKey(0)
    out = train_fn(rng)
    
    print("Training finished!")
    
if __name__ == "__main__":
    main()
