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
def load_gait_trajectory(env, npz_path):
    data = np.load(npz_path)
    qpos = data['qpos']
    qvel = data['qvel']
    N_steps = qpos.shape[0]

    model = env.get_model()
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
    
    class DummyConf:
        pass
        
    config = DummyConf()
    config.experiment = DummyConf()
    config.experiment.env_params = env_params
    config.experiment.task_factory = DummyConf()
    config.experiment.task_factory.name = "ImitationFactory"
    config.experiment.task_factory.params = {}
    config.experiment.validation = DummyConf()
    config.experiment.validation.active = False
    config.experiment.n_seeds = 1
    config.experiment.debug = True
    config.experiment.num_steps = 200
    config.experiment.total_timesteps = int(300e6)
    config.experiment.update_epochs = 4
    config.experiment.proportion_env_reward = 0.0
    config.experiment.num_minibatches = 32
    config.experiment.gamma = 0.99
    config.experiment.gae_lambda = 0.95
    config.experiment.clip_eps = 0.2
    config.experiment.init_std = 0.2
    config.experiment.learnable_std = False
    config.experiment.ent_coef = 0.0
    config.experiment.vf_coef = 0.5
    config.experiment.max_grad_norm = 0.5
    config.experiment.activation = "tanh"
    config.experiment.anneal_lr = False
    config.experiment.weight_decay = 0.0
    config.experiment.normalize_env = True
    config.experiment.lr = 1e-4
    config.experiment.num_envs = 2048
    config.experiment.hidden_layers = [512, 256]

    print("Loading trajectory...")
    traj = load_gait_trajectory(None, npz_path)
    custom_dataset_conf = CustomDatasetConf()
    custom_dataset_conf.traj = traj

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
