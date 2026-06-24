import numpy as np
from loco_mujoco.task_factories import ImitationFactory, DefaultDatasetConf

print("Creating Unitree G1 Environment...")
# We use the UnitreeG1 environment and load a default 'walk' dataset if available
# If UnitreeG1 doesn't have a specific dataset, it might fallback to standard retargeting
env = ImitationFactory.make("UnitreeG1",
                            default_dataset_conf=DefaultDatasetConf(["walk"]),
                            n_substeps=20)

print("Playing Trajectory. You should see a MuJoCo window pop up!")
env.play_trajectory(n_episodes=2, n_steps_per_episode=500, render=True)
