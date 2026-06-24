import mujoco
from loco_mujoco.trajectory import Trajectory
from loco_mujoco.environments.humanoids.kbot import KBot
import matplotlib.pyplot as plt

def render_frame():
    env = KBot()
    model = env.model
    data = env.data

    traj = Trajectory.load("loco-mujoco/kbot_walk_retargeted.npz")
    frame = 50 # Let's grab the 50th frame to see it mid-stride
    
    data.qpos[:] = traj.data.qpos[frame]
    data.qvel[:] = traj.data.qvel[frame]
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, height=480, width=640)
    renderer.update_scene(data, camera=-1)
    pixels = renderer.render()

    plt.imsave("kbot_walk_frame50.png", pixels)
    print("Saved render to kbot_walk_frame50.png")

if __name__ == "__main__":
    render_frame()
