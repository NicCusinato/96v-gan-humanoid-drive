import mujoco
import mujoco.viewer
import time
import numpy as np
from PIL import Image

def main():
    model = mujoco.MjModel.from_xml_path("c:/96v_gan_humanoid_drive/MuJoCo/kbot/robot.mjcf")
    data = mujoco.MjData(model)
    
    # Step once to update kinematics
    mujoco.mj_forward(model, data)
    
    # Create renderer to save image
    renderer = mujoco.Renderer(model, 480, 640)
    renderer.update_scene(data, camera=-1)
    img = renderer.render()
    
    # Save image
    Image.fromarray(img).save("c:/96v_gan_humanoid_drive/MuJoCo/kbot_default_pose.png")
    print("Saved KBot default pose to kbot_default_pose.png")

if __name__ == "__main__":
    main()
