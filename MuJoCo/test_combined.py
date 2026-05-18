import mujoco
import mujoco.viewer
import time
import os

# Set the path to the combined scene MJCF file
current_dir = os.path.dirname(os.path.abspath(__file__))
scene_path = os.path.join(current_dir, "kbot_in_scene.xml")

print(f"Loading combined scene from: {scene_path}")

try:
    # Load the model
    # Specifying the assets path helps MuJoCo find meshes referenced in included files
    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)

    # Optional: Print some info about the model
    print(f"Model loaded successfully.")
    print(f"Number of bodies: {model.nbody}")
    print(f"Number of joints: {model.njnt}")
    print(f"Number of actuators: {model.nu}")

    # Launch the viewer
    print("Launching MuJoCo viewer (passive)...")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Initial camera setup (optional)
        viewer.cam.distance = 3.0
        viewer.cam.elevation = -20
        
        start_time = time.time()
        while viewer.is_running():
            step_start = time.time()

            # Step the simulation
            mujoco.mj_step(model, data)

            # Sync the viewer
            viewer.sync()

            # Maintain real-time simulation speed
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
            
            # Auto-close after 2 minutes for testing
            if time.time() - start_time > 120:
                break

    print("Simulation finished.")

except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
