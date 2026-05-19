import os
import shutil

current_dir = os.path.dirname(os.path.abspath(__file__))

kbot_dir = os.path.join(current_dir, "kbot")
urdf_dest_dir = os.path.join(kbot_dir, "urdf")
urdf_meshes_dest_dir = os.path.join(urdf_dest_dir, "meshes")

# 1. Create target directories
os.makedirs(urdf_meshes_dest_dir, exist_ok=True)
print(f"Created directories:\n - {urdf_dest_dir}\n - {urdf_meshes_dest_dir}")

# 2. Move URDF files
urdf_files = ["robot.urdf", "robot_legs.urdf"]
for f in urdf_files:
    src = os.path.join(kbot_dir, f)
    dst = os.path.join(urdf_dest_dir, f)
    if os.path.exists(src):
        shutil.move(src, dst)
        print(f"Moved URDF: {src} -> {dst}")

# 3. Move converted OBJ/MTL meshes
meshes_src_dir = os.path.join(kbot_dir, "meshes")
if os.path.exists(meshes_src_dir):
    for filename in os.listdir(meshes_src_dir):
        if filename.startswith("converted_"):
            src = os.path.join(meshes_src_dir, filename)
            dst = os.path.join(urdf_meshes_dest_dir, filename)
            shutil.move(src, dst)
            print(f"Moved Mesh: {filename} -> {urdf_meshes_dest_dir}")

print("Successfully moved all converted URDF files and assets!")
