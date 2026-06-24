import os

current_dir = os.path.dirname(os.path.abspath(__file__))
kbot_dir = os.path.join(current_dir, "kbot")
urdf_meshes_dir = os.path.join(kbot_dir, "urdf", "meshes")

if os.path.exists(urdf_meshes_dir):
    print(f"Cleaning directory: {urdf_meshes_dir}")
    removed_count = 0
    for filename in os.listdir(urdf_meshes_dir):
        if filename.endswith(".obj") or filename.endswith(".mtl"):
            file_path = os.path.join(urdf_meshes_dir, filename)
            os.remove(file_path)
            removed_count += 1
            print(f" Removed: {filename}")
    print(f"Cleaned up {removed_count} files.")
else:
    print("Directory not found.")
