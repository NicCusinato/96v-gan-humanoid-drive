import os
import re
import shutil
import xml.etree.ElementTree as ET

current_dir = os.path.dirname(os.path.abspath(__file__))

kbot_dir = os.path.join(current_dir, "kbot")
robot_mjcf = os.path.join(kbot_dir, "robot.mjcf")
urdf_dir = os.path.join(kbot_dir, "urdf")
urdf_meshes_dir = os.path.join(urdf_dir, "meshes")

# 1. Parse robot.mjcf to get the mesh mappings
print("Parsing mesh assets from robot.mjcf...")
tree = ET.parse(robot_mjcf)
root = tree.getroot()

asset = root.find("asset")
if asset is None:
    raise ValueError("Asset block not found in robot.mjcf!")

mesh_mappings = {}  # maps name in mjcf -> source file path
for mesh in asset.findall("mesh"):
    name = mesh.attrib.get("name")
    file_path = mesh.attrib.get("file")
    if name and file_path:
        mesh_mappings[name] = file_path
        print(f" Found mesh mapping: '{name}' -> '{file_path}'")

# 2. Copy the original STL files to the urdf/meshes directory under their mjcf names
os.makedirs(urdf_meshes_dir, exist_ok=True)
print("\nCopying original STL files to urdf/meshes/...")
for name, file_path in mesh_mappings.items():
    src = os.path.abspath(os.path.join(kbot_dir, file_path))
    dst = os.path.join(urdf_meshes_dir, name)
    if os.path.exists(src):
        shutil.copy2(src, dst)
        print(f" Copied STL: {src} -> {dst}")
    else:
        print(f" Warning: Source STL not found: {src}")

# 3. Modify URDF files to point to STL meshes instead of converted OBJ meshes
urdf_files = ["robot.urdf", "robot_legs.urdf"]
for urdf_name in urdf_files:
    urdf_path = os.path.join(urdf_dir, urdf_name)
    if not os.path.exists(urdf_path):
        print(f"Warning: URDF file {urdf_path} does not exist.")
        continue
        
    print(f"\nProcessing URDF file: {urdf_path}...")
    with open(urdf_path, "r", encoding="utf-8") as f:
        urdf_content = f.read()
        
    updated_content = urdf_content
    # Find and replace each mesh reference
    # Format of old references: meshes/converted_<name>_<hash>.obj
    for name in mesh_mappings.keys():
        # Match pattern: meshes/converted_<name>_<any hex characters>.obj
        # Note: name could contain special characters, so we escape it
        escaped_name = re.escape(name)
        pattern = rf"meshes/converted_{escaped_name}_[a-f0-9]+\.obj"
        replacement = f"meshes/{name}"
        
        matches = re.findall(pattern, updated_content)
        if matches:
            updated_content = re.sub(pattern, replacement, updated_content)
            print(f"  Replaced mesh reference for '{name}': {len(matches)} occurrences")
            
    # Write updated URDF file
    with open(urdf_path, "w", encoding="utf-8") as f:
        f.write(updated_content)
    print(f"Successfully updated {urdf_name}")

print("\nAll conversions and file organization completed successfully!")
