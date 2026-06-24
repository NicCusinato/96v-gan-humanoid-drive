import os
import xml.etree.ElementTree as ET

current_dir = os.path.dirname(os.path.abspath(__file__))
urdf_dir = os.path.join(current_dir, "kbot", "urdf")

urdf_files = ["robot.urdf", "robot_legs.urdf"]

print("==================================================================")
print("             SIMPLIFYING URDF MODELS FOR SIMSCAPE                 ")
print("==================================================================")

for name in urdf_files:
    input_path = os.path.join(urdf_dir, name)
    base_name, ext = os.path.splitext(name)
    output_path = os.path.join(urdf_dir, f"{base_name}_simplified{ext}")
    
    if not os.path.exists(input_path):
        print(f"Warning: File not found: {input_path}")
        continue
        
    print(f"Simplifying {input_path} -> {output_path}...")
    
    # Parse the XML file
    # We register the namespace if any, but URDF typically does not have one
    tree = ET.parse(input_path)
    root = tree.getroot()
    
    # Find all <link> elements and remove their <visual> children
    link_count = 0
    visuals_removed = 0
    for link in root.findall(".//link"):
        link_count += 1
        # Find all <visual> children and remove them
        visuals = link.findall("visual")
        for visual in visuals:
            link.remove(visual)
            visuals_removed += 1
            
    # Write the simplified URDF
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    print(f" Completed: processed {link_count} links, removed {visuals_removed} visual tags.")

print("==================================================================")
