import os
import shutil
import sys
import subprocess

current_dir = os.path.dirname(os.path.abspath(__file__))

try:
    from mjcf_urdf_simple_converter import convert
except ImportError:
    print("Installing mjcf-urdf-simple-converter...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "mjcf-urdf-simple-converter"])
    from mjcf_urdf_simple_converter import convert

# Paths
kbot_dir = os.path.join(current_dir, "kbot")
robot_mjcf = os.path.join(kbot_dir, "robot.mjcf")
robot_xml_tmp = os.path.join(kbot_dir, "robot.xml")
robot_urdf = os.path.join(kbot_dir, "robot.urdf")

robot_legs_mjcf = os.path.join(kbot_dir, "robot_legs.mjcf")
robot_legs_xml_tmp = os.path.join(kbot_dir, "robot_legs.xml")
robot_legs_urdf = os.path.join(kbot_dir, "robot_legs.urdf")

print("==================================================================")
print("             CONVERTING MJCF ROBOT MODELS TO URDF                 ")
print("==================================================================")

try:
    print(f"Creating temporary {robot_xml_tmp}...")
    shutil.copy2(robot_mjcf, robot_xml_tmp)
    print(f"Converting {robot_xml_tmp} -> {robot_urdf}...")
    convert(robot_xml_tmp, robot_urdf)
    print("Conversion of full robot model completed successfully!")
except Exception as e:
    print(f"Error converting full robot model: {e}")
finally:
    if os.path.exists(robot_xml_tmp):
        os.remove(robot_xml_tmp)
        print("Cleaned up temporary robot.xml")

try:
    print(f"Creating temporary {robot_legs_xml_tmp}...")
    shutil.copy2(robot_legs_mjcf, robot_legs_xml_tmp)
    print(f"Converting {robot_legs_xml_tmp} -> {robot_legs_urdf}...")
    convert(robot_legs_xml_tmp, robot_legs_urdf)
    print("Conversion of legs-only robot model completed successfully!")
except Exception as e:
    print(f"Error converting legs-only robot model: {e}")
finally:
    if os.path.exists(robot_legs_xml_tmp):
        os.remove(robot_legs_xml_tmp)
        print("Cleaned up temporary robot_legs.xml")

print("==================================================================")
