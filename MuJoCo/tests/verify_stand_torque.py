import mujoco
import numpy as np
import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, '..', 'controllers'))

from wbc_controller import MinimalWBC
from gait_generator import GaitGenerator, GaitMode

# Run test_kbot_legs to generate the legs-only model if it doesn't exist
# Actually, test_kbot_legs will run the viewer. We don't want that.
# Let's extract the model generation logic.

import xml.etree.ElementTree as ET

def generate_legs_only_model():
    original_robot_path = os.path.join(current_dir, "..", "kbot", "robot.mjcf")
    tree = ET.parse(original_robot_path)
    root = tree.getroot()
    torso_body = root.find(".//body[@name='torso']")
    if torso_body is not None:
        arms_to_remove = []
        for b in torso_body.findall("body"):
            name = b.attrib.get("name", "")
            if "ShldYokeDrive" in name or name.startswith("KC_") or name.startswith("KD_C_"):
                arms_to_remove.append(b)
        for arm in arms_to_remove:
            torso_body.remove(arm)
        base_body = root.find(".//body[@name='base']")
        if base_body is not None:
            base_visual = base_body.find("./geom[@name='base_visual']")
            if base_visual is not None:
                base_body.remove(base_visual)
        torso_origin = torso_body.find("./geom[@name='torso_origin']")
        if torso_origin is not None: torso_body.remove(torso_origin)
        torso_visual = torso_body.find("./geom[@name='torso_visual']")
        if torso_visual is not None: torso_body.remove(torso_visual)
        
        pelvis_visual = ET.Element("geom", {"name": "pelvis_visual", "type": "box", "size": "0.06 0.12 0.04", "pos": "-0.025 0 0.0", "material": "torso_material", "class": "visual"})
        torso_body.append(pelvis_visual)
        torso_collision = ET.Element("geom", {"name": "torso_collision", "type": "box", "size": "0.12 0.15 0.08", "pos": "-0.025 0 0.1", "class": "collision"})
        torso_body.append(torso_collision)
        imu_body = torso_body.find("body[@name='imu']")
        if imu_body is not None: torso_body.remove(imu_body)
        
        left_femur = torso_body.find(".//body[@name='KD_D_301L_L_Femur_Lower_Drive']")
        if left_femur is not None: left_femur.append(ET.Element("geom", {"name": "L_femur_collision", "type": "capsule", "fromto": "0 0 0 0 0 -0.2", "size": "0.045", "class": "collision"}))
        right_femur = torso_body.find(".//body[@name='KD_D_301R']")
        if right_femur is not None: right_femur.append(ET.Element("geom", {"name": "R_femur_collision", "type": "capsule", "fromto": "0 0 0 0 0 -0.2", "size": "0.045", "class": "collision"}))
        left_shin = torso_body.find(".//body[@name='KD_D_401L_L_Shin_Drive']")
        if left_shin is not None: left_shin.append(ET.Element("geom", {"name": "L_shin_collision", "type": "capsule", "fromto": "0 0 0 0 -0.29 0", "size": "0.035", "class": "collision"}))
        right_shin = torso_body.find(".//body[@name='KD_D_401R']")
        if right_shin is not None: right_shin.append(ET.Element("geom", {"name": "R_shin_collision", "type": "capsule", "fromto": "0 0 0 0 -0.29 0", "size": "0.035", "class": "collision"}))
        
    actuator = root.find("actuator")
    if actuator is not None:
        motors_to_remove = [motor for motor in actuator.findall("motor") if any(x in motor.attrib.get("name", "") for x in ["shoulder", "elbow", "wrist"])]
        for motor in motors_to_remove: actuator.remove(motor)
    
    sensor = root.find("sensor")
    if sensor is not None:
        sensors_to_remove = [s for s in sensor if "imu" in s.attrib.get("name", "") or "imu" in s.attrib.get("site", "")]
        for s in sensors_to_remove: sensor.remove(s)
        
    robot_legs_path = os.path.join(current_dir, "..", "kbot", "robot_legs.mjcf")
    tree.write(robot_legs_path, encoding="utf-8", xml_declaration=True)
    
    original_scene_path = os.path.join(current_dir, "..", "scene", "kbot_in_scene.xml")
    with open(original_scene_path, "r") as f: scene_content = f.read()
    scene_content = scene_content.replace('<include file="kbot/robot.mjcf"/>', '<include file="kbot/robot_legs.mjcf"/>')
    legs_scene_path = os.path.join(current_dir, "..", "scene", "kbot_legs_in_scene.xml")
    with open(legs_scene_path, "w") as f: f.write(scene_content)
    return legs_scene_path

robot_legs_path = os.path.join(current_dir, "..", "kbot", "robot_legs.mjcf")

# Setup simulation
wbc = MinimalWBC(robot_legs_path, knee_bend=0.45)
gait_gen = GaitGenerator(mode=GaitMode.STAND)

max_l_ankle_torque = 0.0
max_r_ankle_torque = 0.0

dt = wbc.model.opt.timestep

for i in range(int(5.0 / dt)):
    current_time = wbc.data.time
    targets = gait_gen.get_targets(current_time)
    wbc.step(targets)
    
    # Wait 2 seconds for robot to settle into static standing
    if current_time > 2.0:
        l_ankle_torque = abs(wbc.data.ctrl[4])
        r_ankle_torque = abs(wbc.data.ctrl[9])
        
        if l_ankle_torque > max_l_ankle_torque:
            max_l_ankle_torque = l_ankle_torque
        if r_ankle_torque > max_r_ankle_torque:
            max_r_ankle_torque = r_ankle_torque

print("="*50)
print("STATIC STANDING TORQUE VERIFICATION")
print(f"Max L_Ankle Torque (t > 2.0s): {max_l_ankle_torque:.4f} Nm")
print(f"Max R_Ankle Torque (t > 2.0s): {max_r_ankle_torque:.4f} Nm")
print("="*50)
