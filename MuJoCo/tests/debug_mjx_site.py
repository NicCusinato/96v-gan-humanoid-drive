import mujoco
from mujoco import mjx
import os

xml_path = "loco-mujoco/loco_mujoco/models/kbot_v2/kbot_v2.xml"
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

print("Standard MuJoCo site_xmat shape:", data.site_xmat.shape)

mjx_model = mjx.put_model(model)
mjx_data = mjx.put_data(model, data)

print("MJX site_xmat shape:", mjx_data.site_xmat.shape)
