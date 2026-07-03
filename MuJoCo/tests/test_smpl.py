import sys
try:
    import torch
    print("torch ok")
    import smplx
    print("smplx ok")
    from smplx import SMPLH
    print("SMPLH ok")
    
    # Try importing loco_mujoco's parser
    import loco_mujoco.smpl.parser
    print("loco_mujoco.smpl.parser ok")
    
except Exception as e:
    print("Error:", e)
