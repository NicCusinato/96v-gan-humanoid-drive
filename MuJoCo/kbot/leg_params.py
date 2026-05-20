"""
Leg Parameter File for 3-DOF Planar Sagittal Leg Model (Model B).
This file contains the lumped parameters derived from robot_legs.mjcf for the left leg.
Units: Metric (meters, kilograms, radians, seconds, Newton-meters).
"""

import numpy as np

# 1. Kinematic Link Lengths (projected onto the sagittal X-Z plane, in meters)
# l1: Hip-to-Knee, l2: Knee-to-Ankle, l3: Ankle-to-Foot Center
l1 = 0.3854205951336591
l2 = 0.2915475941516295
l3 = 0.04780164910585583
lengths = np.array([l1, l2, l3])

# Nominal joint-to-joint vectors at q=0 (X-Z plane)
v1 = np.array([-0.01799998, -0.38500005])  # hip -> knee
v2 = np.array([0.03000019, -0.28999998])   # knee -> ankle
v3 = np.array([0.02899999, -0.03799998])   # ankle -> foot site

# 2. Lumped Masses (in kilograms)
# m1: Thigh (combining yoke, roll drive, and lower femur drive)
# m2: Shank (shin drive)
# m3: Foot (foot bushing)
m1 = 5.296577
m2 = 1.684707
m3 = 0.608966
masses = np.array([m1, m2, m3])

# 3. Lumped Center of Mass Offsets (relative to the respective joint origin in X-Z coordinates, in meters)
# r1: Thigh COM relative to Hip Pitch joint
# r2: Shank COM relative to Knee joint
# r3: Foot COM relative to Ankle joint
r1 = np.array([-0.00317863, -0.19441240])
r2 = np.array([0.02411602, -0.10430798])
r3 = np.array([0.01654899, -0.03114199])
com_offsets = [r1, r2, r3]

# 4. Lumped Moments of Inertia (rotation around Y-axis in sagittal plane, I_yy, in kg*m^2)
# Evaluated at the lumped link COM
I1 = 0.12715176
I2 = 0.01466400
I3 = 0.00189300
inertias = np.array([I1, I2, I3])

# 5. Joint Limits (in radians)
joint_limits = {
    'hip_pitch': [-1.047198, 2.216568],
    'knee_pitch': [0.0, 2.705260],
    'ankle_pitch': [-1.134464, 0.261799]
}

# 6. Actuator and Drive Specifications
# Gear ratios (N) and motor rotor inertias (Jm) derived from Robstride specs
# Reflected Armature (J_reflected) = N^2 * Jm.
# J_reflected is stored in the MJCF model as 'armature'.
# Gear ratios: RS-04 (9:1), RS-02 (7.75:1)
gear_ratios = np.array([9.0, 9.0, 7.75])

# Motor rotor inertias (in kg*m^2)
# Hip/Knee (RS-04): Jm = 0.04 / 9^2 = 0.000493827
# Ankle (RS-02): Jm = 0.0042 / 7.75^2 = 0.000069927
motor_inertias = np.array([
    0.04 / (9.0**2),     # Hip Pitch
    0.04 / (9.0**2),     # Knee Pitch
    0.0042 / (7.75**2)   # Ankle Pitch
])

# Reflected motor inertias at the joint (in kg*m^2)
reflected_inertias = gear_ratios**2 * motor_inertias  # [0.04, 0.04, 0.0042]

# 7. Joint Friction and Damping
# Damping (B) and Coulomb friction (frictionloss) from MJCF
damping_coefs = np.array([0.0, 0.0, 0.0])
coulomb_friction = np.array([0.2, 0.2, 0.1])
stiction_torque = 1.3 * coulomb_friction  # Estimate stiction as 130% of Coulomb friction

# Torque Constants (Kt, in N*m/Arms)
torque_constants = np.array([2.1, 2.1, 1.22])

# Peak Torques (in N*m)
max_torques = np.array([120.0, 120.0, 17.0])

# =============================================================================
# MODEL A: KNEE DYNO SPECIFIC REDUCED PARAMETERS (HIP LOCKED, ANKLE LOCKED)
# =============================================================================
# Equivalent distal inertia of the shank + foot lumped about the knee joint:
# I_distal = I2 + m2*||r2||^2 + I3 + m3*||v2 + r3||^2
I_distal = 0.09999014  # kg*m^2

# Reflected motor inertia of Knee Actuator (RS-04): J_reflected = N^2 * Jm
J_reflected = reflected_inertias[1]  # 0.04 kg*m^2

# Total equivalent inertia seen at the knee joint:
I_total = I_distal + J_reflected  # 0.13999014 kg*m^2

# Combined mass of shank + foot:
m_distal = m2 + m3  # 2.293673 kg

# Combined COM location relative to the knee joint at q_knee = 0:
r_distal = (m2 * r2 + m3 * (v2 + r3)) / m_distal  # [0.030064, -0.091796] m
d_com_distal = np.linalg.norm(r_distal)  # 0.096593 m

# Gravity torque formula constants:
# G_knee(q_knee) = - g * (a_g * cos(q_knee) + b_g * sin(q_knee))
# where g = 9.81 m/s^2 (or 9.80665 m/s^2 for MuJoCo match)
a_g = m2 * r2[0] + m3 * (v2[0] + r3[0])  # 0.06897519 kg*m
b_g = m2 * r2[1] + m3 * (v2[1] + r3[1])  # -0.37129112 kg*m

