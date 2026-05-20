% leg_params.m
% Lumped parameters for the 3-DOF Planar Sagittal Leg Model (Model B).
% Derived from robot_legs.mjcf for the left leg.
% Units: Metric (meters, kilograms, radians, seconds, Newton-meters).

% 1. Kinematic Link Lengths (projected onto the sagittal X-Z plane, in meters)
% l1: Hip-to-Knee, l2: Knee-to-Ankle, l3: Ankle-to-Foot Center
leg_params.l1 = 0.3854205951336591;
leg_params.l2 = 0.2915475941516295;
leg_params.l3 = 0.04780164910585583;
leg_params.lengths = [leg_params.l1; leg_params.l2; leg_params.l3];

% Nominal joint-to-joint vectors at q=0 (X-Z plane)
leg_params.v1 = [-0.01799998; -0.38500005];  % hip -> knee
leg_params.v2 = [0.03000019; -0.28999998];   % knee -> ankle
leg_params.v3 = [0.02899999; -0.03799998];   % ankle -> foot site

% 2. Lumped Masses (in kilograms)
% m1: Thigh, m2: Shank, m3: Foot
leg_params.m1 = 5.296577;
leg_params.m2 = 1.684707;
leg_params.m3 = 0.608966;
leg_params.masses = [leg_params.m1; leg_params.m2; leg_params.m3];

% 3. Lumped Center of Mass Offsets (relative to the respective joint origin in X-Z coordinates, in meters)
% r1: Thigh COM relative to Hip Pitch joint
% r2: Shank COM relative to Knee joint
% r3: Foot COM relative to Ankle joint
leg_params.r1 = [-0.00317863; -0.19441240];
leg_params.r2 = [0.02411602; -0.10430798];
leg_params.r3 = [0.01654899; -0.03114199];
leg_params.com_offsets = {leg_params.r1, leg_params.r2, leg_params.r3};

% 4. Lumped Moments of Inertia (rotation around Y-axis in sagittal plane, I_yy, in kg*m^2)
% Evaluated at the lumped link COM
leg_params.I1 = 0.12715176;
leg_params.I2 = 0.01466400;
leg_params.I3 = 0.00189300;
leg_params.inertias = [leg_params.I1; leg_params.I2; leg_params.I3];

% 5. Joint Limits (in radians)
leg_params.joint_limits.hip_pitch = [-1.047198, 2.216568];
leg_params.joint_limits.knee_pitch = [0.0, 2.705260];
leg_params.joint_limits.ankle_pitch = [-1.134464, 0.261799];

% 6. Actuator and Drive Specifications
% Gear ratios (N) and motor rotor inertias (Jm) derived from Robstride specs
% Reflected Armature (J_reflected) = N^2 * Jm.
% Gear ratios: RS-04 (9:1), RS-02 (7.75:1)
leg_params.gear_ratios = [9.0; 9.0; 7.75];

% Motor rotor inertias (in kg*m^2)
% Hip/Knee (RS-04): Jm = 0.04 / 9^2 = 0.000493827
% Ankle (RS-02): Jm = 0.0042 / 7.75^2 = 0.000069927
leg_params.motor_inertias = [ ...
    0.04 / (9.0^2); ...     % Hip Pitch
    0.04 / (9.0^2); ...     % Knee Pitch
    0.0042 / (7.75^2) ...   % Ankle Pitch
];

% Reflected motor inertias at the joint (in kg*m^2)
leg_params.reflected_inertias = leg_params.gear_ratios.^2 .* leg_params.motor_inertias;  % [0.04; 0.04; 0.0042]

% 7. Joint Friction and Damping
% Damping (B) and Coulomb friction (frictionloss) from MJCF
leg_params.damping_coefs = [0.0; 0.0; 0.0];
leg_params.coulomb_friction = [0.2; 0.2; 0.1];
leg_params.stiction_torque = 1.3 * leg_params.coulomb_friction;  % Estimate stiction as 130% of Coulomb friction

% Torque Constants (Kt, in N*m/Arms)
leg_params.torque_constants = [2.1; 2.1; 1.22];

% Peak Torques (in N*m)
leg_params.max_torques = [120.0; 120.0; 17.0];

% =============================================================================
% MODEL A: KNEE DYNO SPECIFIC REDUCED PARAMETERS (HIP LOCKED, ANKLE LOCKED)
% =============================================================================
% Equivalent distal inertia of the shank + foot lumped about the knee joint:
% I_distal = I2 + m2*||r2||^2 + I3 + m3*||v2 + r3||^2
leg_params.I_distal = 0.09999014;  % kg*m^2

% Reflected motor inertia of Knee Actuator (RS-04): J_reflected = N^2 * Jm
leg_params.J_reflected = leg_params.reflected_inertias(2);  % 0.04 kg*m^2

% Total equivalent inertia seen at the knee joint:
leg_params.I_total = leg_params.I_distal + leg_params.J_reflected;  % 0.13999014 kg*m^2

% Combined mass of shank + foot:
leg_params.m_distal = leg_params.m2 + leg_params.m3;  % 2.293673 kg

% Combined COM location relative to the knee joint at q_knee = 0:
v2_plus_r3 = leg_params.v2 + leg_params.r3;
leg_params.r_distal = (leg_params.m2 * leg_params.r2 + leg_params.m3 * v2_plus_r3) / leg_params.m_distal;  % [0.030064; -0.091796] m
leg_params.d_com_distal = norm(leg_params.r_distal);  % 0.096593 m

% Gravity torque formula constants:
% G_knee(q_knee) = - g * (a_g * cos(q_knee) + b_g * sin(q_knee))
% where g = 9.81 m/s^2 (or 9.80665 m/s^2 for MuJoCo match)
leg_params.a_g = leg_params.m2 * leg_params.r2(1) + leg_params.m3 * v2_plus_r3(1);  % 0.06897519 kg*m
leg_params.b_g = leg_params.m2 * leg_params.r2(2) + leg_params.m3 * v2_plus_r3(2);  % -0.37129112 kg*m

