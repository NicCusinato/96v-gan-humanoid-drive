# K-Bot Legs-Only Control Section

## 1. Notation and generalized coordinates
We model the modified K-Bot as a floating-base humanoid with leg joints only. The generalized coordinates are

\[
q = \begin{bmatrix} q_b \\ q_j \end{bmatrix}
\]

where \(q_b\) are the 6 floating-base coordinates and \(q_j\) are the actuated leg joints. The generalized velocities are denoted by \(v = \dot{q}\). The main variables used in this section are:

- \(q_b \in \mathbb{R}^6\): base position and orientation.
- \(q_j \in \mathbb{R}^{n_j}\): actuated leg joint angles.
- \(v \in \mathbb{R}^{n_v}\): generalized velocity vector.
- \(\tau \in \mathbb{R}^{n_j}\): joint torque vector.
- \(\lambda \in \mathbb{R}^{n_c}\): contact wrench at the feet.
- \(J_c(q) \in \mathbb{R}^{n_c \times n_v}\): contact Jacobian.
- \(S \in \mathbb{R}^{n_j \times n_v}\): selection matrix mapping generalized forces to joint torques.

Bold lower-case letters denote vectors, bold upper-case letters denote matrices, and superscript \(T\) denotes matrix transpose.

## 2. Floating-base dynamics
Under the standard rigid-body assumption, the dynamics of the floating-base humanoid can be written as

\[
M(q)\dot{v} + c(q,v) + g(q) = S^T\tau + J_c(q)^T\lambda,
\]

where

- \(M(q) \in \mathbb{R}^{n_v \times n_v}\) is the mass matrix,
- \(c(q,v)\) collects Coriolis and centrifugal terms,
- \(g(q)\) is the generalized gravity vector,
- \(S^T\tau\) injects joint torques into generalized coordinates,
- \(J_c(q)^T\lambda\) is the contribution of foot contact forces [web:179][web:176].

This equation is valid for both stance and swing; in stance, the contact forces \(\lambda\) enforce non-penetration and friction constraints at the feet [web:179].

## 3. Reduced leg dynamics
Partitioning the dynamics into floating-base and joint components yields a reduced model for the actuated leg joints:

\[
M_j(q)\ddot{q}_j + h_j(q,\dot{q}) = \tau + J_{c,j}(q)^T\lambda,
\]

where

- \(M_j(q) \in \mathbb{R}^{n_j \times n_j}\) is the joint-space inertia,
- \(h_j(q,\dot{q})\) collects Coriolis, centrifugal, and gravity effects in joint space,
- \(J_{c,j}(q)\) is the contact Jacobian projected onto the actuated joints [web:176][web:179].

For static standing, we take \(\dot{q} = 0\) and \(\ddot{q} = 0\). The reduced dynamics then simplify to

\[
0 = \tau_{\text{stand}} + J_{c,j}(q)^T \lambda - h_j(q,0),
\]

which can be rearranged as

\[
\tau_{\text{stand}} = h_j(q,0) - J_{c,j}(q)^T \lambda.
\]

If both feet are firmly in contact and \(\lambda\) is chosen such that the net wrench balances the robot’s weight and external loads, \(\tau_{\text{stand}}\) is the joint torque required to hold the given posture [web:77][web:176]. In practice, we obtain this quantity from inverse dynamics or bias-force computation.

## 4. MuJoCo dynamics interface
MuJoCo exposes the dynamics through:

- The mass matrix \(M(q)\),
- The bias-force vector \(qfrc\_bias(q,v)\), which equals \(c(q,v) + g(q)\) in the absence of external forces,
- Inverse dynamics via `mj_inverse`, which computes generalized forces given \((q,\dot{q},\ddot{q})\) [web:68][web:159].

For gravity compensation and standing analysis, we use the bias forces with zero velocity and acceleration:

1. Set \(\dot{q} = 0\), \(\ddot{q} = 0\).
2. Evaluate bias forces \(qfrc\_bias(q,0)\).
3. Extract the actuated-joint components as the gravity-compensation torque \(\tau_g(q)\).

This provides a model-consistent estimate of the torque needed to counter gravity and static loads for a given configuration [web:68][web:77].

## 5. Joint-space PD with gravity compensation
We adopt a joint-space proportional–derivative (PD) controller with gravity compensation. Let \(q_d(t)\) and \(\dot{q}_d(t)\) denote the desired joint positions and velocities. The control law is

\[
\tau = K_p (q_d - q_j) + K_d (\dot{q}_d - \dot{q}_j) + \tau_g(q),
\]

where

- \(K_p = \operatorname{diag}(k_{p,1},\dots,k_{p,n_j})\) is the proportional gain matrix,
- \(K_d = \operatorname{diag}(k_{d,1},\dots,k_{d,n_j})\) is the derivative gain matrix,
- \(\tau_g(q)\) is the gravity-compensation torque from the model [web:68][web:71].

To respect actuator limits, we apply elementwise torque saturation:

\[
\tau_i \leftarrow \operatorname{clip}(\tau_i, -\tau_{i,\max}, \tau_{i,\max}), \quad i = 1,\dots,n_j.
\]

Neglecting Coriolis and contact effects, and assuming \(\tau_g \approx h_j(q,0)\), the closed-loop error dynamics for joint \(i\) are approximately

\[
\ddot{e}_i + \frac{k_{d,i}}{m_{j,i}} \dot{e}_i + \frac{k_{p,i}}{m_{j,i}} e_i \approx 0,
\]

with tracking error \(e_i = q_{d,i} - q_{j,i}\) and effective inertia \(m_{j,i}\) at joint \(i\). This facilitates gain tuning in terms of desired damping ratio and natural frequency [web:176][web:179].

## 6. Torque-to-current mapping
To relate joint torques to electrical current, we assume a rotary actuator with torque constant \(K_t\), gear ratio \(N\), and gearbox efficiency \(\eta_g\). Neglecting friction, the motor torque \(\tau_m\) and armature current \(I\) satisfy

\[
\tau_m \approx K_t I,
\]

and the joint torque is related to motor torque by

\[
\tau \approx N \eta_g \tau_m.
\]

Combining these relationships gives

\[
I \approx \frac{\tau}{K_t N \eta_g}.
\]

In the simplest case we take \(\eta_g \approx 1\) and refine the model with measured efficiency and friction as needed. This approximation is sufficient to compare peak and RMS current across standing, squat, and disturbance tests.

## 7. Assumptions
The analysis and controller rely on the following assumptions:

1. Rigid-body links and ideal joints; link flexibilities and backlash are neglected [web:179].
2. Any modeled friction is included in \(h_j(\cdot)\) and thus partially compensated by \(\tau_g\) [web:68].
3. During standing and squat tests, both feet remain in full, non-slipping contact with the ground, and the contact solver enforces the associated constraints [web:179].
4. Motions are sufficiently slow that unmodeled actuator bandwidth limits have limited impact on stability.
5. The inertial parameters in the MJCF model approximate the real robot; remaining errors are handled in validation.

## 8. Controller pseudocode
The PD + gravity-compensation controller can be expressed in MuJoCo-style pseudocode as follows:

```python
# Inputs at each control step:
#   model          : MuJoCo MjModel
#   data           : MuJoCo MjData
#   q_des          : desired joint positions (n_j,)
#   qd_des         : desired joint velocities (n_j,)
#   Kp, Kd         : gain vectors (n_j,)
#   tau_max        : torque limits (n_j,) or None
#   joint_idx      : indices of actuated joints in qpos/qvel
#   actuator_idx   : indices of actuators in data.ctrl

def leg_pd_gravity_step(model, data, q_des, qd_des, Kp, Kd, tau_max=None):
    # 1. Read current generalized state
    q = data.qpos.copy()
    v = data.qvel.copy()

    # 2. Extract actuated joint states
    q_j = q[joint_idx]
    v_j = v[joint_idx]

    # 3. Compute bias forces (includes gravity)
    mujoco.mj_forward(model, data)
    tau_bias = data.qfrc_bias.copy()
    tau_g = tau_bias[joint_idx]

    # 4. PD term
    e_pos = q_des - q_j
    e_vel = qd_des - v_j
    tau_pd = Kp * e_pos + Kd * e_vel

    # 5. Combine PD and gravity terms
    tau_cmd = tau_pd + tau_g

    # 6. Apply torque saturation
    if tau_max is not None:
        tau_cmd = np.clip(tau_cmd, -tau_max, tau_max)

    # 7. Write commands and step simulation
    data.ctrl[:] = 0.0
    data.ctrl[actuator_idx] = tau_cmd
    mujoco.mj_step(model, data)

    return tau_cmd
```

This routine is used in simulation to generate joint torque and current estimates under standing, squat, and disturbance tests, and also serves as a template for the real-time controller on the physical robot.
