# K-Bot Legs-Only Control Report Draft

## 1. System model
We model the modified K-Bot as a floating-base humanoid with leg joints only. The generalized coordinates are

$$
q = \begin{bmatrix} q_b \\ q_j \end{bmatrix}
$$

where $q_b$ are the 6 floating-base coordinates and $q_j$ are the actuated leg joints. The rigid-body dynamics are

$$
M(q)\dot{v} + c(q,v) + g(q) = S^T\tau + J_c(q)^T\lambda
$$

where $M(q)$ is the mass matrix, $c(q,v)$ contains Coriolis and centrifugal terms, $g(q)$ is gravity, $\tau$ is the joint torque vector, and $J_c(q)^T\lambda$ represents the contact wrenches at the feet.

For standing and slow motion, this is the core equation we need.

## 2. Reduced leg model
If we only care about the legs, the actuated dynamics can be written as

$$
M_j(q)\ddot{q}_j + h_j(q,\dot{q}) = \tau + J_{c,j}^T\lambda
$$

where $h_j(q,\dot{q})$ collects Coriolis, centrifugal, and gravity effects projected into joint space. In static standing, $\dot{q}=0$ and $\ddot{q}=0$, so the required holding torque is the gravity/contact balance.

If the feet are fixed on the ground, the contact wrench term balances the base load and the joint torques hold posture.

## 3. Control law
For the thesis baseline, use joint-space PD with gravity compensation:

$$
\tau = K_p(q_d - q) + K_d(\dot{q}_d - \dot{q}) + \tau_g(q)
$$

where $\tau_g(q)$ is gravity compensation. Add torque saturation if needed:

$$
\tau \leftarrow \mathrm{clip}(\tau, -\tau_{max}, \tau_{max})
$$

This is simple, stable for slow motion, and easy to validate.

## 4. Current estimate
With motor torque constant $K_t$ and gear ratio $N$, the approximate current is

$$
I \approx \frac{\tau}{K_t N}
$$

You can add friction and efficiency afterward if needed. This is enough for comparing standing, stepping, and disturbance cases.

## 5. Validation tests
Use these tests:

1. Static standing pose hold.
2. Single-joint step response.
3. Small squat trajectory.
4. Push disturbance while standing.
5. Repeatability across at least 5 trials.

Metrics: RMS position error, peak torque, RMS torque, estimated current, and recovery time.

## 6. MuJoCo implementation
In MuJoCo, use the model's inverse dynamics or bias forces for gravity compensation, then apply the PD law in Python. For a floating-base humanoid, the key MuJoCo quantities are `qfrc_bias`, `qfrc_actuator`, and the joint states `qpos`, `qvel`.

## 7. Baseline controller equation
A practical implementation is

$$
\tau_{cmd} = K_p(q_d - q) + K_d(\dot{q}_d - \dot{q}) + \tau_g
$$

with $\tau_g$ taken from inverse dynamics or bias forces, and then mapped to actuator command values.

## 8. What to write in the report
A clean report sequence is:

- System description.
- Floating-base dynamics.
- Reduced leg dynamics.
- Control law.
- Torque-to-current estimation.
- Validation tests.

This makes the control strategy easy to explain and defend.
