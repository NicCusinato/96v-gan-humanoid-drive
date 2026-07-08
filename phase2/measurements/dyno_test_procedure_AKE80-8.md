# Dyno Test Procedure — CubeMars AKE80-8 with Inertia Leg & LocoMuJoCo Policy Replay

**Phase 2 | Status: Draft | Date: 2026-07-07**
**Related KANs:** KAN-56 (comparative test framework)
**Motor Under Test:** CubeMars AKE80-8 KV30 (QDD actuator, 8:1 planetary)
**Board Under Test:** 96V GaN inverter (KAN-54) evaluated at both 48V (Si-equivalent baseline) and 96V.

---

## 1. Objectives & Thesis Contribution

### 1.1 Purpose

This procedure defines the comprehensive **hardware-in-the-loop validation** of the 96V GaN drive
hypothesis under realistic humanoid gait loading. It builds upon the static $(T, \omega)$ sweep
methodology from [KAN-56](../../../docs/KAN-56_test_plan.md) by first establishing a full steady-state
performance baseline, and then extending the testing to subject the motor and inverter to
**dynamic, time-varying torque/speed profiles** extracted directly from trained LocoMuJoCo
locomotion policies. This bridges the gap between:

- **Simulation** — where we have full-body dynamics, trained policies, and predicted joint
  trajectories (from `MuJoCo/policies/Baseline_Active_Policy/`)
- **Hardware** — where we measure real electrical losses, real thermal behaviour, and real
  switching waveforms on an actual motor driven by a real inverter

### 1.2 Thesis Claims Supported

| Claim | Test(s) That Provide Evidence |
|-------|-------------------------------|
| 96V GaN achieves higher cycle-averaged efficiency than a 48V Si-equivalent baseline under gait loading | T5, T6 |
| GaN's lower switching losses enable higher $f_{sw}$ and smaller dead time, improving Z-width (transparent impedance) | T2, T3 |
| Higher bus voltage reduces $I_{RMS}$ and thermal stress for the same mechanical output | T1, T4 |
| Policy-driven load profiles produce different loss distributions than static grids | T1 vs T5/T6 |
| Passive (activation-modulated) policies reduce electrical CoT versus active baselines | T8 |
| GaN zero-reverse-recovery enables faster regen braking during dynamic motions | T7 |

### 1.3 Why the AKE80-8?

The CubeMars AKE80-8 is a commercially available QDD actuator with an 8:1 planetary gearbox.
Its specifications bracket the KBot leg actuators:

| Parameter | AKE80-8 | KBot Hip/Knee (RS-04) | KBot Ankle (RS-02) |
|-----------|---------|----------------------|-------------------|
| Gear ratio | 8:1 | 9:1 | 7.75:1 |
| Peak torque | 30 Nm | 120 Nm | 17 Nm |
| $K_t$ | 0.32 Nm/A | 2.1 Nm/A | 1.22 Nm/A |
| Rated voltage | 48 V | — | — |
| Pole pairs | 21 | — | — |

The AKE80-8 can serve as a **surrogate joint actuator** for dyno testing: its gear ratio and
QDD architecture are representative of humanoid leg joints, while its smaller torque rating
keeps the dyno power requirements manageable. All results are expressed in per-unit or
normalised form so they transfer to the full-scale RS-04/RS-02 drives.

> **96V operation note:** The AKE80-8 is rated at 48V nominal. At 96V, the motor sees 2×
> voltage headroom, which (a) doubles the theoretical no-load speed ceiling from 195 to ~390 rpm,
> (b) halves the phase current for the same shaft power ($P = V \cdot I$), reducing $I^2 R$
> winding losses by up to 4×, and (c) gives the FOC current loop more voltage margin for faster
> transient response. The motor winding insulation must be verified for 96V operation before
> testing.

---

## 2. AKE80-8 Motor Characterization

### 2.1 Datasheet Parameters

| Parameter | Symbol | Value | Unit | Source |
|-----------|--------|-------|------|--------|
| Torque constant | $K_t$ | 0.32 | Nm/A | Datasheet |
| Back-EMF constant | $K_e$ | 33 | V/krpm | Datasheet |
| Phase-to-phase resistance | $R_{pp}$ | 870 | mΩ | Datasheet |
| Phase resistance (star) | $R_{ph}$ | 435 | mΩ | $R_{pp}/2$ |
| Phase-to-phase inductance | $L_{pp}$ | 990 | µH | Datasheet |
| Phase inductance (star) | $L_{ph}$ | 495 | µH | $L_{pp}/2$ |
| Motor constant | $K_m$ | 0.34 | Nm/√W | Datasheet |
| Pole pairs | $p$ | 21 | — | Datasheet |
| Gear ratio | $N$ | 8 | :1 | Datasheet |
| Peak output torque | $\tau_{peak}$ | 30 | Nm | Datasheet |
| Rated output torque | $\tau_{rated}$ | 12 | Nm | Datasheet |
| Peak current | $I_{peak}$ | 12 | A | Datasheet |
| Rated current | $I_{rated}$ | 4.8 | A | Datasheet |
| No-load speed | $\omega_{NL}$ | 195 | rpm | Datasheet (at 48V) |
| Rated speed | $\omega_{rated}$ | 150 | rpm | Datasheet |
| Electrical time constant | $\tau_e$ | 1.13 | ms | Datasheet |
| Backlash | — | 9 | arcmin | Datasheet |
| Weight | — | 570 | g | Datasheet |

### 2.2 Pre-Test Measurements (To Be Filled In)

Before any dynamic testing, measure and record these on the bench with the motor disconnected
from the dyno:

| Parameter | Measured Value | Instrument | Date |
|-----------|---------------|-----------|------|
| $R_{pp}$ at 25°C | _______ mΩ | LCR meter / 4-wire DMM | |
| $L_{pp}$ at 1 kHz | _______ µH | LCR meter | |
| $K_e$ (spin at known RPM, measure open-circuit Vpp) | _______ V/krpm | Scope + tachometer | |
| Cogging torque peak | _______ Nm | Torque sensor, slow manual rotation | |
| Rotor inertia $J_m$ (optional) | _______ kg·m² | Spin-down deceleration test | |

### 2.3 Torque–Speed Envelope

The operating envelope shifts dramatically between 48V and 96V supply:

```
Torque (Nm)
  30 ┤ ████████                          Peak (transient < 1s)
     │ ████████
  12 ┤ ████████████████████               Rated (continuous)
     │ █████████████████████████
   0 ┤──────────────────────────────────── Speed (rpm)
     0    50   100   150   195  ~390
         ◄── 48V ──►  ◄─── 96V ───►
```

At 96V, the constant-torque region extends further because the inverter has more bus voltage
to overcome the back-EMF at high speed:

$$
\omega_{max}(V_{bus}) = \frac{V_{bus} - I \cdot R_{ph}}{K_e} \approx \frac{V_{bus}}{K_e}
$$

This is the primary electrical advantage being tested: for a given gait trajectory
$(q(t), \dot{q}(t), \tau(t))$, the 96V drive operates deeper inside its constant-torque
region, where the current loop has more voltage margin and the efficiency is highest.

---

## 3. Inertia Leg Design — Pendulum Arm Fixture

### 3.1 Concept

The test stand uses the simplest possible layout: the AKE80-8 motor is **clamped to the edge
of a sturdy table** with its output shaft pointing horizontally. A rigid bar (the "leg") is
bolted to the output shaft and hangs downward under gravity, free to swing in the vertical
plane.

```
    ┌─────────────────────────┐
    │        TABLE            │
    │                         │
    │   ┌──────────┐          │
    │   │ AKE80-8  │◄── Clamped to table edge
    │   │  Motor   │
    └───┤          ├──────────┘
        └────┬─────┘
             │ Output shaft (horizontal)
             │
        ┌────┴────┐
        │ Coupler │
        └────┬────┘
             │
             │  ◄── Steel / aluminium bar ("leg")
             │
             │       Length L, adjustable masses
             │
             ●  ◄── Clamp-on mass (m_tip)
             │
             ▼ gravity
```

This approach has three major advantages over a traditional flywheel-on-horizontal-shaft dyno:

1. **Free gravity loading** — the pendulum arm experiences real configuration-dependent
   gravitational torque $G(q) = -m g L_{COM} \sin(q)$, exactly like a real leg joint. No
   software feedforward or load motor is needed to emulate gravity.
2. **Physically intuitive** — it literally looks like a swinging leg segment, making the test
   setup easy to explain in the thesis and to visitors.
3. **Simple construction** — a bar, a coupler, and some clamp-on masses. No precision
   machining of flywheel discs required.

### 3.2 Inertia Targets

The KBot's sagittal-plane leg model (from
[`leg_params.py`](../../MuJoCo/kbot/leg_params.py)) defines the inertia the motor should
"see" at the joint side (after the gearbox). Three configurations:

#### Configuration A — Knee Joint Equivalent

From [`leg_params.py`](../../MuJoCo/kbot/leg_params.py) Model A (hip locked, ankle locked):

| Quantity | Symbol | Value | Source |
|----------|--------|-------|--------|
| Shank + foot inertia about knee | $I_{distal}$ | 0.1000 kg·m² | `leg_params.py` L89 |
| Reflected motor inertia at joint | $J_{reflected}$ | 0.04 kg·m² | $N^2 J_m$ (RS-04) |
| **Total joint-side inertia** | $I_{total}$ | **0.1400 kg·m²** | `leg_params.py` L95 |
| Distal mass | $m_{distal}$ | 2.294 kg | $m_2 + m_3$ |
| Distal COM distance from knee | $d_{COM}$ | 0.0966 m | `leg_params.py` L102 |
| Gravity torque constants | $a_g, b_g$ | 0.069, −0.371 kg·m | `leg_params.py` L107-108 |

#### Configuration B — Hip Pitch Equivalent

| Quantity | Symbol | Value | Derivation |
|----------|--------|-------|------------|
| Thigh inertia at COM | $I_1$ | 0.1272 kg·m² | `leg_params.py` L41 |
| Thigh mass | $m_1$ | 5.297 kg | `leg_params.py` L25 |
| Thigh COM distance | $\|r_1\|$ | 0.1944 m | `leg_params.py` L34 |
| Thigh parallel-axis at hip | $I_{1,hip}$ | $I_1 + m_1 \|r_1\|^2$ = 0.327 kg·m² | Parallel axis theorem |
| Shank+foot about hip (approx.) | $I_{2+3,hip}$ | ≈ 0.441 kg·m² | $I_{distal} + m_{distal} \cdot l_1^2$ |
| Reflected motor inertia | $J_{reflected}$ | 0.04 kg·m² | $N^2 J_m$ (RS-04) |
| **Total joint-side inertia** | $I_{hip,total}$ | **≈ 0.808 kg·m²** | Sum |
| Total distal mass | $m_{leg}$ | 7.590 kg | $m_1 + m_2 + m_3$ |

#### Configuration C — Ankle Joint Equivalent

| Quantity | Symbol | Value | Source |
|----------|--------|-------|--------|
| Foot inertia at COM | $I_3$ | 0.00189 kg·m² | `leg_params.py` L43 |
| Foot mass | $m_3$ | 0.609 kg | `leg_params.py` L27 |
| Foot COM distance | $\|r_3\|$ | 0.0353 m | `leg_params.py` L36 |
| Foot parallel-axis at ankle | $I_{3,ankle}$ | $I_3 + m_3 \|r_3\|^2$ = 0.00265 kg·m² | |
| Reflected motor inertia | $J_{reflected}$ | 0.0042 kg·m² | $N^2 J_m$ (RS-02) |
| **Total joint-side inertia** | $I_{ankle,total}$ | **≈ 0.0069 kg·m²** | Sum |

### 3.3 Pendulum Arm Sizing

The pendulum arm must match **two quantities simultaneously**: the rotational inertia
$I_{target}$ and the gravity torque magnitude $m g L_{COM}$ of the real leg segment.

For a simple arm consisting of a light rigid bar (mass negligible) with a concentrated tip
mass $m_{tip}$ at distance $L$ from the pivot:

$$
I_{arm} = m_{tip} \cdot L^2
$$
$$
G_{arm}(q) = -m_{tip} \cdot g \cdot L \cdot \sin(q)
$$

Matching to the KBot values requires:
$$
m_{tip} = \frac{I_{target}}{L^2}, \qquad L = \frac{I_{target}}{m_{distal} \cdot d_{COM}}
$$

where $m_{distal} \cdot d_{COM}$ is the first mass moment that sets the gravity torque
amplitude. If the bar itself has significant mass $m_{bar}$ distributed uniformly over
length $L_{bar}$, add $\frac{1}{3} m_{bar} L_{bar}^2$ to the inertia.

#### Pendulum Arm Sizing Table

| Config | $I_{target}$ (kg·m²) | Bar Length $L$ | Tip Mass $m_{tip}$ | Gravity Torque Peak | Notes |
|--------|---------------------|----------------|--------------------|--------------------|-------|
| **A — Knee** | 0.140 | 0.30 m | 1.56 kg | 4.58 Nm | Matches KBot shank length |
| **A — Knee** | 0.140 | 0.40 m | 0.88 kg | 3.44 Nm | Longer bar, lighter mass |
| **B — Hip** | 0.808 | 0.40 m | 5.05 kg | 19.8 Nm | Heavy — use thick steel bar |
| **B — Hip** | 0.808 | 0.50 m | 3.23 kg | 15.8 Nm | More manageable mass |
| **C — Ankle** | 0.0069 | 0.10 m | 0.69 kg | 0.68 Nm | Short stub + small mass |

> **Recommended starting configuration:** Config A with $L = 0.30$ m (matches the KBot shank
> length $l_2 = 0.292$ m) and $m_{tip} \approx 1.6$ kg. Use a 25 mm × 6 mm steel flat bar
> ($\approx 0.35$ kg for 300 mm length) with a 1.2 kg clamp-on mass at the tip. The bar's own
> inertia ($\frac{1}{3} \times 0.35 \times 0.30^2 = 0.0105$ kg·m²) is small relative to the
> tip contribution ($1.2 \times 0.30^2 = 0.108$ kg·m²), totalling $\approx 0.119$ kg·m².
> Adjust tip mass position along the bar to fine-tune to the exact 0.140 kg·m² target.

### 3.4 Gravity Loading — Built In

Unlike a horizontal-shaft flywheel dyno, the pendulum arm provides **real gravitational
loading** automatically. The gravity torque at joint angle $q$ (measured from the vertical
hang-down position) is:

$$
G_{pend}(q) = -m_{tip} \cdot g \cdot L \cdot \sin(q)
$$

For Config A ($m_{tip} = 1.56$ kg, $L = 0.30$ m):
- Peak gravity torque: $1.56 \times 9.81 \times 0.30 = 4.59$ Nm
- Compare to KBot knee gravity peak: $g \cdot \sqrt{a_g^2 + b_g^2} = 9.81 \times 0.377 = 3.70$ Nm

The pendulum slightly over-estimates the KBot knee gravity torque (4.59 vs 3.70 Nm) because
the KBot's COM is offset forward, not purely below the joint. This is an acceptable
approximation for drive-system testing — the motor sees realistic gravity-like loading, and
the exact magnitude difference is within 25%.

> **For the non-policy tests (T1–T4 in §5), gravity loading simply adds realism.** The motor
> swings the pendulum back and forth while we measure efficiency. For later policy-replay
> tests, the gravity torque naturally shows up in the measured shaft torque.

### 3.5 Fixture Bill of Materials

| Item | Specification | Qty | Purpose |
|------|--------------|-----|--------|
| Table clamp / L-bracket | Steel, ≥ M8 bolts, rated ≥ 50 Nm reaction | 1 | Secure motor to table edge |
| Shaft coupler | Rigid jaw coupler, Ø8 mm bore (match AKE80-8 shaft) | 1 | Connect motor to arm |
| Steel flat bar | 25 × 6 mm, 1045 steel, 300–500 mm length | 1 | Pendulum arm |
| Clamp-on masses | 0.5 kg and 1.0 kg, split-collar type | 2–3 | Adjustable inertia |
| Shaft collar / set screws | Match coupler bore | 2 | Retain masses on bar |
| End stop bolts (optional) | M6 shoulder bolts in clamp bracket | 2 | Limit swing angle to ±90° |

### 3.6 Range-of-Motion Considerations

The pendulum can swing freely through 360° if the table overhang is sufficient. For most
gait tests, the joint angle excursion is modest:

| Gait Mode | Typical Joint Excursion | Pendulum Swing |
|-----------|------------------------|----------------|
| Standing / static tests | ±5° | Negligible |
| Squat | 0–60° (knee) | Moderate |
| Walking | ±30° (hip pitch) | Moderate |
| Jump | 0–90° (knee) | Large |

Ensure the table overhang allows at least ±100° of unobstructed swing. Install mechanical
end stops (rubber bumpers on shoulder bolts) at ±100° to prevent the arm from wrapping around
and hitting the table or wiring in case of a control fault.

---

## 4. Policy-to-Hardware Pipeline

### 4.1 Overview

The pipeline converts a trained LocoMuJoCo policy into real-time motor commands:

```
┌──────────────────┐    ┌─────────────────┐    ┌──────────────────┐    ┌──────────┐
│  Trained PPO     │    │  MuJoCo Rollout  │    │  Single-Joint    │    │  MCU     │
│  Policy (.pkl)   │───►│  q(t), q̇(t),    │───►│  Trajectory      │───►│  Inverter│
│                  │    │  τ(t) @ 50 Hz    │    │  Extraction      │    │  FOC     │
└──────────────────┘    └─────────────────┘    └──────────────────┘    └──────────┘
                                                       │
                                                       ▼
                                                ┌──────────────┐
                                                │  CSV / Binary │
                                                │  trajectory   │
                                                │  file         │
                                                └──────────────┘
```

### 4.2 Step 1 — Policy Rollout in Simulation

Run each trained policy (from `MuJoCo/policies/Baseline_Active_Policy/`) in the full KBot
MuJoCo model and record the complete state trajectory:

```python
# Pseudocode — run in the LocoMuJoCo training environment
for policy_name in ["walk_07_12", "run_38_03", "jump_75_01"]:
    policy = load_policy(f"policies/Baseline_Active_Policy/{policy_name}/PPOJax_saved.pkl")
    env = make_kbot_env(model_path="kbot/robot_legs.mjcf")
    
    trajectory = []
    obs = env.reset()
    for step in range(N_steps):
        action = policy(obs)
        obs, reward, done, info = env.step(action)
        trajectory.append({
            "t": step * env.dt,
            "qpos": env.data.qpos.copy(),        # All joint positions
            "qvel": env.data.qvel.copy(),        # All joint velocities  
            "ctrl": env.data.ctrl.copy(),        # All actuator commands
            "qfrc_actuator": env.data.qfrc_actuator.copy()  # Actual joint torques
        })
    save_trajectory(trajectory, f"phase2/measurements/trajectories/{policy_name}.npz")
```

Alternatively, use the existing [`GaitReplay`](../../MuJoCo/controllers/gait_replay.py) class
with pre-recorded `.npz` clips to replay existing motion capture data through the WBC
controller and record the resulting joint torques.

### 4.3 Step 2 — Single-Joint Trajectory Extraction

Extract the trajectory for the specific joint being tested (e.g., left knee pitch):

```python
# Extract single-joint trajectory for dyno replay
joint_idx = 18  # dof_left_knee_04 nn_id from metadata.json

q_joint = trajectory["qpos"][:, joint_qpos_idx]     # Joint-side position [rad]
qdot_joint = trajectory["qvel"][:, joint_qvel_idx]   # Joint-side velocity [rad/s]
tau_joint = trajectory["qfrc_actuator"][:, joint_idx] # Joint-side torque [Nm]

# Convert to motor-side quantities for the AKE80-8
N = 8  # AKE80-8 gear ratio
q_motor = N * q_joint               # Motor position [rad]
qdot_motor = N * qdot_joint          # Motor velocity [rad/s]
tau_motor = tau_joint / N            # Motor torque [Nm]
I_motor = tau_motor / Kt_AKE80      # Motor current [A]  (Kt = 0.32 Nm/A)
```

### 4.4 Step 3 — Torque Scaling

The KBot RS-04 knee actuator ($K_t = 2.1$ Nm/A, $N = 9$, $\tau_{peak} = 120$ Nm) is
significantly larger than the AKE80-8. Direct replay of RS-04 torques would saturate the
AKE80-8 (30 Nm peak). We apply per-unit normalisation:

$$
\tau_{AKE80}(t) = \tau_{policy}(t) \cdot \frac{\tau_{peak,AKE80}}{\tau_{peak,RS\text{-}04}}
= \tau_{policy}(t) \cdot \frac{30}{120} = 0.25 \cdot \tau_{policy}(t)
$$

This preserves the **shape** of the torque waveform (which determines the loss profile) while
scaling the **amplitude** to fit within the AKE80-8's capability. All efficiency results are
then reported in per-unit form:

$$
\eta(\hat{\tau}, \hat{\omega}) \quad \text{where} \quad \hat{\tau} = \frac{\tau}{\tau_{rated}}, \quad \hat{\omega} = \frac{\omega}{\omega_{rated}}
$$

Similarly, scale velocity to respect the AKE80-8 speed limit:

$$
\omega_{AKE80}(t) = \omega_{policy}(t) \cdot \frac{\omega_{NL,AKE80}}{\omega_{NL,RS\text{-}04}}
$$

### 4.5 Step 4 — Gravity Compensation (Not Required)

Because the pendulum arm fixture (§3.4) provides **real gravitational loading**, there is no
need to inject a software gravity feedforward into the torque command. The motor naturally
experiences configuration-dependent gravity torque from the swinging arm.

The policy-derived torque command is sent directly:

$$
\tau_{cmd}(t) = \tau_{AKE80,scaled}(t)
$$

> **Note:** If the pendulum arm's gravity torque magnitude differs significantly from the
> KBot joint's gravity torque (see §3.4 comparison), a small correction term can optionally
> be added: $\Delta G = G_{KBot}(q) - G_{pend}(q)$, scaled appropriately. For this first
> test campaign, this correction is omitted — the 25% difference is acceptable for
> drive-system characterisation.

### 4.6 Step 5 — Real-Time Streaming to MCU

The final trajectory is streamed to the inverter MCU as a time-indexed table:

| Time (ms) | $q_{ref}$ (rad) | $\dot{q}_{ref}$ (rad/s) | $\tau_{ff}$ (Nm) | Mode |
|-----------|-----------------|-------------------------|-------------------|------|
| 0 | 0.000 | 0.000 | 0.120 | POS+FF |
| 20 | 0.015 | 0.750 | 0.135 | POS+FF |
| 40 | 0.058 | 1.420 | 0.189 | POS+FF |
| ... | ... | ... | ... | ... |

The MCU interpolates between table entries at its internal control rate (≥20 kHz current loop).
The control mode is PD position tracking with torque feedforward:

$$
\tau_{cmd} = K_p(q_{ref} - q) + K_d(\dot{q}_{ref} - \dot{q}) + \tau_{ff}
$$

This is identical to the controller in
[`kbot_leg_control_math.md`](../../docs/kbot_leg_control_math.md) §5, ensuring consistency
between simulation and hardware.

### 4.7 Which Policies to Replay

| Policy | Source | Joint Loading Character | Primary Tests |
|--------|--------|------------------------|---------------|
| `walk` | Active & Passive Policies | Periodic, moderate amplitude sinusoidal loading | T5, T8 |
| `run` | Active & Passive Policies | High frequency periodic loading | T8 |
| `jump` | Active & Passive Policies | Impulsive, high peak torque, regen braking | T7, T8 |
| `squat` | Active Policy | Slow, high-torque, quasi-static | T6 |
| `step_in_place` | Active Policy | Periodic with impact transients | T6 |

---

## 5. Test Matrix

All tests are run on the **96V GaN inverter board** driving the AKE80-8 motor with the pendulum
arm. To demonstrate the benefits of the GaN architecture without needing a separate Si board,
we define a **"Si-equivalent baseline"** (48V bus, 20–50 kHz $f_{sw}$, 500 ns dead time) and
compare it against the **"GaN-advanced"** configuration (96V bus, up to 100 kHz $f_{sw}$,
<100 ns dead time).

Tests are ordered so that **hardware characterisation comes first** (T1–T4), followed by
**policy-driven dynamic loading** (T5–T8).

**Test order:** T1 → T2 → T3 → T4 → T5 → T6 → T7 → T8.
Allow the motor to cool to ambient between tests.

### T1 — Static Efficiency Map & Voltage Comparison

**Purpose:** Establish the $(T, \omega)$ efficiency grid as a reference, and prove the
bus voltage advantage (48V vs 96V) under steady-state conditions.

| Parameter | Value |
|-----------|-------|
| Configurations | **48V** (Si-baseline $f_{sw}$/DT) vs **96V** (GaN $f_{sw}$/DT) |
| Load | Pendulum arm swinging at constant amplitude (set by position command) |
| Torques | 1, 3, 6, 9, 12 Nm (output shaft) |
| Speeds | 10, 30, 60, 100, 150 rpm (output shaft) |
| Hold time | 10 s per point (after thermal settling) |
| Logging | V_bus, I_bus, τ_shaft, ω_shaft, T_case, T_heatsink |

**Analysis:** Generate efficiency heatmap $\eta(T, \omega)$ for each configuration.
Compare $I_{RMS}$ between 48V and 96V (expecting ≈50% reduction at 96V). These maps
are the fundamental dataset — later policy tests will overlay their operating trajectories
onto these maps.

### T2 — Switching Frequency Sweep

**Purpose:** Quantify the efficiency vs. switching frequency trade-off, showing that
GaN can operate at high $f_{sw}$ without the severe efficiency penalty seen in Si.

| Parameter | Value |
|-----------|-------|
| Profile | Fixed sinusoidal oscillation (e.g. ±30° at 1 Hz on pendulum arm) |
| $f_{sw}$ values | 20, 50, 75, 100 kHz |
| Duration | 60 s per frequency |
| Controlled variables | Same bus voltage (96V), same motion profile, same dead time |

**Analysis:**
- Plot $\eta_{cycle}$ vs. $f_{sw}$ (expect a relatively flat curve for GaN)
- Current ripple $\Delta I_{pp}$ vs. $f_{sw}$
- EMI spectrum comparison at each $f_{sw}$ (if spectrum analyser available)

### T3 — Dead Time & Z-Width Characterisation

**Purpose:** Directly measure the control benefits of GaN's switching speed. Smaller
dead time reduces voltage distortion, which in turn improves the "Z-width" — the range
of mechanical impedances (from free-swinging transparency to stiff position holding) the
drive can render without instability.

| Parameter | Value |
|-----------|-------|
| Profile | Zero-torque command (transparency) & high-stiffness position hold |
| Dead times | 500 ns (Si-equivalent), 200 ns, 100 ns, 50 ns (GaN-optimised) |
| Controlled variables | $f_{sw}$ = 50 kHz, 96V bus |

**Analysis:**
- **Z-width:** Measure uncommanded drag torque (friction) during manual backdriving at zero torque.
- Voltage distortion and phase current THD at different dead times.
- Show that 50 ns dead time allows significantly higher virtual stiffness before limit cycles (chatter) occur.

### T4 — Thermal Soak

**Purpose:** Characterise steady-state thermal performance at worst-case operating point
(highest losses from T1 map). Determines the continuous rating of the drive under realistic
loading.

| Parameter | Value |
|-----------|-------|
| Configurations | **48V** (Si-baseline) vs **96V** (GaN-advanced) |
| Profile | Continuous oscillation at the worst-case (T,ω) from T1 |
| Duration | 20–30 min or until $dT/dt < 0.5$ °C/min |
| Logging | T_case, T_heatsink, T_winding, T_ambient @ 1 Hz; electrical @ 10 kHz |

**Analysis:**
- Thermal time constant $\tau_{th}$ from exponential fit
- Maximum continuous torque at thermal limit for each configuration
- Show that 96V operation allows significantly longer sustained peak torque due to lower $I^2 R$ heating.

### Phase A Conclusion — Parameter Optimization

Before proceeding to Phase B, the data from T1–T4 must be used to **select the optimal FOC parameters** for the GaN drive. These locked-in parameters will define the "GaN-advanced" configuration used for all policy replay tests:

1. **Optimal $f_{sw}$:** Chosen from T2 to balance switching losses against current ripple.
2. **Optimal Dead Time:** Chosen from T3 to maximize Z-width (transparency) without causing shoot-through.
3. **FOC Tuning:** Current loop PI gains ($K_p, K_i$) optimized for the selected $f_{sw}$ and 96V bus to ensure maximum tracking bandwidth.

---

### Phase B — Policy-Driven Dynamic Loading

> **Prerequisites:** T1–T4 complete. Measurement chain validated. Optimal GaN parameters ($f_{sw}$, dead time, PI gains) locked in based on Phase A results. Policy trajectory files exported (see §4). Pendulum arm inertia verified.

### T5 — Walking Policy Replay

**Purpose:** Measure cycle-averaged electrical efficiency under realistic periodic walking loading.

| Parameter | Value |
|-----------|-------|
| Policy | `walk` policy trajectory (active) |
| Duration | 60 s continuous (≈60 gait cycles at 1 Hz stepping) |
| Pendulum arm | Config A (knee) or Config B (hip) |
| Logging rate | ≥ 10 kHz synchronised |

**Analysis:**
- Cycle-averaged efficiency: $\eta_{cycle} = \frac{\int_0^{T_{cycle}} P_{out} \, dt}{\int_0^{T_{cycle}} P_{in} \, dt}$
- $I_{RMS}$ per gait cycle
- Thermal rise over 60 s
- Loss segregation: conduction ($I^2 R$) vs switching ($f_{sw} \cdot E_{sw}$)
- Overlay the trajectory's $(T, \omega)$ trace on the T1 efficiency map

### T6 — Squat and Step-in-Place Replay

**Purpose:** Characterise performance under two additional gait primitives:
- **Squat** — high-torque, low-speed quasi-static loading (worst case for conduction losses)
- **Step-in-place** — periodic loading with impact-like transients

| Parameter | Value |
|-----------|-------|
| Policy | `squat` and `step_in_place` policies |
| Profile | 0.3 Hz (squat) and 1 Hz (step) |
| Duration | 30 s squat + 60 s stepping |
| Focus metrics | Peak $\tau$, peak $I$, bus ripple (squat); CoT, transient quality (step) |

**Analysis:**
- Squat: GaN's lower $R_{DS(on)}$ advantage under high-current, low-speed stress
- Step: Electrical Cost of Transport $CoT_e$, current tracking at swing/stance transitions

### T7 — Jump Policy Replay

**Purpose:** Characterise peak power delivery and regenerative braking under highly dynamic
impulsive loading.

| Parameter | Value |
|-----------|-------|
| Profile | `jump` policy trajectory (active) |
| Duration | 2 s per jump × 5 repetitions, 30 s rest between |
| Focus metric | Peak current, peak power, regen energy, bus voltage transient |

**Analysis:**
- Peak instantaneous power $P_{peak} = \tau_{peak} \cdot \omega_{peak}$
- Regenerative energy captured: $E_{regen} = \int_{braking} P_{bus} \, dt$ (when $I_{bus} < 0$)
- GaN advantage: zero-reverse-recovery diode loss during regen commutation

### T8 — Active vs. Passive Policy Comparison

**Purpose:** Directly measure the electrical energy savings of the activation-modulated passive
policy (from the [Hybrid Constrained-Activation framework](../../MuJoCo/docs/passive_policy_research.md))
versus the standard active baseline.

| Parameter | Value |
|-----------|-------|
| Profiles | Active vs. Passive implementations of `walk`, `run`, and `jump` |
| Duration | 60 s each |
| Controlled variables | Same pendulum arm, same inverter, same motor, same speed |

**Analysis:**
- $\Delta I_{RMS} = I_{RMS,active} - I_{RMS,passive}$
- $\Delta P_{loss} = P_{loss,active} - P_{loss,passive}$
- $\Delta \eta = \eta_{passive} - \eta_{active}$
- Per-joint activation parameter $\alpha(t)$ correlation with measured current reduction

> **Note:** This test requires the passive policy to be fully trained. If not ready by the time
> the dyno is operational, run T8 later as a follow-up. Tests T1–T7 are all executable with
> existing hardware and active baseline policies.

---

## 6. Instrumentation & Measurement Setup

### 6.1 Equipment List

| Instrument | Purpose | Min. Specification |
|------------|---------|-------------------|
| DC power supply (programmable) | Bus voltage source | 0–100 V, ≥ 15 A, current-limited |
| Power analyser or precision shunt + DMM | DC bus power: $P_{in} = V_{bus} \cdot I_{bus}$ | ±0.1% accuracy, ≥ 100 kHz BW |
| Inline torque sensor | Shaft torque $\tau_{shaft}$ | ≥ 50 Nm range, ≤ 0.1% FS accuracy |
| Rotary encoder | Motor position $\theta$, velocity $\omega$ | ≥ 4096 CPR, index pulse |
| Current probes (×3) | Phase currents $i_a, i_b, i_c$ | Hall-effect or Rogowski, ≥ 500 kHz BW |
| Differential voltage probes (×2) | $V_{bus}$ and $V_{phase}$ | ≥ 100 V, ≥ 100 MHz BW |
| Thermocouples (×4) | $T_{GaN/FET}$, $T_{heatsink}$, $T_{winding}$ (if accessible), $T_{ambient}$ | Type K, ±1°C |
| Oscilloscope | Switching waveform capture ($V_{DS}$, $V_{GS}$, $i_{phase}$) | ≥ 200 MHz, ≥ 1 GS/s |
| DAQ system | Synchronised multi-channel logging | ≥ 10 kHz sample rate, ≥ 8 channels |
| LISN (optional) | Conducted EMI measurement | Per CISPR standards |
| Spectrum analyser (optional) | EMI frequency analysis | 150 kHz – 30 MHz |
| Thermal camera (optional) | Hotspot mapping | FLIR or equivalent |

### 6.2 Measurement Architecture

```
                    ┌─────────────┐
    DC Supply ──────┤ Power       │
    (48V or 96V)    │ Analyser    ├──── V_bus, I_bus, P_in  ──► DAQ Ch 1-3
                    └──────┬──────┘
                           │ DC Bus
                    ┌──────▼──────┐
                    │  Inverter   │
                    │  Board      │──── T_case (TC) ────────► DAQ Ch 7
                    │  (GaN/Si)   │
                    └──────┬──────┘
                      3-ph │ AC    ──── i_a, i_b, i_c ─────► DAQ Ch 4-6
                    ┌──────▼──────┐         (current probes)
                    │  AKE80-8    │
                    │  Motor      │──── T_winding (TC) ─────► DAQ Ch 8
                    └──────┬──────┘
                           │ Shaft
                    ┌──────▼──────┐
                    │  Torque     │──── τ_shaft ────────────► DAQ Ch 9
                    │  Sensor     │──── ω_shaft ────────────► DAQ Ch 10
                    └──────┬──────┘
                           │
                    ┌──────▼──────┐
                    │  Pendulum   │
                    │  Arm + Mass │
                    └──────│──────┘
                           │
                           ● tip mass
                           │
                           ▼ gravity
```

### 6.3 Wiring & Grounding Rules

1. **Star ground** — single ground reference point for all measurement equipment
2. **Identical DC bus wiring** for both inverter boards (same cable length, gauge, connectors)
3. **Shield all analogue signals** — twisted-pair or coax from sensors to DAQ
4. **Keep current probe loops small** — route phase wires through probes close to inverter output
5. **Oscilloscope probe ground** — use tip-and-barrel differential probing for $V_{DS}$; do NOT use scope ground clip at high-side

### 6.4 Safety Requirements

1. **96V hazard** — while below typical "dangerous voltage" thresholds, 96V can deliver
   lethal current through low-impedance paths. Use insulated connectors, shrouded terminals,
   and do not work on energised circuits.
2. **Emergency stop (E-stop)** — hardware E-stop that disconnects DC bus. Reachable from the
   operator position. Tested before every session.
3. **Over-current protection** — DC supply current limit set to 15 A. Inverter-side
   over-current trip at 15 A phase (both SW and HW).
4. **Swing zone clearance** — the pendulum arm sweeps a vertical arc. Mark a clear zone
   on the floor (radius = arm length + 15 cm margin). No hands, cables, or equipment in
   the swing zone during operation. Install mechanical end stops at ±100°.
5. **Thermal shutdown** — software thermal limit at $T_{case} = 100°C$. Manual abort if any
   component exceeds 80°C unexpectedly.
6. **Motor insulation check** — before applying 96V, verify motor winding insulation with a
   megohmmeter (hipot) at ≥ 250V DC, confirm ≥ 100 MΩ.

---

## 7. Test Procedures (Step-by-Step)

### 7.1 Common Pre-Test Checklist

Run this before **every** test session (adapted from KAN-56 §4.1):

- [ ] Visually inspect motor, clamp, coupler, pendulum arm, torque sensor, and all wiring
- [ ] Verify pendulum arm masses are secure (clamp screws tight, no play)
- [ ] Confirm swing zone is clear of cables, tools, and personnel
- [ ] Verify gate drive supplies (+5V for GaN, +12V for Si gate drivers) with no HV applied
- [ ] Verify encoder readings: rotate shaft by hand, confirm position and velocity are correct
- [ ] Verify thermocouple readings: all reading ambient ±2°C
- [ ] Set DC supply current limit (15 A)
- [ ] Power up at reduced bus voltage (20V), run no-load spin at low speed, verify:
  - Phase currents are balanced
  - No audible noise, vibration, or hot spots
  - Switching waveforms ($V_{DS}$, $V_{GS}$) are clean on scope
- [ ] Ramp to full bus voltage (48V or 96V), repeat checks
- [ ] Arm and test E-stop
- [ ] Start DAQ logging, verify all channels are acquiring
- [ ] Record ambient temperature and humidity in log
- [ ] Document any deviations in `RESEARCH_LOG.md`

### 7.2 Per-Test Procedure Template

For each test (T1–T8):

1. **Load the trajectory file** for this test onto the MCU (or select the correct profile)
2. **Configure test parameters** (bus voltage, switching frequency, pendulum arm config)
3. **Start DAQ recording** — filename format: `YYYY-MM-DD_T{N}_{board}_{fsw}kHz_{Vbus}V.csv`
4. **Start the trajectory replay** on the inverter
5. **Monitor** temperatures, currents, and bus voltage in real time during the test
6. **Wait** for the prescribed duration
7. **Stop trajectory replay** and allow motor to coast to a stop
8. **Stop DAQ recording**
9. **Allow motor to cool** to within 5°C of ambient before next test
10. **Transfer raw data** to `data_raw/YYYY-MM-DD_phase2_dyno_T{N}/`
11. **Log observations** in `RESEARCH_LOG.md`

---

## 8. Data Processing & Analysis

### 8.1 Primary Metrics

All analysis scripts should be placed in `phase2/measurements/analysis/` and produce
publication-ready figures.

#### Instantaneous Efficiency
$$
\eta(t) = \frac{P_{out}(t)}{P_{in}(t)} = \frac{\tau_{shaft}(t) \cdot \omega_{shaft}(t)}{V_{bus}(t) \cdot I_{bus}(t)}
$$

#### Cycle-Averaged Efficiency
$$
\eta_{cycle} = \frac{\int_0^{T_{cycle}} \tau \cdot \omega \, dt}{\int_0^{T_{cycle}} V_{bus} \cdot I_{bus} \, dt}
= \frac{E_{mech,cycle}}{E_{elec,cycle}}
$$

#### RMS Phase Current
$$
I_{RMS} = \sqrt{\frac{1}{T} \int_0^T i_a^2(t) \, dt}
$$

#### Electrical Cost of Transport
$$
CoT_e = \frac{E_{elec,cycle}}{m_{robot} \cdot g \cdot d_{stride}}
$$

where $m_{robot} = 21.07$ kg (legs-only mass from `leg_params.py`: $2 \times (m_1 + m_2 + m_3)
+ \text{torso}$), $g = 9.81$ m/s², and $d_{stride}$ is the displacement per gait cycle from
the policy. Since we are testing a single joint, report this as a **per-joint contribution** to
the total CoT.

#### Loss Segregation

$$
P_{total} = P_{in} - P_{out}
$$
$$
P_{Cu} = 3 \cdot I_{RMS}^2 \cdot R_{ph}(T_{winding}) \quad \text{(copper losses)}
$$
$$
P_{Fe} \approx P_{total} - P_{Cu} - P_{sw} \quad \text{(iron/core losses, by subtraction)}
$$
$$
P_{sw} \approx f_{sw} \cdot (E_{on} + E_{off}) \quad \text{(switching losses, from waveform analysis)}
$$

Note: $R_{ph}$ must be temperature-corrected using the measured winding temperature:
$$
R_{ph}(T) = R_{ph,25°C} \cdot \left(1 + \alpha_{Cu}(T - 25°C)\right), \quad \alpha_{Cu} = 0.00393 \, °C^{-1}
$$

### 8.2 Thermal Analysis

- **Thermal time constant** $\tau_{th}$: fit an exponential $T(t) = T_\infty - (T_\infty - T_0) e^{-t/\tau_{th}}$ to the soak test temperature data
- **Thermal resistance** $R_{th,j-a} = (T_{case} - T_{ambient}) / P_{loss}$ at steady state

### 8.3 Statistical Requirements

- Each test must be repeated **≥ 3 times** (minimum)
- Report **mean ± standard deviation** for all primary metrics
- Use the same ambient conditions (within ±3°C) for all repeats
- If any repeat deviates by > 2σ, investigate and either explain or discard (with justification)

### 8.4 Key Comparison Plots (for Thesis)

| Figure | X-axis | Y-axis | Curves |
|--------|--------|--------|--------|
| Efficiency map | Speed (rpm) | Torque (Nm) | Colour = η; one map per inverter |
| Walking efficiency time series | Time (s) | η(t), I(t), τ(t) | GaN vs Si overlay |
| Cycle efficiency bar chart | Test (T1–T8) | η_cycle (%) | GaN vs Si side-by-side |
| I_RMS comparison | Gait mode | I_RMS (A) | 48V vs 96V |
| Thermal rise | Time (s) | ΔT (°C) | GaN vs Si during T2 |
| Switching freq sweep | f_sw (kHz) | η_cycle (%) | GaN and Si |
| Loss breakdown | Component | P_loss (W) | Stacked bar: P_Cu, P_sw, P_Fe |
| Voltage comparison | Metric | Value | 48V vs 96V on same plot |

---

## 9. Expected Results & Thesis Integration

### 9.1 Predicted Outcomes

Based on the inverter specifications and motor parameters:

| Metric | Expected GaN Advantage | Reasoning |
|--------|----------------------|-----------|
| Cycle η (walking) | +2–4% absolute | Lower $R_{DS(on)}$ + zero reverse recovery |
| $I_{RMS}$ (96V vs 48V) | −45–50% | $I \propto P/V$; halving at doubled voltage |
| Thermal rise | −30–40% | $P_{Cu} \propto I^2$; 4× reduction at 96V |
| Max usable $f_{sw}$ | 100 kHz (GaN) vs 50 kHz (Si) | GaN zero-recovery allows aggressive timing |
| η sensitivity to $f_{sw}$ | Flat to 100 kHz (GaN); −1%/25 kHz (Si) | GaN switching losses scale weakly with frequency |
| Regen braking efficiency | +5–10% (GaN) | No diode recovery losses during braking commutation |

### 9.2 Thesis Chapter Mapping

| Test | Chapter Section | Figure/Table |
|------|----------------|--------------|
| T1 | Ch. 4: Motor & Inverter Characterization | Fig. 4.x: Efficiency map overlay |
| T2 | Ch. 4: Switching Frequency Study | Fig. 4.x: η vs f_sw |
| T3 | Ch. 4: Voltage Architecture | Fig. 4.x: 48V vs 96V comparison |
| T4 | Ch. 4: Thermal Performance | Fig. 4.x: Thermal soak curves |
| T5 | Ch. 5: Dynamic Performance | Fig. 5.x: Walking cycle efficiency |
| T6 | Ch. 5: Dynamic Performance | Fig. 5.x: Squat & stepping CoT |
| T7 | Ch. 5: Dynamic Performance | Fig. 5.x: Jump peak power & regen |
| T8 | Ch. 6: Policy-Hardware Co-design | Fig. 6.x: Active vs passive energy |

### 9.3 If Results Disagree with Predictions

If the measured GaN advantage is smaller than predicted:
1. Check thermal management — if GaN runs hotter than expected, $R_{DS(on)}$ rises and advantage erodes
2. Check layout parasitics — high loop inductance can cause ringing and increase switching losses
3. Check dead time — too-conservative dead time on GaN wastes the zero-recovery advantage
4. Revisit the loss model: are iron losses (not captured in the conduction/switching model) dominating?

Document all discrepancies in `RESEARCH_LOG.md` with hypotheses and follow-up actions.

---

## 10. File & Data Organisation

All files produced by this procedure should follow the project convention:

```
96v_gan_humanoid_drive/
├── phase2/
│   └── measurements/
│       ├── dyno_test_procedure_AKE80-8.md   ← This document
│       ├── trajectories/                     ← Extracted single-joint policy CSVs
│       │   ├── walk_07_12_knee.csv
│       │   ├── squat_knee.csv
│       │   ├── step_in_place_knee.csv
│       │   └── jump_75_01_knee.csv
│       └── analysis/                         ← MATLAB/Python analysis scripts
│           ├── plot_efficiency_map.py
│           ├── plot_cycle_efficiency.py
│           ├── loss_segregation.py
│           └── thermal_analysis.py
├── data_raw/
│   ├── 2026-XX-XX_phase2_dyno_T1/
│   ├── 2026-XX-XX_phase2_dyno_T2/
│   └── ...
└── notebooks/
    └── RESEARCH_LOG.md                       ← Log every test session here
```

---

## Appendix A — AKE80-8 Winding Insulation Verification for 96V

The AKE80-8 is rated for 48V. Operating at 96V requires:

1. **Phase-to-ground insulation test**: Apply 250V DC between any phase terminal and the
   motor housing using a megohmmeter. Confirm ≥ 100 MΩ insulation resistance.
2. **Phase-to-phase insulation test**: Apply 250V DC between any two phase terminals.
   Confirm ≥ 100 MΩ.
3. **Dielectric withstand (hipot)**: Apply 500V AC (or 700V DC) for 60 seconds between
   phases and ground. No breakdown.
4. **Thermal derating**: At 96V, if the motor is driven at higher speed or power than the
   48V rating, monitor winding temperature carefully. The insulation class (typically Class B
   or F for CubeMars motors) determines the maximum allowable temperature.

> **If the motor fails any insulation test, do NOT proceed with 96V testing.** Use 48V only,
> or source a motor with verified higher voltage insulation.

---

## Appendix B — Pendulum Arm Fixture Drawing Reference

A detailed technical drawing should be created in CAD (Fusion 360 or SolidWorks) for the
pendulum arm fixture assembly. Key dimensions to specify:

- Table clamp geometry and bolt pattern
- Shaft coupler bore diameter (match AKE80-8 output shaft, typically Ø8 mm)
- Steel flat bar cross-section, length, and material (1045 steel)
- Clamp-on mass dimensions and attachment method (set screw / split collar)
- Mass positions along bar for each inertia configuration (A, B, C)
- Mechanical end stop locations (±100° from vertical)
- Overall swing envelope (for safety zone marking)

Store drawing files in `phase2/mechanical/pendulum_arm/`.

---

*This is a living document. Update as hardware is built, tests are executed, and results come in.
Link updates to the corresponding Jira ticket and log changes in `RESEARCH_LOG.md`.*
