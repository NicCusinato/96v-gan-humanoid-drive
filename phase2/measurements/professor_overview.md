# 96V GaN Humanoid Joint Drive — Research Overview & Test Plan
**For:** [Professor Name]  
**Prepared by:** Nicholas Cusinato  
**Date:** 2026-07-14  
**Status:** Planning / In Progress

---

## Overview

This research designs, builds, and validates a **96V GaN-based motor drive system** for humanoid robot leg joints. The core hypothesis is that switching from a conventional 48V Si-MOSFET inverter to a 96V GaN inverter (using EPC2305 eGaN FETs on the EPC91200 evaluation board) produces measurable, publishable improvements in:

- **Efficiency** — lower phase current at higher voltage reduces copper losses
- **Thermal performance** — cooler operation at the same mechanical output
- **Control transparency (Z-width)** — GaN's faster switching enables smaller dead-times, reducing torque distortion
- **Regenerative braking** — zero reverse-recovery in GaN allows cleaner energy capture

The work is structured as three progressive phases:

| Phase | What | Motor | Board |
|-------|------|-------|-------|
| **1 — Simulation** | Theoretical baseline; FOC pre-tuning | Both motors modelled | MATLAB / Simulink |
| **2a — Rig validation** | Validate test setup and methodology at lower power | **AKE80-8** (ankle surrogate) | EPC91200 |
| **2b — Drive characterisation** | Full incremental upgrade tests — actual thesis data | **Custom AKE90-KV35** | EPC91200 (same board) |
| **3 — Leg build & validation** | Full system gait validation in physical leg | **Custom AKE90-KV35** (knee + hip) | EPC91200 (same board) |

> **Why this sequence?** Starting with the AKE80-8 lets us shake out the test rig, instrumentation chain, and methodology at lower risk before running the custom AKE90 motors through the full test matrix. The same EPC91200 board is used from Phase 2a through Phase 3 — demonstrating the board's versatility and ensuring a clean apples-to-apples comparison at every step.

---

## Phase 1 — Simulation (Complete / Ongoing)

A Simulink/Simscape virtual dynamometer sweeps the motor + inverter across a torque–speed grid. Outputs theoretical efficiency maps and loss breakdowns before any hardware is powered.

**Deliverables:** Efficiency maps (48V vs 96V), loss breakdown charts (switching, conduction, copper, iron), optimal switching frequency prediction, pre-tuned FOC PI gains.

---

## Phase 2 — Drive Characterisation Tests

### Test Structure — Incremental Upgrade Layers

Tests are structured so that **exactly one variable changes per layer**. This makes each result unambiguous and directly publishable.

```
Layer 0  Simulation baseline (already done)
Layer 1  48V → 96V voltage uplift          (same controller, same fsw)
Layer 2  Switching frequency optimisation  (96V, sweep fsw 20–100 kHz)
Layer 3  Dead-time optimisation            (96V, optimal fsw, sweep DT 500→50 ns)
Layer 4  Regenerative braking             (96V, optimal fsw+DT, 4-quadrant)
Layer 5  Controller comparison            (96V, optimal fsw+DT, 4 controllers)
```

Each layer produces the same set of outputs: **efficiency (η), loss breakdown (P_Cu / P_sw / P_Fe), thermal rise (ΔT_FET, ΔT_winding), phase current quality (I_RMS, THD).**

### Controller Architecture Comparison (Layer 5)

Four control policy architectures are tested in sequence. All hardware parameters (voltage, fsw, dead time) are locked in from Layers 1–3. Only the control law changes between runs. The four architectures span from a classical baseline to our novel hybrid design, with the two published papers serving as direct reference points.

| Controller | Architecture | What it adds |
|-----------|--------------|--------------|
| **Standard RL** | Classical RL with a fixed PD joint controller and an energy penalty term in the reward function: τ = k_p(q\*−q) + k_d(q̇\*−q̇), with energy penalised as a soft reward coefficient. | Baseline. Simple to implement but known to collapse when the energy penalty is large — the policy freezes or curls up rather than walking efficiently. Establishes the performance floor. |
| **Duke Humanoid** *(arXiv 2409.19795)* | The policy outputs both joint targets **and** a per-joint activation parameter α ∈ [0,1]. The applied torque is gated: **τ = α · (k_p(q\*−q) + k_d(q̇\*−q̇))**. A passive reward r = 1/‖α‖ encourages the robot to relax joints (e.g. knee during swing), exploiting natural pendulum dynamics. A curriculum decays a minimum activation floor α₀ from 0.5 → 0 during training to prevent premature collapse. | The robot can **disengage joints** during passive phases of the gait, harvesting free pendulum energy instead of fighting it. Measurable: lower average torque output and lower I_RMS during swing phases. |
| **ECO — Energy-Constrained Optimisation** *(Energy-Constrained Optimization with RL for Humanoid Walking)* | Treats energy as a **strict inequality constraint** using PPO-Lagrangian (Constrained RL) rather than a soft penalty. The energy cost C₁ = Σ\|τⱼ·q̇ⱼ\| is constrained: J_C1(π) ≤ b₁. A Lagrange multiplier λ is updated via dual gradient descent after every PPO update, automatically increasing or decreasing the energy pressure based on whether the constraint is violated. | Solves the policy-collapse problem: by separating energy into a hard constraint, the policy cannot trade away locomotion quality to farm low energy. The Lagrangian multiplier self-tunes, removing the need for manual reward weight tuning. Measurable: lower electrical CoT with stable, high-quality gait. |
| **Modified — Hybrid Constrained-Activation (our design)** | Combines both papers into a single framework. The actor outputs [a_q, a_α] as in Duke. The PPO-Lagrangian energy constraint from ECO replaces the soft energy penalty. An additional regen bonus is tied to specific gait phases (heel strike, swing deceleration) via a secondary lower-bound constraint E[C_regen] ≥ ε_regen, explicitly training the robot to yield to impacts and harvest regenerative energy through the GaN inverter's Quadrant IV operation. | Best of all worlds: joint relaxation (Duke) + stable constrained training (ECO) + regen-aware policy (ours). The primary novel contribution of the thesis — closes the loop between high-level policy and low-level GaN inverter hardware. |

Each controller is evaluated on: electrical CoT (energy per stride), cycle-averaged efficiency η, I_RMS, peak joint torque, thermal rise ΔT_FET, and gait tracking quality.


### Two Parallel Test Paths

**Path A — Full Active Dynamometer** *(preferred)*  
Motor coupled to a load motor. Provides full torque–speed efficiency maps and regenerative braking tests.

**Path B — Pendulum Arm** *(fallback if load motor unavailable)*  
Motor drives a calibrated pendulum arm that provides real gravitational loading. Gives efficiency at discrete operating points, thermal characterisation, and controller comparison. Still fully publishable.

> Both paths are being designed in parallel. Path B can proceed immediately with existing equipment.

### Data Outputs per Layer

| Output | Format |
|--------|--------|
| Efficiency contour map (Path A) | η(T, ω) heatmap — standard motor datasheet format |
| Efficiency at fixed points (Path B) | Bar chart / table, 3 operating points |
| Loss breakdown | Stacked bar: P_Cu, P_sw, P_Fe per configuration |
| Switching frequency sweep | η and ΔI_pp vs f_sw curves |
| Dead-time sweep | THD and backdrive drag vs dead-time |
| Regen waveforms (Path A) | Scope capture: V_DS commutation, I_bus sign reversal |
| Controller comparison | Step response, tracking RMS error, thermal rise, η |

---

## Phase 3 — Leg Build & Full System Validation

This is the culminating phase and the primary thesis deliverable. The validated drive system from Phase 2 is integrated into a **physical leg assembly** using the custom AKE90-KV35 motors for the knee and hip joints. Phase 3 closes the loop between simulation (Phase 1) and component-level hardware tests (Phase 2) by showing the full system working as a unit under real gait loading.

**Motor assignment in the leg:**

| Joint | Motor | Notes |
|-------|-------|-------|
| Hip pitch | Custom AKE90-KV35 | Same frame and gear ratio as standard AKE90; re-wound for 96V (more turns, lower current per turn for the same torque output) |
| Knee | Custom AKE90-KV35 | Same as hip — 96V winding, same mechanical interface and gear ratio |
| Ankle | AKE80-8 | Standard unit; already fully validated in Phase 2a |

### Leg Design Objectives

- Mechanical leg structure housing the AKE90-KV35 motors and EPC91200 inverters
- Joint range of motion and inertia matched to humanoid gait requirements
- Full electronics integration: motor drives, encoders, power management
- Thermal management: motor and FET cooling under sustained gait cycles

### Phase 3 Test Plan

| Test | What it validates |
|------|------------------|
| No-load joint sweep | Friction, backlash, encoder calibration |
| Static torque hold | Drive system structural integrity under load |
| Slow gait cycle (manual) | Electronics integration, sensor wiring, control loop |
| Automated gait replay | Policy trajectory from simulation replayed on hardware |
| Efficiency comparison (sim vs real) | Validates Phase 1 simulation model against measured data |
| Sustained gait — thermal soak | Continuous rating of the full integrated system |
| Controller comparison on leg | Repeat Layer 5 controller tests on actual leg — real-world result |

### Phase 3 — Key Publishable Results

- **Measured vs. simulated efficiency** — does the virtual dyno model predict real hardware accurately?
- **System-level CoT (Cost of Transport)** — electrical energy per stride, comparison between controllers
- **Thermal performance under gait** — does the 96V drive stay within safe thermal limits during a full gait cycle?
- **Gait tracking quality** — does the leg track a human-like trajectory with sufficient bandwidth using each controller?

---

## Parts & Requirements Summary

### Phase 2 — Path B (Pendulum, can start now)

| Item | Qty | Source |
|------|-----|--------|
| **AKE80-8 motor** | 1 | Already available |
| EPC91200 inverter board | 1 | Already available |
| Motor mount bracket (PAHT-CF 3D print) | 1 | Print in-lab |
| Table clamp / L-bracket (steel, ≥M8) | 1 | Buy / fabricate |
| Shaft coupler (rigid, bore matched to AKE80-8 shaft OD) | 1 | Buy |
| Steel flat bar pendulum arm (25×6 mm, 300–500 mm) | 1 | Buy (stock) |
| Split-collar clamp masses (0.5 kg, 1.0 kg) | 2–3 | Buy |
| DC power supply (0–100V, ≥15A) | 1 | Lab equipment |
| Oscilloscope (≥200 MHz) | 1 | Lab equipment |
| Current probes ×3 (Hall-effect, ≥500 kHz BW) | 3 | Lab / borrow |
| DAQ system (≥10 kHz, ≥8 ch) | 1 | Lab equipment |
| Thermocouples Type K ×4 | 4 | Buy |

### Phase 2 — Path A (Dyno, additional items)

| Item | Qty | Source |
|------|-----|--------|
| Load motor (BLDC/PMSM, ≥1.5 kW, regen-capable) | 1 | Buy / borrow |
| Load motor driver (regenerative) | 1 | Buy / borrow |
| Inline torque sensor (≥15 Nm, ≤0.1% FS) | 1 | Buy |
| Dyno base plate — CNC aluminium (10–20 mm) | 1 | CNC required |
| Motor alignment plate — CNC aluminium | 1 | CNC required |
| Flexible shaft coupling (Lovejoy, ≥20 Nm) | 1 | Buy |

### Phase 3 — Leg Build

| Item | Qty | Source |
|------|-----|--------|
| **Custom AKE90-KV35 motors (knee + hip)** | 2+ | Custom wound — same frame/gear as standard AKE90, 96V winding |
| **AKE80-8 (ankle, carried over from Phase 2)** | 1 | Already available |
| EPC91200 inverter boards | 2–3 | Already available / additional units |
| Leg structural frame / links | 1 set | CNC aluminium or PAHT-CF (see below) |
| Joint bearings | 2–4 | Buy |
| Absolute encoder (≥12-bit) | 2–3 | Buy |
| 96V-rated power wiring harness | 1 | Make |
| Thermal management (heatsink + fan or pad) | per joint | Buy / design |

### 3D Print vs. CNC Guidance

| Component | Method | Reason |
|-----------|--------|--------|
| Motor mount / bracket | 3D print PAHT-CF (with brass heat-set inserts) | Low sustained load, complex geometry, heat resistant (HDT 170–185°C) |
| Pendulum arm adapter | 3D print PAHT-CF | Low stress, easy to iterate |
| Leg links (if lightly loaded) | 3D print PAHT-CF | Rapid iteration, adequate strength in-plane (~100 MPa) |
| Dyno base plate | **CNC aluminium** | Precision shaft alignment required; deflection affects torque measurement |
| Motor alignment plate | **CNC aluminium** | Same — rigid precision required |
| Leg links (if high joint loads) | **CNC aluminium** | When sustained torque reaction exceeds PAHT-CF Z-axis capability (~30–50 MPa) |

---

## Open Questions for Discussion

1. **Load motor availability** — Is a load motor available from the lab, or does one need to be sourced? This determines whether Path A or Path B runs first.
2. **CNC access** — The dyno base plate and motor alignment plate require CNC. Is lab CNC available, or should these be outsourced?
3. **Phase 3 leg spec** — What are the joint torque and range-of-motion requirements? This determines whether PAHT-CF structural links are sufficient or CNC aluminium is needed.

---

*Full detailed step-by-step test procedures, instrumentation setup, data analysis methods, safety requirements, and the policy replay pipeline are in the [full procedure document](dyno_test_procedure_AKE80-8.md).*
