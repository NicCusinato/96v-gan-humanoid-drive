# 96V GaN Humanoid Joint Drive — Research Overview & Test Plan 
**Prepared by:** Nicholas Cusinato  
**Date:** 2026-07-14  

---

## Project Summary

Designing and validating a 96V GaN-based motor drive (EPC91200, EPC2305 eGaN FETs) for humanoid robot leg joints. Compared against a 48V Si-equivalent baseline, this project aims to prove:

- **Higher efficiency** — lower phase current at doubled voltage reduces copper losses by up to 4×
- **Lower thermal load** — cooler FETs and windings at the same shaft power output
- **Reduced total drive loss** — switching, conduction, and iron losses compared across voltage and frequency configurations
- **Improved backdrivability (Z-width)** — faster GaN switching enables smaller dead-times, reducing torque distortion and improving transparency
- **Effective regenerative braking** — zero reverse-recovery in GaN allows clean 4-quadrant operation; regen energy captured and quantified
- **Better gait tracking** — higher control bandwidth and lower friction enables more accurate trajectory following
- **More human-like motion** — passive policy (Duke + ECO hybrid) reduces unnecessary joint activation, lowering CoT and producing smoother, more natural gait


| Phase | Description | Motor | Hardware |
|-------|-------------|-------|----------|
| **1 — Simulation** | Virtual dyno baseline; FOC pre-tuning | Both modelled | MATLAB/Simulink |
| **2a — Rig Validation** | Validate test setup at lower power before committing custom motors | AKE80-8 (ankle surrogate) | EPC91200 |
| **2b — Drive Characterisation** | Full test matrix — actual thesis data | Custom AKE90-KV35 | EPC91200 (same board) |
| **3 — Leg Build & Validation** | Integrate drive into physical leg, validate under real gait | Custom AKE90-KV35 (knee + hip) + AKE80-8 (ankle) | EPC91200 |

---

## Phase 1 — Simulation

Simulink/Simscape virtual dynamometer used to generate theoretical baselines and pre-tune FOC gains before any hardware runs. All simulation plots act as predicted reference curves that real data will be plotted against.

| Test | Output |
|------|--------|
| Torque × speed efficiency sweep (48V and 96V) | Efficiency contour maps (η vs T, ω) for both voltage configs |
| Loss breakdown at each operating point | Stacked bar charts: P_Cu, P_sw (switching), P_Fe (iron) |
| Switching frequency sweep (20–100 kHz, 48V and 96V) | Total loss vs f_sw; motor loss vs f_sw; inverter switching loss vs f_sw (separate plots) |
| Voltage comparison at fixed operating point | η, I_RMS, loss split — 48V vs 96V side by side |
| FOC PI gain tuning | Pre-validated current loop gains for hardware |

---

## Phase 2 — Drive Characterisation Tests

Tests are run incrementally — **one variable changes at a time** — so each result directly attributes an improvement to a specific design choice. Phase 2a runs the AKE80-8 to validate the test rig; Phase 2b repeats with the AKE90-KV35 for thesis data.

### Test Hardware Options

| Path | Setup | What it enables |
|------|-------|-----------------|
| **Path A — Active Dyno** *(preferred)* | Motor coupled to a load motor with torque sensor | Full efficiency map, regen tests, any operating point |
| **Path B — Pendulum Arm** *(can start now)* | Motor drives a calibrated pendulum arm | Discrete efficiency points, thermal, controller comparison |

---

### T1 — Voltage Comparison: 48V vs 96V

Same motor, same GaN board, same controller (standard RL), same f_sw. Bus voltage is the only variable.

| Measured | Expected result |
|----------|----------------|
| η(T, ω) efficiency map at 48V and 96V | 96V map shows higher η across most of the grid |
| I_RMS at same torque output | ~50% lower at 96V |
| Copper loss P_Cu = 3·I²·R | ~75% lower at 96V |
| Switching loss P_sw | ~2× higher at 96V (same f_sw, higher voltage per switch event) |
| Peak FET and winding temperature | Lower at 96V for same shaft power |

**Plots:** Efficiency contour maps (one per voltage), I_RMS bar comparison, loss breakdown stacked bars (48V vs 96V), ΔΗ difference map.

---

### T2 — Switching Frequency Sweep

Fixed: 96V bus, standard RL controller. Variable: f_sw swept at 20 / 40 / 60 / 80 / 100 kHz. Run at fixed mid-load operating point (≈50% rated torque, ≈50% rated speed). **Also run at 48V** to show the comparison.

| Measured | 48V result | 96V result |
|----------|-----------|-----------|
| Total actuator loss vs f_sw | Increases with f_sw (Si-like behaviour) | Flatter curve — GaN switching losses scale less severely |
| Motor loss vs f_sw | Approximately flat (copper loss not frequency dependent) | Same |
| Inverter switching loss vs f_sw | Linear increase | Linear increase, but from a lower base at same duty |
| Phase current ripple ΔI_pp vs f_sw | Decreases as ~1/f_sw | Same |

**Plots:** Total loss vs f_sw (48V + 96V on same plot), motor loss vs f_sw (separate subplot), inverter switching loss vs f_sw (separate subplot), ΔI_pp vs f_sw. Decision: lock in optimal f_sw.

---

### T3 — Dead-Time Sweep

Fixed: 96V bus, optimal f_sw from T2, standard RL. Variable: dead time swept at 500 / 300 / 200 / 100 / 50 ns.

| Measured | Plot |
|----------|------|
| Phase current THD vs dead time | Line plot — THD decreases as DT reduces |
| Torque ripple vs dead time | Line plot |
| Backdrive drag torque at I_cmd = 0 (Z-width) | Line plot — drag drops, transparency improves |
| Minimum safe dead time (shoot-through boundary) | Annotated on plot |

**Decision:** Lock in optimal dead time.

---

### T4 — Regenerative Braking (Path A only)

Fixed: 96V bus, optimal f_sw, optimal DT. Variable: braking torque level.

| Measured | Plot |
|----------|------|
| Regen energy captured E_regen = ∫ V_bus · |I_bus| dt | Bar chart by braking level |
| V_DS switching waveform during regen commutation | Scope capture — shows zero Qrr vs Si body diode ringing |
| Bus voltage transient on step into regen | Scope trace |
| Regen efficiency at each braking level | Line plot |

---

### T5 — Controller Comparison

Fixed: 96V, optimal f_sw, optimal DT. Variable: control law. Each controller is tested in sequence. The two published papers (Duke, ECO) are reproduced as reference baselines; our modified design is compared against both.

| Controller | Architecture summary |
|-----------|---------------------|
| **Standard RL** | Fixed PD torque controller with soft energy penalty in reward. Known to collapse at high penalty weight. |
| **Duke Humanoid** *(arXiv 2409.19795)* | Policy outputs joint targets + per-joint activation α ∈ [0,1]. Torque gated as τ = α·PD(q\*, q). Passive reward 1/‖α‖ encourages joint relaxation during swing. |
| **ECO** *(Energy-Constrained Opt. with RL)* | Energy treated as hard inequality constraint via PPO-Lagrangian. C₁ = Σ\|τ·q̇\| constrained ≤ b₁. Lagrange multiplier self-tunes — prevents policy collapse. |
| **Modified (our design)** | Combines Duke's α gating with ECO's constrained training. Adds regen bonus tied to gait phases (heel strike, swing deceleration) to exploit high voltage GaN Quadrant IV operation. |

| Measured (same metrics for all 4 controllers) | Plot |
|----------------------------------------------|------|
| Cycle-averaged efficiency η | Bar chart — 4 controllers side by side |
| Electrical Cost of Transport (CoT) per stride | Bar chart |
| I_RMS | Bar chart |
| Peak FET temperature | Bar chart |
| Gait tracking RMS error | Bar chart |
| Step response (bandwidth) | Overlaid step response curves |

---

## Phase 3 — Leg Build & System Validation

The validated drive from Phase 2 is integrated into a physical leg. Phase 3 validates the full system under real gait loading and closes the loop between simulation predictions and hardware measurements.

**Motor assignment:**

| Joint | Motor | Notes |
|-------|-------|-------|
| Hip pitch | Custom AKE90-KV35 | Same frame + gear ratio as standard AKE90; re-wound for 96V |
| Knee | Custom AKE90-KV35 | Same |
| Ankle | AKE80-8? | Already validated in Phase 2a |

**Tests:**

| Test | Data collected |
|------|---------------|
| No-load joint sweep | Friction map, backlash, encoder calibration |
| Static torque hold | Structural integrity verification |
| Automated gait replay | Simulated trajectory replayed on hardware |
| Efficiency (sim vs real) | Measured η overlaid on Phase 1 simulation maps |
| Thermal soak — sustained gait | ΔT_FET and ΔT_winding vs time under repeated gait cycles |
| Controller comparison on leg | Repeat T5 metrics on real leg — CoT, η, tracking quality |

**Key publishable outputs:** Sim-vs-real efficiency overlay, system CoT per controller, thermal performance under sustained gait, gait tracking quality curves.

---

## Parts & Requirements

### Phase 2 — Path B (Pendulum — can start immediately)

| Item | Qty | Source |
|------|-----|--------|
| AKE80-8 motor | 1 | Available |
| EPC91200 inverter board | 1 | Available |
| Motor mount bracket | 1 | **3D print PAHT-CF** (brass inserts at bolt locations) |
| Table clamp / L-bracket (steel, ≥M8) | 1 | Buy / weld |
| Shaft coupler (bore matched to AKE80-8 shaft OD) | 1 | Buy |
| Steel flat bar pendulum arm (25×6 mm, 300–500 mm) | 1 | Buy |
| Split-collar masses (0.5 kg + 1.0 kg) | 2–3 | Buy |
| DC power supply (0–100V, ≥15A) | 1 | Lab |
| Oscilloscope (≥200 MHz) | 1 | Lab |
| Current probes ×3 (Hall, ≥500 kHz BW) | 3 | Lab / borrow |
| DAQ (≥10 kHz, ≥8 ch) | 1 | Lab |
| Thermocouples Type K ×4 | 4 | Buy |

### Phase 2 — Path A (Dyno — additional items)

| Item | Qty | Source |
|------|-----|--------|
| Load motor (BLDC/PMSM, ≥1.5 kW, regen-capable) | 1 | Buy / borrow / lab? |
| Load motor driver (regenerative) | 1 | Buy / borrow / lab? |
| Inline torque sensor (≥15 Nm, ≤0.1% FS) | 1 | Buy |
| Motor alignment plate | 1 | **CNC aluminium** |
| Dyno base plate (10–20 mm) | 1 | **CNC aluminium** |
| Flexible shaft coupling (≥20 Nm rated) | 1 | Buy |

### Phase 3 — Leg Build

| Item | Qty | Source |
|------|-----|--------|
| Custom AKE90-KV35 motors (knee + hip) | 2+ | Custom wound |
| EPC91200 inverter boards | 2–3 | Additional units |
| Leg structural links | 1 set | PAHT-CF (light load) or **CNC aluminium** (high load joints) |
| Joint bearings | 2–4 | Buy |
| Absolute encoders (≥12-bit) | 2–3 | Buy |
| 96V wiring harness | 1 | Make |
| Thermal management (heatsink + fan) | Per joint | Buy / design |

### 3D Print (PAHT-CF) vs CNC

| Part | Method |
|------|--------|
| Motor mount brackets, arm adapters, spacers | **3D print PAHT-CF** — use brass heat-set inserts; print bolt loads in XY plane |
| Leg links with low-to-moderate joint loads | **3D print PAHT-CF** — adequate strength in-plane (~100 MPa) |
| Motor alignment plates, dyno base plate | **CNC aluminium** — shaft alignment and stiffness critical |
| Leg links at high joint loads | **CNC aluminium** — PAHT-CF Z-axis strength (~30–50 MPa) insufficient for sustained torque reaction |

---

## Open Questions

| Question | Impact |
|----------|--------|
| Is a load motor available in the lab? | Determines if Path A or B runs first |
| CNC access — lab or outsource? | Affects timeline for Path A dyno setup |
| Phase 3 leg joint torque and ROM spec | Determines PAHT-CF vs aluminium for structural links |

---

*Full step-by-step procedures, instrumentation setup, safety requirements, and policy replay pipeline: [dyno_test_procedure_AKE80-8.md](dyno_test_procedure_AKE80-8.md)*
