# KAN-56 Test Plan – 96V GaN vs 48V MOSFET

This document defines the experimental procedure for **KAN-56 – Define comparative test framework: metrics, test conditions, and measurement methodology for 96V GaN vs 48V MOSFET**. It is intended to be thesis-grade and *board-agnostic*, so it can be re-used for:

- 96V GaN dev board (KAN-54)
- 48V Si MOSFET dev board (KAN-55)
- Future custom GaN board

All boards under test (BUT) must use **the same motor, leg mechanism, and control structure** for fair comparison.

---

## 0. Validation Hierarchy: RL Load vs Motor-Coupled Bench

To reduce dyno time and accelerate progression to robot-level testing, testing is divided into three evidence tiers. Each result must be labelled with the rig used.

### Tier 1 — Inverter-only three-phase RL load

**Purpose:** isolate and characterize the inverter as a power-processing subsystem.

Use a calibrated three-phase air-core inductor/RL load whose inductance matches the motor winding under comparison. This rig is the primary platform for repetitive inverter characterization.

Valid Tier 1 outputs:

- Double-pulse switching characterization: \(E_{on}\), \(E_{off}\), switch-node overshoot, ringing, dv/dt, and estimated commutation-loop inductance.
- Inverter conduction and switching loss versus \(V_{dc}\), RMS phase current, switching frequency, and dead time.
- Phase-current ripple and THD versus \(V_{dc}\), load inductance, and \(f_{sw}\).
- Dead-time distortion, especially low-current zero-crossing behaviour.
- Gate-drive robustness, Miller-induced conduction, protection timing, and thermal maps at controlled electrical loading.
- Conducted-EMI screening at matched electrical operating points.

**Required calculations:**

- DC input power: \(P_{dc} = V_{dc} I_{dc}\).
- Three-phase RL load resistive loss: \(P_{load,R} = 3 I_{rms}^2 R_{load}(T)\), using load resistance measured at operating temperature.
- Inverter loss: \(P_{inv} = P_{dc} - P_{load,R}\), with phase-terminal electrical power used as an independent cross-check where available.
- Report **inverter efficiency/loss**, not drive efficiency.

**Do not claim from Tier 1 alone:**

- Mechanical/drive efficiency \(\eta_{drive} = P_{mech}/P_{dc}\).
- Voltage-limited torque–speed boundary.
- Regenerated energy or DC-bus rise under four-quadrant loading.
- Torque ripple, torque-step response, torque bandwidth, backdrive drag, or impedance/Z-width.

### Tier 2 — Motor-coupled bench / pendulum

**Purpose:** validate the inverter with the real QDD motor and obtain actuator-relevant but lower-complexity evidence.

Use the pendulum or equivalent mechanical load for:

- Motor parameter calibration and current-loop commissioning.
- Discrete efficiency points at known mechanical load.
- Low-speed trajectory replay.
- Thermal validation under representative current waveforms.
- Initial torque-step and impedance screening.

### Tier 3 — Dyno anchor and validation dataset

**Purpose:** provide the smaller, decisive dataset required for actuator-level thesis claims.

Reserve dyno time for:

- Full torque–speed and \(V_{dc}\times f_{sw}\) efficiency maps.
- Voltage-saturation boundary and back-EMF headroom measurements.
- Four-quadrant regeneration and DC-bus-rise tests.
- Torque-step response, current/torque bandwidth, and Z-width measurements.
- Final thermal validation at representative joint-duty operating points.
- Calibration points used to validate the RL-load inverter-loss model.

### RL-load-to-dyno calibration

At a minimum, repeat three matched operating points on both the RL load and dyno:

1. Light load / low current.
2. Mid load / nominal current.
3. High load / peak continuous current.

For each point, match \(V_{dc}\), RMS phase current, fundamental electrical frequency where practical, \(f_{sw}\), dead time, and thermal mounting condition. Compare inverter loss and device temperature. Use the residuals as a quantified correction/uncertainty term for Tier 1 results.

---

## 1. Objectives

1. Quantitatively compare **96V GaN** and **48V Si MOSFET** drive systems on the same legged platform.
2. Separate **inverter-level** performance from **actuator-level** performance using the Tier 1–3 validation hierarchy.
3. Benchmark across:
   - Inverter loss and efficiency on the RL load.
   - Drive efficiency \(\eta_{drive} = P_{mech}/P_{dc}\) on motor-coupled tests.
   - Thermal performance versus electrical and mechanical load.
   - Output impedance bandwidth (Z-width).
   - Current control bandwidth.
   - Switching loss indicators and waveform quality.
   - Current ripple and THD.
   - EMI, where available.
   - Power density.
4. Produce plots and tables that can be dropped directly into thesis chapters and papers.

---

## 2. Test Matrix and Operating Points

### 2.1 Electrical RL-load grid

Run this grid first on the three-phase RL load. It provides the majority of repeatable inverter characterization data.

- Bus voltage: \(V_{dc} = [24, 48, 72, 96]\) V, subject to board and load ratings.
- RMS phase current: selected to cover light, nominal, and peak continuous inverter current.
- Switching frequency: \(f_{sw} = [20, 40, 60, 80, 100, 120]\) kHz, refined after initial loss-minimum screening.
- Dead time: nominal value plus a bounded sweep around it.
- Load inductance: matched to the relevant motor winding; record actual inductance and temperature-dependent resistance.

For each condition, log DC power, phase currents, phase-terminal power where available, gate/switch-node waveforms where required, and device/heatsink temperatures.

### 2.2 Motor-coupled operating grid

Define a torque–speed grid derived from humanoid gait requirements, MuJoCo WBC trajectories, and Simscape models:

- Torques: e.g. \(T = [0.5, 1, 2, 3, 4]\) Nm, tuned to the leg capability.
- Speeds: e.g. \(\omega = [10, 50, 100, 200]\) rad/s equivalent joint speeds.

Run the full grid on the dyno where available. Use the pendulum for representative low-speed and thermal points when dyno access is constrained.

### 2.3 Test cases

For each board:

1. **RL-load inverter mapping:** loss, ripple, THD, dead-time sensitivity, thermal rise, and switching characterization.
2. **Dyno steady-state mapping:** hold torque and speed, then log electrical, mechanical, and thermal variables after settling.
3. **Dynamic tests:** torque/current steps and small-signal perturbations to measure current-loop bandwidth.
4. **Trajectory replay:** replay representative single-joint gait-derived current/torque trajectories on the pendulum or dyno.
5. **Thermal soak:** run the worst-case RL-load and motor-coupled points until temperatures converge.
6. **EMI sweep:** measure conducted emissions at a fixed operating condition, preferably repeated at selected switching frequencies.

---

## 3. Measurement Setup

### 3.1 Instrumentation

Minimum recommended instrumentation:

- **Power analyzer** or calibrated DC shunt/differential voltage sensing for \(V_{dc}\) and \(I_{dc}\).
- **PWM-capable three-phase power analyzer** for phase-terminal electrical power and loss separation.
- **Current probes** with sufficient bandwidth for phase current, ripple, and switching-event verification.
- **Voltage probes** for DC bus, phase voltages, and switch nodes.
- **Temperature measurement:** thermocouples on GaN/MOSFET case or nearby PCB hotspot, heatsink, load inductor, and motor winding where accessible; optional thermal camera.
- **DAQ / logging:** synchronized currents, voltages, temperatures, rotor position/velocity, and torque.
- **Torque sensor:** required for Tier 3 efficiency, bandwidth, torque ripple, and Z-width measurements.
- **EMI:** LISN and spectrum analyzer for conducted emissions; near-field probes if available.

### 3.2 Wiring and Safety

- Use identical DC bus wiring, DC-link capacitance, and filtering for both boards where possible.
- Use a limited-energy double-pulse fixture for initial high-voltage switching tests.
- Ensure proper isolation for probes at 96 V.
- Include emergency stop, over-current protection, over-voltage protection, and thermal protection.
- Treat 96 V DC testing under the applicable high-voltage safety procedure.

---

## 4. Test Procedures

### 4.1 Common Pre-test Checklist

For each board:

1. Visually inspect board, connectors, and heatsinks.
2. Verify gate-drive supplies, logic rails, and bootstrap/charge-pump outputs with no high voltage applied.
3. Verify dead time and switching waveforms at reduced bus voltage with no power load.
4. Confirm current-sensor offsets, voltage-sensor scaling, protection thresholds, and DAQ synchronization.
5. For RL-load tests, measure load inductance and resistance, including resistance at operating temperature.

Document deviations in `RESEARCH_LOG.md`.

### 4.2 RL-Load Inverter Loss Mapping

For each RL-load grid point:

1. Set \(V_{dc}\), \(f_{sw}\), dead time, and commanded current amplitude.
2. Allow currents and temperatures to settle.
3. Log a fixed window, typically 5–10 s or longer for thermal points.
4. Record \(V_{dc}\), \(I_{dc}\), phase currents, phase-terminal power, load temperature/resistance, and device/heatsink temperatures.
5. Compute:
   - \(P_{dc} = V_{dc}I_{dc}\).
   - \(P_{load,R} = 3I_{rms}^2R_{load}(T)\).
   - \(P_{inv} = P_{dc} - P_{load,R}\).
   - Inverter loss versus current, voltage, \(f_{sw}\), and dead time.
6. Repeat for all boards using identical scripts and operating points.

### 4.3 Switching Characterization

1. Use a limited-energy double-pulse fixture.
2. Step bus voltage through the planned ladder, for example 12, 24, 48, 72, and 96 V.
3. Step turn-off current through the relevant range, for example 20–55 A, subject to device and board limits.
4. Capture switch-node voltage, gate-source voltages, and phase/branch current.
5. Extract \(E_{on}\), \(E_{off}\), overshoot, ringing frequency, dv/dt, and estimated loop inductance.
6. Repeat identically for each board.

### 4.4 Motor-Coupled Steady-State Efficiency Mapping

For each \((T, \omega)\) point:

1. Set the controller to track the desired torque and speed.
2. Wait for temperatures and currents to settle.
3. Log for a fixed window:
   - \(V_{dc}\), \(I_{dc}\).
   - Phase currents and phase-terminal electrical power.
   - Mechanical torque and speed.
   - Device, heatsink, and winding temperatures.
4. Compute:
   - \(P_{in} = V_{dc}I_{dc}\).
   - \(P_{out} = T\omega\).
   - \(\eta_{drive} = P_{out}/P_{in}\).
   - Separated inverter loss using phase-terminal electrical measurements where available.
5. Repeat for all grid points and boards.

### 4.5 Thermal Performance

1. Select worst-case points from both the RL-load loss map and motor-coupled efficiency map.
2. Run until temperatures converge or \(dT/dt\) becomes negligible.
3. Log temperatures, electrical variables, mechanical load where applicable, and ambient temperature.
4. Estimate thermal resistance only when the power dissipated in the measured thermal path is known or bounded.
5. Compare boards at matched electrical loading for Tier 1 and matched mechanical operating points for Tier 2/3.

### 4.6 Current Control Bandwidth

1. Use a motor-coupled operating point; do not derive actuator current-loop bandwidth from the RL load alone.
2. Inject a small step or sinusoidal modulation into the \(I_d/I_q\) reference.
3. Log commanded and measured currents.
4. Compute gain/phase versus frequency or equivalent bandwidth from the step response.
5. Repeat with identical control code, sampling rate, and loop gains where possible.

### 4.7 Z-Width / Output Impedance

1. Operate the joint in virtual stiffness/damping control.
2. Apply controlled external perturbations using a known inertia, weights, actuator, or instrumented manual excitation.
3. Measure joint torque and displacement.
4. Derive impedance over frequency and identify the stable stiffness/damping region.
5. Compare boards only at matched control parameters and successful task execution.

### 4.8 Switching Loss and Waveform Quality

1. Use double-pulse results for device-level switching energy.
2. Use RL-load and motor-coupled tests for in-circuit waveform quality at representative operating points.
3. Capture phase voltage/current, gate-source voltage, and switch-node voltage.
4. Compare overshoot, ringing, dv/dt, current ripple, and THD.

### 4.9 EMI

1. Connect the LISN and spectrum analyzer to the DC supply.
2. Run a fixed RL-load operating point for repeatable inverter-level screening.
3. Sweep the frequency range of interest.
4. Repeat selected motor-coupled points if actuator-level EMI behaviour is required.
5. Compare boards at identical bus wiring, DC-link capacitance, load, and switching frequency.

---

## 5. Data Management and Post-Processing

- Store raw logs in `data_raw/YYYY-MM-DD_phaseX_<rig>_<desc>/`, where `<rig>` is `rl_load`, `pendulum`, or `dyno`.
- Store analysis scripts under `phase3/efficiency_maps/` and `phase3/sensitivity/`.
- Generate from scripts:
  - Inverter-loss heatmaps versus current, \(V_{dc}\), and \(f_{sw}\).
  - Drive-efficiency heatmaps versus torque and speed.
  - Temperature-versus-time plots.
  - Ripple and THD plots.
  - Current-loop Bode plots.
  - Z-width plots.
  - EMI spectra.
- Every plot must record the rig, board ID, motor/winding, bus voltage, switching frequency, dead time, and uncertainty method.

---

## 6. Pass/Fail and Comparison Criteria

Criteria are separated by evidence tier.

### Tier 1 — RL load

- Inverter loss at matched \(V_{dc}\), \(I_{rms}\), \(f_{sw}\), and dead time.
- Switching-energy and overshoot margins.
- Current ripple and THD.
- Thermal rise at matched electrical loading.
- Protection behaviour and waveform quality.

### Tier 2/3 — Motor-coupled bench and dyno

- Drive efficiency \(\eta_{drive}\) at matched torque–speed points.
- Voltage-saturation time and voltage-headroom boundary.
- Torque-step response and current/torque bandwidth.
- Regenerated energy and DC-bus rise.
- Stable impedance/Z-width region.
- Thermal rise at representative joint-duty operating points.

Example acceptance language:

- **Inverter loss:** GaN board achieves a measured reduction in \(P_{inv}\) over the relevant operating region.
- **Drive efficiency:** GaN board achieves a defined absolute or relative improvement at selected motor-coupled operating points.
- **Thermal:** lower device/heatsink temperature at matched loading, or higher allowable continuous current/torque before thermal limit.
- **Z-width:** larger stable impedance region without instability at matched control parameters.
- **EMI:** no unacceptable increase in conducted emissions; ideally comparable or lower with appropriate filtering.

---

## 7. Re-use for Custom GaN Board

When the custom GaN board is implemented, repeat the same Tier 1–3 procedures with identical scripts and operating points. Treat the following as a board-design-space comparison:

- 96V GaN dev board.
- 48V Si MOSFET dev board.
- Custom 96V GaN board.

The RL-load campaign should be completed first for each board, followed by motor-coupled pendulum/dyno validation. This preserves a clean A/B/C comparison while minimizing dyno occupancy and tying the final hardware contribution back to the original research questions.
