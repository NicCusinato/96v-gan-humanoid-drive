# KAN-56 Test Plan – 96V GaN vs 48V MOSFET

This document defines the experimental procedure for **KAN-56 – Define comparative test framework: metrics, test conditions, and measurement methodology for 96V GaN vs 48V MOSFET**. It is intended to be thesis-grade and *board-agnostic*, so it can be re-used for:

- 96V GaN dev board (KAN-54)
- 48V Si MOSFET dev board (KAN-55)
- Future custom GaN board

All boards under test (BUT) must use **the same motor, leg mechanism, and control structure** for fair comparison.

---

## 1. Objectives

1. Quantitatively compare **96V GaN** and **48V Si MOSFET** drive systems on the same legged platform.
2. Benchmark across:
   - Efficiency \(\eta = P_\text{out} / P_\text{in}\)
   - Thermal performance (junction and case temperatures vs load)
   - Output impedance bandwidth (Z-width)
   - Current control bandwidth (closed-loop Bode)
   - Switching loss indicators (waveforms, E\_on/E\_off estimates)
   - EMI (conducted, and radiated if available)
   - Power density (W/cm³ or W/kg of inverter)
3. Produce plots and tables that can be dropped directly into thesis chapters and papers.

---

## 2. Test Matrix and Operating Points

### 2.1 Operating Grid

Define a torque–speed grid derived from humanoid gait requirements (MuJoCo WBC trajectories and Simscape models):

- Torques: e.g. \(T = [0.5, 1, 2, 3, 4]\) Nm (tune based on leg capability)
- Speeds: e.g. \(\omega = [10, 50, 100, 200]\) rad/s equivalent joint speeds

For each \((T, \omega)\) pair, run steady-state tests on both boards.

### 2.2 Test Cases

For each board:

1. **Steady-state mapping:**
   - Hold torque and speed (using impedance / torque control) and log voltages, currents, temperatures once steady state is reached.
2. **Dynamic tests:**
   - Step load changes (e.g. 50% → 100% torque) to probe current control loop bandwidth.
   - Small-signal perturbations (current injection or commanded torque sinusoid) to measure Z-width.
3. **Thermal soak:**
   - Long-duration run at worst-case operating point until temperatures reach steady state.
4. **EMI sweep:**
   - Measure conducted emissions over the relevant frequency range under a standard operating condition.

---

## 3. Measurement Setup

### 3.1 Instrumentation

Minimum recommended instrumentation:

- **Power analyzer** (or calibrated shunt + differential voltage sensing) for DC bus and mechanical output power.
- **Current probes** (Hall-effect or Rogowski) for phase currents.
- **Voltage probes** for DC bus and phase voltages.
- **Temperature measurement:**
  - Thermocouples on MOSFET/GaN case, heatsink, and key PCB hotspots.
  - Optional thermal camera for hotspot mapping.
- **DAQ / logging:**
  - High-speed DAQ or MCU logging synchronized currents, voltages, temperatures, rotor position/velocity, and torque.
- **EMI:**
  - LISN + spectrum analyzer for conducted EMI; near-field probe if available for radiated noise around the board.

### 3.2 Wiring and Safety

- Use identical **DC bus wiring** and filtering for both boards where possible.
- Ensure **proper isolation** for probes at 96V.
- Include emergency stop and over-current protections.

---

## 4. Test Procedures

### 4.1 Common Pre-test Checklist

For each board (GaN and MOSFET):

1. Visually inspect board, connectors, and heatsinks.
2. Verify gate drive supplies, logic rails, and bootstrap/charge pump outputs with no high-voltage applied.
3. Verify dead time and switching waveforms at reduced bus voltage (e.g. 20–30 V) with no load.
4. Confirm encoder/sensor readings are valid and correctly scaled.

Document any deviations in RESEARCH_LOG.md.

### 4.2 Steady-State Efficiency Mapping

For each \((T, \omega)\) point and each board:

1. Set the controller to track the desired torque and speed.
2. Wait for temperatures and currents to settle.
3. Log for a fixed window (e.g. 5–10 s):
   - DC bus voltage \(V_\text{bus}\)
   - DC bus current \(I_\text{bus}\)
   - Phase currents \(i_a, i_b, i_c\)
   - Motor electrical angle and mechanical speed
   - Temperatures (devices and heatsink)
4. Compute:
   - Electrical input power \(P_\text{in} = V_\text{bus} I_\text{bus}\)
   - Mechanical output power \(P_\text{out} = T \omega\)
   - Efficiency \(\eta = P_\text{out} / P_\text{in}\).
5. Repeat for all grid points for both boards.

### 4.3 Thermal Performance

1. Choose 1–2 worst-case points from the efficiency map (highest losses).
2. Run each board at those conditions until temperatures converge (e.g. 20–30 min, or until dT/dt is small).
3. Log temperatures and electrical variables.
4. Compute an approximate junction-to-ambient thermal resistance based on estimated device power loss and temperature rise.

### 4.4 Current Control Bandwidth

1. Fix a nominal operating point (e.g. mid-speed, mid-torque).
2. Inject a small step or sinusoidal modulation into the current reference (Id/Iq) while logging actual currents.
3. Use MATLAB/Python to compute a Bode-like response of the current loop:
   - Gain and phase vs frequency (for sinusoidal sweep) or equivalent bandwidth from step response.
4. Repeat for both boards with identical control code (same sampling frequency, same loop gains where possible).

### 4.5 Z-width (Output Impedance) Measurement

1. Operate the leg in a **virtual joint stiffness / impedance control** mode.
2. Apply small external perturbations at the joint (either with a dedicated actuator, weights, or manual excitation with a force sensor).
3. Measure joint torque and displacement, and derive joint impedance over frequency.
4. Compare how “stiff but transparent” each drive feels—this is particularly relevant for interaction tasks.

### 4.6 Switching Loss and Waveform Quality

1. At a representative operating point, capture high-resolution waveforms of:
   - Phase voltage, phase current
   - Gate-source voltage
2. Estimate turn-on and turn-off energy per event using datasheet curves + measured currents and voltages.
3. Compare:
   - Overshoot, ringing, and dv/dt between GaN and MOSFET boards.

### 4.7 EMI

1. With LISN and spectrum analyzer connected to the DC supply, run a fixed operating point on each board.
2. Sweep the frequency range of interest and capture conducted emission spectra.
3. If possible, repeat with different switching frequencies and filter configurations.

---

## 5. Data Management and Post-Processing

- Store all raw logs in `data_raw/YYYY-MM-DD_phaseX_<desc>/`.
- Create a dedicated analysis script (MATLAB or Python) under `phase3/efficiency_maps/` and `phase3/sensitivity/` to:
  - Generate efficiency heatmaps vs torque/speed for each board.
  - Plot temperature vs time for thermal soaks.
  - Plot Bode curves for current loop and Z-width.
  - Compare EMI spectra.

All plots should be generated from scripts (no manual Excel plots) for reproducibility.

---

## 6. Pass/Fail and Comparison Criteria

Example criteria (to be refined as results come in):

- **Efficiency:** GaN board achieves ≥X% absolute or ≥Y% relative improvement over MOSFET at key operating points.
- **Thermals:** Lower device or heatsink temperature for the same operating point, or higher allowable continuous torque before thermal limit.
- **Z-width:** Higher impedance bandwidth without instability for the same control parameters.
- **Current loop:** Comparable or higher closed-loop bandwidth with sufficient phase margin.
- **EMI:** No unacceptable increase in conducted emissions; ideally comparable or lower with appropriate filtering.
- **Power density:** Higher W/cm³ or W/kg for GaN board.

These criteria will be translated directly into thesis figures and discussion.

---

## 7. Re-use for Custom GaN Board

When the custom GaN board is implemented, **re-run this entire test plan unchanged**, treating:

- 96V GaN dev board
- 48V MOSFET dev board
- Custom 96V GaN board

as three points in a design space. This preserves a clean A/B/C comparison and ties the final hardware contribution back to the original research questions.
