# TI FOC in MATLAB/Simulink: Hardware & Parameter Porting Guide
**Target System:** LAUNCHXL-F28069M + EPC9147B Interface + EPC91200 GaN Inverter + AKE80-8 KV30 Motor

---

## 1. Overview & Architecture

When porting TI InstaSPIN / MotorWare code to MATLAB/Simulink (using **Motor Control Blockset (MCB)** or **Embedded Coder Support Package for C2000**), the default templates assume TI standard inverter boards (e.g., `BOOSTXL-DRV8301` or `BOOSTXL-DRV8305`).

Because you are using the **EPC91200 GaN Inverter** via the **EPC9147B Interface Board**, the analog front-end (AFE) sensing scaling and motor parameters must be updated in your MATLAB parameter scripts (e.g., `mcb_pmsm_foc_..._data.m` or Simulink Model Workspace).

---

## 2. Parameter Reference Table (Quick Copy-Paste)

Add or overwrite these variables in your MATLAB initialization script (e.g., `setup_data.m` / `mcb_pmsm_foc_f28069m_data.m`):

```matlab
%% ==========================================
%% Motor Parameters: AKE80-8 KV30
%% ==========================================
pmsm.p           = 21;                  % Number of pole pairs (42 poles)
pmsm.Rs          = 0.435;               % Stator phase resistance (Ohms, phase-to-neutral)
pmsm.Ld          = 0.000495;            % d-axis inductance (H) -> 495 uH
pmsm.Lq          = 0.000495;            % q-axis inductance (H) -> 495 uH
pmsm.Ke          = 19.24;               % Back-EMF constant (Vpk line-to-line / krpm electrical)
pmsm.FluxPM      = 0.002785;            % PM Flux linkage (Weber / V-s) [Derived from KV=30, p=21]
pmsm.N_max       = 900;                 % Rated mechanical speed (RPM) @ 30V DC
pmsm.I_rated     = 10.0;                % Rated phase current (A peak)
pmsm.I_max       = 2.0;                 % Test clamp current for 1A DC bench supply (A peak)
pmsm.N_rated     = 900;                 % Base speed (RPM)

%% ==========================================
%% Inverter Parameters: EPC91200 + EPC9147B
%% ==========================================
inverter.V_dc            = 30.0;        % Nominal DC bus voltage (V)
inverter.V_max           = 152.3;       % ADC Full-Scale Voltage (V) [Calibrated]
inverter.I_max           = 137.5;       % Maximum peak measurable phase current (+/- 137.5A)
inverter.I_trip          = 20.0;        % Overcurrent software protection trip threshold (A)

% Current Sensing Front-End (Allegro ACS37003)
inverter.ISenseVoltPerAmp = 0.012;      % Sensor Sensitivity: 12 mV/A (0.012 V/A)
inverter.ISenseVref      = 1.65;        % Zero-current reference voltage (V)
inverter.ISenseMax       = 275.0;       % Peak-to-Peak ADC Current Span (3.3V / 0.012 V/A)
inverter.ISenseOffset    = 0.50;        % ADC per-unit offset (1.65V / 3.3V = 0.50 pu -> 2048 counts)

% Voltage Sensing Front-End
inverter.VSenseMax       = 152.3;       % Full-scale voltage corresponding to 3.3V on ADC
inverter.VSenseOffset    = 0.0;         % DC offset (0V input = 0V on ADC)

%% ==========================================
%% Timing & PWM Settings
%% ==========================================
pwm.Frequency    = 30e3;                % PWM Switching Frequency: 30 kHz (GaN optimal: 30k-50k)
pwm.Period       = 90e6 / pwm.Frequency;% TBPRD in Up-Down count mode (for 90 MHz SYSCLK: 1500 counts)
pwm.Deadtime     = 0.05e-6;             % Deadtime (s) -> 50 ns (EPC GaN switches fast)
pwm.DeadtimeFED  = 5;                   % Deadtime in clock cycles (5 cycles @ 90MHz = 55.5 ns)
pwm.DeadtimeRED  = 5;

% Control Loop Sample Rates
Ts.CurrentLoop   = 1 / pwm.Frequency;   % Current controller execution rate (30 kHz = 33.3 us)
Ts.SpeedLoop     = 1e-3;                % Speed controller execution rate (1 kHz = 1 ms)
```

---

## 3. Detailed Hardware Mapping & Calculations

### A. Current Sense Calibration (ACS37003 on EPC91200)
- **Sensor Type:** Allegro `ACS37003LLUTR-050B3` Hall-effect IC.
- **Sensitivity:** $12\text{ mV/A} = 0.012\text{ V/A}$.
- **Zero-Current Bias Voltage:** $1.65\text{ V}$ ($V_{DD} / 2$).
- **ADC Full-Scale Range:** $0\text{ to }3.3\text{ V}$ (12-bit ADC $\rightarrow 0 \text{ to } 4095$ counts).
- **Full-Scale Current Span:**
  $$\text{Full-Scale Span} = \frac{3.3\text{ V}}{0.012\text{ V/A}} = 275.0\text{ A Peak-to-Peak}$$
- **Current Measurement Range:** $-137.5\text{ A}$ to $+137.5\text{ A}$.
- **ADC Offset / Bias:**
  $$\text{Offset pu} = \frac{1.65\text{ V}}{3.3\text{ V}} = 0.50\text{ pu} \quad (\text{Offset Counts} = 2048)$$

### B. DC Bus & Phase Voltage Sense Calibration
- **Measured Hardware Scaling:** $36.0\text{ V}_{\text{in}} \rightarrow 0.2364\text{ pu}$ ($968\text{ counts}$).
- **ADC Full-Scale Voltage:**
  $$V_{\text{full-scale}} = \frac{36.0\text{ V}}{0.2364\text{ pu}} = \mathbf{152.3\text{ V}}$$
- **ADC Offset:** $0.0\text{ pu}$ ($0\text{ V} = 0\text{ counts}$).

### C. Motor Flux Linkage Calculation (AKE80-8 KV30)
In MATLAB Motor Control Blockset, `pmsm.FluxPM` (Webers) is calculated from the motor's $KV$ and pole pairs ($p = 21$):
$$\omega_{m,\text{rated}} = 30 \frac{\text{RPM}}{\text{V}} \times \frac{2\pi}{60} = \pi \frac{\text{rad/s}}{\text{V}}$$
$$K_e \text{ (V}_{\text{peak, L-L}}/\text{rad/s}_\text{elec}\text{)} = \frac{\sqrt{3} \times 60}{2\pi \times KV \times p} = \frac{\sqrt{3} \times 60}{2\pi \times 30 \times 21} \approx 0.02626\text{ V}\cdot\text{s}$$
$$\text{FluxPM } (\Psi_m) = \frac{K_e}{\sqrt{3}} = \frac{60}{2\pi \times KV \times p} = \frac{60}{2\pi \times 30 \times 21} \approx \mathbf{0.002785\text{ Wb (V-s)}}$$

---

## 4. Simulink Block & Model Changes

### 1. In ADC Interface Subsystem (`ADC_Interface` / Hardware Driver Blocks)
In the C2000 ADC Conversion block or script:
- **Gain / Conversion Equation for Phase Currents:**
  $$I_{\text{phase}} (A) = \left( \frac{\text{ADC\_Counts}}{4096} - 0.50 \right) \times 275.0\text{ A}$$
  *(Or if using per-unit):*
  $$I_{\text{pu}} = \left( \frac{\text{ADC\_Counts} - 2048}{2048} \right)$$
- **Gain / Conversion Equation for DC Bus Voltage:**
  $$V_{\text{dc}} (V) = \left( \frac{\text{ADC\_Counts}}{4096} \right) \times 152.3\text{ V}$$

> [!WARNING]
> In TI standard examples for BOOSTXL-DRV8301, the default current offset is `0.50` or `0.83` and voltage scale is `66.3V` / `26.3V`. **You must replace these with 152.3V and 275.0A.**

---

### 2. In Current Control Subsystem (`Current_PI_Controllers`)
Because $L_d = 495\ \mu\text{H}$ is very small, use the standard Motor Control Blockset PI autotuner or calculate gains directly:
- **Desired Current Loop Bandwidth:** $\omega_{bw} = 2\pi \times 1000\text{ rad/s}$ ($1.0\text{ kHz}$)
- **Current Proportional Gain ($K_p$):**
  $$K_{p,i} = L_d \times \omega_{bw} = 0.000495 \times (2\pi \times 1000) \approx \mathbf{3.11}$$
- **Current Integral Gain ($K_i$):**
  $$K_{i,i} = R_s \times \omega_{bw} = 0.435 \times (2\pi \times 1000) \approx \mathbf{2733.2}$$

---

### 3. In Quadrature Encoder (QEP) Subsystem (When Encoder Arrives)
When using the sensored FOC template (`mcb_pmsm_foc_qep_f28069m.slx`):
- **eQEP Module:** Set to `eQEP1` on LaunchPad GPIO-20 (QEPA), GPIO-21 (QEPB), GPIO-23 (QEPI).
- **Encoder Resolution:**
  - If your encoder has $N$ pulses per revolution (PPR), the eQEP in 4x quadrature mode gives $4N$ counts per revolution.
  - Set `encoder.CountsPerRev = 4 * N;`
- **Zero-Angle Calibration:**
  - Run the open-loop alignment routine (d-axis alignment) to find the electrical offset angle `encoder.OffsetAngle` before enabling closed-loop speed control.

---

## 5. Bench Supply Protection Checklist

When running tests on a 1.0 A current-limited power supply:
1. **Speed Reference Ramp:** Limit acceleration ramp rate in Simulink to $\le 500\text{ RPM/s}$ ($0.5\text{ krpm/s}$).
2. **Current Saturation Limit:** Set the output saturation of the Speed PI controller (which commands $I_q^*$) to a maximum of **$2.0\text{ A}$** (to prevent winding saturation and power supply tripping).
3. **No ID / Startup Tuning:** Do not run sensorless open-loop drag algorithms at $<50\text{ RPM}$ with 21 pole pairs; use the encoder once connected.
