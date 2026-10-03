# 96V GaN Humanoid Drive: Offset Calculations & Hardware Verification

This directory contains the configured MathWorks Motor Control Blockset (MCB) models for **Open-Loop Hardware Verification & ADC Offset Calibration** and **eQEP Encoder Electrical Position Offset Calibration**, tailored specifically for the **TI LAUNCHXL-F28069M + EPC9147B/EPC91200 GaN Inverter + CubeMars AKE80-8 KV30** drive system.

---

## 1. Hardware Pinout & Mapping (BoosterPack Site 2: J5–J8)

| Signal | LaunchPad Pin / Header | MCU Peripheral | Hardware Function / Wire |
| :--- | :--- | :--- | :--- |
| **Phase U High** | J5-1 / GPIO-06 | `EPWM4A` | Phase A GaN High-Side Gate |
| **Phase U Low** | J5-2 / GPIO-07 | `EPWM4B` | Phase A GaN Low-Side Gate |
| **Phase V High** | J5-3 / GPIO-08 | `EPWM5A` | Phase B GaN High-Side Gate |
| **Phase V Low** | J5-4 / GPIO-09 | `EPWM5B` | Phase B GaN Low-Side Gate |
| **Phase W High** | J5-5 / GPIO-10 | `EPWM6A` | Phase C GaN High-Side Gate |
| **Phase W Low** | J5-6 / GPIO-11 | `EPWM6B` | Phase C GaN Low-Side Gate |
| **Inverter Gate Enable**| Header J5 / GPIO-52 | `GPIO-52` (DO) | Active HIGH Gate Driver Enable |
| **Phase U Current** | Header J7 / ADCINA3 | `ADC-A3` (SOC0) | Phase A Inverter Shunt Current |
| **Phase V Current** | Header J7 / ADCINB3 | `ADC-B3` (SOC1) | Phase B Inverter Shunt Current |
| **ADC SOC Trigger** | Internal | `ePWM4 ADCSOCA`| Synchronous ADC sample on PWM period |
| **QEP Channel A** | Header J4 / GPIO-20 | `eQEP1A` | AS5047P ABI Quad Channel A |
| **QEP Channel B** | Header J4 / GPIO-21 | `eQEP1B` | AS5047P ABI Quad Channel B |
| **QEP Index Pulse** | Header J4 / GPIO-23 | `eQEP1I` | AS5047P ABI Index Marker |
| **Host Serial SCI** | Virtual COM Port | `SCI-A` (GPIO-28/29) | COM7 Link to Host Simulink models |

---

## 2. Key Motor & Inverter Parameters

- **Motor**: CubeMars AKE80-8 KV30
  - Pole Pairs ($p$): **21** (42 magnetic poles)
  - Base Speed: **1440 RPM** (@ 48 V nominal)
  - Encoder ABI Slits: **1024 lines** ($1024 \times 4 = 4096$ counts/rev in 4x quadrature mode)
  - Phase Resistance ($R_s$): **0.435 $\Omega$**
  - Phase Inductance ($L_d = L_q$): **495 $\mu\text{H}$**
- **Inverter Timing & Protection**:
  - Switching Frequency ($f_{\text{sw}}$): **20 kHz** (`PWM_Counter_Period = 2250` @ 90 MHz system clock)
  - GaN Deadband: **5 clock cycles ($\approx 55.5\,\text{ns}$)** (`REDperiod = 5`, `FEDperiod = 5`, Active High Complementary)
  - Current Zero-Bias: **Calibrated on bench to 2058 counts ($I_a$, 1.658V) and 2057 counts ($I_b$, 1.657V)** (Nominal: 2048 counts / 1.65V)

---

## 3. System 1: Open-Loop Control & ADC Offset Calibration

### Files:
- **Target Model**: `OpenloopMotorControlF28069mLaunchPad.slx`
- **Host Model**: `OpenloopMotorControlHost.slx`
- **Initialization Callback**: `OpenloopMotorControlCallback.m`

### Purpose:
1. Verifies inverter gate pulses and phase wiring ($U \to V \to W$) using scalar (Volts/Hz) open-loop rotation without requiring position feedback.
2. Measures raw ADC resting offsets when gates are enabled and unpowered to calibrate zero-current sensor biases ($I_a, I_b$).

### How to Run:
1. In MATLAB, navigate to this folder and run:
   ```matlab
   setup_offset_calculations
   ```
2. Open the target model `OpenloopMotorControlF28069mLaunchPad.slx`.
3. In the **Hardware** tab, click **Build, Deploy & Start** to flash the code to the LAUNCHXL-F28069M LaunchPad.
4. Open the host model `OpenloopMotorControlHost.slx`.
5. Ensure the **Host Serial Setup** block is set to **COM7** (5.625 MBaud).
6. Click **Run** on the host model.
7. Use the toggle switch in the host model to start/stop open-loop rotation, ramp the commanded speed slider, and view real-time ADC count readings.

---

## 4. System 2: Quadrature Encoder (QEP) Offset Calibration

### Files:
- **Target Model**: `EncoderOffsetCalibrationF28069mLaunchPad.slx`
- **Host Model**: `OffsetCalibrationF28069mHost.slx`
- **Initialization Callback**: `OffsetCalibrationCallback.m`

### Purpose:
Calculates the angular offset between the rotor $d$-axis magnetic pole and the physical encoder index pulse ($Z$). This value is required for Field-Oriented Control (FOC) to achieve maximum torque per ampere.

### Two-Phase Calibration Routine:
1. **Rotor Alignment Phase ($t = 0 - 2.5\,\text{s}$)**:
   A DC voltage vector is injected along the Phase A axis ($\theta_e = 0^\circ$). The rotor physically aligns to electrical zero.
2. **Open-Loop Spin Phase ($t = 2.5 - 7.5\,\text{s}$)**:
   The controller commands an open-loop rotating voltage vector at `calibSpeed = 60 RPM` until the first encoder index pulse is detected. The eQEP counter captures the count latch (`QPOSILAT`), computes the offset in per-unit (PU), and transmits it to the host model over COM7 (115,200 baud).

### How to Run:
1. Ensure the motor is mechanically decoupled or free to rotate under no-load.
2. In MATLAB, open `EncoderOffsetCalibrationF28069mLaunchPad.slx`.
3. Click **Build, Deploy & Start** on the **Hardware** tab.
4. Open `OffsetCalibrationF28069mHost.slx`.
5. Ensure **Host Serial Setup** is set to **COM7** (115,200 baud).
6. Click **Run** on the host model.
7. The host model will show the motor spinning, confirm correct direction (green indicator), and display the final calibrated **Position Offset (PU)**.
8. Store this offset in your FOC parameter script (`motor.PositionOffset` or `pmsm.PositionOffset`).

---

## 5. System 3: AS5047P SPI Absolute + eQEP Hardware Synchronization

### Documentation:
- **Full Engineering Guide**: [`AS5047P_SPI_QEP_HARDWARE_SYNC_GUIDE.md`](AS5047P_SPI_QEP_HARDWARE_SYNC_GUIDE.md)

### Key Purpose:
Eliminates open-loop index calibration in closed-loop FOC. By sampling the AS5047P 14-bit absolute SPI angle at $t = 0$ and pre-seeding the hardware eQEP counter (`EQep1Regs.QPOSCNT = spi_raw >> 2`), the drive boots with instantaneous closed-loop position awareness, enabling full torque at $t = 0$ with zero motion.

### Validated Test Suite Results:
- **Stage 1 (Active Spin Calibration)**: Measured $\text{PositionOffset} = 0.025635\text{ PU}$ ($9.23^\circ$ mechanical, $193.8^\circ$ electrical).
- **Stage 2 (Hand-Spin Dynamic Tracking)**: Bidirectional tracking confirmed across 11 full revolutions over 15.8 seconds.
- **Stage 3 (Stationary Power Cycle)**: SPI booted at $0.7658\text{ PU}$ ($275.7^\circ$), QEP initialized at $0.7640\text{ PU}$ ($275.0^\circ$) — tracking error $< 0.87^\circ$ mechanical.
- **Stage 4 (Displaced Cold Boot)**: Displaced shaft by hand while unpowered. SPI booted at $0.9517\text{ PU}$ ($342.6^\circ$), QEP initialized at $0.9442\text{ PU}$ ($339.9^\circ$) — confirmed true absolute tracking.

