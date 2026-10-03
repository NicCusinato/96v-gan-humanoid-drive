# Hardware Mapping & Project Understanding Guide
## Motor Control Blockset™ (MCB) Algorithm Export Workflow for Custom Hardware

---

### Executive Summary & System Philosophy

The **Algorithm Export Workflow** represents MathWorks' recommended Model-Based Design (MBD) architecture for deploying Field-Oriented Control (FOC) onto **any custom motor control hardware** (e.g., custom GaN inverter boards, TI C2000 microcontrollers, STM32 MCUs, NXP, Infineon, or FPGA/SoC platforms). 

Instead of relying on hardware-specific Simulink Target Support Packages (which bundle peripheral drivers directly into the Simulink model), this workflow enforces a clean **architectural decoupling**:
1. **Control Algorithm Domain (Simulink & Embedded Coder®):** Contains pure, target-independent ANSI/ISO C code implementing motor control mathematics (Clarke/Park transformations, PI current/speed loops, space-vector modulation, quadrature decoding, and per-unit normalization).
2. **Hardware Abstraction Domain (User/Vendor BSP):** Developed either in MCU-native IDEs (e.g., STM32CubeIDE / STM32CubeMX, TI Code Composer Studio / Theia, Keil $\mu$Vision) or written manually. It configures microcontroller clock trees, PWM complementary timers, ADC conversions, encoder interfaces, DMA channels, and communication transceivers.
3. **Integration Interface Layer:** A deterministic, low-overhead C-function boundary where hardware ISRs call generated algorithm step functions passing raw hardware feedback and retrieving normalized duty cycle commands.

```
+-----------------------------------------------------------------------------------------+
|                                    HOST COMPUTER                                        |
|   +---------------------------------------------------------------------------------+   |
|   | Host Simulink Model (open_loop_host.slx / qep_calibration_host.slx /            |   |
|   |                      foc_qep_host.slx)                                          |   |
|   | - Real-time command sliders (Start/Stop, Closed Loop, Speed RPM)                |   |
|   | - High-speed serial monitoring scopes (Phase Currents, Speed, Position, Offset) |   |
|   +---------------------------------------+-----------------------------------------+   |
+-------------------------------------------|---------------------------------------------+
                                     UART / USB Serial
                                (4 MBaud, Packet Framing)
+-------------------------------------------|---------------------------------------------+
|                                    TARGET HARDWARE                                      |
|                                                                                         |
|  +----------------------------------------v------------------------------------------+  |
|  | HARDWARE DRIVER LAYER (Board Support Package / main.c / HAL / C2000 Drivers)      |  |
|  |                                                                                   |  |
|  |  +---------------------+   +---------------------+   +-------------------------+  |  |
|  |  | PWM Generation      |   | Injected ADC        |   | QEP Quadrature & Index  |  |  |
|  |  | Center-Aligned      |   | Shunt Current Sense |   | Timer Encoder Interface |  |  |
|  |  | (TIM1 / ePWM1-3)    |   | (ADC1 / AdcA-B)     |   | (TIM2 / eQEP1)          |  |  |
|  |  +----------^----------+   +----------+----------+   +------------+------------+  |  |
|  |             |                         |                           |               |  |
|  |             | (Compare Registers)     | (Ia, Ib Raw Counts)       | (Counts & Z)  |  |
|  |             |                         v                           v               |  |
|  |  +----------+-----------------------------------------------------+------------+  |  |
|  |  | High-Frequency Current Loop ISR (20 kHz / 50 us Synchronous with PWM Valley)|  |  |
|  |  +------------------------------------+----------------------------------------+  |  |
|  |                                       |                                           |  |
|  |  +------------------------------------+----------------------------------------+  |  |
|  |  | Medium-Frequency Speed Loop ISR / Timer (1 kHz - 2 kHz / 500 us - 1 ms)      |  |  |
|  |  +------------------------------------+----------------------------------------+  |  |
|  +---------------------------------------|-------------------------------------------+  |
|                                          | Function Calls / Per-Unit Scaling            |
|  +---------------------------------------v-------------------------------------------+  |
|  | EXPORTED ALGORITHM LAYER (Simulink Embedded Coder ANSI C Artifacts)               |  |
|  | - Current Control Step: Clarke / Park, D-Q Decoupling, PI Regulators, SVPWM      |  |
|  | - Speed Control Step: Speed Error PI, Anti-Windup, Acceleration Ramp             |  |
|  | - Calibration Step: Electrical Angle Alignment, Index Latch, Sensor Bias Nulling  |  |
|  +-----------------------------------------------------------------------------------+  |
+-----------------------------------------------------------------------------------------+
```

---

### Folder & Architecture Directory Breakdown

The folder `AlgorithmExportWorkflowsForCustomHardware_Example_20260930_133212` contains the complete MATLAB Project and supporting artifacts:

```
AlgorithmExportWorkflowsForCustomHardware_Example_20260930_133212/
├── AlgorithmExportWorkflowsForCustomHardwareExample.m  # Example launcher script
├── mcb_FOCAlgorithmExportDemoStart.m                  # Project startup & path loader
├── mcb-custom-hw-foc-host-model.png                   # Architectural host model preview
└── FOCAlgorithmExportDemo/                            # MATLAB Project root
    ├── FOCAlgorithmExportDemo.prj                     # MATLAB Project configuration
    ├── FOC_QEP.ioc                                    # STM32CubeMX hardware peripheral config
    ├── open_loop/                                     # WORKFLOW STEP 1: Scalar V/f & ADC Offsets
    │   ├── open_loop_algorithm.slx                    # Simulink model for code generation
    │   ├── open_loop_data.m                           # Target, inverter & motor initialization
    │   ├── open_loop_host.slx                         # Host PC UI for open-loop validation
    │   └── STM32Code/
    │       └── main.c                                 # Driver integration reference for Step 1
    ├── qep_calibration/                               # WORKFLOW STEP 2: Quadrature Offset Calibration
    │   ├── qep_calibration_algorithm.slx              # Simulink model for encoder alignment
    │   ├── qep_calibration_data.m                     # QEP calibration dataset
    │   ├── qep_calibration_host.slx                   # Host PC UI for encoder index capture
    │   └── STM32Code/
    │       └── main.c                                 # Driver integration reference for Step 2
    ├── foc_qep/                                       # WORKFLOW STEP 3: Closed-Loop FOC Execution
    │   ├── current_control_algorithm.slx              # Inner loop algorithm (fast task, 20 kHz)
    │   ├── speed_control_algorithm.slx                # Outer loop algorithm (slow task, 1-2 kHz)
    │   ├── foc_qep_data.m                             # Parameters, PU bases, tuned PI gains
    │   ├── foc_qep_host.slx                           # Host PC control & monitoring dashboard
    │   ├── foc_qep_sim.slx                            # Closed-loop desktop simulation model
    │   └── STM32Code/
    │       └── main.c                                 # Dual-rate driver integration for Step 3
    └── work/
        ├── cache/                                     # Simulink build cache & SLXC files
        └── code/                                      # Embedded Coder output (.c and .h files)
```

---

### Hardware Mapping Matrix (Reference STM32F302R8 + X-NUCLEO-IHM07M1)

The peripheral setup defined in [FOC_QEP.ioc](file:///c:/96v_gan_humanoid_drive/phase1/simulation/MyFOC%20code/AlgorithmExportWorkflowsForCustomHardware_Example_20260930_133212/FOCAlgorithmExportDemo/FOC_QEP.ioc) establishes the reference hardware mapping. This matrix maps directly to custom boards (such as TI C2000 LaunchPads or GaN inverter boards):

| MCU Pin | Signal Name | Hardware Function | Peripheral & Mode | Configuration Details | Custom Hardware / GaN Inverter Equivalent |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **PA8** | `TIM1_CH1` | Phase U High/Low PWM | **TIM1** PWM Generation 1 | PWM Mode 2, Center-Aligned 1, Period = 1600 counts (64 MHz clock $\rightarrow$ 20 kHz) | TI C2000 `EPWM1A` (High-Side GaN) / `EPWM1B` (Low-Side GaN) |
| **PA9** | `TIM1_CH2` | Phase V High/Low PWM | **TIM1** PWM Generation 2 | PWM Mode 2, Center-Aligned 1, Period = 1600 counts | TI C2000 `EPWM2A` / `EPWM2B` |
| **PA10**| `TIM1_CH3` | Phase W High/Low PWM | **TIM1** PWM Generation 3 | PWM Mode 2, Center-Aligned 1, Period = 1600 counts | TI C2000 `EPWM3A` / `EPWM3B` |
| **PC10**| `CH1_EN` | Phase U Inverter Gate Enable | **GPIO** Output (Push-Pull) | Active High (`ENABLE_INV = 1`) | Gate Driver Enable / Clear Fault (`EN_GATE` / `SD`) |
| **PC11**| `CH2_EN` | Phase V Inverter Gate Enable | **GPIO** Output (Push-Pull) | Active High (`ENABLE_INV = 1`) | Gate Driver Enable / Clear Fault |
| **PC12**| `CH3_EN` | Phase W Inverter Gate Enable | **GPIO** Output (Push-Pull) | Active High (`ENABLE_INV = 1`) | Gate Driver Enable / Clear Fault |
| **PA0** | `ADC1_IN1` | Phase A Shunt Current ($I_a$) | **ADC1** Injected Channel 1 | Triggered by TIM1 TRGO (PWM valley), 2.5 cycles sampling | TI C2000 `ADCINA0` (SOCA triggered by ePWM1 CMPC/CTR=0) |
| **PC1** | `ADC1_IN7` | Phase B Shunt Current ($I_b$) | **ADC1** Injected Channel 2 | Triggered by TIM1 TRGO (PWM valley), 2.5 cycles sampling | TI C2000 `ADCINB0` (SOCA triggered by ePWM1 CMPC/CTR=0) |
| **PA15**| `TIM2_CH1` | QEP Encoder Channel A | **TIM2** Encoder Interface | `TIM_ENCODERMODE_TI12`, 4x count resolution, Period = 65535 | TI C2000 `EQEP1A` |
| **PB3** | `TIM2_CH2` | QEP Encoder Channel B | **TIM2** Encoder Interface | `TIM_ENCODERMODE_TI12`, 4x count resolution, Period = 65535 | TI C2000 `EQEP1B` |
| **PB10**| `QEP_INDEX`| QEP Index Pulse ($Z$) | **EXTI** Line 10 Interrupt | GPIO Falling Edge Trigger (`GPIO_MODE_IT_FALLING`) | TI C2000 `EQEP1I` (Index Event Latch) |
| **PC4** | `USART1_TX`| Telemetry to Host PC | **USART1** TX (DMA Mode) | 4,000,000 Baud (4 MBaud), DMA1 Channel 4, Circular / Normal | TI C2000 `SCIA_TX` (DMA / FIFO Enabled) |
| **PC5** | `USART1_RX`| Commands from Host PC | **USART1** RX (DMA Mode) | 4,000,000 Baud (4 MBaud), DMA1 Channel 5, Circular Reception | TI C2000 `SCIA_RX` (DMA / FIFO Enabled) |
| **—**   | `TIM6` | Speed Controller Timer Base | **TIM6** Internal Timer | Overflow period = 500 $\mu$s (Period = 32,000 at 64 MHz) | TI C2000 CPUTimer0 / ePWM4 periodic interrupt |
| **PD2** | `TEST` | Profiling / Latency Pin | **GPIO** Output (Push-Pull) | High on ISR entry, Low on ISR exit (monitored via oscilloscope) | Dedicated Diagnostic GPIO Pin |
| **PB13**| `LD2` | Status LED | **GPIO** Output | Green LED Indicator | Diagnostic Heartbeat LED |
| **PC13**| `B1` | User Pushbutton | **EXTI** Line 13 Interrupt | Falling Edge Trigger | Safety / Manual Stop Button |

---

### Step-by-Step Engineering Workflows

#### Workflow 1: Open-Loop Control & Current Sensor ADC Offset Calibration
*Documentation Reference:* [MathWorks Open-Loop Control & ADC Offset Guide](https://www.mathworks.com/help/mcb/gs/algorithm-export-custom-hardware-open-loop-adc-offset.html)

```
[Host Model: open_loop_host.slx]
       |
  (ENABLE_INV, SPEED_REF) via UART (4 MBaud)
       v
[Target MCU: main.c / open_loop_algorithm_step]
       |
       +---> Generates rotating voltage vector (V/f scalar control)
       +---> Reads Injected ADC Channels (PA0: Ia, PC1: Ib)
       +---> Outputs PWM duty cycles to TIM1
       |
  (Ia, Ib Raw Counts) via UART DMA (framed: 'ss' ... 'ee')
       v
[Host Scope / Signal Statistics]
       |
       +---> Disconnect DC supply & motor phases
       +---> Run Host Model -> Record Median ADC count for Ia & Ib
       +---> Save to inverter.CtSensAOffset and inverter.CtSensBOffset
```

1. **Engineering Objective:**
   - Verify inverter power stage operation, PWM complementary gating, dead-time, and phase connection integrity without needing closed-loop sensors.
   - Measure the exact hardware zero-current voltage offsets ($V_{mid} \approx 1.65\text{ V}$ on a $3.3\text{ V}$ reference) introduced by operational amplifiers, resistor tolerances, and PCB traces.
2. **Algorithm Execution Flow:**
   - Evaluated inside the fast ADC Injected Conversion Complete Callback (`HAL_ADCEx_InjectedConvCpltCallback`) running at 20 kHz (50 $\mu$s period).
   - Generates synthetic 3-phase sinusoidal duty cycles with fixed voltage-to-frequency ratio ($V/f$), maintaining `VOLTAGE_AMP_LOW_LIMIT` at low speeds to overcome stator resistance $R_s$.
3. **C Function Signature:**
   ```c
   void open_loop_algorithm_initialize(void);
   void open_loop_algorithm_step(
       uint8_T ENABLE_INV,              /* Inverter Enable (0 = Disabled, 1 = Enabled) */
       real32_T VOLTAGE_AMP_LOW_LIMIT,  /* Minimum voltage amplitude (per-unit, e.g. 0.15) */
       real32_T SPEED_REF,              /* Reference speed in RPM */
       real32_T duty_vals[3]            /* Output: 3-phase duty cycles in per-unit [0.0, 1.0] */
   );
   ```
4. **ADC Calibration Procedure:**
   - Disconnect motor leads and DC bus power.
   - Power logic circuitry and launch `open_loop_host.slx`.
   - Extract the median raw ADC count for Phase A and Phase B.
   - **Verified Bench Measurement Results:**
     * `inverter.CtSensAOffset = 2058;` ($1.658\text{ V}$)
     * `inverter.CtSensBOffset = 2057;` ($1.657\text{ V}$)
   - These calibrated values are saved in [foc_qep_data.m](file:///c:/96v_gan_humanoid_drive/phase1/simulation/MyFOC%20code/AlgorithmExportWorkflowsForCustomHardware_Example_20260930_133212/FOCAlgorithmExportDemo/foc_qep/foc_qep_data.m).

---

#### Workflow 2: Quadrature Encoder Offset Calibration
*Documentation Reference:* [MathWorks Quadrature Encoder Offset Calibration Guide](https://www.mathworks.com/help/mcb/gs/algorithm-export-custom-hardware-quadrature-encoder-offset.html)

```
[Host Model: qep_calibration_host.slx]
       |
  (ENABLE_INV = 1) via UART
       v
[Target MCU: qep_calibration_algorithm_step]
       |
       |-- Mode 0: Align rotor d-axis to stator phase A (sets theta_e = 0)
       |-- Mode 1: Slowly ramp electrical angle forward (open-loop spin)
       |
[Index Pulse Occurs on PB10 (EXTI Interrupt)]
       |
       +---> Latch TIM2 counter value -> QEP_INDEX_COUNT
       +---> Calculate angular difference: (QEP_INDEX_COUNT - QEP_COUNT_AT_D_AXIS)
       |
  (position_meas, mode) sent back to Host Model
       v
[Host UI: QEP Offset Display]
       |
       +---> Verify positive position slope (if negative, swap two motor phases)
       +---> Capture QEP Offset (pu) [0.0 to 1.0]
       +---> Save to pmsm.PositionOffset in foc_qep_data.m
```

1. **Engineering Objective:**
   - Field-Oriented Control relies on knowing the instantaneous rotor magnetic flux angle ($\theta_e$).
   - A relative quadrature encoder only tracks incremental pulses ($A$ and $B$) and has no absolute reference until the physical index ($Z$) notch passes the optical/magnetic readhead.
   - This calibration measures the constant mechanical/electrical angular distance between the rotor $d$-axis magnetic pole and the physical index mark.
2. **Calibration Phasing Sequence:**
   - **Phase 1 (Lock Mode / Alignment):** The algorithm applies a static direct current vector along Phase A ($\theta_e = 0$). The rotor swings and aligns its magnetic pole with Phase A.
   - **Phase 2 (Open-Loop Ramp):** The algorithm slowly accelerates the voltage vector, forcing the motor to rotate forward.
   - **Phase 3 (Index Latch):** When the encoder index line toggles, EXTI line 10 fires instantly:
     ```c
     void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
         if (GPIO_Pin == QEP_INDEX_Pin) {
             QEP_INDEX_COUNT = __HAL_TIM_GET_COUNTER(&htim2);
         }
     }
     ```
   - The algorithm computes the normalized difference:
     $$\text{PositionOffset} = \frac{\theta_{electrical, index}}{2\pi} \pmod 1 \quad [\text{per-unit}]$$
3. **C Function Signature:**
   ```c
   void qep_calibration_algorithm_initialize(void);
   void qep_calibration_algorithm_step(
       uint8_T ENABLE_INV,              /* Inverter enable flag */
       uint16_T QEP_INDEX_COUNT,        /* Latched TIM2 count at EXTI index pulse */
       uint16_T QEP_COUNT,              /* Current continuous TIM2 counter value */
       real32_T duty_vals[3],           /* Output: 3-phase PWM duty cycles [0.0, 1.0] */
       real32_T *mode,                  /* Output: Calibration state machine mode (0, 1, 2) */
       real32_T *position_meas          /* Output: Current measured electrical angle [0.0, 1.0] */
   );
   ```
4. **Validation Check:**
   - Verify on the time scope that the position ramp has a **positive slope**.
   - If the slope is negative, the encoder count direction is inverted relative to motor phase sequence. **Remedy:** Disconnect DC power and swap any two motor phase connections (e.g., swap Phase V and Phase W).
   - Enter the calibrated value into `pmsm.PositionOffset` in [foc_qep_data.m](file:///c:/96v_gan_humanoid_drive/phase1/simulation/MyFOC%20code/AlgorithmExportWorkflowsForCustomHardware_Example_20260930_133212/FOCAlgorithmExportDemo/foc_qep/foc_qep_data.m).
   - **Bench Calibrated Value:** `pmsm.PositionOffset = 0.0132` (per-unit position).
   - **Host Model Setting:** In `OffsetCalibrationF28069mHost/Serial Communication/Check Direction/Subsystem/check motor is running`, threshold is set to `motor.calibSpeed * 0.8` (48 RPM) instead of multiplying by pole pairs, matching mechanical RPM output from the speed measurement block.

---

#### Workflow 3: Closed-Loop Dual-Rate Field-Oriented Control (FOC)
*Documentation Reference:* [MathWorks Field-Oriented Control Guide](https://www.mathworks.com/help/mcb/gs/algorithm-export-custom-hardware-field-oriented-control.html)

```
================================================================================================
FAST TASK: CURRENT CONTROL LOOP (20 kHz / 50 us Period) - ADC Injected ISR
================================================================================================
  Raw ADC Ia, Ib counts
            |
            v
  Offset Subtraction & PU Normalization:
    Ia_pu = (Ia - CtSensAOffset) * Gain_pu
    Ib_pu = (Ib - CtSensBOffset) * Gain_pu
    Ic_pu = -(Ia_pu + Ib_pu)
            |
            v
  Clarke Transformation (abc -> alpha, beta):
    I_alpha = Ia_pu
    I_beta  = (Ia_pu + 2*Ib_pu) / sqrt(3)
            |
            v
  Rotor Flux Angle Calculation (TIM2 Count + PositionOffset):
    theta_e = mod( (Count - IndexCount) * PPR_ratio + PositionOffset, 1.0 )
            |
            v
  Park Transformation (alpha, beta -> d, q):
    Id =  I_alpha * cos(theta_e) + I_beta * sin(theta_e)
    Iq = -I_alpha * sin(theta_e) + I_beta * cos(theta_e)
            |
            v
  PI Current Regulators:
    Vd_ref = PI_d( Id_ref - Id ) - Feedforward_Decoupling
    Vq_ref = PI_q( Iq_ref - Iq ) + BackEMF_Decoupling
            |
            v
  Inverse Park Transformation (d, q -> alpha, beta):
    V_alpha = Vd_ref * cos(theta_e) - Vq_ref * sin(theta_e)
    V_beta  = Vd_ref * sin(theta_e) + Vq_ref * cos(theta_e)
            |
            v
  Space Vector PWM (SVPWM) Duty Calculation:
    duty_vals[0..2] in [0.0, 1.0]
            |
            v
  Convert to Timer Compare Registers:
    TIM1->CCR1 = (1 - duty_vals[0]) * ARR;
    TIM1->CCR2 = (1 - duty_vals[1]) * ARR;
    TIM1->CCR3 = (1 - duty_vals[2]) * ARR;

================================================================================================
SLOW TASK: SPEED CONTROL LOOP (1-2 kHz / 500 us - 1 ms Period) - TIM6 Periodic ISR
================================================================================================
  Speed Reference (RPM) & Measured Speed PU (from QEP differentiator + filter)
            |
            v
  PI Speed Regulator with Anti-Windup & Saturation:
    Iq_ref = PI_speed( Speed_Ref_pu - Speed_Meas_pu )
    Id_ref = 0  (Surface PMSM / Maximum Torque Per Ampere below base speed)
            |
            +------------------------> Passed to Current Loop via shared volatile IDQ_REF
```

1. **Dual-Rate Execution Architecture:**
   - **Current Controller (`current_control_algorithm.slx`):** Executed inside `HAL_ADCEx_InjectedConvCpltCallback` at **20 kHz** (every 50 $\mu$s). High bandwidth ensures fast current transient tracking and torque regulation.
   - **Speed Controller (`speed_control_algorithm.slx`):** Executed inside `HAL_TIM_PeriodElapsedCallback` on TIM6 at **2 kHz** (every 500 $\mu$s). Lower bandwidth matches mechanical motor/load inertia.
2. **Current Loop C Function Signature:**
   ```c
   void current_control_algorithm_initialize(void);
   void current_control_algorithm_step(
       uint8_T ENABLE_INV,              /* Inverter enable flag */
       uint8_T ENABLE_CL_HOST,          /* Host closed-loop enable request */
       uint16_T Ia,                     /* Raw Phase A ADC injected conversion count */
       uint16_T Ib,                     /* Raw Phase B ADC injected conversion count */
       uint16_T QEP_INDEX_COUNT,        /* Captured encoder counter value at Z-index */
       uint16_T QEP_COUNT,              /* Instantaneous encoder counter value */
       real32_T SPEED_REF,              /* Reference speed commanded from host */
       real32_T IDQ_REF[2],             /* Input from speed controller: [Id_ref, Iq_ref] */
       real32_T *SPEED_MEAS_PU,         /* Output: Filtered measured speed in per-unit */
       real32_T duty_vals[3],           /* Output: 3-phase duty cycles [0.0, 1.0] */
       boolean_T *CL_ENABLE,            /* Output: Internal closed-loop active flag */
       real32_T *pos_meas,              /* Output: Computed electrical angle [0.0, 1.0] */
       real32_T Iab_meas_pu[2]          /* Output: Phase currents [Ia, Ib] in per-unit */
   );
   ```
3. **Speed Loop C Function Signature:**
   ```c
   void speed_control_algorithm_initialize(void);
   void speed_control_algorithm_step(
       uint8_T ENABLE_INV,              /* Inverter enable flag */
       uint8_T CL_ENABLE,               /* Closed-loop enabled indicator from current loop */
       real32_T SPEED_REF,              /* Commanded speed reference in RPM */
       real32_T SPEED_MEAS_PU,          /* Filtered measured speed from current loop */
       real32_T IDQ_REF[2]              /* Output: Computed [Id_ref, Iq_ref] current commands */
   );
   ```
4. **PWM Compare Register Mapping:**
   In [foc_qep/STM32Code/main.c](file:///c:/96v_gan_humanoid_drive/phase1/simulation/MyFOC%20code/AlgorithmExportWorkflowsForCustomHardware_Example_20260930_133212/FOCAlgorithmExportDemo/foc_qep/STM32Code/main.c), the duty cycle is inverted to match PWM Mode 2:
   ```c
   for (int i = 0; i < 3; i++) {
       duty_vals[i] = (1.0f - duty_vals[i]) * htim1.Init.Period;
   }
   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)duty_vals[0]);
   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)duty_vals[1]);
   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)duty_vals[2]);
   ```

---

### Per-Unit (PU) Normalization & Gain Derivation

To make the algorithm portable across integer DSPs and 32-bit floating point MCUs, all internal quantities are scaled to a dimensionless range $[-1.0, +1.0]$:

| Parameter | Mathematical Definition | Physical Meaning |
| :--- | :--- | :--- |
| **Voltage Base ($V_{base}$)** | $V_{base} = \frac{V_{dc}}{\sqrt{3}}$ | Maximum peak phase voltage synthesizable by linear SVPWM without overmodulation. |
| **Current Base ($I_{base}$)** | $I_{base} = \frac{V_{ref,ADC}}{2 \cdot R_{shunt} \cdot A_{opamp}}$ | Peak measurable neutral phase current before ADC saturation. |
| **Speed Base ($N_{base}$)** | $N_{base} = \frac{V_{dc}}{\sqrt{3} \cdot K_e}$ | Theoretical no-load speed at full DC link voltage, where $K_e$ is back-EMF constant. |
| **Power Base ($P_{base}$)** | $P_{base} = \frac{3}{2} \cdot V_{base} \cdot I_{base}$ | Total 3-phase apparent electrical power capability of the inverter. |

#### Automated Gain Synthesis via `foc_qep_data.m`
The MATLAB data script calls Motor Control Blockset utility functions:
- `mcb.getPMSMParameters('BLY171D')`: Loads pole pairs ($p$), phase resistance ($R_s$), direct/quadrature inductances ($L_d, L_q$), and torque constant ($K_t$).
- `mcb.getPUSystemParameters(pmsm, inverter)`: Computes base values for current, voltage, frequency, and impedance.
- `mcb.getPIControllerParameters(pmsm, inverter, PU_System, T_pwm, 5*Ts, Ts_speed)`: Automatically derives zero-placement PI gains:
  $$K_{p,current} = \frac{L_q \cdot \omega_{bw}}{V_{base} / I_{base}}, \quad K_{i,current} = \frac{R_s \cdot \omega_{bw}}{V_{base} / I_{base}} \cdot T_s$$
  $$\omega_{bw} \approx \frac{2\pi}{10 \cdot T_{pwm}} \quad (\text{Targeting } 10\times \text{ bandwidth separation})$$

---

### Host-Target Serial Communication Protocol

The host Simulink models communicate with the target MCU over a high-speed asynchronous serial link:
- **Baud Rate:** 4,000,000 bps (4 MBaud), 8 data bits, 1 stop bit, no parity.
- **Hardware Architecture:** STM32 USART1 linked to DMA1 Channel 5 (Circular RX) and DMA1 Channel 4 (Normal TX).

```
HOST TO TARGET PACKET (Binary IEEE 754 Single Precision Floats):
+--------------------+------------------------+---------------------+
| Byte 0 .. 3        | Byte 4 .. 7            | Byte 8 .. 11        |
| Inverter Enable    | Closed-Loop Host Enable| Speed Reference RPM |
| (float: 0.0 / 1.0) | (float: 0.0 / 1.0)     | (float: e.g. 1500.0)|
+--------------------+------------------------+---------------------+

TARGET TO HOST PACKET (Sample Streaming with Delimiters):
Sample 0 (Header):
+--------------------+--------------------+--------------------+--------------------+
| Byte 0: 's' (0x73) | Byte 1: 's' (0x73) | Byte 2..3: Debug 1 | Byte 4..5: Debug 2 |
+--------------------+--------------------+--------------------+--------------------+
Samples 1 .. 198:
+--------------------+--------------------+
| Byte 0..1: Debug 1 | Byte 2..3: Debug 2 |
+--------------------+--------------------+
Sample 199 (Tail):
+--------------------+--------------------+--------------------+--------------------+
| Byte 0..1: Debug 1 | Byte 2..3: Debug 2 | Byte 4: 'e' (0x65) | Byte 5: 'e' (0x65) |
+--------------------+--------------------+--------------------+--------------------+
```
- In Step 1 (Open-Loop): `Debug 1 = Ia (Raw Counts)`, `Debug 2 = Ib (Raw Counts)`.
- In Step 2 (QEP Calib): `Debug 1 = Position [0..65535]`, `Debug 2 = Calibration Mode`.
- In Step 3 (FOC): `Debug 1 = Normalized Ia_pu [0..65535]`, `Debug 2 = Normalized Speed_pu [0..65535]`.

---

### Migration Guide to Custom Hardware (e.g., 96V GaN Inverter + TI C2000 / Custom MCU)

When porting this exported algorithm to the **96V GaN Humanoid Drive** (or a TI TMS320F280039C / F28069M controller):

```
+-----------------------------------------------------------------------------------------+
| STEP 1: CONFIGURE SYSTEM PARAMETERS                                                     |
| Update foc_qep_data.m with 96V Inverter specifics:                                      |
| - inverter.V_dc = 96.0;                                                                 |
| - inverter.Rshunt = [Custom Shunt Ohms];                                                |
| - inverter.ISenseVoltPerAmp = [Current Sense Amplifier Gain * Rshunt];                  |
| - target.CPU_frequency = 100e6 or 120e6 (for C2000 F280039C) / 64e6;                    |
| - target.PWM_frequency = 20e3 to 50e3 (GaN allows 40-100 kHz with ultra-low losses);   |
+-----------------------------------------------------------------------------------------+
                                           |
                                           v
+-----------------------------------------------------------------------------------------+
| STEP 2: CODE GENERATION WITH EMBEDDED CODER                                             |
| 1. Open current_control_algorithm.slx and speed_control_algorithm.slx                   |
| 2. In Model Settings -> Hardware Implementation:                                        |
|    Set Device Vendor to 'Texas Instruments' or 'ARM Compatible'                         |
| 3. Set Code Interface -> Function Interface to match argument list                      |
| 4. Click 'Build' -> Embedded Coder generates clean C source in work/code/               |
+-----------------------------------------------------------------------------------------+
                                           |
                                           v
+-----------------------------------------------------------------------------------------+
| STEP 3: CONFIGURE TI C2000 / CUSTOM MCU PERIPHERALS                                     |
| - ePWM1, ePWM2, ePWM3: Up-Down Count Mode (Center-Aligned), dead-band for GaN (<50 ns).  |
| - ADC Trigger: ePWM1 SOCA triggered on Counter Underflow (CTR=0, Valley of PWM).        |
| - eQEP1: 4x quadrature decoding, latch on index (IEL) to capture QEP_INDEX_COUNT.       |
| - Timer / ePWM4: 500 us / 1 ms periodic interrupt for speed controller.                |
| - SCIA: FIFO/DMA mode for host packet telemetry.                                        |
+-----------------------------------------------------------------------------------------+
                                           |
                                           v
+-----------------------------------------------------------------------------------------+
| STEP 4: HARDWARE BRING-UP SEQUENCE                                                      |
| 1. Run open_loop_algorithm to verify GaN gate switching & compute ADC Offsets.          |
| 2. Run qep_calibration_algorithm to latch index and set pmsm.PositionOffset.           |
| 3. Run dual-rate FOC in current_control_algorithm and speed_control_algorithm.          |
+-----------------------------------------------------------------------------------------+
```

---

### Relevant Documentation & Reference Links

1. **MathWorks Primary Topic & Workflow Guides:**
   - [Algorithm-Export Workflows for Custom Hardware](https://www.mathworks.com/help/mcb/gs/algorithm-export-workflow-for-custom-hardware.html)
   - [Step 1: Open-Loop Control and ADC Offset Calibration](https://www.mathworks.com/help/mcb/gs/algorithm-export-custom-hardware-open-loop-adc-offset.html)
   - [Step 2: Quadrature Encoder Offset Calibration](https://www.mathworks.com/help/mcb/gs/algorithm-export-custom-hardware-quadrature-encoder-offset.html)
   - [Step 3: Field-Oriented Control](https://www.mathworks.com/help/mcb/gs/algorithm-export-custom-hardware-field-oriented-control.html)

2. **Foundational Control & MBD Engineering Concepts:**
   - [Field-Oriented Control (FOC) Principles](https://www.mathworks.com/help/mcb/gs/implement-motor-speed-control-by-using-field-oriented-control-foc.html)
   - [Per-Unit (PU) System in Motor Control Blockset](https://www.mathworks.com/help/mcb/gs/per-unit-system.html)
   - [Program Control Flow of MCB Examples](https://www.mathworks.com/help/mcb/gs/program-control-flow-of-mcb-examples.html)
   - [Host-Target Serial Communication Protocol](https://www.mathworks.com/help/mcb/gs/communication-between-host-and-target.html)
   - [Estimate PMSM Parameters Using Custom Hardware](https://www.mathworks.com/help/mcb/gs/estimate-pmsm-parameters-using-custom-hardware.html)
   - [Estimate Control Gains and Tune PI Parameters](https://www.mathworks.com/help/mcb/gs/estimate-control-gains-from-motor-parameters.html)
