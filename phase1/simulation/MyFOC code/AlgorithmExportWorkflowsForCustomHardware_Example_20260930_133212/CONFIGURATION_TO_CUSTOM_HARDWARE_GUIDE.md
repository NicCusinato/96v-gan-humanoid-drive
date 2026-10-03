# Step-by-Step Configuration Guide: Adapting Algorithm Export to Custom Hardware
## Target: EPC91200 GaN Inverter + CubeMars AKE80-8 Motor + TI LAUNCHXL-F28069M / C2000

---

### Overview & Architecture Adaptation

The generic MathWorks example was configured for an STMicroelectronics NUCLEO board, an ST inverter shield, and a BLY171D motor. To adapt this Algorithm Export Workflow to your **96V GaN Humanoid Drive** hardware stack (**EPC91200 GaN Inverter + CubeMars AKE80-8 KV30 Actuator + TI TMS320F28069M LaunchPad**), you must perform four distinct configuration stages:

```
[STAGE 1: Model Parameterization]
Update data scripts (foc_qep_data.m, open_loop_data.m, qep_calibration_data.m)
with AKE80-8, EPC91200, and C28x 90MHz clock parameters.
               │
               ▼
[STAGE 2: Embedded Coder Target Configuration]
Set Hardware Implementation to TI C2000 / FPU32 in Simulink.
Re-generate ANSI C code into work/code/.
               │
               ▼
[STAGE 3: C2000 Peripheral & Driver Mapping in CCS Theia]
Implement ePWM (center-aligned + GaN dead-band), ADC SOCA triggering,
eQEP index latching, and dual-rate ISR execution.
               │
               ▼
[STAGE 4: Hardware Bring-Up Execution (3 Sequential Workflows)]
1. Open-Loop V/f run -> Measure Ia/Ib ADC offsets.
2. QEP Calibration -> Latch index & verify positive slope.
3. Closed-Loop FOC -> Spin up with current & speed regulators.
```

---

### Stage 1: Parameterize the MATLAB Data Scripts

Before generating code, you must update [foc_qep_data.m](file:///c:/96v_gan_humanoid_drive/phase1/simulation/MyFOC%20code/AlgorithmExportWorkflowsForCustomHardware_Example_20260930_133212/FOCAlgorithmExportDemo/foc_qep/foc_qep_data.m) (and the matching `open_loop_data.m` / `qep_calibration_data.m` files) with your actual motor, inverter, and microcontroller hardware specifications.

#### 1.1 Microcontroller Target Parameters (`target`)
```matlab
%% Set Target Parameters (TI TMS320F28069M)
target.CPU_frequency        = 90e6;                 % 90 MHz SYSCLK for F28069M
PWM_frequency               = 20e3;                 % 20 kHz initial test frequency (expandable to 40-50 kHz for GaN)
T_pwm                       = 1/PWM_frequency;      % 50 microseconds
target.PWM_frequency        = PWM_frequency;
% Up-down counter period = CPU_freq / (2 * PWM_freq)
target.PWM_Counter_Period   = round(target.CPU_frequency / (2 * target.PWM_frequency)); % 2250 counts
target.ADC_Vref             = 3.3;                  % 3.3V ADC reference voltage
target.ADC_MaxCount         = 4095;                 % 12-bit ADC full-scale (2^12 - 1)
target.SCI_baud_rate        = 1.25e6;               % 1.25 MBaud (or 4 MBaud if supported by FTDI transceiver)
```

#### 1.2 CubeMars AKE80-8 KV30 Actuator Parameters (`pmsm`)
*Do not use `mcb.getPMSMParameters('BLY171D')`*. Configure the struct explicitly:
```matlab
%% CubeMars AKE80-8 KV30 Motor Parameters
pmsm.p                      = 21;                   % 21 Pole Pairs
pmsm.Rs                     = 0.435;                % Phase resistance (Ohms) (0.870 ohm line-to-line / 2)
pmsm.Ld                     = 495e-6;               % Direct-axis inductance (495 uH)
pmsm.Lq                     = 495e-6;               % Quadrature-axis inductance (495 uH)
pmsm.J                      = 1.47175e-4;           % Rotor inertia (kg*m^2)
pmsm.I_rated                = 4.8;                  % Continuous rated current (Amps RMS)
pmsm.I_max                  = 12.0;                 % Peak maximum current (Amps)
pmsm.Ke                     = 0.01016 * 1.5 * pmsm.p; % Back-EMF constant (Vpk / (rad_e/s))
pmsm.Kt                     = 0.32;                 % Torque constant (N*m/A)
pmsm.PositionOffset         = 0.0132;               % Calibrated bench offset (Step 2 bench result)
```

#### 1.3 EPC91200 GaN Inverter Parameters (`inverter`)
```matlab
%% EPC91200 3-Phase GaN Inverter Parameters
inverter.model              = 'EPC91200_EPC2305';
inverter.V_dc               = 48.0;                 % 48V bench power supply (keep at 48V before moving to 96V)
inverter.I_trip             = 25.0;                 % Overcurrent trip threshold (Amps)
inverter.Rds_on             = 2.2e-3;               % EPC2305 GaN RDS(on) (2.2 mOhms)
inverter.Rshunt             = 0.005;                % 5 mOhm current shunt (check EPC91200 schematic)
inverter.ISenseVoltPerAmp   = 0.100;                % Net V/A gain (Rshunt * OpAmp Gain)
inverter.ISenseVref         = 3.3;                  % Current sense amplifier Vref
inverter.ISenseMax          = inverter.ISenseVref / (2 * inverter.ISenseVoltPerAmp); % Full scale current
inverter.CtSensAOffset      = 2058;                 % Measured hardware offset for Phase A (1.658V)
inverter.CtSensBOffset      = 2057;                 % Measured hardware offset for Phase B (1.657V)
inverter.EnableLogic        = 1;                    % Active high gate driver enable (EN / nEN check)
inverter.invertingAmp       = -1;                   % Inverted if opamp inverts current direction
```

#### 1.4 Synthesize Controller Gains Automatically
Run the automated gain derivation lines at the bottom of `foc_qep_data.m`:
```matlab
PU_System = mcb.getPUSystemParameters(pmsm, inverter);
PI_params = mcb.getPIControllerParameters(pmsm, inverter, PU_System, T_pwm, 5*Ts, Ts_speed);
```
This automatically computes the zero-placement current PI gains ($K_p, K_i$) matched to the $L/R$ time constant ($1.14\text{ ms}$) of the AKE80-8 motor.

---

### Stage 2: Embedded Coder Target Configuration

For each of the models:
1. `open_loop_algorithm.slx`
2. `qep_calibration_algorithm.slx`
3. `current_control_algorithm.slx`
4. `speed_control_algorithm.slx`

Configure the model properties in Simulink:
1. **Solver Settings:**
   - Type: `Fixed-step`
   - Solver: `discrete (no continuous states)`
   - Fixed-step size: `Ts` ($50\,\mu\text{s}$ for current loop and open-loop) or `Ts_speed` ($500\,\mu\text{s}$ for speed loop).
2. **Hardware Implementation:**
   - Device vendor: `Texas Instruments`
   - Device type: `C2000` (or `ARM Compatible -> ARM Cortex-M` if targeting an STM32 MCU).
   - Floating-point support: `Single precision` (matches F28069M FPU32).
3. **Code Generation:**
   - System target file: `ert.tlc` (Embedded Coder).
   - Language: `C`.
   - Utility function generation: `Shared location`.
4. **Code Interface Mappings:**
   - In Simulink toolstrip: **C Code $\rightarrow$ Code Interface $\rightarrow$ Default Code Mappings**.
   - Verify that the step function takes inputs/outputs as arguments (rather than global structures) so that they match the C function prototypes.
5. **Build Models:**
   - Press **Ctrl+B** (or `slbuild('model_name')`).
   - The generated ANSI C code (`*.c` and `*.h`) is deposited into `work/code/`.

---

### Stage 3: Map C2000 Silicon Drivers & Peripherals (in CCS Theia)

Integrate the generated C code into your Code Composer Studio project (or your custom firmware project). The peripheral mapping for the TI LAUNCHXL-F28069M + EPC91200 adapter is configured as follows:

```
+---------------------------------------------------------------------------------------+
| TI TMS320F28069M PERIPHERAL CONFIGURATION                                             |
+---------------------------------------------------------------------------------------+
| 1. ePWM1 / ePWM2 / ePWM3 (or ePWM4-6 depending on adapter routing):                   |
|    - Time Base: Center-Aligned Up-Down Count (TB_COUNT_UPDOWN)                        |
|    - Period: TBPRD = (90 MHz) / (2 * 20 kHz) = 2250 counts                            |
|    - Action Qualifier: Set on CMPA Up, Clear on CMPA Down                             |
|    - Dead-Band (DB): Active High Complementary (AHC).                                 |
|      * CRITICAL FOR GaN: Set RED and FED to 30-50 ns (e.g. 3-5 counts @ 90MHz).       |
|        Never use IGBT/MOSFET dead-times (1-2 us) on GaN FETs!                         |
|                                                                                       |
| 2. ADC-A & ADC-B (Current Sensing):                                                  |
|    - Trigger: ePWM1 SOCA triggered at Counter Zero (TBCTR = 0x0000).                   |
|      This samples Ia and Ib at the exact midpoint of the low-side conduction cycle.   |
|    - Channels: ADCINA0 (Phase A shunt), ADCINB0 (Phase B shunt).                      |
|    - Resolution: 12-bit, Single-ended.                                                |
|    - Interrupt: ADCINT1 triggered on completion of SOC1 conversion.                   |
|                                                                                       |
| 3. eQEP1 (Quadrature Encoder Interface):                                              |
|    - QDECCTL: Quadrature count mode (4x resolution).                                  |
|    - QPOSMAX: Set to (Lines * 4 - 1). For 1024-line encoder = 4095 counts.            |
|    - Absolute Pre-Seeding: EQep1Regs.QPOSCNT initialized on boot from AS5047P SPI.    |
|    - Reset Mode: Reset on max position (preserves absolute alignment in FOC).         |
|                                                                                       |
| 4. SPI-A (AS5047P Absolute Encoder Interface):                                       |
|    - Pins: GPIO16 (SIMO), GPIO17 (SOMI), GPIO18 (CLK), GPIO19 (CSn).                 |
|    - Role: Reads 14-bit absolute angle on bootup to seed QEP counter with zero motion.|
|    - Full Documentation: AS5047P_SPI_QEP_HARDWARE_SYNC_GUIDE.md in OFFSET CALCULATIONS|
|                                                                                       |
| 5. Periodic Timer (CPUTimer0):                                                        |
|    - Interrupt Rate: 500 us (2 kHz) for speed control loop execution.                 |
|                                                                                       |
| 6. SCI-A (Serial Host Interface):                                                     |
|    - Baud Rate: 1.25 MBaud or 4 MBaud with 16-level TX/RX FIFO.                       |
+---------------------------------------------------------------------------------------+
```

#### Dual-Rate ISR Structure in C:
```c
// =========================================================================
// FAST TASK: Current Loop ISR (20 kHz / 50 us) - Triggered by ADCINT1
// =========================================================================
interrupt void adc_isr(void)
{
    // 1. Set Diagnostic Test GPIO High (for oscilloscope CPU load profiling)
    GpioDataRegs.GPASET.bit.GPIO12 = 1;

    // 2. Read Raw ADC conversion values
    uint16_t Ia_raw = AdcResult.ADCRESULT0; // ADCINA0
    uint16_t Ib_raw = AdcResult.ADCRESULT1; // ADCINB0

    // 3. Read Instantaneous and Latched Index Encoder counts
    uint16_t qep_count = EQep1Regs.QPOSCNT;
    uint16_t qep_index = EQep1Regs.QPOSILAT;

    // 4. Execute Exported Current Control Algorithm Step Function
    current_control_algorithm_step(
        ENABLE_INV,
        ENABLE_CL_HOST,
        Ia_raw,
        Ib_raw,
        qep_index,
        qep_count,
        SPEED_REF_RPM,
        (real32_T *)IDQ_REF,
        (real32_T *)&SPEED_MEAS_PU,
        duty_vals,
        (boolean_T *)&CL_ENABLE,
        &pos_meas,
        Iab_meas_pu
    );

    // 5. Convert Normalized Duty Cycles [0.0, 1.0] to ePWM Compare Counts
    // Period = 2250 counts
    EPwm1Regs.CMPA.half.CMPA = (uint16_t)(duty_vals[0] * EPwm1Regs.TBPRD);
    EPwm2Regs.CMPA.half.CMPA = (uint16_t)(duty_vals[1] * EPwm2Regs.TBPRD);
    EPwm3Regs.CMPA.half.CMPA = (uint16_t)(duty_vals[2] * EPwm3Regs.TBPRD);

    // 6. Reset Diagnostic GPIO & Acknowledge Interrupt
    GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// =========================================================================
// SLOW TASK: Speed Loop ISR (2 kHz / 500 us) - Triggered by CPUTimer0
// =========================================================================
interrupt void cpu_timer0_isr(void)
{
    // Execute Exported Speed Control Algorithm Step Function
    speed_control_algorithm_step(
        ENABLE_INV,
        CL_ENABLE,
        SPEED_REF_RPM,
        SPEED_MEAS_PU,
        (real32_T *)IDQ_REF
    );

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
```

---

### Stage 4: Hardware Bring-Up Execution (The 3 Sequential Workflows)

Execute the three workflows sequentially. **Never skip directly to closed-loop FOC on a high-power GaN stage.**

#### 4.1 Safety Pre-Flight Checklist
- [ ] Bench power supply set to **48.0 V** with current limit clamped to **1.5 A**.
- [ ] Motor unloaded (rotor spinning freely in air, no rigid humanoid knee coupling yet).
- [ ] Verify with an oscilloscope that complementary GaN gate signals exhibit proper dead-time ($\ge 30\,\text{ns}$) and **no shoot-through overlap** while gate power is active and DC bus is 0V.

#### 4.2 Workflow Step 1: Open-Loop & ADC Offset Measurement
1. Build and flash the `open_loop` project to the C2000 controller.
2. Launch `open_loop_host.slx` on the PC and connect over the COM port.
3. Switch **Motor** to **Start** at a low test speed ($60\text{ RPM}$).
4. Confirm smooth rotation and observe the sinusoidal current traces in the scope.
5. **Calibrate Offsets (VERIFIED ON BENCH):**
   - Inverter gate enable disabled (`Stop` / `ENABLE_INV = 0`), 48V DC bus energized to power onboard 12V/5V sensor regulators.
   - **Measured Median Values:**
     * **Phase A ($I_a$):** `2058 counts` ($1.658\text{ V}$)
     * **Phase B ($I_b$):** `2057 counts` ($1.657\text{ V}$)
   - Updated into `foc_qep_data.m` and `setup_offset_calculations.m`.

#### 4.3 Workflow Step 2: Quadrature Encoder Index Calibration
1. Build and flash the `qep_calibration` project.
2. Launch `qep_calibration_host.slx`.
3. Set **Calibration** to **Start**:
   - The algorithm applies a static vector to Phase A (rotor snaps to $\theta_e = 0$).
   - The algorithm ramps forward open-loop until the index line fires the latch.
4. Check the position scope trace:
   - **Positive slope:** Phase sequence matches encoder forward direction.
   - **Negative slope:** Stop immediately! Power down and swap motor phases V and W. Re-run.
5. **Calibrated Physical Offset (VERIFIED ON BENCH):**
   - **Measured Value:** `pmsm.PositionOffset = 0.025635 PU` ($9.23^\circ$ mechanical, $193.8^\circ$ electrical).
   - Validated across bidirectional hand rotation and cold boots.
6. **AS5047P SPI Pre-Seeded Zero-Motion Startup:**
   - Because `seed_qep_from_spi()` pre-loads `EQep1Regs.QPOSCNT = spi_raw >> 2` at $t=0$, Step 3 FOC can run immediately upon power-up without executing an open-loop alignment twitch or index search!


#### 4.4 Workflow Step 3: Closed-Loop FOC Validation
1. Build and flash the `foc_qep` project containing both `current_control_algorithm` and `speed_control_algorithm`.
2. Launch `foc_qep_host.slx`.
3. Start the motor in **Open Loop** mode at $100\text{ RPM}$ to begin rotation.
4. Flip the switch to **Closed Loop**:
   - As soon as the index pulse is verified, the controller transitions smoothly into full Field-Oriented Control.
5. Test speed step changes ($100 \rightarrow 500 \rightarrow 1000\text{ RPM}$) while monitoring phase current RMS and $I_q$ tracking on the PC host scope.
