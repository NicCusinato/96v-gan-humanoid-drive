# FOC Signal Extraction Reference

## Where to Find Each Signal in FOC_Hardware_Deployment.slx

### Current Signals

#### **Ia, Ib, Ic (Phase Currents - Raw ADC Counts)**
- **Source**: ADC blocks (likely named `c2802xADC_Ia`, `c2802xADC_Ib`, `c2802xADC_Ic` or similar)
- **Output Type**: uint16 (0-4095 ADC counts)
- **Scaling to Amps**: 
  ```matlab
  Ia_amps = (adc_a - offset_a) * (3.3 / 4095) / (0.002 * 20);
  % Breakdown:
  % (adc_a - offset_a)  → remove bias
  % * 3.3 / 4095        → convert to voltage
  % / (0.002 * 20)      → convert to current (sensor gain = 0.002 V/A, amp gain = 20)
  ```
- **Typical Range**: 2000-2100 (unpowered), 2000-4000 (under load)
- **Location in Model**: Usually connected to PWM inverter or main control block

#### **Id, Iq (d/q Axis Currents - After Park Transform)**
- **Source**: Park transform block output (e.g., `Park_Transform`, `dq_currents`, or inside a subsystem)
- **Output Type**: single/double (already in amps)
- **Typical Range**: -10 to +10 A
- **What They Mean**:
  - **Id**: Direct (magnetizing) axis current - keeps flux aligned
  - **Iq**: Quadrature (torque-producing) axis current - creates torque
- **Location in Model**: Inside main FOC controller subsystem (usually after Clarke transform)
- **How to Find**: 
  - Look for blocks titled "Clarke Transform" → outputs Iα, Iβ
  - Then "Park Transform" → outputs Id, Iq (with rotor angle as input)

#### **Rotor Position (Mechanical)**
- **Source**: eQEP decoder block output (e.g., `eQEP_1`, `qep_count`, `position`)
- **Output Type**: uint32 (pulse count) or normalized (0-1)
- **Raw Count Range**: 0-4095 for 1024 PPR encoder in 4x mode
- **Conversion to Degrees**: `position_deg = count * 360 / 4096`
- **Location in Model**: Usually at root level, coming from encoder interface

#### **SPI Rotor Position (AS5047P)**
- **Source**: SPI read block output (likely `AS5047P_Read`, `spi_position`, or inside read function)
- **Output Type**: uint16 (14-bit, so 0-16383)
- **Conversion to Degrees**: `position_deg = count * 360 / 16384`
- **Location in Model**: Encoder read section, probably early in execution order

---

### Voltage Signals

#### **Vd, Vq (Voltage Commands - d/q Frame)**
- **Source**: SVPWM (Space Vector PWM) block input (the voltage commands going INTO the modulator)
- **Output Type**: single/double (volts)
- **Typical Range**: -48 to +48 V (depends on your system voltage)
- **What They Mean**:
  - **Vd**: Voltage applied to align/strengthen magnetic field
  - **Vq**: Voltage applied to produce torque (orthogonal to field)
- **Location in Model**: PI speed controller output → SVPWM input
- **How to Find**:
  - Look for PI controller blocks in the speed/torque control section
  - Their outputs typically feed directly to SVPWM

#### **Vdc (DC Link Voltage - Raw ADC)**
- **Source**: ADC block for voltage sensing (e.g., `c2802xADC_Vdc`, `voltage_sense`)
- **Output Type**: uint16 (0-4095 ADC counts)
- **Scaling to Volts**: 
  ```matlab
  Vdc_volts = adc_vdc * (3.3 * 46.4545 / 4095);
  % Breakdown:
  % 46.4545 = (100k + 2.2k) / 2.2k  → divider ratio
  % 3.3 / 4095                      → ADC scaling
  ```
- **Typical Range**: 1000-3500 counts (when 48V nominal)
- **Location in Model**: Voltage sensing path, usually separate from current sensing

#### **Modulation Index (Optional, but useful)**
- **Source**: Inside SVPWM or main controller (check for variable named `m`, `modulation`, `duty_cycle`)
- **Range**: 0.0 to 1.0 (or 0 to 100%)
- **Meaning**: How much of available voltage is being used
- **Typical Values**:
  - Open-loop startup: 0.05-0.10 (5-10%)
  - Closed-loop ramping: 0.06-0.15 (6-15%)
  - Full speed: 0.3-0.9 (30-90%)

---

### Control/Reference Signals

#### **Speed Command (Reference)**
- **Source**: Command decoder or state machine (look for input named `spd_ref`, `spd_cmd`, `w_target`)
- **Output Type**: int16 or single (RPM)
- **Typical Range**: -2000 to +2000 RPM (depends on motor)
- **Location in Model**: Early in the control loop, coming from serial/CAN/user interface

#### **Speed Actual (Feedback)**
- **Source**: Encoder speed calculation block (e.g., `speed_filter`, `speed_calc`)
- **Output Type**: single (RPM)
- **How It's Calculated**: Speed = (encoder_count - previous_count) / delta_time
- **Typical Filter**: Low-pass filter (alpha = 0.8-0.95) to smooth noisy measurements
- **Location in Model**: Usually right after encoder read

#### **Rotor Electrical Angle (theta_e)**
- **Source**: Angle calculation block (typically: `theta_e = theta_m * POLE_PAIRS + offset`)
- **Output Type**: single (radians, 0 to 2π)
- **Breakdown**:
  - `theta_m` = mechanical angle from encoder (0 to 2π/pole_pairs)
  - `POLE_PAIRS` = 21 for your motor
  - `offset` = encoder offset (~0.2592 rad from your measurement)
- **Formula**: `theta_e = mechanical_angle * pole_pairs + offset`
- **Location in Model**: Inside main FOC controller, output of Clarke/Park calculation

---

### State/Control Status Signals

#### **State Echo / Control Mode**
- **Source**: State machine in main controller
- **Output Type**: uint8 (0-255)
- **Typical Values**:
  - `0` = STOP
  - `1` = Open-loop 60 RPM
  - `2` = Open-loop 120 RPM
  - `4` = Alignment (fixed 0° electrical)
  - `5` = Closed-loop FOC 100 RPM
  - `6` = Closed-loop FOC 200 RPM
  - `51` = Startup ramp (Phase 1 of closed-loop)
- **Location in Model**: Main controller state machine output

#### **Gate Enable Signal**
- **Source**: Gate driver enable block (e.g., `gate_enable`, `PWM_enable`)
- **Output Type**: boolean (0 or 1) or uint8 (0-1)
- **Meaning**:
  - `1` = PWM outputs active, gates conducting
  - `0` = PWM stopped, gates off (freewheeling)
- **Location in Model**: Global gate enable signal, usually affects all PWM blocks

---

## How to Extract Signals: Step-by-Step

### Method 1: Using From/Goto Blocks (Recommended for Quick Integration)

```
1. In FOC_Hardware_Deployment.slx, find signal of interest (e.g., Id output)
2. Right-click the signal line → Create → Goto
3. Set name to `Id_signal`
4. Go to telemetry section (where Format_Telemetry_FOC block is)
5. Right-click → Create → From
6. Set name to `Id_signal`
7. Connect From block to Format_Telemetry_FOC `Id` input
8. Repeat for all 9 signals needed
```

### Method 2: Adding Output Ports (More Professional)

```
1. Find the main FOC controller subsystem (usually largest subsystem)
2. Double-click to enter edit mode
3. Right-click in empty space → Add Port → Output
4. Name it `Id` and connect to Id calculation block
5. Repeat for all FOC signals needed
6. Exit subsystem
7. Now external signals available as outputs of FOC subsystem
8. Connect to Format_Telemetry_FOC block directly
```

### Method 3: Extracting from Inside a Large Subsystem

If signals are deeply nested:
```
1. Open FOC subsystem and navigate to where signal is generated
2. Right-click signal → Go to Named Ports → Create Goto
3. Name: `Id_signal`
4. This "promotes" the signal to model-wide availability
5. Create From blocks at top level to access it
```

---

## Typical FOC Model Structure (For Reference)

```
FOC_Hardware_Deployment.slx
├── ADC_Ia ────────────────────────┐
├── ADC_Ib ────────────────────────├──┐
├── ADC_Ic ────────────────────────┤  │
├── ADC_Vdc ──────────────────────┐│  │
├── eQEP ──────────────────────────┤  │
├── SPI_Read ──────────────────────┤  │
│                                   │  │
└─ FOC_Motor_Control (MAIN) ◄──────┤  │
    ├── Clarke Transform            │  │
    │   ├── [outputs: Iα, Iβ]      │  │
    │   └── (input: Ia, Ib, Ic) ◄──┘  │
    │                                   │
    ├── Park Transform ◄─ Rotor Angle  │
    │   ├── [outputs: Id, Iq]         │
    │   └── (input: Iα, Iβ)           │
    │                                   │
    ├── PI Speed Controller            │
    │   └── [outputs: Vd_ref, Vq_ref]  │
    │                                   │
    └── SVPWM ◄─ Electrical Angle      │
        ├── [outputs: Vabc or PWM]      │
        └── (input: Vd_ref, Vq_ref)     │
            
                      ↓ (extract these 9 signals)
                      
    Signals needed for telemetry:
    1. Id (from Park output)
    2. Iq (from Park output)
    3. theta_e (from rotor angle calc)
    4. Vd (from PI controller)
    5. Vq (from PI controller)
    6. speed_actual (from encoder calc)
    7. speed_cmd (from command interface)
    8. state_echo (from state machine)
    9. gate_enable (global)
```

---

## Signal Verification Checklist

Once you've connected all signals, verify each with these checks:

| Signal | Expected Range | Check On Dashboard | Pass? |
|--------|-----------------|-------------------|-------|
| Ia | ±20 A (raw: 0-4095) | Should be sinusoidal at motor speed | ☐ |
| Ib | ±20 A (raw: 0-4095) | 120° phase-shifted from Ia | ☐ |
| Ic | ±20 A (raw: 0-4095) | 120° phase-shifted from Ib | ☐ |
| Id | ±10 A | Should be relatively constant during steady-state | ☐ |
| Iq | ±10 A | Should vary with speed command | ☐ |
| theta_e | 0 to 2π rad | Should increase linearly at constant speed | ☐ |
| Vd | ±48 V (scaled) | Should be semi-constant for field alignment | ☐ |
| Vq | ±48 V (scaled) | Should vary with speed controller output | ☐ |
| speed_actual | ±2000 RPM | Should match command after ramp | ☐ |
| speed_cmd | ±2000 RPM | Should match selected command (60/100/200) | ☐ |
| state_echo | 0-255 | Should show state (1=OL60, 5=CL100, 51=RAMP) | ☐ |
| gate_enable | 0 or 1 | Should be 1 when running, 0 when stopped | ☐ |
| Vdc | 0-96 V (scaled) | Should match your battery voltage | ☐ |

---

## Common Mistakes to Avoid

❌ **Mistake 1**: Connecting raw ADC count for current, forgetting to apply offset
- ✓ **Fix**: Check offset_a/offset_b values in Dashboard code

❌ **Mistake 2**: Connecting Ia/Ib/Ic instead of Id/Iq
- ✓ **Fix**: Id/Iq are the key signals; they show torque-producing vs alignment current

❌ **Mistake 3**: theta_e not increasing (static angle)
- ✓ **Fix**: Verify encoder position is actually changing; check that angle calculation includes offset

❌ **Mistake 4**: Voltage commands (Vd/Vq) always zero
- ✓ **Fix**: Check PI controller is enabled and receiving speed error

❌ **Mistake 5**: Gate enable always 0 (never runs)
- ✓ **Fix**: Gate enable signal might be named differently (check: `enable`, `run`, `gate_output`)

---

## Next: Run the Setup Script

Once you understand where signals are:

```matlab
cd 'c:\96v_gan_humanoid_drive\phase1\simulation\MyFOC code'
FOC_Telemetry_Setup  % Prints detailed instructions
```

This will guide you through the exact steps for your model. 🎯
