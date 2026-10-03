# FOC Hardware Deployment + Dashboard Integration Guide

## Overview

You want to "hookup FOC hardware deployment to the dashboard and also read a bunch of signals throughout the actual FOC."

This guide walks you through:
1. **Adding enhanced telemetry** (32 bytes vs current 17) to capture FOC internals
2. **Extracting FOC signals** from inside the motor control loop
3. **Deploying to hardware** and monitoring on the Enhanced Dashboard

## What Signals Are Available?

The FOC control loop internally calculates/measures these critical signals:
- **Phase currents**: Ia, Ib, Ic (raw ADC counts or amps)
- **d/q currents**: Id, Iq (after Clarke + Park transform)
- **Rotor angle**: 
  - Mechanical: from encoder (eQEP or SPI)
  - Electrical: mechanical angle × pole pairs + offset
- **Voltage commands**: Vd, Vq (from PI controller → SVPWM)
- **Speed**: actual RPM (filtered) + command RPM
- **Speed error**: command - actual (for closed-loop diagnostics)
- **Gate drive status**: enabled/disabled

## Enhanced Telemetry Packet (32 bytes)

```
Byte Range  | Name                      | Type    | Range/Scale
─────────────────────────────────────────────────────────────────
0-2         | Sync [170, 85, 170]       | uint8   | Fixed pattern
3-4         | eQEP count                | uint16  | 0-4095 counts
5-6         | Speed actual              | int16   | ±2000 RPM
7-8         | Ia (raw ADC)              | uint16  | 0-4095
9-10        | Ib (raw ADC)              | uint16  | 0-4095
11-12       | Ic (raw ADC)              | uint16  | 0-4095
13-14       | Vdc (raw ADC)             | uint16  | 0-4095
15-16       | SPI count                 | uint16  | 0-16383
17-18       | Id (d-axis current)       | int16   | ±10 A (scaled 3276.7)
19-20       | Iq (q-axis current)       | int16   | ±10 A (scaled 3276.7)
21-22       | Theta electrical          | uint16  | 0-65535 = 0-2π rad
23-24       | Speed command             | int16   | ±2000 RPM
25-26       | Vd (voltage command)      | int16   | ±5 V (scaled 6553.4)
27-28       | Vq (voltage command)      | int16   | ±5 V (scaled 6553.4)
29          | State echo                | uint8   | 0=STOP, 1=OL60, 5=CL100, ...
30          | Gate enable               | uint8   | 0=OFF, 1=ON
31          | End byte [69]             | uint8   | Fixed 0x45
```

## Step 1: Prepare FOC_Hardware_Deployment.slx

### 1.1 Copy the Model
```matlab
% In MATLAB command window:
copyfile('FOC_Hardware_Deployment.slx', 'FOC_Hardware_Deployment_Dashboard.slx');
open FOC_Hardware_Deployment_Dashboard.slx
```

### 1.2 Identify Where FOC Signals Live

These signals come from inside the FOC control loop (typically in a subsystem):
- **Phase currents Ia, Ib, Ic**: From ADC blocks → after current sensing scaling
- **d/q currents Id, Iq**: From the Clarke+Park transform section
- **Rotor angle theta_e**: From encoder position + offset calculation
- **Voltage commands Vd, Vq**: From the SVPWM (Space Vector PWM) modulator input
- **Speed command**: From your command state machine or reference input
- **Gate enable**: From the overall gate drive enable signal

**Action**: Explore inside `FOC_Motor_Control` subsystem to locate these blocks/signals.

### 1.3 Export Signals (Choose One Method)

**Option A: Use From/Goto Blocks** (Easier)
- Right-click on each signal → Create → Goto
- Name each goto: `Id_signal`, `Iq_signal`, `theta_e_signal`, etc.
- Then create corresponding From blocks in the telemetry section

**Option B: Add Output Ports** (Cleaner)
- Right-click FOC_Motor_Control subsystem → Edit
- Add output ports for: Id, Iq, theta_e, Vd, Vq, speed_cmd
- Connect internal signals to these ports
- Save and exit

## Step 2: Create Enhanced Telemetry Block

### 2.1 Add New MATLAB Function Block

1. In your model at root level, add a **MATLAB Function** block
2. Double-click to edit
3. Change the function signature to:

```matlab
function tx_packet = Format_Telemetry_FOC(...
    qep_cnt, speed_rpm, adc_a, adc_b, adc_c, adc_vdc, spi_raw, ...
    Id, Iq, theta_e_rad, Vd, Vq, spd_cmd, state_echo, gate_enable)
```

### 2.2 Paste the Function Body

```matlab
    % Pack all signals into 32-byte telemetry packet
    
    % Convert to appropriate types
    sync_bytes = uint8([170, 85, 170]);
    qep_u16 = uint16(qep_cnt);
    spd_i16 = typecast(int16(round(speed_rpm)), 'uint16');
    adc_a_u16 = uint16(adc_a);
    adc_b_u16 = uint16(adc_b);
    adc_c_u16 = uint16(adc_c);
    adc_vdc_u16 = uint16(adc_vdc);
    spi_u16 = uint16(spi_raw);
    
    % Scale currents to int16 range: ±10A → ±32767 counts
    % Scaling factor: 32767 / 10 = 3276.7
    Id_i16 = typecast(int16(round(Id * 3276.7)), 'uint16');
    Iq_i16 = typecast(int16(round(Iq * 3276.7)), 'uint16');
    
    % Convert angle from radians to uint16: 0-2π → 0-65535
    theta_u16 = uint16(mod(theta_e_rad, 2*pi) / (2*pi) * 65535);
    
    % Scale voltages to int16 range: ±5V → ±32767 counts
    % Scaling factor: 32767 / 5 = 6553.4
    Vd_i16 = typecast(int16(round(Vd * 6553.4)), 'uint16');
    Vq_i16 = typecast(int16(round(Vq * 6553.4)), 'uint16');
    
    spd_cmd_i16 = typecast(int16(round(spd_cmd)), 'uint16');
    state = uint8(state_echo);
    gate = uint8(gate_enable);
    end_byte = uint8(69);
    
    % Assemble 32-byte packet
    tx_packet = uint8([
        sync_bytes(1); sync_bytes(2); sync_bytes(3);
        bitshift(qep_u16, -8); bitand(qep_u16, 255);
        bitshift(spd_i16, -8); bitand(spd_i16, 255);
        bitshift(adc_a_u16, -8); bitand(adc_a_u16, 255);
        bitshift(adc_b_u16, -8); bitand(adc_b_u16, 255);
        bitshift(adc_c_u16, -8); bitand(adc_c_u16, 255);
        bitshift(adc_vdc_u16, -8); bitand(adc_vdc_u16, 255);
        bitshift(spi_u16, -8); bitand(spi_u16, 255);
        bitshift(Id_i16, -8); bitand(Id_i16, 255);
        bitshift(Iq_i16, -8); bitand(Iq_i16, 255);
        bitshift(theta_u16, -8); bitand(theta_u16, 255);
        bitshift(spd_cmd_i16, -8); bitand(spd_cmd_i16, 255);
        bitshift(Vd_i16, -8); bitand(Vd_i16, 255);
        bitshift(Vq_i16, -8); bitand(Vq_i16, 255);
        state; gate; end_byte
    ]);
end
```

### 2.3 Connect Inputs

Connect these signals to the Format_Telemetry_FOC block:
1. From current sensing: `adc_a`, `adc_b`, `adc_c`
2. From voltage sensing: `adc_vdc`
3. From encoders: `qep_cnt`, `spi_raw`
4. From FOC loop (use From/Goto blocks):
   - `Id_signal` → `Id` input
   - `Iq_signal` → `Iq` input
   - `theta_e_signal` → `theta_e_rad` input
   - `Vd_signal` → `Vd` input
   - `Vq_signal` → `Vq` input
   - `spd_cmd_signal` → `spd_cmd` input
   - `state_signal` → `state_echo` input
   - `gate_signal` → `gate_enable` input

## Step 3: Update Serial TX

Replace the old telemetry block connection:

**Old (17 bytes)**:
```
Format_Telemetry output [17 bytes] → SCI_Tx input
```

**New (32 bytes)**:
```
Format_Telemetry_FOC output [32 bytes] → SCI_Tx input
```

Make sure SCI_Tx is configured to output uint8 data.

## Step 4: Deploy to Hardware

1. Build and deploy `FOC_Hardware_Deployment_Dashboard.slx` to LAUNCHXL-F28069M
2. Verify it compiles without errors
3. Don't panic if you see warnings about expanded telemetry—that's normal

## Step 5: Run the Enhanced Dashboard

In MATLAB command window:
```matlab
% Make sure you're in the MyFOC code directory
cd 'c:\96v_gan_humanoid_drive\phase1\simulation\MyFOC code'

% Run the enhanced dashboard
Enhanced_Telemetry_Dashboard_FOC();
```

The dashboard will:
- Connect to COM7 at 5625000 baud
- Wait for packets from the hardware
- Display 9 plots showing all FOC signals
- Update in real-time as the motor runs

## What You'll See

### During Open-Loop 60 RPM Spin:
- **Phase Currents**: Should show symmetric, 120° phase-shifted sinusoids
- **d/q Currents**: Id ≈ constant (alignment current), Iq ≈ near zero
- **Voltage Commands**: Vd/Vq should be mostly constant
- **Rotor Angle**: Should increase linearly (constant speed)
- **Speed**: Should stabilize around 60 RPM

### During Closed-Loop FOC at 100 RPM:
- **Phase 1 (Ramp, 0-1.5s)**: Speed increases to target, Id/Iq stabilize
- **Phase 2 (Steady-state, >1.5s)**: Speed = reference, PI controller maintains torque
- **Speed Error**: Should drop to near zero (PI control working)
- **Vd/Vq**: Will vary as PI controller adjusts torque

### If Motor Stalls:
- **Imag** (current magnitude) will be high (1-5A) but speed won't increase
- **Speed Error** will be large (command - actual ≈ 100 RPM)
- Check: Is offset correct? Is gate enabled? Are voltage commands reasonable?

## Troubleshooting

### "No packets received"
- Check COM7 is correct (run `serialportlist('available')`)
- Check LAUNCHXL is programmed and running
- Check gate enable signal is active

### "Ia/Ib/Ic look wrong (not 120° phase-shifted)"
- ADC offset values may be incorrect for your setup
- Check current sensor operation with multimeter
- Verify offset_a, offset_b, offset_c values match your bias voltage

### "Id/Iq values don't look right"
- Check that Clarke/Park transform has correct encoder feedback
- Verify encoder offset is correct (should be ~0.26 from your earlier measurement)
- Check rotor angle signal matches electrical angle

### "Motor won't reach closed-loop speed"
- This could indicate encoder offset still not perfectly aligned
- Try manual offset tuning: ±0.02 rad steps
- Or run the open-loop diagnostic to re-measure offset

## Next: Serial Performance Verification

Current bandwidth usage:
- Packet size: 32 bytes
- Update rate: Should be same as FOC loop rate (check your PWM frequency)
- Example: 1 kHz FOC loop × 32 bytes = 32 kB/s = 256 kb/s ≈ 4.5% of 5625 kb/s
- **Result**: Very safe, with room for even more signals if needed

## Signal Scaling Reference

When extracting values from the Dashboard:

```matlab
% From 32-byte packet:
Id_amps = double(typecast(uint16([pkt(18), pkt(19)]), 'int16')) / 3276.7;
Iq_amps = double(typecast(uint16([pkt(20), pkt(21)]), 'int16')) / 3276.7;
Vd_volts = double(typecast(uint16([pkt(26), pkt(27)]), 'int16')) / 6553.4;
Vq_volts = double(typecast(uint16([pkt(28), pkt(29)]), 'int16')) / 6553.4;
theta_rad = double(pkt(22)*256 + pkt(23)) / 65535 * 2 * pi;
spd_rpm = double(typecast(uint16([pkt(6), pkt(7)]), 'int16'));
```

## Summary Checklist

- [ ] Copied FOC_Hardware_Deployment.slx → FOC_Hardware_Deployment_Dashboard.slx
- [ ] Located Id, Iq, theta_e, Vd, Vq signals in FOC loop
- [ ] Created/updated Format_Telemetry_FOC block with 32-byte packet
- [ ] Connected all FOC signals to Format_Telemetry_FOC
- [ ] Updated SCI_Tx to handle 32-byte packets
- [ ] Built and deployed to LAUNCHXL-F28069M
- [ ] Ran Enhanced_Telemetry_Dashboard_FOC.m
- [ ] Verified packets are arriving on Dashboard
- [ ] Checked phase currents, d/q currents, voltage commands
- [ ] Validated motor behavior (speed tracking, current behavior)

Once this is done, you'll have **complete visibility into every part of your FOC controller**. 🎯
