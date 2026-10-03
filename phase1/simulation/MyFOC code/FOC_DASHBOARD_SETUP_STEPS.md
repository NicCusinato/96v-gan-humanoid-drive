# FOC Hardware Deployment → Dashboard Integration
## Complete Setup Guide Using Simulink Agentic Toolkit MCP

---

## Overview

This guide walks you through integrating comprehensive telemetry into **FOC_Hardware_Deployment.slx** using the Simulink Agentic Toolkit (SATK) MCP server to create **FOC_Hardware_Deployment_Dashboard.slx**.

**What you'll accomplish:**
- ✓ Copy the FOC model to a new "Dashboard" version
- ✓ Add a Format_Telemetry_FOC MATLAB Function block
- ✓ Wire 15 FOC signals to the telemetry formatter
- ✓ Connect the 32-byte telemetry packet to the SCI serial output
- ✓ Deploy to LAUNCHXL-F28069M
- ✓ Run the Enhanced_Telemetry_Dashboard_FOC.m to visualize all 9 metrics

---

## Step 1: Create the Dashboard Model (Manual)

### 1A: Copy the base model
```
Source:  c:\96v_gan_humanoid_drive\phase1\simulation\MyFOC code\FOC_Hardware_Deployment.slx
Dest:    c:\96v_gan_humanoid_drive\phase1\simulation\MyFOC code\FOC_Hardware_Deployment_Dashboard.slx
```

**How:**
- Open Windows Explorer
- Navigate to the MyFOC code folder
- Right-click FOC_Hardware_Deployment.slx → Copy
- Right-click in empty space → Paste
- Rename to `FOC_Hardware_Deployment_Dashboard.slx`

### 1B: Open the new model in Simulink
```matlab
open_system('FOC_Hardware_Deployment_Dashboard')
```

---

## Step 2: Add the Telemetry Formatter Block

**Using Simulink Library Browser:**

1. With FOC_Hardware_Deployment_Dashboard.slx open, press **Ctrl+L** to open Library Browser
2. Search for "MATLAB Function" in the search box
3. Drag-and-drop **MATLAB Function** (from Simulink → Functions) onto the right side of your model
4. Name it: `Format_Telemetry_FOC`

**Or programmatically:**
```matlab
add_block('simulink/Functions/MATLAB Function', ...
    'FOC_Hardware_Deployment_Dashboard/Format_Telemetry_FOC', ...
    'Position', [1200, 100, 1500, 600]);
```

---

## Step 3: Populate the Format_Telemetry_FOC Block

### 3A: Double-click the Format_Telemetry_FOC block

### 3B: Copy the entire function code below and paste it into the editor:

```matlab
function tx_packet = Format_Telemetry_FOC(qep_cnt, speed_rpm, adc_a, adc_b, adc_c, adc_vdc, spi_raw, Id, Iq, theta_e_rad, Vd, Vq, spd_cmd, state_echo, gate_enable)
  %% 32-Byte Telemetry Packet Formatter for FOC Hardware Deployment
  %% Inputs: 15 signals from FOC controller
  %% Output: 32-byte packet ready for serial transmission
  
  % Prepare sync and data bytes
  sync_bytes = uint8([170, 85, 170]);
  qep_u16 = uint16(qep_cnt);
  spd_i16 = typecast(int16(round(speed_rpm)), 'uint16');
  adc_a_u16 = uint16(adc_a);
  adc_b_u16 = uint16(adc_b);
  adc_c_u16 = uint16(adc_c);
  adc_vdc_u16 = uint16(adc_vdc);
  spi_u16 = uint16(spi_raw);
  
  % Scale d/q currents: ±10A range → ±32767 counts (factor 3276.7)
  Id_i16 = typecast(int16(round(Id * 3276.7)), 'uint16');
  Iq_i16 = typecast(int16(round(Iq * 3276.7)), 'uint16');
  
  % Convert rotor electrical angle: 0-2π rad → 0-65535 counts
  theta_u16 = uint16(mod(theta_e_rad, 2*pi) / (2*pi) * 65535);
  
  % Scale voltage commands: ±5V range → ±32767 counts (factor 6553.4)
  Vd_i16 = typecast(int16(round(Vd * 6553.4)), 'uint16');
  Vq_i16 = typecast(int16(round(Vq * 6553.4)), 'uint16');
  
  % Prepare control signals
  spd_cmd_i16 = typecast(int16(round(spd_cmd)), 'uint16');
  state = uint8(state_echo);
  gate = uint8(gate_enable);
  end_byte = uint8(69);
  
  % Pack all values into 32-byte array [SYNC(3) + DATA(28) + END(1)]
  tx_packet = uint8([ ...
      sync_bytes(1); sync_bytes(2); sync_bytes(3); ...
      bitshift(qep_u16, -8); bitand(qep_u16, 255); ...
      bitshift(spd_i16, -8); bitand(spd_i16, 255); ...
      bitshift(adc_a_u16, -8); bitand(adc_a_u16, 255); ...
      bitshift(adc_b_u16, -8); bitand(adc_b_u16, 255); ...
      bitshift(adc_c_u16, -8); bitand(adc_c_u16, 255); ...
      bitshift(adc_vdc_u16, -8); bitand(adc_vdc_u16, 255); ...
      bitshift(spi_u16, -8); bitand(spi_u16, 255); ...
      bitshift(Id_i16, -8); bitand(Id_i16, 255); ...
      bitshift(Iq_i16, -8); bitand(Iq_i16, 255); ...
      bitshift(theta_u16, -8); bitand(theta_u16, 255); ...
      bitshift(spd_cmd_i16, -8); bitand(spd_cmd_i16, 255); ...
      bitshift(Vd_i16, -8); bitand(Vd_i16, 255); ...
      bitshift(Vq_i16, -8); bitand(Vq_i16, 255); ...
      state; gate; end_byte ...
  ]);
end
```

### 3C: Click "OK" or close the MATLAB Function dialog

---

## Step 4: Wire the Input Signals to Format_Telemetry_FOC

The Format_Telemetry_FOC block now has **15 input ports** and **1 output port**.

| Port | Signal Name | Source Block | Notes |
|------|-------------|--------------|-------|
| **u1** | qep_cnt | eQEP block | Rotor position (4096 CPR) |
| **u2** | speed_rpm | Speed calculation | Derived from eQEP |
| **u3** | adc_a | ADC_Ia | Phase A current ADC |
| **u4** | adc_b | ADC_Ib | Phase B current ADC |
| **u5** | adc_c | ADC_Ic | Phase C current ADC |
| **u6** | adc_vdc | ADC_Vbus | DC link voltage ADC |
| **u7** | spi_raw | AS5047P read | SPI encoder position |
| **u8** | Id | FOC_Motor_Control (inside) | d-axis current from Park transform |
| **u9** | Iq | FOC_Motor_Control (inside) | q-axis current from Park transform |
| **u10** | theta_e_rad | FOC_Motor_Control (inside) | Electrical angle |
| **u11** | Vd | FOC_Motor_Control (inside) | d-axis voltage command |
| **u12** | Vq | FOC_Motor_Control (inside) | q-axis voltage command |
| **u13** | spd_cmd | Speed command input | Target speed setpoint |
| **u14** | state_echo | Open_Loop_Controller (inside) | State machine byte (STOP/OL60/CL/RAMP) |
| **u15** | gate_enable | Gate_Enable block | Gate driver enable flag |

### 4A: Wire External Signals (u1-u7, u13-u15)
These are easily visible at the root level:

1. Right-click on ADC_Ia block output → **Create** → **Goto**
   - Name it: `adc_a_signal`
2. In root level, right-click → **Create** → **From**
   - Name it: `adc_a_signal`
   - Connect to Format_Telemetry_FOC **u3**
3. Repeat for **adc_b**, **adc_c**, **adc_vdc**, **speed_rpm**

### 4B: Wire Internal FOC Signals (u8-u12, u14)
These require navigation inside subsystems:

**For Id, Iq, theta_e, Vd, Vq:**
1. Double-click the **FOC_Motor_Control** subsystem (or wherever your FOC controller is)
2. Find the Park Transform output (should show Id/Iq)
3. Right-click on the Id output → **Create** → **Goto**
   - Name it: `Id_signal`
4. Exit subsystem, create **From** block at root, connect to **u8**
5. Repeat for Iq, theta_e, Vd, Vq

**For state_echo:**
1. Find the **Open_Loop_Controller** subsystem
2. Locate the state machine output (should be a uint8)
3. Create **Goto** block named: `state_signal`
4. Create **From** block at root, connect to **u14**

---

## Step 5: Wire the Telemetry Output to Serial

### 5A: Find your SCI serial transmission block
Look for: **SCI_Tx** or **c28xsci_tx** or similar

### 5B: Connect Format_Telemetry_FOC output (y1) to SCI_Tx input
- Click on **Format_Telemetry_FOC** output port (y1)
- Drag and drop line to **SCI_Tx** input
- Or use Goto/From pattern:
  - Right-click Format_Telemetry_FOC y1 → **Create** → **Goto**
  - Name: `telemetry_packet`
  - Create **From** block, connect to SCI_Tx input

---

## Step 6: Build and Deploy

### 6A: Verify model integrity
```matlab
slcheck('FOC_Hardware_Deployment_Dashboard')  % Check for block errors
```

### 6B: Build the model
- Ctrl+B (or **Tools** → **Build**)
- This generates the .out file for the LAUNCHXL

### 6C: Program the board
- Connect LAUNCHXL-F28069M via JTAG
- In Simulink, click **Tools** → **Run on Target Hardware** (or similar)
- Or use **Processor in the Loop (PIL)** if available

---

## Step 7: Run the Enhanced Dashboard

Once the model is deployed and running on hardware:

```matlab
% In MATLAB command window, run:
Enhanced_Telemetry_Dashboard_FOC()
```

This will:
- Auto-connect to COM7 at 5625000 baud
- Parse 32-byte packets in real-time
- Display 9 subplots:
  1. Phase currents (Ia, Ib, Ic)
  2. d/q axis currents (Id, Iq)
  3. Current magnitude (|I|)
  4. d/q axis voltages (Vd, Vq)
  5. Motor position (eQEP vs SPI encoder)
  6. Electrical angle (θ_e)
  7. Speed actual vs command
  8. Speed error
  9. DC link voltage (Vdc)

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| **"Block not found" error** | Make sure all Goto/From blocks are properly named and connected |
| **Dashboard shows zeros** | Check that: (1) serial cable is connected, (2) baud rate is 5625000, (3) model is running on hardware |
| **Missing internal signals** | Use `model_read` to explore FOC_Motor_Control subsystem structure and find where Id/Iq/theta_e originate |
| **Build fails** | Run `slcheck('FOC_Hardware_Deployment_Dashboard')` to see structural issues |
| **JTAG connection fails** | Verify board is in debug mode; check USB cable; restart Simulink if needed |

---

## Bandwidth Verification

**Telemetry packet size:** 32 bytes  
**Update rate:** 1 kHz (FOC loop frequency)  
**Serial bandwidth:** 32 bytes × 1000 Hz × 8 bits/byte = 256 kb/s  
**Available bandwidth:** 5625000 baud = 5625 kb/s  
**Utilization:** 256 / 5625 = **4.5%** ✓ (Plenty of headroom)

---

## Key Design Parameters

### Scaling Factors (in Format_Telemetry_FOC function)
- **d/q Currents:** ±10A range mapped to ±32767 counts → factor **3276.7**
- **Voltages (Vd/Vq):** ±5V range mapped to ±32767 counts → factor **6553.4**
- **Rotor Angle:** 0-2π rad mapped to 0-65535 counts → factor **65535/(2π)**

### Packet Structure (32 bytes total)
```
Byte 0-2:   Sync markers [170, 85, 170]
Byte 3-4:   eQEP position (u16, 0-4095)
Byte 5-6:   Speed (i16, ±2000 RPM)
Byte 7-14:  Phase currents Ia/Ib/Ic + Vdc (4× u16 each)
Byte 15-16: SPI position (u16)
Byte 17-20: Id/Iq (2× i16, scaled)
Byte 21-22: Rotor electrical angle (u16)
Byte 23-24: Speed command (i16)
Byte 25-28: Vd/Vq (2× i16, scaled)
Byte 29:    State byte (STOP=0, OL60=1, CL=2, RAMP=3)
Byte 30:    Gate enable (0=OFF, 1=ON)
Byte 31:    End marker [69]
```

---

## Files Reference

| File | Purpose |
|------|---------|
| `FOC_Hardware_Deployment_Dashboard.slx` | Your working model (created in Step 1) |
| `Enhanced_Telemetry_Dashboard_FOC.m` | Real-time plotting dashboard (run in Step 7) |
| `FOC_Motor_Control_Params.m` | Motor init script (use existing) |
| `FOC_DASHBOARD_INTEGRATION_GUIDE.md` | Detailed reference guide |
| `FOC_SIGNAL_EXTRACTION_REFERENCE.md` | Signal location reference |

---

## Next Steps After Successful Deployment

1. ✓ Monitor motor behavior in real-time via Enhanced_Telemetry_Dashboard_FOC
2. ✓ Tune FOC PI controller gains based on live Id/Iq feedback
3. ✓ Log telemetry to file for post-test analysis
4. ✓ Integrate with higher-level humanoid controller

---

**Questions?** Refer to FOC_DASHBOARD_INTEGRATION_GUIDE.md for more detail, or examine the existing Hardware_Diagnostics_Test.slx as a working example of telemetry integration.
