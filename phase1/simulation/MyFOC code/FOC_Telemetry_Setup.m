%% FOC Hardware Deployment + Dashboard Integration
% This script sets up FOC_Hardware_Deployment for Dashboard telemetry
% Captures: Phase currents, d/q currents, voltages, rotor angle, speed loop state

clear all; close all; clc;

fprintf('=== FOC Hardware Deployment Dashboard Integration ===\n\n');

%% NEW TELEMETRY PACKET STRUCTURE (32 bytes)
fprintf('Enhanced Telemetry Packet:\n');
fprintf('─────────────────────────────────────────────\n');
fprintf('Byte 0-2:     Sync bytes [170, 85, 0xAA]\n');
fprintf('Byte 3-4:     Rotor position (eQEP counts 0-4095) [u16]\n');
fprintf('Byte 5-6:     Speed (RPM, signed int16) [i16]\n');
fprintf('Byte 7-8:     Phase A current raw counts [u16]\n');
fprintf('Byte 9-10:    Phase B current raw counts [u16]\n');
fprintf('Byte 11-12:   Phase C current raw counts [u16]\n');
fprintf('Byte 13-14:   DC link voltage raw counts [u16]\n');
fprintf('Byte 15-16:   SPI rotor position (AS5047P 14-bit) [u16]\n');
fprintf('Byte 17-18:   d-axis current (Idq, signed, scaled) [i16]\n');
fprintf('Byte 19-20:   q-axis current (Iqq, signed, scaled) [i16]\n');
fprintf('Byte 21-22:   Rotor electrical angle (0-65535 = 0-2π) [u16]\n');
fprintf('Byte 23-24:   Speed reference/command (RPM, signed) [i16]\n');
fprintf('Byte 25-26:   Vd voltage command (scaled, signed) [i16]\n');
fprintf('Byte 27-28:   Vq voltage command (scaled, signed) [i16]\n');
fprintf('Byte 29:      State byte (0=STOP, 1=OL60, ..., 5=CL100, ...)\n');
fprintf('Byte 30:      Gate enable flag (0/1)\n');
fprintf('Byte 31:      End byte [0x45]\n');
fprintf('─────────────────────────────────────────────\n');
fprintf('Total: 32 bytes\n\n');

%% INSTRUCTIONS FOR MODIFICATION

fprintf('MANUAL SETUP REQUIRED:\n\n');

fprintf('Step 1: Create FOC_Hardware_Deployment_Dashboard.slx\n');
fprintf('────────────────────────────────────────────────\n');
fprintf('  1a. Copy FOC_Hardware_Deployment.slx → FOC_Hardware_Deployment_Dashboard.slx\n');
fprintf('  1b. Open FOC_Hardware_Deployment_Dashboard.slx in Simulink\n\n');

fprintf('Step 2: Extract FOC Signals\n');
fprintf('────────────────────────────────────────────────\n');
fprintf('  Inside the FOC control loop, you need to access:\n');
fprintf('    - Phase currents (Ia, Ib, Ic): From ADC blocks\n');
fprintf('    - d/q currents (Id, Iq): From the Clarke/Park transform section\n');
fprintf('    - Rotor angle (theta_e): From encoder + offset\n');
fprintf('    - Voltage commands (Vd, Vq): From SVPWM block\n');
fprintf('    - Speed command: From command interface\n\n');

fprintf('Step 3: Create Enhanced Format_Telemetry Block\n');
fprintf('────────────────────────────────────────────────\n');
fprintf('  Add a new MATLAB Function block with this signature:\n\n');
fprintf('  function tx_packet = Format_Telemetry_FOC(...\n');
fprintf('      qep_cnt, speed_rpm, adc_a, adc_b, adc_c, adc_vdc, spi_raw, ...\n');
fprintf('      Id, Iq, theta_e_rad, Vd, Vq, spd_cmd, state_echo, gate_enable)\n');
fprintf('    %% Convert all values to uint8 pairs\n');
fprintf('    sync_bytes = uint8([170, 85, 170]);\n');
fprintf('    qep_u16 = uint16(qep_cnt);\n');
fprintf('    spd_i16 = typecast(int16(round(speed_rpm)), ''uint16'');\n');
fprintf('    adc_a_u16 = uint16(adc_a);\n');
fprintf('    adc_b_u16 = uint16(adc_b);\n');
fprintf('    adc_c_u16 = uint16(adc_c);\n');
fprintf('    adc_vdc_u16 = uint16(adc_vdc);\n');
fprintf('    spi_u16 = uint16(spi_raw);\n');
fprintf('    %% Scale currents: convert amps to int16 (range ±10A = ±32767 counts)\n');
fprintf('    Id_i16 = typecast(int16(round(Id * 3276.7)), ''uint16'');\n');
fprintf('    Iq_i16 = typecast(int16(round(Iq * 3276.7)), ''uint16'');\n');
fprintf('    %% Convert angle from radians to 0-65535\n');
fprintf('    theta_u16 = uint16(mod(theta_e_rad, 2*pi) / (2*pi) * 65535);\n');
fprintf('    %% Scale voltages: ±5V = ±32767 counts\n');
fprintf('    Vd_i16 = typecast(int16(round(Vd * 6553.4)), ''uint16'');\n');
fprintf('    Vq_i16 = typecast(int16(round(Vq * 6553.4)), ''uint16'');\n');
fprintf('    spd_cmd_i16 = typecast(int16(round(spd_cmd)), ''uint16'');\n');
fprintf('    state = uint8(state_echo);\n');
fprintf('    gate = uint8(gate_enable);\n');
fprintf('    end_byte = uint8(0x45);\n\n');
fprintf('    %% Pack all into 32-byte packet\n');
fprintf('    tx_packet = uint8([\n');
fprintf('        sync_bytes(1:3);\n');
fprintf('        bitshift(qep_u16, -8), bitand(qep_u16, 255);\n');
fprintf('        bitshift(spd_i16, -8), bitand(spd_i16, 255);\n');
fprintf('        bitshift(adc_a_u16, -8), bitand(adc_a_u16, 255);\n');
fprintf('        bitshift(adc_b_u16, -8), bitand(adc_b_u16, 255);\n');
fprintf('        bitshift(adc_c_u16, -8), bitand(adc_c_u16, 255);\n');
fprintf('        bitshift(adc_vdc_u16, -8), bitand(adc_vdc_u16, 255);\n');
fprintf('        bitshift(spi_u16, -8), bitand(spi_u16, 255);\n');
fprintf('        bitshift(Id_i16, -8), bitand(Id_i16, 255);\n');
fprintf('        bitshift(Iq_i16, -8), bitand(Iq_i16, 255);\n');
fprintf('        bitshift(theta_u16, -8), bitand(theta_u16, 255);\n');
fprintf('        bitshift(spd_cmd_i16, -8), bitand(spd_cmd_i16, 255);\n');
fprintf('        bitshift(Vd_i16, -8), bitand(Vd_i16, 255);\n');
fprintf('        bitshift(Vq_i16, -8), bitand(Vq_i16, 255);\n');
fprintf('        state; gate; end_byte\n');
fprintf('    ]);\n');
fprintf('  end\n\n');

fprintf('Step 4: Connect Signals to Format_Telemetry Block\n');
fprintf('────────────────────────────────────────────────\n');
fprintf('  You need to route these signals from inside FOC_Motor_Control to the block:\n');
fprintf('    - Use From/Goto blocks to access internal FOC signals\n');
fprintf('    - Or add output ports to FOC subsystem\n');
fprintf('    - Signals needed:\n');
fprintf('        Id, Iq (from Park transform output)\n');
fprintf('        theta_e (from encoder + offset calculation)\n');
fprintf('        Vd, Vq (from PI controller outputs)\n');
fprintf('        spd_cmd (from command decoder)\n\n');

fprintf('Step 5: Update Serial TX Block\n');
fprintf('────────────────────────────────────────────────\n');
fprintf('  Change SCI_Tx to send Format_Telemetry_FOC output (32 bytes instead of 17)\n\n');

fprintf('Step 6: Update Dashboard Parser\n');
fprintf('────────────────────────────────────────────────\n');
fprintf('  Run: Enhanced_Telemetry_Dashboard_FOC.m\n');
fprintf('  This parses the 32-byte packets and displays FOC diagnostics\n\n');

%% HELPFUL CODE SNIPPETS

fprintf('\n=== CODE SNIPPETS ===\n\n');

fprintf('Scaling Reference:\n');
fprintf('─────────────────\n');
fprintf('Current (A) ← → int16 counts: multiply/divide by 3276.7\n');
fprintf('  Example: Id_amps = 5.5 A → Id_i16 = int16(5.5 * 3276.7) = 18022\n\n');

fprintf('Voltage (V) ← → int16 counts: multiply/divide by 6553.4\n');
fprintf('  Example: Vd_volts = 2.3 V → Vd_i16 = int16(2.3 * 6553.4) = 15073\n\n');

fprintf('Angle (rad) ← → uint16 counts: (angle / 2π) * 65535\n');
fprintf('  Example: theta = π/2 rad → theta_u16 = uint16(π/2 / 2π * 65535) = 16384\n\n');

fprintf('To extract from packet:\n');
fprintf('─────────────────────\n');
fprintf('Id_amps = double(typecast(uint16([packet(17), packet(18)]), ''int16'')) / 3276.7;\n');
fprintf('Vd_volts = double(typecast(uint16([packet(25), packet(26)]), ''int16'')) / 6553.4;\n');
fprintf('theta_rad = double(typecast(uint16([packet(21), packet(22)]), ''uint16'')) / 65535 * 2 * pi;\n\n');

fprintf('\n=== NEXT STEPS ===\n');
fprintf('1. Copy FOC_Hardware_Deployment.slx → FOC_Hardware_Deployment_Dashboard.slx\n');
fprintf('2. Modify Format_Telemetry to include FOC signals\n');
fprintf('3. Route Id, Iq, theta_e, Vd, Vq from FOC controller\n');
fprintf('4. Re-deploy to hardware\n');
fprintf('5. Run Enhanced_Telemetry_Dashboard_FOC.m\n');
