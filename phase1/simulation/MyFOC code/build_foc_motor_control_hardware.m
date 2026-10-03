%% build_foc_motor_control_hardware.m
% Automatically creates FOC_Motor_Control_Hardware.slx from FOC_Motor_Control.slx
% Fully aligns:
%  1. Speed units (rad/s in PI controller, RPM in telemetry)
%  2. Current scaling (ACS37003 12 mV/A on EPC91200 + boot-time auto-zero offset)
%  3. Voltage telemetry range (+/-100V scaled by 327.67, eliminating int16 overflow)
%  4. C2000 F28069M LaunchPad Site 2 (J5-J8) ePWM, ADC, eQEP, and SCI Transmit (115200 baud)
%  5. COMPLETE HARDWARE SAFEGUARDING:
%     - Switch4 clamps PWM counts to [0,0,0] (CMPA=0, High-Side FETs 100% OFF) when stopped
%     - ADCINB7 physical DC link voltage sensing with 12.0V interlock (prevents runaway if unpowered)
%     - Soft-start speed acceleration rate limiter (50 RPM/sec)
%     - Current clamp tightened to +/- 1.2A for bench DC supply safety

fprintf('=================================================================\n');
fprintf('  BUILDING FOC_MOTOR_CONTROL_HARDWARE.SLX (WITH SAFETY INTERLOCKS)\n');
fprintf('=================================================================\n\n');

src_model = 'FOC_Motor_Control';
dst_model = 'FOC_Motor_Control_Hardware';

% Ensure base workspace parameters are loaded from FOC_Motor_Control_Params.m
run('FOC_Motor_Control_Params.m');
if ~exist('target', 'var') || ~isfield(target, 'PWM_Counter_Period')
    target.ClockFreq_Hz = 90e6;
    PWM_frequency = 20e3;
    target.PWM_Counter_Period = target.ClockFreq_Hz / (2 * PWM_frequency); % 2250
    assignin('base', 'target', target);
    assignin('base', 'PWM_frequency', PWM_frequency);
    assignin('base', 'PWM_period_counts', target.PWM_Counter_Period);
end

% 1. Close open models if loaded
if bdIsLoaded(dst_model), close_system(dst_model, 0); end
if ~bdIsLoaded(src_model), load_system(src_model); end

% 2. Save copy as destination model
fprintf('Step 1: Copying %s -> %s...\n', src_model, dst_model);
save_system(src_model, dst_model);

% 3. Configure Hardware Board & Discrete Solver
fprintf('Step 2: Configuring target hardware and solver...\n');
set_param(dst_model, 'HardwareBoard', 'TI Piccolo F28069M LaunchPad');
set_param(dst_model, 'SystemTargetFile', 'ert.tlc');
set_param(dst_model, 'SolverType', 'Fixed-step');
set_param(dst_model, 'Solver', 'FixedStepDiscrete');
set_param(dst_model, 'FixedStep', '50e-6'); % 20 kHz ISR rate
set_param(dst_model, 'StopTime', 'inf');

% Configure SCI_A at 115200 Baud
cs = getActiveConfigSet(dst_model);
data = codertarget.data.getData(cs);
data.SCI_A.UserBaudRate = '115200';
data.SCI_A.BaudRate = 114796;
data.SCI_A.BaudRatePrescaler = 97;
codertarget.data.setData(cs, data);

% 4. Delete Physical Simulation & Simscape Blocks
fprintf('Step 3: Removing Simscape physical simulation blocks and redundant routing...\n');
s_blks = find_system(dst_model, 'SearchDepth', 1, 'BlockType', 'SimscapeBlock');
for i = 1:length(s_blks)
    delete_block(s_blks{i});
end

extra_blks = {'Solver Configuration', 'Gate Driver', 'PWM Generator', ...
              'PS-Simulink Converter', 'PS-Simulink Converter1', ...
              'PS-Simulink Converter2', 'PS-Simulink Converter3', ...
              'Simulink-PS Converter', 'Scope1', 'Scope3', 'i-Tracking', ...
              'Ideal Angular Velocity Source', 'Ideal Torque Sensor', ...
              'Ideal Rotational Motion Sensor', 'Thermal Mass', ...
              'Thermal Reference', 'Convective Heat Transfer', ...
              'Electrical Reference', 'Mechanical Rotational Reference', ...
              'Mechanical Rotational Reference1', 'Voltage Source', ...
              'Current Sensor (Three-Phase)', 'Converter (Three-Phase)', ...
              'PMSM', 'Resistor', 'Inductor', 'Constant1', ...
              'Goto10', 'Goto11', 'Goto12', 'Goto13', 'Goto14', 'Goto15', 'Goto9', 'Gain1', ...
              'From17', 'From18', 'From20', 'From21', 'From16', 'Mux2', 'Mux3', 'Mux4', 'Mux5'};
for i = 1:length(extra_blks)
    b = [dst_model '/' extra_blks{i}];
    if ~isempty(find_system(dst_model, 'SearchDepth', 1, 'Name', extra_blks{i}))
        delete_block(b);
    end
end

% 5. Set Constant Blocks to single-precision & PIDs to Discrete-time with External Reset
fprintf('Step 4: Setting single-precision constants and discrete PIDs with auto-reset...\n');
const_list = {'Constant', 'Constant3', 'Constant4', 'Constant5', 'Constant7', 'i_d_ref'};
for i = 1:length(const_list)
    b = [dst_model '/' const_list{i}];
    if ~isempty(find_system(dst_model, 'SearchDepth', 1, 'Name', const_list{i}))
        set_param(b, 'OutDataTypeStr', 'single');
    end
end

pid_list = {'PID Controller', 'PID Controller1', 'PID Controller2'};
for i = 1:length(pid_list)
    b = [dst_model '/' pid_list{i}];
    set_param(b, 'TimeDomain', 'Discrete-time', 'SampleTime', '50e-6', 'ExternalReset', 'level');
end

% Set Sampling Delay sample time to inherited (-1)
if ~isempty(find_system(dst_model, 'SearchDepth', 1, 'Name', 'Sampling Delay'))
    set_param([dst_model '/Sampling Delay'], 'SampleTime', '-1');
end

% Add Is_Stopped comparator (gate_enable == 0) to hold PIDs in reset when stopped
add_block('simulink/Logic and Bit Operations/Compare To Zero', [dst_model '/Is_Stopped'], ...
    'relop', '==', 'Position', [420 120 460 140]);
add_block('simulink/Signal Routing/From', [dst_model '/From_gate_en_reset'], ...
    'GotoTag', 'gate_enable', 'Position', [340 120 400 140]);
add_line(dst_model, 'From_gate_en_reset/1', 'Is_Stopped/1');

% Wire Is_Stopped/1 to all 3 PID reset inports (Inport 2)
for i = 1:length(pid_list)
    add_line(dst_model, 'Is_Stopped/1', sprintf('%s/2', pid_list{i}));
end

% 6. Add Heartbeat LED & PWM Output with Switch4 Stop Gating
fprintf('Step 5: Adding C2000 hardware peripheral blocks and PWM stop gating...\n');
calib_model = 'Official_Encoder_Calib_Site2';
if ~bdIsLoaded(calib_model), load_system(calib_model); end

diag_model = 'Hardware_Diagnostics_Test';
if ~bdIsLoaded(diag_model), load_system(diag_model); end

add_block([calib_model '/Heartbeat LED'], [dst_model '/Heartbeat LED'], 'Position', [50 50 150 100]);
add_block([calib_model '/Offset Calculation/PWM Output'], [dst_model '/PWM Output'], 'Position', [1350 250 1500 350]);

% Add Scale_to_PWM_Counts (Gain 2250)
add_block('simulink/Math Operations/Gain', [dst_model '/Scale_to_PWM_Counts'], ...
    'Gain', '2250', 'Position', [1080 260 1130 290]);
add_line(dst_model, 'Saturation/1', 'Scale_to_PWM_Counts/1');

% Add Zero Constant for PWM stop state (CMPA = 0 -> High-Side GaN FETs completely OFF)
add_block('simulink/Sources/Constant', [dst_model '/Constant_Zero_PWM'], ...
    'Value', 'single([0.0; 0.0; 0.0])', 'OutDataTypeStr', 'single', ...
    'Position', [1080 320 1180 340]);

% Add From block for gate_enable to control Switch4
add_block('simulink/Signal Routing/From', [dst_model '/From_gate_en_pwm'], ...
    'GotoTag', 'gate_enable', 'Position', [1100 290 1170 305]);

% Add Switch4 between Scale_to_PWM_Counts and PWM Output
add_block('simulink/Signal Routing/Switch', [dst_model '/Switch4'], ...
    'Criteria', 'u2 >= Threshold', 'Threshold', '0.5', ...
    'Position', [1220 250 1260 340]);

% Wire Switch4: Inport 1 = Scale_to_PWM_Counts, Inport 2 = gate_enable, Inport 3 = Zero
add_line(dst_model, 'Scale_to_PWM_Counts/1', 'Switch4/1');
add_line(dst_model, 'From_gate_en_pwm/1', 'Switch4/2');
add_line(dst_model, 'Constant_Zero_PWM/1', 'Switch4/3');
add_line(dst_model, 'Switch4/1', 'PWM Output/1');

% 7. Add Feedback Subsystem (ADC Currents + ADCINB7 DC Bus + eQEP + Demux + Auto-Zero)
fprintf('Step 6: Adding Feedback subsystem with ACS37003 current and ADCINB7 Vdc sensing...\n');
fb = [dst_model '/Feedback'];
add_block('simulink/Ports & Subsystems/Subsystem', fb, 'Position', [50 200 220 440]);
delete_block([fb '/In1']); delete_block([fb '/Out1']);

% Current ADC (Phase A & B)
add_block([calib_model '/Offset Calculation/IA//IB Measurement'], [fb '/ADC'], 'Position', [50 50 180 100]);
add_block('simulink/Signal Routing/Demux', [fb '/Demux_ADC'], 'Outputs', '2', 'Position', [210 55 220 105]);

% eQEP Position Sensor
add_block([calib_model '/Offset Calculation/eQEP'], [fb '/eQEP'], 'Position', [50 130 180 180]);
set_param([fb '/eQEP'], 'useModule', 'eQEP1');
set_param([fb '/eQEP'], 'pcMaximumvalue', '4*1024 - 1');

% DC Bus Voltage ADC (BoosterPack Site 2: ADCINB7 via SOC2)
add_block([diag_model '/ADC_VDC'], [fb '/ADC_VDC'], 'Position', [50 210 180 260]);
add_block('simulink/Signal Routing/Demux', [fb '/Demux_VDC'], 'Outputs', '1', 'Position', [210 225 220 245]);

% Add Global Gotos inside Feedback for raw telemetry signals
add_block('simulink/Signal Routing/Goto', [fb '/Goto_adc_a'], ...
    'GotoTag', 'adc_a', 'TagVisibility', 'global', 'Position', [250 15 320 35]);
add_block('simulink/Signal Routing/Goto', [fb '/Goto_adc_b'], ...
    'GotoTag', 'adc_b', 'TagVisibility', 'global', 'Position', [250 35 320 55]);
add_block('simulink/Signal Routing/Goto', [fb '/Goto_qep_cnt'], ...
    'GotoTag', 'qep_cnt', 'TagVisibility', 'global', 'Position', [250 185 320 205]);
add_block('simulink/Signal Routing/Goto', [fb '/Goto_adc_vdc'], ...
    'GotoTag', 'adc_vdc', 'TagVisibility', 'global', 'Position', [250 225 320 245]);

fn_fb = sprintf('%s/Feedback_Processing', fb);
add_block('simulink/User-Defined Functions/MATLAB Function', fn_fb, 'Position', [270 50 430 180]);
sf = sfroot;
ch_fb = sf.find('Path', fn_fb, '-isa', 'Stateflow.EMChart');
fb_script = sprintf([...
    'function [i_abc, th_e, w_m, calib_done] = Feedback_Processing(u_meas_raw, v_meas_raw, raw_qep)\n', ...
    '%%#codegen\n', ...
    'i_abc = zeros(3, 1, ''single'');\n', ...
    'th_e = single(0);\n', ...
    'w_m = single(0);\n', ...
    'calib_done = single(0);\n', ...
    'persistent offset_u offset_v calib_count cal_complete prev_th_m w_m_flt\n', ...
    'if isempty(calib_count)\n', ...
    '    offset_u = single(2048.0);\n', ...
    '    offset_v = single(2048.0);\n', ...
    '    calib_count = single(0);\n', ...
    '    cal_complete = false;\n', ...
    '    prev_th_m = single(0);\n', ...
    '    w_m_flt = single(0);\n', ...
    'end\n', ...
    'N_CALIB = single(10000);\n', ...
    'u_meas = single(u_meas_raw);\n', ...
    'v_meas = single(v_meas_raw);\n', ...
    'if ~cal_complete\n', ...
    '    if calib_count < N_CALIB\n', ...
    '        calib_count = calib_count + single(1.0);\n', ...
    '        offset_u = offset_u + (u_meas - offset_u) / calib_count;\n', ...
    '        offset_v = offset_v + (v_meas - offset_v) / calib_count;\n', ...
    '    else\n', ...
    '        cal_complete = true;\n', ...
    '    end\n', ...
    'end\n', ...
    'K_I = single(3.3 / (4095.0 * 0.012));\n', ...
    'i_a = (u_meas - offset_u) * K_I;\n', ...
    'i_b = (v_meas - offset_v) * K_I;\n', ...
    'i_c = -(i_a + i_b);\n', ...
    'i_abc(1) = i_a;\n', ...
    'i_abc(2) = i_b;\n', ...
    'i_abc(3) = i_c;\n', ...
    'TWO_PI = single(6.283185307179586);\n', ...
    'th_m = (single(raw_qep) / single(4096.0)) * TWO_PI;\n', ...
    'TH_OFFSET = single(1.790708);\n', ...
    'th_e_unmod = single(21.0) * th_m + TH_OFFSET;\n', ...
    'th_e = mod(th_e_unmod, TWO_PI);\n', ...
    'if th_e < single(0)\n', ...
    '    th_e = th_e + TWO_PI;\n', ...
    'end\n', ...
    'd_th = th_m - prev_th_m;\n', ...
    'if d_th > single(3.14159265)\n', ...
    '    d_th = d_th - TWO_PI;\n', ...
    'elseif d_th < single(-3.14159265)\n', ...
    '    d_th = d_th + TWO_PI;\n', ...
    'end\n', ...
    'prev_th_m = th_m;\n', ...
    'w_m_raw = d_th * single(20000.0);\n', ...
    'ALPHA = single(0.03);\n', ...
    'w_m_flt = w_m_flt + ALPHA * (w_m_raw - w_m_flt);\n', ...
    'w_m = w_m_flt;\n', ...
    'if cal_complete\n', ...
    '    calib_done = single(1.0);\n', ...
    'else\n', ...
    '    calib_done = single(0.0);\n', ...
    'end\n']);
ch_fb.Script = fb_script;

% Add Outports to Feedback
add_block('simulink/Sinks/Out1', [fb '/i_abc'], 'Position', [500 60 530 75]);
add_block('simulink/Sinks/Out1', [fb '/th_e'], 'Position', [500 95 530 110]);
add_block('simulink/Sinks/Out1', [fb '/w_m'], 'Position', [500 130 530 145]);
add_block('simulink/Sinks/Out1', [fb '/calib_done'], 'Position', [500 165 530 180]);
add_block('simulink/Sinks/Out1', [fb '/vdc_raw'], 'Position', [500 230 530 245]);

add_line(fb, 'ADC/1', 'Demux_ADC/1');
add_line(fb, 'Demux_ADC/1', 'Feedback_Processing/1');
add_line(fb, 'Demux_ADC/2', 'Feedback_Processing/2');
add_line(fb, 'eQEP/1', 'Feedback_Processing/3');

add_line(fb, 'ADC_VDC/1', 'Demux_VDC/1');
add_line(fb, 'Demux_VDC/1', 'vdc_raw/1');

add_line(fb, 'Feedback_Processing/1', 'i_abc/1');
add_line(fb, 'Feedback_Processing/2', 'th_e/1');
add_line(fb, 'Feedback_Processing/3', 'w_m/1');
add_line(fb, 'Feedback_Processing/4', 'calib_done/1');

add_line(fb, 'Demux_ADC/1', 'Goto_adc_a/1');
add_line(fb, 'Demux_ADC/2', 'Goto_adc_b/1');
add_line(fb, 'eQEP/1', 'Goto_qep_cnt/1');
add_line(fb, 'Demux_VDC/1', 'Goto_adc_vdc/1');

% Main GOTO tags for Feedback outputs
add_block('simulink/Signal Routing/Goto', [dst_model '/Goto_i_abc_hw'], 'GotoTag', 'i_abc', 'Position', [260 205 330 225]);
add_block('simulink/Signal Routing/Goto', [dst_model '/Goto_th_e_hw'], 'GotoTag', 'th_e', 'Position', [260 245 330 265]);
add_block('simulink/Signal Routing/Goto', [dst_model '/Goto_w_m_hw'], 'GotoTag', 'w_m', 'Position', [260 285 330 305]);

add_line(dst_model, 'Feedback/1', 'Goto_i_abc_hw/1');
add_line(dst_model, 'Feedback/2', 'Goto_th_e_hw/1');
add_line(dst_model, 'Feedback/3', 'Goto_w_m_hw/1');

% 8. Add Command Subsystem (SCI Rx + Decoder + Vdc Interlock + Soft Ramp)
fprintf('Step 7: Adding Command decoder subsystem with DC bus interlock and soft ramp...\n');
cmd_sys = [dst_model '/Command'];
add_block('simulink/Ports & Subsystems/Subsystem', cmd_sys, 'Position', [50 470 220 620]);
delete_block([cmd_sys '/In1']); delete_block([cmd_sys '/Out1']);
add_block([calib_model '/Serial Receive/SCI Receive'], [cmd_sys '/SCI_Rx'], 'Position', [50 50 160 100]);
set_param([cmd_sys '/SCI_Rx'], 'sciModule', 'A', 'dataType', 'uint8', 'dataDim', '1');

cmd_fn = sprintf('%s/Decode_Command', cmd_sys);
add_block('simulink/User-Defined Functions/MATLAB Function', cmd_fn, 'Position', [220 50 390 180]);
ch_cmd = sf.find('Path', cmd_fn, '-isa', 'Stateflow.EMChart');
cmd_script = sprintf([...
    'function [w_ref, run_cmd, state_echo] = Decode_Command(rx_byte, calib_done, vdc_raw)\n', ...
    '%%#codegen\n', ...
    'persistent run_state target_w w_ref_ramp cur_state\n', ...
    'if isempty(run_state)\n', ...
    '    run_state  = single(0);\n', ...
    '    target_w   = single(0);\n', ...
    '    w_ref_ramp = single(0);\n', ...
    '    cur_state  = single(0);\n', ...
    'end\n', ...
    '%% 1. Physical DC Bus Voltage Calculation (ADCINB7 on BoosterPack Site 2)\n', ...
    'vdc_volts = single(vdc_raw) * single(3.3 * 46.4545 / 4095.0);\n', ...
    'dc_bus_ok = (vdc_volts >= single(12.0)); %% Require at least 12V DC bus to enable gates\n', ...
    '%% 2. Process incoming serial commands\n', ...
    'byte = uint8(rx_byte);\n', ...
    'if byte == uint8(49) || byte == uint8(1)      %% ''1'' -> 100 RPM\n', ...
    '    if dc_bus_ok\n', ...
    '        run_state = single(1);\n', ...
    '        target_w  = single(10.4719755); %% 100 * 2*pi/60 [rad/s]\n', ...
    '        cur_state = single(5);         %% State 5: CL FOC 100 RPM\n', ...
    '    end\n', ...
    'elseif byte == uint8(50) || byte == uint8(2)  %% ''2'' -> 200 RPM\n', ...
    '    if dc_bus_ok\n', ...
    '        run_state = single(1);\n', ...
    '        target_w  = single(20.943951);  %% 200 * 2*pi/60 [rad/s]\n', ...
    '        cur_state = single(6);         %% State 6: CL FOC 200 RPM\n', ...
    '    end\n', ...
    'elseif byte == uint8(48)                      %% ''0'' -> STOP\n', ...
    '    run_state  = single(0);\n', ...
    '    target_w   = single(0);\n', ...
    '    cur_state  = single(0);         %% State 0: STOPPED\n', ...
    'end\n', ...
    '%% 3. Safety Interlock: Force STOP if DC bus voltage is below threshold\n', ...
    'if ~dc_bus_ok\n', ...
    '    run_state  = single(0);\n', ...
    '    target_w   = single(0);\n', ...
    '    cur_state  = single(0);\n', ...
    '    w_ref_ramp = single(0);\n', ...
    'end\n', ...
    '%% 4. Soft-Start Acceleration Rate Limiter (50 RPM/sec = 5.236 rad/s^2)\n', ...
    '%% At 20 kHz (Ts = 50 us), max step = 5.235987756 * 50e-6 = 0.0002618 rad/s\n', ...
    'RAMP_STEP = single(0.0002617994);\n', ...
    'if run_state > single(0.5)\n', ...
    '    if w_ref_ramp < target_w\n', ...
    '        w_ref_ramp = min(w_ref_ramp + RAMP_STEP, target_w);\n', ...
    '    elseif w_ref_ramp > target_w\n', ...
    '        w_ref_ramp = max(w_ref_ramp - RAMP_STEP, target_w);\n', ...
    '    end\n', ...
    'else\n', ...
    '    w_ref_ramp = single(0);\n', ...
    'end\n', ...
    '%% 5. Output Gating\n', ...
    'if calib_done < single(0.5)\n', ...
    '    run_cmd    = single(0);\n', ...
    '    w_ref      = single(0);\n', ...
    '    state_echo = single(51);       %% State 51: STARTUP / CALIBRATION\n', ...
    'elseif ~dc_bus_ok\n', ...
    '    run_cmd    = single(0);\n', ...
    '    w_ref      = single(0);\n', ...
    '    state_echo = single(0);        %% State 0: STOPPED (DC BUS OFF)\n', ...
    'else\n', ...
    '    run_cmd    = run_state;\n', ...
    '    w_ref      = w_ref_ramp;\n', ...
    '    state_echo = cur_state;\n', ...
    'end\n']);
ch_cmd.Script = cmd_script;

add_block('simulink/Sources/In1', [cmd_sys '/calib_done'], 'Position', [50 115 80 130]);
add_block('simulink/Sources/In1', [cmd_sys '/vdc_raw'], 'Position', [50 155 80 170]);
add_block('simulink/Sinks/Out1', [cmd_sys '/w_ref'], 'Position', [450 65 480 80]);
add_block('simulink/Sinks/Out1', [cmd_sys '/run_cmd'], 'Position', [450 105 480 120]);
add_block('simulink/Sinks/Out1', [cmd_sys '/state_echo'], 'Position', [450 145 480 160]);

add_line(cmd_sys, 'SCI_Rx/1', 'Decode_Command/1');
add_line(cmd_sys, 'calib_done/1', 'Decode_Command/2');
add_line(cmd_sys, 'vdc_raw/1', 'Decode_Command/3');
add_line(cmd_sys, 'Decode_Command/1', 'w_ref/1');
add_line(cmd_sys, 'Decode_Command/2', 'run_cmd/1');
add_line(cmd_sys, 'Decode_Command/3', 'state_echo/1');

% Connect Feedback/4 (calib_done) -> Command/1
add_line(dst_model, 'Feedback/4', 'Command/1');

% Connect Feedback/5 (vdc_raw) -> Command/2
add_line(dst_model, 'Feedback/5', 'Command/2');

% Connect Command/2 (run_cmd) -> PWM Output/2 (EnPWM)
add_line(dst_model, 'Command/2', 'PWM Output/2');

% Add Gotos for Command outputs
add_block('simulink/Signal Routing/Goto', [dst_model '/Goto_w_ref_hw'], 'GotoTag', 'w_ref', 'Position', [260 480 330 500]);
add_block('simulink/Signal Routing/Goto', [dst_model '/Goto_gate_en_hw'], 'GotoTag', 'gate_enable', 'Position', [260 520 330 540]);
add_block('simulink/Signal Routing/Goto', [dst_model '/Goto_state_hw'], 'GotoTag', 'state_echo', 'Position', [260 560 330 580]);

add_line(dst_model, 'Command/1', 'Goto_w_ref_hw/1');
add_line(dst_model, 'Command/2', 'Goto_gate_en_hw/1');
add_line(dst_model, 'Command/3', 'Goto_state_hw/1');

% 9. Rewire Speed Loop Reference & Iq Clamp (+/- 1.2 A bench limit)
fprintf('Step 8: Aligning speed loop reference and Iq clamp (+/- 1.2A)...\n');
lh_sub5 = get_param([dst_model '/Subtract5'], 'PortHandles');
if lh_sub5.Inport(1) ~= -1
    l = get_param(lh_sub5.Inport(1), 'Line');
    if l ~= -1, delete_line(l); end
end
add_line(dst_model, 'Command/1', 'Subtract5/1');

% Add Iq Clamp Saturation block (+/- 1.2 A safe limit for bench power supply)
add_block('simulink/Discontinuities/Saturation', [dst_model '/Iq_Clamp'], ...
    'UpperLimit', '1.2', 'LowerLimit', '-1.2', ...
    'Position', [480 345 520 375]);

% Wire: Gain -> Iq_Clamp -> Subtract1:1
lh_sub1 = get_param([dst_model '/Subtract1'], 'PortHandles');
if lh_sub1.Inport(1) ~= -1
    l = get_param(lh_sub1.Inport(1), 'Line');
    if l ~= -1, delete_line(l); end
end
add_line(dst_model, 'Gain/1', 'Iq_Clamp/1');
add_line(dst_model, 'Iq_Clamp/1', 'Subtract1/1');

% 10. Add Telemetry Subsystem (32-byte packet, live Vdc, +/-100V scaling, 100 Hz decimation)
fprintf('Step 9: Adding 32-byte Telemetry Subsystem with live ADCINB7 DC bus voltage...\n');
telem = [dst_model '/Telemetry'];
add_block('simulink/Ports & Subsystems/Subsystem', telem, 'Position', [850 450 1150 670]);
delete_block([telem '/In1']); delete_block([telem '/Out1']);

% Load c280xlib for SCI Transmit
if ~bdIsLoaded('c280xlib'), load_system('c280xlib'); end

% Enabled Subsystem Serial_Send
send_sub = [telem '/Serial_Send'];
add_block('simulink/Ports & Subsystems/Enabled Subsystem', send_sub, 'Position', [450 80 580 180]);
delete_block([send_sub '/Out1']);
add_block('c280xlib/SCI Transmit', [send_sub '/SCI_Tx'], 'Position', [150 50 250 110]);
set_param([send_sub '/SCI_Tx'], 'sciModule', 'A', 'frameSize', '32', 'blockingMode', 'on');
add_line(send_sub, 'In1/1', 'SCI_Tx/1');

% Telemetry inputs inside subsystem via From blocks (including adc_vdc)
from_list = {'qep_cnt', 'w_m', 'adc_a', 'adc_b', 'adc_vdc', 'i_d', 'i_q', 'th_e', 'v_d', 'v_q', 'w_ref', 'state_echo', 'gate_enable'};
y_pos = 20;
for i = 1:length(from_list)
    tag = from_list{i};
    b = [telem '/From_' tag];
    add_block('simulink/Signal Routing/From', b, 'GotoTag', tag, 'Position', [40 y_pos 120 y_pos+18]);
    y_pos = y_pos + 26;
end

fn_telem = sprintf('%s/Format_Telemetry_FOC', telem);
add_block('simulink/User-Defined Functions/MATLAB Function', fn_telem, 'Position', [180 20 360 360]);
telem_script = strjoin({
    'function [tx_packet, send_flag] = Format_Telemetry_FOC(qep_cnt, w_m_rads, adc_a, adc_b, adc_vdc, Id, Iq, th_e, Vd, Vq, w_ref_rads, state_echo, gate_enable)'
    '%#codegen'
    'persistent dec_counter'
    'if isempty(dec_counter)'
    '    dec_counter = single(0);'
    'end'
    'dec_counter = dec_counter + single(1.0);'
    '% Decimate by 200: 20 kHz / 200 = 100 Hz serial rate'
    'if dec_counter >= single(200.0)'
    '    dec_counter = single(0);'
    '    send_flag = true;'
    'else'
    '    send_flag = false;'
    'end'
    'qep_u16 = uint16(qep_cnt);'
    '% Speed unit alignment: convert rad/s -> RPM for dashboard'
    'spd_rpm = w_m_rads * single(9.54929658551372);'
    'spd_i16 = typecast(int16(round(spd_rpm)), ''uint16'');'
    'adc_a_u16 = uint16(adc_a);'
    'adc_b_u16 = uint16(adc_b);'
    'adc_c_val = max(single(0), single(4096.0) - (single(adc_a) + single(adc_b)));'
    'adc_c_u16 = uint16(adc_c_val);'
    'adc_vdc_u16 = uint16(adc_vdc(1));'
    'spi_u16 = qep_u16;'
    'Id_i16 = typecast(int16(round(max(min(Id, single(10.0)), single(-10.0)) * single(3276.7))), ''uint16'');'
    'Iq_i16 = typecast(int16(round(max(min(Iq, single(10.0)), single(-10.0)) * single(3276.7))), ''uint16'');'
    'TWO_PI = single(6.283185307179586);'
    'theta_u16 = uint16(mod(th_e, TWO_PI) / TWO_PI * single(65535.0));'
    'spd_cmd_rpm = w_ref_rads * single(9.54929658551372);'
    'spd_cmd_i16 = typecast(int16(round(spd_cmd_rpm)), ''uint16'');'
    'Vd_i16 = typecast(int16(round(max(min(Vd, single(100.0)), single(-100.0)) * single(327.67))), ''uint16'');'
    'Vq_i16 = typecast(int16(round(max(min(Vq, single(100.0)), single(-100.0)) * single(327.67))), ''uint16'');'
    'tx_packet = zeros(32, 1, ''uint8'');'
    'tx_packet(1) = uint8(170);'
    'tx_packet(2) = uint8(85);'
    'tx_packet(3) = uint8(170);'
    'tx_packet(4) = uint8(bitshift(qep_u16, -8));'
    'tx_packet(5) = uint8(bitand(qep_u16, 255));'
    'tx_packet(6) = uint8(bitshift(spd_i16, -8));'
    'tx_packet(7) = uint8(bitand(spd_i16, 255));'
    'tx_packet(8) = uint8(bitshift(adc_a_u16, -8));'
    'tx_packet(9) = uint8(bitand(adc_a_u16, 255));'
    'tx_packet(10) = uint8(bitshift(adc_b_u16, -8));'
    'tx_packet(11) = uint8(bitand(adc_b_u16, 255));'
    'tx_packet(12) = uint8(bitshift(adc_c_u16, -8));'
    'tx_packet(13) = uint8(bitand(adc_c_u16, 255));'
    'tx_packet(14) = uint8(bitshift(adc_vdc_u16, -8));'
    'tx_packet(15) = uint8(bitand(adc_vdc_u16, 255));'
    'tx_packet(16) = uint8(bitshift(spi_u16, -8));'
    'tx_packet(17) = uint8(bitand(spi_u16, 255));'
    'tx_packet(18) = uint8(bitshift(Id_i16, -8));'
    'tx_packet(19) = uint8(bitand(Id_i16, 255));'
    'tx_packet(20) = uint8(bitshift(Iq_i16, -8));'
    'tx_packet(21) = uint8(bitand(Iq_i16, 255));'
    'tx_packet(22) = uint8(bitshift(theta_u16, -8));'
    'tx_packet(23) = uint8(bitand(theta_u16, 255));'
    'tx_packet(24) = uint8(bitshift(spd_cmd_i16, -8));'
    'tx_packet(25) = uint8(bitand(spd_cmd_i16, 255));'
    'tx_packet(26) = uint8(bitshift(Vd_i16, -8));'
    'tx_packet(27) = uint8(bitand(Vd_i16, 255));'
    'tx_packet(28) = uint8(bitshift(Vq_i16, -8));'
    'tx_packet(29) = uint8(bitand(Vq_i16, 255));'
    'tx_packet(30) = uint8(state_echo);'
    'tx_packet(31) = uint8(gate_enable);'
    'tx_packet(32) = uint8(69);'
}, newline);
ch_telem.Script = telem_script;

% Connect From blocks to Format_Telemetry_FOC inputs
for i = 1:length(from_list)
    add_line(telem, ['From_' from_list{i} '/1'], sprintf('Format_Telemetry_FOC/%d', i));
end

% Connect Format_Telemetry_FOC/1 (tx_packet) -> Serial_Send/1 (In1)
add_line(telem, 'Format_Telemetry_FOC/1', 'Serial_Send/1');

% Connect Format_Telemetry_FOC/2 (send_flag) -> Serial_Send/Enable
add_line(telem, 'Format_Telemetry_FOC/2', 'Serial_Send/Enable');

% 11. Save and validate
fprintf('Step 10: Saving %s...\n', dst_model);
save_system(dst_model);
set_param(dst_model, 'SimulationCommand', 'update');
fprintf('SUCCESS: %s.slx created, assembled, and verified with ZERO compilation errors!\n\n', dst_model);
