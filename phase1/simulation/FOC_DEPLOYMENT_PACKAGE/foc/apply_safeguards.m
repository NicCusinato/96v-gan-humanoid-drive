% apply_hardware_safeguards_and_telemetry.m
% Automatically configures:
% 1. Telemetry in Target Model (streams Iq_measured via SCI Data_Logging)
% 2. Rate Limiter in Host Model (prevents instantaneous step current spikes)

clear; clc;
disp('================================================================');
disp('  APPLYING SAFEGUARDS: TELEMETRY & ACCELERATION RATE LIMITER');
disp('================================================================');

%% 1. Update Target Model Telemetry
target_model = 'mcb_pmsm_foc_qep_f28069LaunchPad';
load_system(target_model);

target_sys = [target_model '/Current Control'];
cs_blk = [target_sys '/Control_System'];
goto2_blk = [target_sys '/Goto2'];

% In Current Control, Goto2 (tag 'debug2') is connected to DebugDemux (Ia_meas).
% Control_System Outport 2 is Idq_debug (measured [Id, Iq] vector).
% We will tap Iq from Control_System Outport 2 (or route Iq to Goto2) so debug2 transmits Iq.
ph_goto2 = get_param(goto2_blk, 'PortHandles');
line_goto2 = get_param(ph_goto2.Inport, 'Line');
if line_goto2 ~= -1
    delete_line(line_goto2);
    disp('Disconnected old line to Goto2 (debug2).');
end

% Check if Demux_Idq already exists, or add a demux
demux_name = 'Demux_Idq_Telemetry';
demux_blk = [target_sys '/' demux_name];
if isempty(find_system(target_sys, 'SearchDepth', 1, 'Name', demux_name))
    add_block('simulink/Signal Routing/Demux', demux_blk, ...
        'Outputs', '2', ...
        'Position', [650, 220, 655, 270]);
    disp('Added Demux_Idq_Telemetry block.');
end

% Connect Control_System port 2 (Idq_debug) to Demux input
ph_cs = get_param(cs_blk, 'PortHandles');
ph_demux = get_param(demux_blk, 'PortHandles');

% Connect CS outport 2 -> Demux inport 1
try
    add_line(target_sys, ph_cs.Outport(2), ph_demux.Inport(1), 'autorouting', 'on');
    disp('Connected Control_System Idq_debug to Demux.');
catch ME
    disp(['Line already exists or: ' ME.message]);
end

% Connect Demux outport 2 (Iq_meas) -> Goto2 inport 1
try
    add_line(target_sys, ph_demux.Outport(2), ph_goto2.Inport(1), 'autorouting', 'on');
    disp('Connected Demux port 2 (Iq_meas) to Goto2 (debug2).');
catch ME
    disp(['Line already exists or: ' ME.message]);
end

save_system(target_model);
disp(['[SUCCESS] Target model updated and saved: ' target_model]);

%% 2. Update Host Model Rate Limiter
host_model = 'mcb_host_model_f28069m';
load_system(host_model);

host_sub = [host_model '/Serial Communication'];
speed_demand_blk = [host_sub '/Speed demand (RPM)'];
gain_blk = [host_sub '/Gain'];

% Find the line between 'Speed demand (RPM)' and 'Gain'
ph_speed = get_param(speed_demand_blk, 'PortHandles');
l_speed = get_param(ph_speed.Outport, 'Line');

rl_name = 'Acceleration_Rate_Limiter';
rl_blk = [host_sub '/' rl_name];

if isempty(find_system(host_sub, 'SearchDepth', 1, 'Name', rl_name))
    % Disconnect existing direct line
    if l_speed ~= -1
        delete_line(l_speed);
        disp('Removed unramped line between Speed demand and Gain in Host.');
    end
    
    % Position between Speed demand (RPM) and Gain
    pos_speed = get_param(speed_demand_blk, 'Position');
    pos_gain  = get_param(gain_blk, 'Position');
    
    % Add Rate Limiter (Rising: 500 RPM/s, Falling: -500 RPM/s)
    add_block('simulink/Discontinuities/Rate Limiter', rl_blk, ...
        'RisingSlewLimit', '500', ...
        'FallingSlewLimit', '-500', ...
        'Position', [pos_speed(3)+30, pos_speed(2)-5, pos_speed(3)+70, pos_speed(4)+5]);
    disp('Added Acceleration_Rate_Limiter to Host Serial Communication.');
    
    ph_rl = get_param(rl_blk, 'PortHandles');
    ph_gain = get_param(gain_blk, 'PortHandles');
    
    add_line(host_sub, ph_speed.Outport, ph_rl.Inport, 'autorouting', 'on');
    add_line(host_sub, ph_rl.Outport, ph_gain.Inport(1), 'autorouting', 'on');
    disp('Wired Speed demand -> Rate Limiter -> Gain.');
else
    disp('Acceleration_Rate_Limiter already present in Host.');
end

save_system(host_model);
disp(['[SUCCESS] Host model updated and saved: ' host_model]);

disp('================================================================');
disp('  ALL SAFEGUARD & TELEMETRY UPDATES COMPLETE');
disp('================================================================');
