% VirtualSpeedDynoParams
% First-pass no-load FOC validation for the EPC91200 + AKE80-8 setup.
%
% This file intentionally runs ONE short, low-risk simulation.  It is the
% simulation-side parameter handoff for the later hardware controller.  The
% hardware model is separate: SAFECOPYFOC_Motor_Control_Hardware.slx.
%
% Important units used by VirtualSpeedDyno:
%   w_ref    = mechanical rotor speed in rad/s at the ideal angular-velocity
%              source input.  The PMSM initial-condition field is displayed
%              with angular_velocity_unit = rpm, but the source input uses SI
%              angular velocity because its Simulink-PS Converter has Unit=1.
%   T_e_ref  = commanded electromagnetic torque in N*m
%   lambda   = per-phase peak PM flux linkage in Wb
%   L_d/L_q  = star-phase inductances in H
%   R        = star-phase resistance in ohm
%
% The model uses an ideal angular-velocity source as the virtual dyno.  This
% means the 0.25 N*m command below is a small torque-producing current-loop
% test; it is not a simulated external mechanical load.

modelName = 'VirtualSpeedDyno';

%% Hardware-aligned motor and inverter data
% AKE80-8 KV30 manufacturer data:
%   Kt = 0.32 N*m/A, R_ll = 0.870 ohm, L_ll = 990 uH, p = 21,
%   48 V rated, 8:1 gearbox, 4.8 A rated and 12 A peak current.
% The published line-to-line R/L values are converted to star phase values
% for the PMSM block.  The L/R result is ~1.14 ms, matching the published
% electrical time constant of 1.13 ms.
motor.name                = 'CubeMars AKE80-8 KV30';
motor.gear_ratio          = 8;
motor.pole_pairs          = 21;
motor.Kt_Nm_per_A         = 0.32;
motor.R_phase_ohm         = 0.870 / 2;
motor.L_phase_H           = 990e-6 / 2;
motor.lambda_Wb           = motor.Kt_Nm_per_A / (1.5 * motor.pole_pairs);
motor.inertia_kg_m2       = 1471.75e-7;
motor.bus_voltage_V       = 48.0;
motor.rated_current_A     = 4.8;
motor.peak_current_A      = 12.0;
motor.electrical_tau_s    = motor.L_phase_H / motor.R_phase_ohm;

% EPC91200 / EPC2305 nominal data.  Switching-energy values are deliberately
% set to a tiny positive numerical floor because the detailed Simscape
% converter requires both values to be greater than zero.  These are not
% datasheet characterizations; replace them with board-level values before
% doing loss or thermal studies.
inverter.name             = 'EPC91200 / EPC2305';
inverter.R_DS_on_ohm      = 2.2e-3;     % EPC2305 typical at 25 C
inverter.V_threshold_V    = 1.1;        % EPC2305 typical threshold
inverter.G_off_S          = 1e-6;       % benign off-state leakage model
inverter.E_on_J           = 1e-12;      % numerical floor only
inverter.E_off_J          = 1e-12;      % numerical floor only
inverter.V_switch_ref_V   = 48.0;
inverter.I_switch_ref_A   = 10.0;
inverter.R_th_jc_ca_K_W   = [0.2, 21.0]; % junction-case; EVB-level placeholder
inverter.M_th_jc_J_K      = [1.5e-4, 112.1]; % model-valid placeholder only

%% Conservative first test point
test.speed_ref_rpm          = 60.0;      % fixed rotor speed, not output-shaft rpm
test.speed_ref_rad_s        = test.speed_ref_rpm * 2*pi/60;
test.torque_ref_Nm          = 0.25;      % small current-loop command, no external load
test.stop_time_s            = 0.100;
test.switching_frequency_Hz = 20e3;     % conservative F28069M first bring-up rate
test.current_loop_bandwidth_Hz = test.switching_frequency_Hz / 10;
test.id_ref_A               = 0.0;

% Core variable names expected by the existing VirtualSpeedDyno model.
lambda  = motor.lambda_Wb;
p       = motor.pole_pairs;
L_d     = motor.L_phase_H;
L_q     = motor.L_phase_H;
R       = motor.R_phase_ohm;
V_bat   = motor.bus_voltage_V;
T_e_ref = test.torque_ref_Nm;
w_ref   = test.speed_ref_rad_s;
fsw     = test.switching_frequency_Hz;

% PI gains use the existing model's pole-zero-cancellation form:
%   Kp = L * 2*pi*fc, Ki = R * 2*pi*fc.
omega_c = 2*pi*test.current_loop_bandwidth_Hz;
Kp_d = L_d * omega_c;
Ki_d = R   * omega_c;
Kp_q = L_q * omega_c;
Ki_q = R   * omega_c;

% Converter variables expected by the model.
V_th       = inverter.V_threshold_V;
R_DS_on    = inverter.R_DS_on_ohm;
G_off      = inverter.G_off_S;
E_on       = inverter.E_on_J;
E_off      = inverter.E_off_J;
V_off_sw   = inverter.V_switch_ref_V;
I_on_sw    = inverter.I_switch_ref_A;
R_th_jc_ca = inverter.R_th_jc_ca_K_W;
M_th_jc    = inverter.M_th_jc_J_K;

% No-load iron-loss data is not yet measured.  Keep the empirical model
% numerically defined but do not invent a loss curve for this control test.
P_oc_iron = [0.0, 0.0, 0.0];
P_sc_iron = [0.0, 0.0, 0.0];
f_iron    = 1000.0;
I_sc_iron = motor.peak_current_A;

%% Non-destructive simulation setup
% SimulationInput keeps this test's variables local to the run and avoids
% overwriting the model file or relying on stale base-workspace values.
in = Simulink.SimulationInput(modelName);
in = in.setModelParameter( ...
    'SimulationMode', 'normal', ...
    'StopTime', num2str(test.stop_time_s, '%.6g'), ...
    'SimscapeLogType', 'all', ...
    'SimscapeLogName', 'simlog', ...
    'SignalLogging', 'on', ...
    'SaveOutput', 'on', ...
    'SaveTime', 'on');

in = in.setVariable('lambda', lambda);
in = in.setVariable('p', p);
in = in.setVariable('L_d', L_d);
in = in.setVariable('L_q', L_q);
in = in.setVariable('R', R);
in = in.setVariable('V_bat', V_bat);
in = in.setVariable('T_e_ref', T_e_ref);
in = in.setVariable('w_ref', w_ref);
in = in.setVariable('fsw', fsw);
in = in.setVariable('Kp_d', Kp_d);
in = in.setVariable('Ki_d', Ki_d);
in = in.setVariable('Kp_q', Kp_q);
in = in.setVariable('Ki_q', Ki_q);
in = in.setVariable('V_th', V_th);
in = in.setVariable('R_DS_on', R_DS_on);
in = in.setVariable('G_off', G_off);
in = in.setVariable('E_on', E_on);
in = in.setVariable('E_off', E_off);
in = in.setVariable('V_off_sw', V_off_sw);
in = in.setVariable('I_on_sw', I_on_sw);
in = in.setVariable('R_th_jc_ca', R_th_jc_ca);
in = in.setVariable('M_th_jc', M_th_jc);
in = in.setVariable('P_oc_iron', P_oc_iron);
in = in.setVariable('P_sc_iron', P_sc_iron);
in = in.setVariable('f_iron', f_iron);
in = in.setVariable('I_sc_iron', I_sc_iron);

% Override the PMSM inertia for the run without changing VirtualSpeedDyno.slx.
% The angular-velocity source still enforces the test speed, but the correct
% inertia makes the parameter set ready for a future free-acceleration model.
in = in.setBlockParameter([modelName '/PMSM'], ...
    'J', num2str(motor.inertia_kg_m2, '%.12g'));
% The PMSM initial-condition field is in rpm even though w_ref is the
% rad/s source command.  Keep the two interfaces explicit for repeatability.
in = in.setBlockParameter([modelName '/PMSM'], ...
    'angular_velocity', num2str(test.speed_ref_rpm, '%.12g'));
in = in.setBlockParameter([modelName '/i_d_ref'], ...
    'Value', num2str(test.id_ref_A, '%.12g'));

fprintf('VirtualSpeedDyno first-pass test\n');
fprintf('  Motor: %s | Vdc = %.1f V | fsw = %.0f kHz\n', ...
    motor.name, V_bat, fsw/1e3);
fprintf('  Speed: %.1f rpm (%.4f rad/s) rotor | Torque command: %.3f N*m\n', ...
    test.speed_ref_rpm, w_ref, T_e_ref);
fprintf('  Derived: lambda = %.6g Wb | R = %.6g ohm | L = %.6g H | tau_e = %.3f ms\n', ...
    lambda, R, L_d, motor.electrical_tau_s*1e3);
fprintf('  Derived: PI Kp = %.6g | Ki = %.6g | iq command = %.3f A peak\n', ...
    Kp_q, Ki_q, T_e_ref/(1.5*p*lambda));

simOut = sim(in);
fprintf('Simulation completed.\n');
try
    outputNames = simOut.who;
    fprintf('SimulationOutput variables: %s\n', strjoin(outputNames, ', '));
catch
    % Keep completion reporting robust across MATLAB releases.
end
