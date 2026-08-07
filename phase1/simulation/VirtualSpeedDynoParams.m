clear;
close all;
clc;

% =========================================================================
%  High-Fidelity GaN Inverter + PMSM Thesis Data Harvester
%  Target Model : VirtualSpeedDyno
%  Outputs      : 10 thesis figures covering efficiency maps, loss budgets,
%                 frequency sweep, motor loss split, current comparison,
%                 steady-state temperature, temperature-rise profile, and
%                 continuous torque capability.
% =========================================================================

modelName = 'VirtualSpeedDyno';
if ~bdIsLoaded(modelName)
    load_system(modelName);
end

% ── Clear stale accelerator cache (avoids cache write errors) ────────────
% Using 'accelerator' mode (not rapid-accelerator) because Simscape logging
% is incompatible with rapid-accelerator in parsim: workers write to a
% shared temp log path → file conflict → crash.
slprjAccel = fullfile(fileparts(which([modelName '.slx'])), 'slprj', 'accel', modelName);
if exist(slprjAccel, 'dir')
    fprintf('Clearing stale accel cache...\n');
    rmdir(slprjAccel, 's');
end

% ── CRITICAL: Set Simscape logging on the MODEL before parsim compiles it ───
% SimscapeLogType is a code-generation parameter. It must be set via set_param
% on the model BEFORE parsim builds the rapid-accelerator binary.
% setModelParameter() inside SimulationInput is silently ignored for
% code-generation parameters in rapid-accelerator + UseFastRestart mode.
set_param(modelName, 'SimscapeLogType', 'all');
set_param(modelName, 'SimscapeLogName', 'simlog');

% Fix PMSM IC conflict: the Ideal Angular Velocity Source hard-constrains
% speed, which conflicts with the PMSM block's HIGH-priority angular_velocity
% initial condition.  In R2025b the correct API is simscape.block.Selector;
% set_param('angular_velocity_priority') silently fails on this block.
try
    blkSel = simscape.block.Selector([modelName '/PMSM']);
    blkSel.setVariablePriority('angular_velocity', 'Low');
    fprintf('PMSM angular_velocity IC priority set to Low via Simscape API.\n');
catch ME_ic
    % Fallback for older toolbox versions
    % fprintf('  Simscape IC API unavailable; trying set_param fallback.\n');
    warning(ME_ic.identifier, '%s', ME_ic.message);
    try
        set_param([modelName '/PMSM'], 'angular_velocity_priority', 'low');
        fprintf('PMSM IC priority set via set_param fallback.\n');
    catch
        warning('Simscape:ICPriority', 'Could not set PMSM angular_velocity IC priority. IC conflict warning expected.');
    end
end
fprintf('Simscape logging configured on model.\n');

% Save model so all set_param / IC changes are baked in before sim() runs.
save_system(modelName);
fprintf('Model saved.\n');

%% ── PLOT SAVE SETTINGS ──────────────────────────────────────────────────
savePlots = true;   % Set to true to automatically save figures
plotDPI   = 300;    % Resolution for raster formats (PNG)

% Resolve project root from this script's location so all output paths are
% absolute — files are always written to the project directory regardless
% of MATLAB's current working directory at run time.
projRoot   = fileparts(mfilename('fullpath'));
plotFolder = fullfile(projRoot, 'Inverter_Thesis_Plots');
if ~exist(plotFolder, 'dir'); mkdir(plotFolder); end
fprintf('Output folder: %s\n', plotFolder);

%% ── CONVERTER PARAMETER SETS ──────────────────────────────────────────────
% EPC91200 board — EPC2305 eGaN FETs (6 FETs in 3-phase bridge)
%
% !! CRITICAL: E_on / E_off units are JOULES per switching event.
% At 100 kHz with 6 FETs: P_sw = 6 * (E_on + E_off) * fsw
% Old values of 6.2e-6 / 1.8e-6 J gave: 6*(6.2+1.8)e-6*100e3 = 4800 W — impossible!
% Correct EPC2305 values from datasheet hard-switching test (48V bus, 10A):
%   E_on  ≈  90 nJ  (turn-on including Qoss energy)
%   E_off ≈  20 nJ  (turn-off; GaN has no body-diode Qrr → very low)
% At 96V these scale linearly with voltage: ×(96/48)=×2
% We embed the 48V reference values and scale inside Simscape via V_off_sw.
%
% R_DS_on: 3.0 mΩ max, 2.2 mΩ typical (from EPC2305 datasheet)
% V_off_sw: the bus voltage at which E_on/E_off were measured (48V test bench)
% I_on_sw:  the current at which E_on/E_off were measured (10A for characterisation)
% Simscape scales: E_actual = E_ds * (V_bus/V_off_sw) * (I/I_on_sw)
conv_GaN.V_th       = 1.4;       % Gate threshold voltage [V]  (EPC2305 typical)
conv_GaN.R_DS_on    = 2.2e-3;    % Drain-source on-resistance [Ohm] (EPC2305 typical)
conv_GaN.G_off      = 1e-6;      % Off-state conductance [1/Ohm]
conv_GaN.E_on       = 90e-9;     % Turn-on switching energy  [J]  @ V_off_sw, I_on_sw
conv_GaN.E_off      = 20e-9;     % Turn-off switching energy [J]  @ V_off_sw, I_on_sw
conv_GaN.V_off_sw   = 48.0;      % Reference bus voltage for switching loss data [V]
conv_GaN.I_on_sw    = 10.0;      % Reference current for switching loss data [A]
conv_GaN.R_th_jc_ca = [0.2, 6.5]; % Junction-to-case, case-to-ambient [K/W] (EPC91200)
conv_GaN.M_th_jc    = [1.5e-4, 112.1]; % Thermal masses [J/K]
% NOTE: conv_Si removed — same GaN board used for both 48V and 96V tests.

%% ── IRON LOSS PARAMETER SETS ──────────────────────────────────────────────
% AKE90-KV35 iron losses — Simscape \"Empirical\" model
% Format: [P_hysteresis, P_eddy, P_excess] at f_iron [Hz] and I_sc_iron [A]
% Simscape scales: P_hyst ∝ f, P_eddy ∝ f², P_excess ∝ f^1.5
%
% !! IMPORTANT: These are ESTIMATED placeholders for simulation only.
%    They MUST be replaced with measured values from the T1 no-load spin test:
%    Spin motor at known speeds with I_cmd=0, measure P_in = V*I,
%    subtract bearing friction, fit Steinmetz coefficients to remainder.
%
% Reference frequency: rated electrical frequency at 48V
%   f_rated = KV * V_bat * p_pairs / 60 = 35 * 48 * 21 / 60 = 588 Hz
%   (= 1680 RPM mechanical)
% Using rated speed as reference avoids large extrapolation errors.
%
% Estimated values for AKE90-KV35 class outrunner at 588 Hz:
%   OC: hysteresis ≈ 8W, eddy ≈ 15W, excess ≈ 2W  (flux-driven, no-load)
%   SC: hysteresis ≈ 10W, eddy ≈ 18W, excess ≈ 2.5W (field-driven, stall)
% These are in the right order of magnitude for a ~1.5 kW-class outrunner
% at full electrical frequency. Adjust once T1 measurements are available.
ironLoss_baseline.P_oc_iron = [8.0, 15.0, 2.0];   % [P_hyst, P_eddy, P_excess] OC [W]
ironLoss_baseline.P_sc_iron = [10.0, 18.0, 2.5];  % [P_hyst, P_eddy, P_excess] SC [W]
ironLoss_baseline.f_iron    = 588.0;               % Rated electrical frequency [Hz] @ 48V, 1680 RPM
ironLoss_baseline.I_sc_iron = 25.0;                % Short-circuit RMS current [A] (I_rated)

% Co-designed motor iron losses (empirical)
% Note: Please update if the co-designed motor has different empirical data
ironLoss_codesign           = ironLoss_baseline;

%% ── MOTOR PARAMETER SETS ─────────────────────────────────────────────────
% AKE90-KV35 measured parameters (from datasheet / aifitlab characterisation):
%   KV = 35 RPM/V   => Ke = 1/(KV * 2*pi/60) = 0.2728 V/(rad/s)  (line-to-line peak)
%   Kt = 0.272 Nm/A  (peak, line-to-line)
%   Flux linkage (per-phase peak): lambda = Kt / (3/2 * p) = 0.272/(1.5*21) = 0.00863 Wb
%   Phase-to-phase resistance   = 164 mOhm  => R_phase = 82 mOhm  (per star phase)
%   Phase-to-phase inductance   = 235 uH    => L_phase  = 117.5 uH (per star phase)
%   For SPMSM (surface mount): Ld ≈ Lq ≈ L_phase = 117.5 uH
%   Pole pairs: p = 21
%
% Config A — 48 V bus, GaN inverter ("Si-equivalent baseline" operating point)
motorA.lambda = 0.00863;  % Flux linkage (peak, per-phase) [Wb]  derived: Kt/(1.5*p)
motorA.p      = 21;        % Number of pole pairs
motorA.L_d    = 117.5e-6;  % d-axis inductance [H]  = L_phase (SPMSM)
motorA.L_q    = 117.5e-6;  % q-axis inductance [H]  = L_phase (SPMSM)
motorA.R      = 0.082;     % Stator phase resistance [Ω]  = R_ph-ph/2
motorA.V_bat  = 48.0;      % DC bus voltage [V]
motorA.label  = '48 V GaN (Si-equiv. baseline)';
motorA.conv   = conv_GaN;
motorA.ironLoss = ironLoss_baseline;

% Config B — 96 V bus, GaN inverter ("GaN-advanced" operating point)
% Rewound motor with double the turns (2x Kt)
motorB        = motorA;
motorB.lambda = motorA.lambda * 2;  % Flux linkage doubles
motorB.L_d    = motorA.L_d * 4;     % Inductance quadruples
motorB.L_q    = motorA.L_q * 4;
motorB.R      = motorA.R * 4;       % Resistance quadruples
motorB.V_bat  = 96.0;
motorB.label  = '96 V GaN (rewound double Kt)';
motorB.conv   = conv_GaN;
motorB.ironLoss = ironLoss_baseline;

motorConfigs  = {motorA, motorB};
nConfigs      = numel(motorConfigs);

%% ── OPERATIONAL TEST GRID ──────────────────────────────────────────────────
% AKE90-KV35 motor-shaft operating envelope:
%   KV = 35 RPM/V  |  Kt = 0.272 Nm/A  |  I_rated ≈ 25A RMS
%   Motor-shaft peak torque: Kt * I_peak = 0.272 * 40 ≈ 10.9 Nm
%   Motor-shaft rated torque: Kt * I_rated = 0.272 * 25 ≈ 6.8 Nm
%   No-load speed @ 48V: 35 * 48 * (2*pi/60) = 175.9 rad/s (motor shaft)
%   No-load speed @ 96V: 35 * 96 * (2*pi/60) = 351.9 rad/s (motor shaft)
%   Joint speed with 8:1 gearbox: motor_shaft / 8 → 22–44 rad/s at joint
%   We test the motor shaft directly (before gearbox) on the dyno.
torque_sweep = [1.0, 2.5, 4.5, 6.8, 9.0];   % [N·m] motor shaft (up to ~I_peak)
speed_sweep  = [20, 50, 100, 140, 175];       % [rad/s] motor shaft (up to ~KV*V_bat)
nT = length(torque_sweep);
nS = length(speed_sweep);
speed_sweep_RPM = speed_sweep .* (60 / (2*pi));
[X_Speed_RPM, Y_Torque] = meshgrid(speed_sweep_RPM, torque_sweep);

%% ── FREQUENCY SWEEP PARAMETERS ────────────────────────────────────────────
fsw_sweep = [20e3, 40e3, 60e3, 80e3, 100e3];  % [Hz]
nFsw      = length(fsw_sweep);
% Fixed operating points for frequency sweep
fsw_T_ref    = 4.5;   % [N·m]  ≈ mid-rated (67% of I_rated)
fsw_w_ref    = 100.0; % [rad/s]
fsw_T_ref_fl = 9.0;   % [N·m]  peak/full load
fsw_w_ref_fl = 100.0; % [rad/s]

%% ── GAIT PROFILE FOR TEMPERATURE-RISE TRACE ──────────────────────────────
% Representative single-step cycle: stance → swing → stance (3 s total)
gait_time   = [0, 0.4, 0.8, 1.2, 1.6, 2.0, 2.4, 2.8, 3.0];  % [s] (knot pts)
gait_torque = [3.5, 4.375, 2.5, 0.5, 2.5, 4.375, 3.5, 1.5, 3.5]; % [N·m]
gait_speed  = [20, 25, 15, 5, 15, 25, 20, 10, 20];            % [rad/s]

%% ═══════════════════════════════════════════════════════════════════════════
%  SECTION 1 — Three-Configuration Torque-Speed Efficiency Map Sweeps
%  Outputs: Figures 1-A, 1-B, 1-C (efficiency maps) + data for Fig 4
% ═══════════════════════════════════════════════════════════════════════════

R_cable_total  = 0.016;   % 8 mΩ + 8 mΩ positive/negative rails [Ω]
gan_fets       = {'Q1','Q2','Q3','Q4','Q5','Q6'};
ss_thresh      = 0.040;   % Discard first 40 ms startup transients (needs full FOC settle)
fsw_baseline   = 40000;   % 40 kHz for main sweep

% Preallocate maps struct array before loop to avoid growing it on each iteration
empty_map = struct( ...
    'Efficiency',   zeros(nT, nS), ...
    'Loss_Inv',     zeros(nT, nS), ...
    'Loss_InvSW',   zeros(nT, nS), ...
    'Loss_InvCond', zeros(nT, nS), ...
    'Loss_Motor',   zeros(nT, nS), ...
    'Loss_Cable',   zeros(nT, nS), ...
    'Peak_Tj',      zeros(nT, nS), ...
    'T_winding',    zeros(nT, nS), ...
    'T_housing',    zeros(nT, nS), ...
    'I_rms',        zeros(nT, nS), ...
    'I_peak',       zeros(nT, nS), ...
    'I_dc_rms',     zeros(nT, nS));
maps = repmat(empty_map, 1, nConfigs);

%% ═══════════════════════════════════════════════════════════════════════════
%  SECTION 1 — Three-Configuration Torque-Speed Efficiency Map Sweeps
%  Outputs: Figures 1-A, 1-B, 1-C (efficiency maps) + data for Fig 4
% ═══════════════════════════════════════════════════════════════════════════

% ── Seed base workspace so parsim can validate model parameters before dispatch
% Each parsim worker then receives its own values via setVariable('Workspace','base')
% in the SimulationInput objects built below.
f_c_seed = fsw_baseline / 10;
assignin('base','lambda',  motorA.lambda);  assignin('base','p',       motorA.p);
assignin('base','L_d',     motorA.L_d);     assignin('base','L_q',     motorA.L_q);
assignin('base','R',       motorA.R);       assignin('base','V_bat',   motorA.V_bat);
assignin('base','T_e_ref', torque_sweep(1)); assignin('base','w_ref',   speed_sweep(1));
assignin('base','fsw',     fsw_baseline);
assignin('base','Kp_d', motorA.L_d*2*pi*f_c_seed); assignin('base','Ki_d', motorA.R*2*pi*f_c_seed);
assignin('base','Kp_q', motorA.L_q*2*pi*f_c_seed); assignin('base','Ki_q', motorA.R*2*pi*f_c_seed);
assignin('base','V_th',       motorA.conv.V_th);
assignin('base','R_DS_on',    motorA.conv.R_DS_on);
assignin('base','G_off',      motorA.conv.G_off);
assignin('base','E_on',       motorA.conv.E_on);
assignin('base','E_off',      motorA.conv.E_off);
assignin('base','V_off_sw',   motorA.conv.V_off_sw);
assignin('base','I_on_sw',    motorA.conv.I_on_sw);
assignin('base','R_th_jc_ca', motorA.conv.R_th_jc_ca);
assignin('base','M_th_jc',    motorA.conv.M_th_jc);
assignin('base','P_oc_iron',  motorA.ironLoss.P_oc_iron);
assignin('base','P_sc_iron',  motorA.ironLoss.P_sc_iron);
assignin('base','f_iron',     motorA.ironLoss.f_iron);
assignin('base','I_sc_iron',  motorA.ironLoss.I_sc_iron);

nTotal1 = nConfigs * nT * nS;
simInputs1(1:nTotal1) = Simulink.SimulationInput(modelName);
k = 0;
for cfg = 1:nConfigs
    mc = motorConfigs{cfg};
    for t_idx = 1:nT
        for s_idx = 1:nS
            k = k + 1;
            simInputs1(k) = buildSimIn(modelName, mc, ...
                torque_sweep(t_idx), speed_sweep(s_idx), fsw_baseline);
        end
    end
end

fprintf('\n[SECTION 1] Dispatching %d simulations via parsim...\n', nTotal1);
outs1 = parsim(simInputs1, 'ShowProgress', 'on');

k = 0;
for cfg = 1:nConfigs
    for t_idx = 1:nT
        for s_idx = 1:nS
            k = k + 1;
            if printRunStatus(outs1(k), sprintf('[%d/%d] Cfg%d T=%.2fNm w=%.1frad/s', ...
                    k, nTotal1, cfg, torque_sweep(t_idx), speed_sweep(s_idx)))
                continue;
            end
            try
                res = extractResults(getSimlog(outs1(k)), gan_fets, R_cable_total, ss_thresh);
                maps(cfg).Efficiency(t_idx,s_idx)   = res.efficiency;
                maps(cfg).Loss_Inv(t_idx,s_idx)      = res.inv_loss;
                maps(cfg).Loss_InvSW(t_idx,s_idx)    = res.inv_sw_loss;
                maps(cfg).Loss_InvCond(t_idx,s_idx)  = res.inv_cond_loss;
                maps(cfg).Loss_Motor(t_idx,s_idx)    = res.motor_loss;
                maps(cfg).Loss_Cable(t_idx,s_idx)    = res.cable_loss;
                maps(cfg).Peak_Tj(t_idx,s_idx)       = res.T_junction;
                maps(cfg).T_winding(t_idx,s_idx)     = res.T_winding;
                maps(cfg).T_housing(t_idx,s_idx)     = res.T_housing;
                maps(cfg).I_rms(t_idx,s_idx)         = res.I_rms_phase;
                maps(cfg).I_peak(t_idx,s_idx)        = res.I_peak_phase;
                maps(cfg).I_dc_rms(t_idx,s_idx)      = res.I_dc_rms;
                fprintf('[%d/%d] Cfg %d | T=%.2f Nm | w=%.1f rad/s -> eta=%.1f%% Tj=%.1f degC Idc=%.2fA\n', ...
                    k, nTotal1, cfg, torque_sweep(t_idx), speed_sweep(s_idx), ...
                    res.efficiency, res.T_junction, res.I_dc_rms);
            catch ME
                fprintf('[%d/%d] extract FAILED: %s (in %s line %d)\n', ...
                    k, nTotal1, ME.message, ME.stack(1).name, ME.stack(1).line);
            end
        end
    end
end

%% ═══════════════════════════════════════════════════════════════════════════
%  SECTION 1B — Frequency-Tuned Loss Maps (48 V @ 20 kHz, 96 V @ 80 kHz)
%  Used for Figure 4: inverter-only loss breakdown at config-optimal fsw.
% ═══════════════════════════════════════════════════════════════════════════

% Each config runs at its design switching frequency
% Config A (48 V GaN) → 40 kHz  (Si-equivalent low-frequency baseline)
% Config B (96 V GaN) → 100 kHz (GaN-advanced high-frequency operation)
fsw_per_config = [40e3, 100e3];  % [Hz], one per config

maps_ft = repmat(empty_map, 1, nConfigs);

nTotal1B = nConfigs * nT * nS;
simInputs1B(1:nTotal1B) = Simulink.SimulationInput(modelName);
k = 0;
for cfg = 1:nConfigs
    mc      = motorConfigs{cfg};
    fsw_cfg = fsw_per_config(cfg);
    for t_idx = 1:nT
        for s_idx = 1:nS
            k = k + 1;
            simInputs1B(k) = buildSimIn(modelName, mc, ...
                torque_sweep(t_idx), speed_sweep(s_idx), fsw_cfg);
        end
    end
end

fprintf('\n[SECTION 1B] Dispatching %d frequency-tuned simulations via parsim...\n', nTotal1B);
outs1B = parsim(simInputs1B, 'ShowProgress', 'on');

k = 0;
for cfg = 1:nConfigs
    for t_idx = 1:nT
        for s_idx = 1:nS
            k = k + 1;
            if printRunStatus(outs1B(k), sprintf('[%d/%d] FT Cfg%d T=%.2fNm w=%.1frad/s', ...
                    k, nTotal1B, cfg, torque_sweep(t_idx), speed_sweep(s_idx)))
                continue;
            end
            try
                res = extractResults(getSimlog(outs1B(k)), gan_fets, R_cable_total, ss_thresh);
                maps_ft(cfg).Loss_Cable(t_idx,s_idx)   = res.cable_loss;
                maps_ft(cfg).Loss_InvCond(t_idx,s_idx) = res.inv_cond_loss;
                maps_ft(cfg).Loss_InvSW(t_idx,s_idx)   = res.inv_sw_loss;
                fprintf('[%d/%d] FT Cfg %d | T=%.2f Nm | w=%.1f rad/s -> CableLoss=%.2fW InvCond=%.2fW InvSW=%.2fW\n', ...
                    k, nTotal1B, cfg, torque_sweep(t_idx), speed_sweep(s_idx), ...
                    res.cable_loss, res.inv_cond_loss, res.inv_sw_loss);
            catch ME
                fprintf('[%d/%d] FT extract FAILED: %s (in %s line %d)\n', ...
                    k, nTotal1B, ME.message, ME.stack(1).name, ME.stack(1).line);
            end
        end
    end
end

%% ═══════════════════════════════════════════════════════════════════════════
%  SECTION 2 — Switching-Frequency Sweep (both bus voltages, fixed op-point)
%  Outputs: Figure 5 (total loss vs fsw) + Figure 6 (motor loss split)
% ═══════════════════════════════════════════════════════════════════════════

fsw_configs    = {motorA, motorB};
fsw_cfg_labels = {'48 V GaN', '96 V GaN'};
nFswCfg = 2;

fsw_total_loss = zeros(nFswCfg, nFsw);
fsw_sw_loss    = zeros(nFswCfg, nFsw);
fsw_cond_loss  = zeros(nFswCfg, nFsw);
fsw_motor_loss = zeros(nFswCfg, nFsw);
fsw_cable_loss = zeros(nFswCfg, nFsw);

fsw_total_loss_fl = zeros(nFswCfg, nFsw);
fsw_sw_loss_fl    = zeros(nFswCfg, nFsw);
fsw_cond_loss_fl  = zeros(nFswCfg, nFsw);
fsw_motor_loss_fl = zeros(nFswCfg, nFsw);
fsw_cable_loss_fl = zeros(nFswCfg, nFsw);

nTotal2 = nFswCfg * nFsw * 2;
simInputs2(1:nTotal2) = Simulink.SimulationInput(modelName);
k = 0;
% Mid Load Sweep
for fcfg = 1:nFswCfg
    mc = fsw_configs{fcfg};
    for f_idx = 1:nFsw
        k = k + 1;
        simInputs2(k) = buildSimIn(modelName, mc, fsw_T_ref, fsw_w_ref, fsw_sweep(f_idx));
    end
end
% Full Load Sweep
for fcfg = 1:nFswCfg
    mc = fsw_configs{fcfg};
    for f_idx = 1:nFsw
        k = k + 1;
        simInputs2(k) = buildSimIn(modelName, mc, fsw_T_ref_fl, fsw_w_ref_fl, fsw_sweep(f_idx));
    end
end

fprintf('\n[SECTION 2] Dispatching %d frequency-sweep simulations via parsim...\n', nTotal2);
outs2 = parsim(simInputs2, 'ShowProgress', 'on');

k = 0;
% Extract Mid Load
for fcfg = 1:nFswCfg
    for f_idx = 1:nFsw
        k = k + 1;
        if printRunStatus(outs2(k), sprintf('S2 run %d: %s fsw=%.0fkHz (Mid Load)', k, fsw_cfg_labels{fcfg}, fsw_sweep(f_idx)/1e3))
            continue;
        end
        try
            res = extractResults(getSimlog(outs2(k)), gan_fets, R_cable_total, ss_thresh);
            fsw_total_loss(fcfg,f_idx) = res.inv_loss + res.motor_loss + res.cable_loss;
            fsw_sw_loss(fcfg,f_idx)    = res.inv_sw_loss;
            fsw_cond_loss(fcfg,f_idx)  = res.inv_cond_loss;
            fsw_motor_loss(fcfg,f_idx) = res.motor_loss;
            fsw_cable_loss(fcfg,f_idx) = res.cable_loss;
            fprintf('  %s | fsw=%.0f kHz (Mid Load) -> Total Loss=%.2f W\n', ...
                fsw_cfg_labels{fcfg}, fsw_sweep(f_idx)/1e3, fsw_total_loss(fcfg,f_idx));
        catch ME
            fprintf('  S2 run %d extract FAILED: %s (in %s line %d)\n', ...
                k, ME.message, ME.stack(1).name, ME.stack(1).line);
        end
    end
end
% Extract Full Load
for fcfg = 1:nFswCfg
    for f_idx = 1:nFsw
        k = k + 1;
        if printRunStatus(outs2(k), sprintf('S2 run %d: %s fsw=%.0fkHz (Full Load)', k, fsw_cfg_labels{fcfg}, fsw_sweep(f_idx)/1e3))
            continue;
        end
        try
            res = extractResults(getSimlog(outs2(k)), gan_fets, R_cable_total, ss_thresh);
            fsw_total_loss_fl(fcfg,f_idx) = res.inv_loss + res.motor_loss + res.cable_loss;
            fsw_sw_loss_fl(fcfg,f_idx)    = res.inv_sw_loss;
            fsw_cond_loss_fl(fcfg,f_idx)  = res.inv_cond_loss;
            fsw_motor_loss_fl(fcfg,f_idx) = res.motor_loss;
            fsw_cable_loss_fl(fcfg,f_idx) = res.cable_loss;
            fprintf('  %s | fsw=%.0f kHz (Full Load) -> Total Loss=%.2f W\n', ...
                fsw_cfg_labels{fcfg}, fsw_sweep(f_idx)/1e3, fsw_total_loss_fl(fcfg,f_idx));
        catch ME
            fprintf('  S2 run %d extract FAILED: %s (in %s line %d)\n', ...
                k, ME.message, ME.stack(1).name, ME.stack(1).line);
        end
    end
end

%% ═══════════════════════════════════════════════════════════════════════════
%  SECTION 3 — Gait-Profile Temperature-Rise Trace
%  Outputs: Figure 9 (temperature rise vs time)
% ═══════════════════════════════════════════════════════════════════════════

nGait = length(gait_time);
gait_T_junction = zeros(1, nGait);
gait_T_winding  = zeros(1, nGait);
gait_T_housing  = zeros(1, nGait);

simInputs3(1:nGait) = Simulink.SimulationInput(modelName);
for g_idx = 1:nGait
    % Use 96V GaN config for gait trace — represents our primary hardware config
    simInputs3(g_idx) = buildSimIn(modelName, motorB, ...
        gait_torque(g_idx), gait_speed(g_idx), 100e3);
end

fprintf('\n[SECTION 3] Dispatching %d gait-profile simulations via parsim...\n', nGait);
outs3 = parsim(simInputs3, 'ShowProgress', 'on');

for g_idx = 1:nGait
    if printRunStatus(outs3(g_idx), sprintf('S3 gait %d: t=%.1fs T=%.2fNm w=%.0frad/s', ...
            g_idx, gait_time(g_idx), gait_torque(g_idx), gait_speed(g_idx)))
        continue;
    end
    try
        res = extractResults(getSimlog(outs3(g_idx)), gan_fets, R_cable_total, ss_thresh);
        gait_T_junction(g_idx) = res.T_junction;
        gait_T_winding(g_idx)  = res.T_winding;
        gait_T_housing(g_idx)  = res.T_housing;
        fprintf('  t=%.1f s | T=%.2f Nm | w=%.0f rad/s -> Tj=%.1f degC\n', ...
            gait_time(g_idx), gait_torque(g_idx), gait_speed(g_idx), res.T_junction);
    catch ME
        fprintf('  S3 gait %d extract FAILED: %s (in %s line %d)\n', ...
            g_idx, ME.message, ME.stack(1).name, ME.stack(1).line);
    end
end

%% ═══════════════════════════════════════════════════════════════════════════
%  SECTION 4 — Continuous Torque Capability (thermal limit at 150°C)
%  Sweep torque at fixed mid-speed for both 48V and 96V until Tj ≥ 150°C
%  Outputs: Figure 10
% ═══════════════════════════════════════════════════════════════════════════

T_LIMIT_DEG = 150;
cont_torque_sweep = linspace(0.5, 5.0, 10);
cont_speed_fixed  = 15.0;
cont_configs      = {motorA, motorB};
cont_labels       = {'48 V GaN', '96 V GaN'};
cont_T_j          = zeros(2, length(cont_torque_sweep));

nContTorque = length(cont_torque_sweep);
nTotal4     = 2 * nContTorque;
simInputs4(1:nTotal4) = Simulink.SimulationInput(modelName);
k = 0;
for ccfg = 1:2
    mc = cont_configs{ccfg};
    for ct_idx = 1:nContTorque
        k = k + 1;
        simInputs4(k) = buildSimIn(modelName, mc, ...
            cont_torque_sweep(ct_idx), cont_speed_fixed, fsw_baseline);
    end
end

fprintf('\n[SECTION 4] Dispatching %d torque-capability simulations via parsim...\n', nTotal4);
outs4 = parsim(simInputs4, 'ShowProgress', 'on');

k = 0;
for ccfg = 1:2
    for ct_idx = 1:nContTorque
        k = k + 1;
        if printRunStatus(outs4(k), sprintf('S4 run %d: %s T=%.2fNm', k, cont_labels{ccfg}, cont_torque_sweep(ct_idx)))
            continue;
        end
        try
            res = extractResults(getSimlog(outs4(k)), gan_fets, R_cable_total, ss_thresh);
            cont_T_j(ccfg, ct_idx) = res.T_junction;
            fprintf('  %s | T=%.2f Nm -> Tj=%.1f degC\n', ...
                cont_labels{ccfg}, cont_torque_sweep(ct_idx), res.T_junction);
        catch ME
            fprintf('  S4 run %d extract FAILED: %s (in %s line %d)\n', ...
                k, ME.message, ME.stack(1).name, ME.stack(1).line);
        end
    end
end

% ── Save extracted data before rendering figures (safeguard against crashes) ──
matPath = fullfile(plotFolder, 'thesis_sim_data.mat');
save(matPath, ...
    'maps', 'maps_ft', 'fsw_per_config', 'nConfigs', 'motorConfigs', ...
    'X_Speed_RPM', 'Y_Torque', 'torque_sweep', 'speed_sweep', 'speed_sweep_RPM', ...
    'fsw_sweep', 'fsw_total_loss', 'fsw_sw_loss', 'fsw_cond_loss', 'fsw_motor_loss', 'fsw_cable_loss', ...
    'fsw_total_loss_fl', 'fsw_sw_loss_fl', 'fsw_cond_loss_fl', 'fsw_motor_loss_fl', 'fsw_cable_loss_fl', ...
    'fsw_cfg_labels', 'nFswCfg', ...
    'gait_time', 'gait_torque', 'gait_speed', 'gait_T_junction', 'gait_T_winding', 'gait_T_housing', ...
    'cont_torque_sweep', 'cont_speed_fixed', 'cont_T_j', 'T_LIMIT_DEG');
fprintf('Simulation data saved to: %s\n', matPath);

%% ── LOCAL HELPER FUNCTIONS ──────────────────────────────────────────────

function simIn = buildSimIn(modelName, mc, T_cmd, w_cmd, fsw_val)
% Compute PI gains from pole-zero cancellation
f_c  = fsw_val / 10;
Kp_d = mc.L_d * 2*pi*f_c;
Ki_d = mc.R   * 2*pi*f_c;
Kp_q = mc.L_q * 2*pi*f_c;
Ki_q = mc.R   * 2*pi*f_c;

% Embed every parameter directly into this SimulationInput object via
% setVariable(). This is the ONLY safe pattern with parsim: each SimulationInput
% carries its own isolated parameter set, completely avoiding the race condition
% that occurs when multiple buildSimIn() calls overwrite the shared base workspace
% and TransferBaseWorkspaceVariables snapshots the last-written values for every
% worker.
simIn = Simulink.SimulationInput(modelName);
simIn = simIn.setModelParameter('SimulationMode', 'rapid-accelerator');
simIn = simIn.setModelParameter('StopTime', '0.500');
simIn = simIn.setModelParameter('SimscapeLogType', 'all');
simIn = simIn.setModelParameter('SimscapeLogName', 'simlog');
simIn = simIn.setVariable('lambda',  mc.lambda);
simIn = simIn.setVariable('p',       mc.p);
simIn = simIn.setVariable('L_d',     mc.L_d);
simIn = simIn.setVariable('L_q',     mc.L_q);
simIn = simIn.setVariable('R',       mc.R);
simIn = simIn.setVariable('V_bat',   mc.V_bat);
simIn = simIn.setVariable('T_e_ref', T_cmd);
simIn = simIn.setVariable('w_ref',   w_cmd);
simIn = simIn.setVariable('fsw',     fsw_val);
simIn = simIn.setVariable('Kp_d',    Kp_d);
simIn = simIn.setVariable('Ki_d',    Ki_d);
simIn = simIn.setVariable('Kp_q',    Kp_q);
simIn = simIn.setVariable('Ki_q',    Ki_q);
simIn = simIn.setVariable('V_th',       mc.conv.V_th);
simIn = simIn.setVariable('R_DS_on',    mc.conv.R_DS_on);
simIn = simIn.setVariable('G_off',      mc.conv.G_off);
simIn = simIn.setVariable('E_on',       mc.conv.E_on);
simIn = simIn.setVariable('E_off',      mc.conv.E_off);
simIn = simIn.setVariable('V_off_sw',   mc.conv.V_off_sw);
simIn = simIn.setVariable('I_on_sw',    mc.conv.I_on_sw);
simIn = simIn.setVariable('R_th_jc_ca', mc.conv.R_th_jc_ca);
simIn = simIn.setVariable('M_th_jc',    mc.conv.M_th_jc);
simIn = simIn.setVariable('P_oc_iron',  mc.ironLoss.P_oc_iron);
simIn = simIn.setVariable('P_sc_iron',  mc.ironLoss.P_sc_iron);
simIn = simIn.setVariable('f_iron',     mc.ironLoss.f_iron);
simIn = simIn.setVariable('I_sc_iron',  mc.ironLoss.I_sc_iron);
end

function res = extractResults(sl, gan_fets, R_cable_total, ss_thresh)
% sl  — Simscape log node (e.g. out.simlog or the base-workspace 'simlog')
%       Pass via getSimlog(out) to handle both sources automatically.

% Time base from switching-loss log (always present)
time_vec  = sl.Converter_Three_Phase.Q1.accumulatedSwitchingLosses.series.time;
ss_start  = find(time_vec >= ss_thresh, 1);
if isempty(ss_start)
    error('extractResults: ss_thresh (%.3f s) is beyond simulation end (%.3f s). Increase StopTime.', ...
        ss_thresh, time_vec(end));
end
ss_dt     = time_vec(end) - time_vec(ss_start);

% ── Inverter losses ──────────────────────────────────────────────────
inv_sw_loss   = 0;
inv_cond_loss = 0;
for q = 1:length(gan_fets)
    sw_E = sl.Converter_Three_Phase.(gan_fets{q}).accumulatedSwitchingLosses.series.values('J');
    t_sw = sl.Converter_Three_Phase.(gan_fets{q}).accumulatedSwitchingLosses.series.time;
    % Use SS-window difference when the signal increments within the window.
    % In rapid-accelerator mode the fixed-step solver may not align with
    % individual switching instants, making (end - ss_start) ≈ 0 even
    % though switching is happening. Fall back to total_energy / total_time
    % (which equals the SS average in true periodic steady state).
    sw_E_ss_delta = sw_E(end) - sw_E(ss_start);
    if sw_E_ss_delta > 1e-9 * sw_E(end)   % window gave a meaningful increment
        inv_sw_loss = inv_sw_loss + sw_E_ss_delta / ss_dt;
    else                                    % flat in window — use total average
        inv_sw_loss = inv_sw_loss + sw_E(end) / t_sw(end);
    end

    cond_P = sl.Converter_Three_Phase.(gan_fets{q}).power_dissipated.series.values('W');
    inv_cond_loss = inv_cond_loss + mean(cond_P(ss_start:end));
end
inv_loss = inv_sw_loss + inv_cond_loss;

% ── Motor losses (copper + iron combined from PMSM.power_dissipated) ─────
motor_P    = sl.PMSM.power_dissipated.series.values('W');
motor_loss = mean(motor_P(ss_start:end));

% ── Cable losses ─────────────────────────────────────────────────────
i_dc  = sl.Resistor.i.series.values('A');
cable_loss = mean((i_dc(ss_start:end).^2) .* R_cable_total);

% ── Mechanical power ─────────────────────────────────────────────────
T_mech   = sl.PMSM.torque.series.values('N*m');
w_mech   = sl.PMSM.angular_velocity.series.values('rad/s');
n_ss     = numel(T_mech) - ss_start + 1;
late_start = ss_start + floor(n_ss * 0.50);
late_start = max(late_start, ss_start + 1);
P_mech   = -mean(T_mech(late_start:end) .* w_mech(late_start:end));

% ── Currents ─────────────────────────────────────────────────────────
try
    i_a = sl.PMSM.i_a.series.values('A');
    i_a_ss = i_a(late_start:end);
    I_rms_phase = sqrt(mean(i_a_ss.^2));
    I_peak_phase = max(abs(i_a_ss));
catch
    I_rms_phase  = NaN;
    I_peak_phase = NaN;
end

% ── Junction temperature (Steady-state analytical calculation) ────────
% Since the simulation runs for only 0.5 s, the large case (112.1 J/K) and
% heatsink (134.6 J/K) thermal masses act as a thermal clamp. In reality,
% the system reaches a steady-state temperature after minutes.
% We calculate this analytically from the average inverter power losses:
% - R_JC = 0.2 K/W (per FET)
% - R_CA_internal = 6.5 K/W (per FET)
% - R_conv = 1/(h*A) = 1/(10.25 * 0.015) = 6.504 K/W (shared)
% - R_CA_eff = 1 / (6/R_CA_internal + 1/R_conv) = 0.9286 K/W
T_ambient = 25.0; % [degC]
R_JC = 0.2;       % [K/W]
R_CA_eff = 0.9286; % [K/W]
max_t_j = T_ambient + (inv_loss / 6) * R_JC + inv_loss * R_CA_eff;

% ── Winding & Housing temperatures (Steady-state analytical calculation) ──
% Similar to the inverter, the motor's thermal time constant is large.
% We calculate steady-state temperatures using typical motor thermal resistances:
% - R_wh = 0.8 K/W (winding to housing)
% - R_ha = 2.0 K/W (housing to ambient)
R_wh = 0.8; % [K/W]
R_ha = 2.0; % [K/W]
T_housing = T_ambient + motor_loss * R_ha;
T_winding = T_housing + motor_loss * R_wh;

% ── Pack into result struct ───────────────────────────────────────────
P_loss_total  = inv_loss + motor_loss + cable_loss;
P_elec        = P_mech + P_loss_total;
if P_mech <= 0 || P_elec <= 0
    warning('extractResults: P_mech=%.3f W, P_elec=%.3f W — not in motoring SS. Returning NaN.', P_mech, P_elec);
    res.efficiency = NaN;
else
    res.efficiency = (P_mech / P_elec) * 100;
end
res.inv_loss      = inv_loss;
res.inv_sw_loss   = inv_sw_loss;
res.inv_cond_loss = inv_cond_loss;
res.motor_loss    = motor_loss;
res.cable_loss    = cable_loss;
res.P_mech        = P_mech;
res.T_junction    = max_t_j;
res.T_winding     = T_winding;
res.T_housing     = T_housing;
res.I_rms_phase   = I_rms_phase;
res.I_peak_phase  = I_peak_phase;
res.I_dc_rms      = sqrt(mean(i_dc(ss_start:end).^2));
end

function sl = getSimlog(out)
% getSimlog  Extract Simscape log from SimulationOutput or base workspace.
%
% When sim(SimulationInput) is used in normal mode, the Simscape log
% can appear in EITHER:
%   (a) out.simlog  — the SimulationOutput object field  [preferred]
%   (b) base workspace variable 'simlog'                 [fallback]
%
% This helper tries (a) first and falls back to (b) so extractResults
% works regardless of which path MATLAB chose.
try
    sl = out.simlog;
    % Validate it has the expected top-level nodes
    if isempty(sl) || ~isobject(sl)
        error('simlog in SimulationOutput is empty');
    end
    % Probe for a known node to confirm it is populated
    sl.Converter_Three_Phase; %#ok<VUNUS>
catch
    % Fall back to base workspace
    try
        sl = evalin('base', 'simlog');
        if isempty(sl)
            error('simlog in base workspace is also empty');
        end
    catch ME2
        error('getSimlog: Simscape log not found in SimulationOutput or base workspace.\nDetails: %s', ME2.message);
    end
end
end

function saveFigure(fig, folder, baseFilename)
% Helper function to save figure in PNG, PDF, and FIG formats
% Create folder if it doesn't exist
if ~exist(folder, 'dir')
    mkdir(folder);
end

% Sanitize filename (replace spaces and non-alphanumeric chars with underscores)
safeName = regexprep(baseFilename, '[^\w\-]', '_');

% Save as PNG (high-resolution raster for quick viewing/reports)
pngPath = fullfile(folder, [safeName, '.png']);
exportgraphics(fig, pngPath, 'Resolution', 300);

% Save as PDF (vector format for academic papers/LaTeX)
pdfPath = fullfile(folder, [safeName, '.pdf']);
exportgraphics(fig, pdfPath, 'ContentType', 'vector');

% Save as FIG (MATLAB figure for future editing/rescaling)
figPath = fullfile(folder, [safeName, '.fig']);
savefig(fig, figPath);

fprintf('Saved figure to: %s (.png, .pdf, .fig)\n', fullfile(folder, safeName));
end

function failed = printRunStatus(simOut, label)
% printRunStatus  Print per-run status from a parsim SimulationOutput.
%   failed = true  → hard error occurred (caller should skip extraction)
%   failed = false → simulation completed (may still have warnings)
%
% Reads both ErrorMessage (hard crash) and
% SimulationMetadata.ExecutionInfo.WarningDiagnostics (soft warnings such
% as the PMSM angular_velocity IC conflict).
failed = false;

% ── Hard error ────────────────────────────────────────────────────
if ~isempty(simOut.ErrorMessage)
    fprintf('  [FAIL] %s\n         Error: %s\n', label, simOut.ErrorMessage);
    failed = true;
    return;
end

% ── Soft warnings from SimulationMetadata ─────────────────────────────
try
    warnDiag = simOut.SimulationMetadata.ExecutionInfo.WarningDiagnostics;
    if ~isempty(warnDiag)
        fprintf('  [WARN] %s\n', label);
        for wi = 1:numel(warnDiag)
            fprintf('         Warning %d: %s\n', wi, warnDiag(wi).message);
        end
    end
catch
    % SimulationMetadata may not exist if sim was trivially short
end
end