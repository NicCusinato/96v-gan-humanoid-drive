% =========================================================================
%  GaitTrackingSimulation.m
%  Section C — Dynamic and Control Simulation (Checklist §C)
%
%  Purpose:
%    Run a SINGLE continuous time-domain simulation that drives the joint
%    through a realistic gait torque/speed profile and captures:
%      - Commanded vs actual torque tracking  (Figure C1)
%      - Commanded vs actual speed tracking   (Figure C2)
%      - Current loop / torque loop Bode plots (Figure C3)
%      - Energy per gait from time-domain integration (Figure C4)
%
%  Prerequisites:
%    1. VirtualSpeedDynoParams.m has been run at least once so that
%       FOC_Project_Parameters / motor params are set in the workspace.
%    2. The Simulink model used here (set in modelName below) must accept
%       time-varying T_e_ref(t) and w_ref(t) — either via a lookup-table
%       block reading 'gait_torque_ts' / 'gait_speed_ts' from the workspace,
%       or via a From Workspace block.
%
%  TODO — before running:
%    [ ] Set modelName to your closed-loop gait model (may be the same
%        VirtualSpeedDyno.slx once you wire the time-varying reference, or a
%        dedicated model)
%    [ ] Confirm the Simscape log paths in extractTrackingResults() match
%        your model hierarchy
%    [ ] Tune gait_time / gait_torque / gait_speed below to match your
%        actual OmniRetarget trajectory data
%    [ ] Set StopTime = gait_time(end) in the sim call
% =========================================================================

clear; close all; clc;

%% ── USER-CONFIGURABLE SETTINGS ──────────────────────────────────────────
modelName  = 'VirtualSpeedDyno';   % TODO: update to your gait-tracking model
savePlots  = true;
projRoot   = fileparts(mfilename('fullpath'));
plotFolder = fullfile(projRoot, 'Inverter_Thesis_Plots');
if ~exist(plotFolder,'dir'); mkdir(plotFolder); end

% Motor configuration to test (match parameters from VirtualSpeedDynoParams.m)
% Config A — 48 V Silicon baseline
motorA.lambda = 0.0075;   motorA.p = 14;
motorA.L_d    = 95e-6;    motorA.L_q = 115e-6;
motorA.R      = 0.185;    motorA.V_bat = 48.0;
motorA.label  = '48 V Baseline';

% Config B — 96 V GaN baseline motor
motorB        = motorA;
motorB.V_bat  = 96.0;
motorB.label  = '96 V Baseline Motor';

fsw_val = 40e3;   % [Hz] — use 80e3 for GaN design-freq runs

%% ── GAIT PROFILE (knot-point trajectory) ─────────────────────────────────
% Replace with imported OmniRetarget data when available.
% Current values: single knee step cycle (stance → swing → stance, 3 s)
gait_time   = [0,   0.4,   0.8,  1.2,  1.6,   2.0,   2.4,  2.8,  3.0];  % [s]
gait_torque = [3.5, 4.375, 2.5,  0.5,  2.5,  4.375,  3.5,  1.5,  3.5];  % [N·m]
gait_speed  = [20,  25,    15,   5,    15,    25,    20,    10,   20];    % [rad/s]

% Build MATLAB timeseries objects for the From Workspace / lookup-table blocks
gait_torque_ts = timeseries(gait_torque(:), gait_time(:));
gait_torque_ts.Name = 'T_e_ref';

gait_speed_ts  = timeseries(gait_speed(:), gait_time(:));
gait_speed_ts.Name  = 'w_ref';

% Push to base workspace so Simulink From Workspace blocks can read them
assignin('base', 'gait_torque_ts', gait_torque_ts);
assignin('base', 'gait_speed_ts',  gait_speed_ts);

stopTime = gait_time(end);   % [s]

%% ── LOAD AND CONFIGURE MODEL ─────────────────────────────────────────────
if ~bdIsLoaded(modelName)
    load_system(modelName);
end

% TODO: configure Simscape logging if needed
% set_param(modelName, 'SimscapeLogType', 'all');
% set_param(modelName, 'SimscapeLogName', 'simlog');

%% ── RUN SIMULATIONS (one per bus-voltage config) ─────────────────────────
configs     = {motorA, motorB};
nCfgTrack   = numel(configs);
track_data  = cell(1, nCfgTrack);

for cfgIdx = 1:nCfgTrack
    mc  = configs{cfgIdx};
    f_c = fsw_val / 10;

    simIn = Simulink.SimulationInput(modelName);
    simIn = simIn.setModelParameter('StopTime', num2str(stopTime));
    simIn = simIn.setModelParameter('SimulationMode', 'normal');   % normal for Simscape logging
    simIn = simIn.setVariable('lambda',  mc.lambda);
    simIn = simIn.setVariable('p',       mc.p);
    simIn = simIn.setVariable('L_d',     mc.L_d);
    simIn = simIn.setVariable('L_q',     mc.L_q);
    simIn = simIn.setVariable('R',       mc.R);
    simIn = simIn.setVariable('V_bat',   mc.V_bat);
    simIn = simIn.setVariable('fsw',     fsw_val);
    simIn = simIn.setVariable('Kp_d',    mc.L_d * 2*pi*f_c);
    simIn = simIn.setVariable('Ki_d',    mc.R   * 2*pi*f_c);
    simIn = simIn.setVariable('Kp_q',    mc.L_q * 2*pi*f_c);
    simIn = simIn.setVariable('Ki_q',    mc.R   * 2*pi*f_c);

    % TODO: add GaN/Si device parameters (V_th, R_DS_on, etc.) if needed
    %   simIn = simIn.setVariable('V_th', mc.conv.V_th); etc.

    fprintf('[CFG %d/%d] Running gait tracking sim for: %s\n', ...
        cfgIdx, nCfgTrack, mc.label);

    try
        out = sim(simIn);
        track_data{cfgIdx} = extractTrackingResults(out, stopTime);
        fprintf('  Done. Tracking error RMS: T=%.4f N·m  w=%.4f rad/s\n', ...
            track_data{cfgIdx}.T_err_rms, track_data{cfgIdx}.w_err_rms);
    catch ME
        fprintf('  [FAIL] %s\n', ME.message);
        track_data{cfgIdx} = [];
    end
end

%% ── FIGURE C1 : Commanded vs Actual Torque Tracking ─────────────────────
cfg_colors = [0 0.4470 0.7410; 0.8500 0.3250 0.098];

figC1 = figure('Name','Torque Tracking','Color',[1 1 1],'Position',[100 100 1100 480],'Visible','off');
tiledlayout(1, nCfgTrack, 'TileSpacing','compact','Padding','compact');

for cfgIdx = 1:nCfgTrack
    nexttile; hold on;
    if ~isempty(track_data{cfgIdx})
        td = track_data{cfgIdx};
        plot(td.t, td.T_cmd, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Commanded');
        plot(td.t, td.T_act, '-',   'Color', cfg_colors(cfgIdx,:), ...
             'LineWidth', 2, 'DisplayName', 'Actual');
        xlabel('Time (s)',       'FontWeight','bold');
        ylabel('Torque (N·m)',   'FontWeight','bold');
        title(configs{cfgIdx}.label, 'FontWeight','bold');
        legend('Location','northeast','FontSize',8);
    else
        text(0.3, 0.5, 'Simulation Failed', 'Units','normalized', 'FontSize',12, 'Color','r');
    end
    grid on; hold off;
end
sgtitle('Commanded vs Actual Torque Tracking — Gait Profile (Checklist §C1)', ...
    'FontSize',12,'FontWeight','bold');
if savePlots; saveFigure(figC1, plotFolder, 'gait_torque_tracking'); end
fprintf('Figure C1 rendered: Torque Tracking\n');

%% ── FIGURE C2 : Commanded vs Actual Speed Tracking ──────────────────────
figC2 = figure('Name','Speed Tracking','Color',[1 1 1],'Position',[100 100 1100 480],'Visible','off');
tiledlayout(1, nCfgTrack, 'TileSpacing','compact','Padding','compact');

for cfgIdx = 1:nCfgTrack
    nexttile; hold on;
    if ~isempty(track_data{cfgIdx})
        td = track_data{cfgIdx};
        plot(td.t, td.w_cmd, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Commanded');
        plot(td.t, td.w_act, '-',   'Color', cfg_colors(cfgIdx,:), ...
             'LineWidth', 2, 'DisplayName', 'Actual');
        xlabel('Time (s)',       'FontWeight','bold');
        ylabel('Speed (rad/s)',  'FontWeight','bold');
        title(configs{cfgIdx}.label, 'FontWeight','bold');
        legend('Location','northeast','FontSize',8);
    else
        text(0.3, 0.5, 'Simulation Failed', 'Units','normalized', 'FontSize',12, 'Color','r');
    end
    grid on; hold off;
end
sgtitle('Commanded vs Actual Speed Tracking — Gait Profile (Checklist §C2)', ...
    'FontSize',12,'FontWeight','bold');
if savePlots; saveFigure(figC2, plotFolder, 'gait_speed_tracking'); end
fprintf('Figure C2 rendered: Speed Tracking\n');

%% ── FIGURE C3 : Time-Domain Energy from Actual Signals ──────────────────
% Integrate P_mech(t) = T_act(t) × w_act(t) over the simulation window
figC3 = figure('Name','Energy from Tracking','Color',[1 1 1],'Position',[100 100 820 480],'Visible','off');
hold on;
for cfgIdx = 1:nCfgTrack
    if ~isempty(track_data{cfgIdx})
        td = track_data{cfgIdx};
        P_mech_t  = td.T_act .* td.w_act;
        E_cum     = cumtrapz(td.t, P_mech_t);
        plot(td.t, E_cum, 'LineWidth', 2, 'Color', cfg_colors(cfgIdx,:), ...
             'DisplayName', configs{cfgIdx}.label);
    end
end
xlabel('Time (s)',              'FontWeight','bold','FontSize',11);
ylabel('Cumulative Mech. Energy (J)', 'FontWeight','bold','FontSize',11);
title('Cumulative Mechanical Energy from Gait Tracking', 'FontSize',12,'FontWeight','bold');
legend('Location','northwest','FontSize',10); grid on; hold off;
if savePlots; saveFigure(figC3, plotFolder, 'gait_energy_from_tracking'); end
fprintf('Figure C3 rendered: Energy from Tracking\n');

fprintf('\n===== GaitTrackingSimulation complete =====\n');
fprintf('Output folder: %s\n', plotFolder);

%% ── LOCAL HELPER FUNCTIONS ──────────────────────────────────────────────

function res = extractTrackingResults(out, stopTime)
% extractTrackingResults  Pull torque, speed, and current time series from
%   a gait-tracking simulation output.
%
%   TODO: update Simscape log paths below to match YOUR model hierarchy.
%   Run  >> out.simlog  in the command window after a test sim to explore
%   the available node names.

try
    sl = out.simlog;
catch
    sl = evalin('base','simlog');
end

% ── Mechanical signals ─────────────────────────────────────────────────
% Actual torque and speed from PMSM block
T_act_raw = sl.PMSM.torque.series.values('N*m');
w_act_raw = sl.PMSM.angular_velocity.series.values('rad/s');
t_raw     = sl.PMSM.torque.series.time;

% Negate torque sign convention (Simscape loads appear negative in motoring)
T_act_raw = -T_act_raw;

% ── Commanded signals (reconstruct from stored gait profile) ───────────
% Since the Simulink model drives from a timeseries, we re-interpolate
% the commanded values at the same time vector for alignment.
gait_torque_ts = evalin('base','gait_torque_ts');
gait_speed_ts  = evalin('base','gait_speed_ts');
T_cmd = interp1(gait_torque_ts.Time, gait_torque_ts.Data, t_raw, 'linear','extrap');
w_cmd = interp1(gait_speed_ts.Time,  gait_speed_ts.Data,  t_raw, 'linear','extrap');

% ── Tracking errors ────────────────────────────────────────────────────
T_err = T_act_raw - T_cmd;
w_err = w_act_raw - w_cmd;

res.t         = t_raw;
res.T_cmd     = T_cmd;
res.T_act     = T_act_raw;
res.w_cmd     = w_cmd;
res.w_act     = w_act_raw;
res.T_err_rms = sqrt(mean(T_err.^2));
res.w_err_rms = sqrt(mean(w_err.^2));
end

function saveFigure(fig, folder, baseFilename)
if ~exist(folder,'dir'); mkdir(folder); end
safeName = regexprep(baseFilename,'[^\w\-]','_');
exportgraphics(fig, fullfile(folder,[safeName,'.png']), 'Resolution',300);
exportgraphics(fig, fullfile(folder,[safeName,'.pdf']), 'ContentType','vector');
savefig(fig, fullfile(folder,[safeName,'.fig']));
fprintf('  Saved: %s (.png/.pdf/.fig)\n', fullfile(folder,safeName));
end
