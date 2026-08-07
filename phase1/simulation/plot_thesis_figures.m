% =========================================================================
%  plot_thesis_figures.m
%  Standalone figure renderer — loads thesis_sim_data.mat and regenerates
%  all thesis figures WITHOUT re-running any simulations.
%
%  Also re-applies the analytical steady-state temperature correction to
%  whatever temperature data is in the .mat file (handles the case where
%  the data was captured before the analytical Tj fix was in place).
%
%  Run this after VirtualSpeedDynoParams.m has completed at least once
%  and saved thesis_sim_data.mat.
% =========================================================================

clear; close all; clc;

%% ── SETTINGS ─────────────────────────────────────────────────────────────
savePlots = true;   % Set false to just display without saving

% ── Locate the .mat file relative to this script ─────────────────────────
projRoot   = fileparts(mfilename('fullpath'));
plotFolder = fullfile(projRoot, 'Inverter_Thesis_Plots');
matPath    = fullfile(plotFolder, 'thesis_sim_data.mat');

if ~exist(matPath, 'file')
    error(['thesis_sim_data.mat not found at:\n  %s\n' ...
        'Run VirtualSpeedDynoParams.m first to generate the simulation data.'], matPath);
end

fprintf('Loading simulation data from:\n  %s\n\n', matPath);
load(matPath);
fprintf('Data loaded successfully.\n');

if ~exist(plotFolder, 'dir'); mkdir(plotFolder); end

%% ── RECONSTRUCT ANY MISSING VARIABLES ────────────────────────────────────
% These may not exist in older .mat files — reconstruct from what is available.

nConfigs = numel(maps);
nT = size(maps(1).Efficiency, 1);
nS = size(maps(1).Efficiency, 2);

% Recover torque / speed sweeps if not saved
if ~exist('torque_sweep','var')
    torque_sweep = [0.5, 1.5, 2.5, 3.5, 4.375];
end
if ~exist('speed_sweep','var')
    speed_sweep = [5, 10, 15, 20, 25];
end
if ~exist('speed_sweep_RPM','var')
    speed_sweep_RPM = speed_sweep .* (60 / (2*pi));
end
if ~exist('T_LIMIT_DEG','var')
    T_LIMIT_DEG = 150;
end

% Motor config labels — rebuild if motorConfigs not present in .mat
if ~exist('motorConfigs','var')
    motorA.label = '48 V GaN (Si-equiv. baseline)';
    motorB.label = '96 V GaN (advanced)';
    motorConfigs = {motorA, motorB};
    fprintf('[WARN] motorConfigs not found in .mat — using default labels.\n');
end

% maps_ft — frequency-tuned inverter-only data for Figure 4
% Requires Section 1B from VirtualSpeedDynoParams.m to have been run.
% If absent, Figure 4 is skipped — scaling approximations are not used
% as they are not suitable for thesis-quality results.
has_maps_ft = exist('maps_ft','var') && ~isempty(maps_ft);
if ~has_maps_ft
    fprintf(['[INFO] maps_ft not found — Figure 4 will be skipped.\n' ...
        '       Re-run VirtualSpeedDynoParams.m to generate it.\n\n']);
    fig4_subtitle  = '';
    fig4_xtlabels  = {};
else
    fig4_subtitle  = 'Inverter Loss Breakdown at Selected Operating Points (48 V GaN @ 40 kHz | 96 V GaN @ 100 kHz)';
    fig4_xtlabels  = {'48V GaN (40 kHz)', '96V GaN (100 kHz)'};
end

%% ── RE-APPLY ANALYTICAL STEADY-STATE TEMPERATURE CORRECTION ──────────────
% The short simulation (0.5 s) cannot heat the large thermal masses.
% We recalculate all temperatures from the loss fields already stored in maps.
%
% Inverter thermal path (EPC2305 GaN FET, 6 FETs sharing one heatsink):
%   R_JC  = 0.2   K/W  (junction-to-case, per FET)
%   R_CA_internal = 6.5 K/W (case-to-ambient internal, per FET)
%   R_conv = 1/(h*A) = 1/(10.25 * 0.015) = 6.504 K/W  (convective, shared)
%   R_CA_eff = 1/(6/6.5 + 1/6.504) = 0.9286 K/W
%
% Motor thermal path (estimated):
%   R_winding_housing = 0.8 K/W
%   R_housing_ambient  = 2.0 K/W

T_amb  = 25.0;
R_JC   = 0.2;
R_CA_eff = 0.9286;
R_wh   = 0.8;
R_ha   = 2.0;

fprintf('Applying analytical steady-state temperature correction...\n');
for cfg = 1:nConfigs
    for t_idx = 1:nT
        for s_idx = 1:nS
            P_inv   = maps(cfg).Loss_Inv(t_idx, s_idx);
            P_motor = maps(cfg).Loss_Motor(t_idx, s_idx);
            maps(cfg).Peak_Tj(t_idx, s_idx)    = T_amb + (P_inv/6)*R_JC + P_inv*R_CA_eff;
            maps(cfg).T_housing(t_idx, s_idx)  = T_amb + P_motor*R_ha;
            maps(cfg).T_winding(t_idx, s_idx)  = maps(cfg).T_housing(t_idx, s_idx) + P_motor*R_wh;
        end
    end
end

% Also correct the gait profile temperatures if available
if exist('gait_T_junction','var') && exist('gait_torque','var')
    % Approximate gait losses from stored data if available, else skip
    fprintf('[INFO] Gait temperatures will retain simulation values (insufficient loss data for correction).\n');
end

% Also correct cont_T_j using maps(1) and maps(2) interpolated at cont_speed_fixed
if exist('cont_torque_sweep','var') && exist('cont_T_j','var')
    for ccfg = 1:min(2, nConfigs)
        for ct_idx = 1:length(cont_torque_sweep)
            % Interpolate loss from maps at cont_speed_fixed
            [~, s_near] = min(abs(speed_sweep - cont_speed_fixed));
            t_vals = torque_sweep;
            P_inv_vec   = maps(ccfg).Loss_Inv(:, s_near);
            P_inv_interp = interp1(t_vals, P_inv_vec, cont_torque_sweep(ct_idx), 'linear', 'extrap');
            cont_T_j(ccfg, ct_idx) = T_amb + (P_inv_interp/6)*R_JC + P_inv_interp*R_CA_eff;
        end
    end
    fprintf('cont_T_j corrected analytically.\n');
end

fprintf('Temperature correction complete.\n\n');

%% ── Figures 1-A/B : 2D Efficiency Block Heatmaps (standard dyno plot) ────────
% Standard industry/thesis format: torque vs speed grid, colour = efficiency.
% imagesc gives crisp discrete blocks matching each physical test point.
nConfigs = numel(maps);
[X_Speed_RPM, Y_Torque] = meshgrid(speed_sweep_RPM, torque_sweep);

cfg_short_labels = {'48 V GaN @ 40 kHz (Si-equiv. baseline)', '96 V GaN @ 100 kHz (advanced)'};

fig1 = figure('Name', 'Efficiency Maps — All Configs', ...
    'Color', [1 1 1], 'Position', [50, 50, 1100, 450], 'Visible', 'off');
tl1 = tiledlayout(1, nConfigs, 'TileSpacing', 'compact', 'Padding', 'compact');

for cfg = 1:nConfigs
    nexttile;
    % Standard motor efficiency contour map
    [C, h] = contourf(X_Speed_RPM, Y_Torque, maps(cfg).Efficiency, 20, 'LineStyle', 'none');
    colormap(turbo); clim([50 100]);
    hold on;
    % Add contour lines with labels for clarity
    [C2, h2] = contour(X_Speed_RPM, Y_Torque, maps(cfg).Efficiency, [60, 70, 80, 85, 90, 95], 'k-', 'LineWidth', 0.5);
    clabel(C2, h2, 'FontSize', 8, 'Color', 'k', 'FontWeight', 'bold', 'LabelSpacing', 200);
    
    % Overlay the actual simulated test points as small dots
    plot(X_Speed_RPM(:), Y_Torque(:), 'w.', 'MarkerSize', 4);
    hold off;
    xlabel('Speed (RPM)',    'FontWeight', 'bold', 'FontSize', 11);
    if cfg == 1
        ylabel('Torque (N\cdotm)', 'FontWeight', 'bold', 'FontSize', 11);
    end
    title(cfg_short_labels{cfg}, 'FontSize', 10, 'FontWeight', 'bold');
    xticks(speed_sweep_RPM(1:4)); yticks(torque_sweep);
    xlim([speed_sweep_RPM(1), speed_sweep_RPM(4)]);
    grid off; box on;
end
cb = colorbar;
cb.Label.String = 'System Efficiency (%)';
cb.Label.FontWeight = 'bold';
cb.Layout.Tile = 'east';
sgtitle('System Efficiency Maps — 48 V GaN vs 96 V GaN', 'FontSize', 13, 'FontWeight', 'bold');
fprintf('Figure 1-A/B rendered: 2D Efficiency Block Heatmaps\n');
if savePlots; saveFigure(fig1, plotFolder, 'efficiency_maps_2D_heatmap'); end

%% ── Figure 1-C : Efficiency Comparison — Curves + Difference Map ──────────
% Top row: η vs Torque at 3 representative speeds, both configs on same axes.
% Bottom left: ΔEfficiency heatmap (96V GaN − 48V GaN).

speeds_to_show = [1, 3, 4];   % low / mid / high speed indices (191, 955, 1337 RPM)
speed_labels   = arrayfun(@(s) sprintf('%.0f RPM', speed_sweep_RPM(s)), ...
    speeds_to_show, 'UniformOutput', false);
cfg_ls  = {'-o', '--s'};
cfg_colors = [0 0.447 0.741;    % Blue  — 48V GaN
              0.850 0.325 0.098]; % Orange — 96V GaN

fig1d = figure('Name', 'Efficiency Comparison', ...
    'Color', [1 1 1], 'Position', [50, 50, 1300, 700], 'Visible', 'off');
tl1d = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

% ── Top row: η vs Torque at each selected speed ────────────────────────────
for si = 1:3
    s_idx = speeds_to_show(si);
    nexttile;
    hold on;
    for cfg = 1:nConfigs
        plot(torque_sweep, maps(cfg).Efficiency(:, s_idx), cfg_ls{cfg}, ...
            'Color', cfg_colors(cfg,:), 'LineWidth', 2, 'MarkerSize', 7, ...
            'DisplayName', cfg_short_labels{cfg});
    end
    xlabel('Torque (N\cdotm)',   'FontWeight', 'bold', 'FontSize', 10);
    ylabel('Efficiency (%)',      'FontWeight', 'bold', 'FontSize', 10);
    title(speed_labels{si},      'FontSize', 10, 'FontWeight', 'bold');
    ylim([50 96]); grid on; hold off;
    if si == 1
        legend('Location', 'southeast', 'FontSize', 8);
    end
end

% ── Bottom: Δη heatmap (96V − 48V) + summary text ────────────────────────
delta_map = maps(2).Efficiency - maps(1).Efficiency;
delta_levels = -10:1:10;

nexttile([1 2]);
[C2, h2] = contourf(X_Speed_RPM, Y_Torque, delta_map, delta_levels);
clabel(C2, h2, 'FontSize', 7, 'Color', [0.15 0.15 0.15]);
colormap(gca, bwr_colormap());
clim([-10 10]);
cb2 = colorbar;
cb2.Label.String = 'Δη (pp)';
cb2.Label.FontWeight = 'bold';
hold on;
contour(X_Speed_RPM, Y_Torque, delta_map, [0 0], 'k-', 'LineWidth', 2);
hold off;
xlabel('Speed (RPM)',  'FontWeight', 'bold', 'FontSize', 10);
ylabel('Torque (N\cdotm)', 'FontWeight', 'bold', 'FontSize', 10);
title('Δη = 96 V GaN − 48 V GaN', 'FontSize', 10, 'FontWeight', 'bold');
xlim([speed_sweep_RPM(1), speed_sweep_RPM(4)]);
grid on; box on;

% Summary statistics tile
nexttile;
axis off;
valid_deltas = delta_map(~isnan(delta_map));
if isempty(valid_deltas)
    mean_diff = NaN; max_diff = NaN; min_diff = NaN;
else
    mean_diff = mean(valid_deltas);
    max_diff  = max(valid_deltas);
    min_diff  = min(valid_deltas);
end
summary_txt = sprintf([ ...
    '96 V GaN vs 48 V GaN\n' ...
    '  Mean Δη = %+.2f pp\n' ...
    '  Max  Δη = %+.2f pp\n' ...
    '  Min  Δη = %+.2f pp'], ...
    mean_diff, max_diff, min_diff);
text(0.05, 0.55, summary_txt, 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Courier New', ...
    'VerticalAlignment', 'middle', 'BackgroundColor', [0.95 0.95 0.95]);
title('Summary Statistics', 'FontSize', 10, 'FontWeight', 'bold');

sgtitle('Efficiency Comparison — 96 V Systems vs 48 V Baseline', ...
    'FontSize', 13, 'FontWeight', 'bold');
fprintf('Figure 1-D rendered: Efficiency Comparison (curves + difference maps)\n');
if savePlots; saveFigure(fig1d, plotFolder, 'efficiency_comparison'); end


%% ── Figure 4 : Inverter Loss Breakdown Bar Charts ────────────────────────
if ~has_maps_ft
    fprintf('[SKIP] Figure 4: maps_ft not available — re-run VirtualSpeedDynoParams.m first.\n');
else
    sel_pts    = {[1,1], [3,3], [5,5]};
    sel_labels = {'Low Load\n(0.5 Nm / 48 RPM)', 'Mid Load\n(2.5 Nm / 143 RPM)', 'Peak Load\n(4.4 Nm / 239 RPM)'};
    loss_categories = {'DC Bus (I²R)', 'Inv. Cond.', 'Inv. Switch.'};
    nSel       = length(sel_pts);
    bar_colors = [0.6 0.6 0.6; 0.2 0.6 0.9; 0.0 0.2 0.7];
    fig4 = figure('Name', 'Inverter Loss Breakdown Bar Charts', ...
        'Color', [1 1 1], 'Position', [100, 700, 950, 480], 'Visible', 'off');
    tiledlayout(1, nSel, 'TileSpacing', 'compact', 'Padding', 'compact');
    for sp = 1:nSel
        ti = sel_pts{sp}(1); si = sel_pts{sp}(2);
        nexttile; hold on;
        bar_data = zeros(nConfigs, 3);
        for cfg = 1:nConfigs
            bar_data(cfg,:) = [ ...
                maps_ft(cfg).Loss_Cable(ti,si), ...
                maps_ft(cfg).Loss_InvCond(ti,si), ...
                maps_ft(cfg).Loss_InvSW(ti,si)];
        end
        b = bar(1:nConfigs, bar_data, 'stacked');
        for k = 1:3; b(k).FaceColor = bar_colors(k,:); end
        xticks(1:nConfigs); xticklabels(fig4_xtlabels);
        xtickangle(15);
        ylabel('Inverter Power Loss (W)', 'FontWeight', 'bold');
        title(sprintf(sel_labels{sp}), 'FontSize', 10, 'FontWeight', 'bold');
        legend(loss_categories, 'Location', 'northeast', 'FontSize', 8);
        grid on; hold off;
    end
    sgtitle(fig4_subtitle, 'FontSize', 12, 'FontWeight', 'bold');
    fprintf('Figure 4 rendered: Inverter Loss Breakdown\n');
    if savePlots; saveFigure(fig4, plotFolder, 'loss_breakdown_bar_charts'); end
end

%% ── Figure 5 : Switching-Frequency Sweep — Total Loss ───────────────────
if exist('fsw_total_loss','var')
    fig5 = figure('Name', 'Switching Frequency Sweep - Total Loss', ...
        'Color', [1 1 1], 'Position', [100, 100, 820, 480], 'Visible', 'off');
    hold on;
    line_styles = {'-o', '--s'};
    for fcfg = 1:nFswCfg
        plot(fsw_sweep/1e3, fsw_total_loss(fcfg,:), line_styles{fcfg}, ...
            'Color', cfg_colors(fcfg,:), 'LineWidth', 2, 'MarkerSize', 7, ...
            'DisplayName', fsw_cfg_labels{fcfg});
    end
    xlabel('Switching Frequency (kHz)', 'FontWeight', 'bold', 'FontSize', 11);
    ylabel('Total Actuator Loss (W)',   'FontWeight', 'bold', 'FontSize', 11);
    title('Switching-Frequency Sweep — Total Loss vs. f_{sw} (48 V vs. 96 V)', ...
        'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'northwest', 'FontSize', 10); grid on; xlim([15 105]);
    hold off;
    fprintf('Figure 5 rendered: Frequency Sweep - Total Loss\n');
    if savePlots; saveFigure(fig5, plotFolder, 'frequency_sweep_total_loss'); end
else
    fprintf('[SKIP] Figure 5: fsw_total_loss not found in .mat\n');
end

%% ── Figure 6 : Motor Loss and Inverter Total Loss vs. Frequency ───────
if exist('fsw_motor_loss','var')
    fig6 = figure('Name', 'Motor & Inverter Loss vs Frequency', ...
        'Color', [1 1 1], 'Position', [800, 100, 1000, 600], 'Visible', 'off');
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    
    % Top-Left: Motor Loss (Mid Load)
    nexttile;
    hold on;
    plot(fsw_sweep/1e3, fsw_motor_loss(1,:), '-o',  'Color', [0.0 0.447 0.741], ...
        'LineWidth', 2, 'MarkerSize', 7, 'DisplayName', 'Motor Loss — 48 V GaN');
    plot(fsw_sweep/1e3, fsw_motor_loss(2,:), '--o', 'Color', [0.0 0.447 0.741]*0.55, ...
        'LineWidth', 2, 'MarkerSize', 7, 'DisplayName', 'Motor Loss — 96 V GaN');
    ylabel('Motor Total Loss (W)', 'FontWeight', 'bold', 'FontSize', 11);
    title('Mid Load (4.5 Nm): Motor Loss', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'northeast', 'FontSize', 9);
    grid on; xlim([15 105]); hold off;

    % Top-Right: Motor Loss (Full Load)
    nexttile;
    hold on;
    if exist('fsw_motor_loss_fl','var')
        plot(fsw_sweep/1e3, fsw_motor_loss_fl(1,:), '-o',  'Color', [0.0 0.447 0.741], ...
            'LineWidth', 2, 'MarkerSize', 7, 'DisplayName', 'Motor Loss — 48 V GaN');
        plot(fsw_sweep/1e3, fsw_motor_loss_fl(2,:), '--o', 'Color', [0.0 0.447 0.741]*0.55, ...
            'LineWidth', 2, 'MarkerSize', 7, 'DisplayName', 'Motor Loss — 96 V GaN');
    end
    title('Full Load (9.0 Nm): Motor Loss', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'northeast', 'FontSize', 9);
    grid on; xlim([15 105]); hold off;

    % Bottom-Left: Inverter Total Loss (Mid Load)
    nexttile;
    hold on;
    plot(fsw_sweep/1e3, fsw_sw_loss(1,:) + fsw_cond_loss(1,:),  '-s',   'Color', [0.850 0.325 0.098], ...
        'LineWidth', 2, 'MarkerSize', 7, 'DisplayName', 'Inv Total Loss — 48 V GaN');
    plot(fsw_sweep/1e3, fsw_sw_loss(2,:) + fsw_cond_loss(2,:), '--s',   'Color', [0.850 0.325 0.098]*0.55, ...
        'LineWidth', 2, 'MarkerSize', 7, 'DisplayName', 'Inv Total Loss — 96 V GaN');
    xlabel('Switching Frequency (kHz)', 'FontWeight', 'bold', 'FontSize', 11);
    ylabel('Inv. Total Loss (W)', 'FontWeight', 'bold', 'FontSize', 11);
    title('Mid Load (4.5 Nm): Inverter Loss', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'northwest', 'FontSize', 9);
    grid on; xlim([15 105]); hold off;

    % Bottom-Right: Inverter Total Loss (Full Load)
    nexttile;
    hold on;
    if exist('fsw_sw_loss_fl','var') && exist('fsw_cond_loss_fl','var')
        plot(fsw_sweep/1e3, fsw_sw_loss_fl(1,:) + fsw_cond_loss_fl(1,:),  '-s',   'Color', [0.850 0.325 0.098], ...
            'LineWidth', 2, 'MarkerSize', 7, 'DisplayName', 'Inv Total Loss — 48 V GaN');
        plot(fsw_sweep/1e3, fsw_sw_loss_fl(2,:) + fsw_cond_loss_fl(2,:), '--s',   'Color', [0.850 0.325 0.098]*0.55, ...
            'LineWidth', 2, 'MarkerSize', 7, 'DisplayName', 'Inv Total Loss — 96 V GaN');
    end
    xlabel('Switching Frequency (kHz)', 'FontWeight', 'bold', 'FontSize', 11);
    title('Full Load (9.0 Nm): Inverter Loss', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'northwest', 'FontSize', 9);
    grid on; xlim([15 105]); hold off;
    
    sgtitle(sprintf('Loss Breakdown vs Switching Frequency (ω = 100 rad/s)'), 'FontSize', 12, 'FontWeight', 'bold');
    fprintf('Figure 6 rendered: Motor & Inverter Loss vs Frequency (Mid & Full Load)\n');
    if savePlots; saveFigure(fig6, plotFolder, 'motor_and_inv_loss_vs_frequency'); end
else
    fprintf('[SKIP] Figure 6: fsw_motor_loss not found in .mat\n');
end

%% ── Figure 7 : Phase Current Comparison ─────────────────────────────────
s_fixed = 3;  % mid-speed index (15 rad/s)
if isfield(maps(1), 'I_rms') && any(maps(1).I_rms(:) ~= 0)
    fig7 = figure('Name', 'Current Comparison 48V vs 96V', ...
        'Color', [1 1 1], 'Position', [100, 100, 820, 480], 'Visible', 'off');
    tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile; hold on;
    for cfg = 1:nConfigs
        mk = {'-o','--s',':^'}; mk = mk{cfg};
        plot(torque_sweep, maps(cfg).I_rms(:,s_fixed), mk, ...
            'Color', cfg_colors(cfg,:), 'LineWidth', 2, 'MarkerSize', 7, ...
            'DisplayName', motorConfigs{cfg}.label);
    end
    xlabel('Torque (N·m)', 'FontWeight', 'bold');
    ylabel('I_{RMS} — Phase (A)', 'FontWeight', 'bold');
    title('RMS Phase Current vs. Torque', 'FontWeight', 'bold');
    legend('Location', 'northwest'); grid on; hold off;

    nexttile; hold on;
    for cfg = 1:nConfigs
        mk = {'-o','--s',':^'}; mk = mk{cfg};
        plot(torque_sweep, maps(cfg).I_peak(:,s_fixed), mk, ...
            'Color', cfg_colors(cfg,:), 'LineWidth', 2, 'MarkerSize', 7, ...
            'DisplayName', motorConfigs{cfg}.label);
    end
    xlabel('Torque (N·m)', 'FontWeight', 'bold');
    ylabel('I_{peak} — Phase (A)', 'FontWeight', 'bold');
    title('Peak Phase Current vs. Torque', 'FontWeight', 'bold');
    legend('Location', 'northwest'); grid on; hold off;

    sgtitle(sprintf('Phase Current Comparison at Equal Mechanical Output (ω = %.0f rad/s)', speed_sweep(s_fixed)), ...
        'FontSize', 13, 'FontWeight', 'bold');
    fprintf('Figure 7 rendered: Current Comparison\n');
    if savePlots; saveFigure(fig7, plotFolder, 'current_comparison_48V_vs_96V'); end
else
    fprintf('[SKIP] Figure 7: I_rms data is all zeros (captured before current logging was added)\n');
end

%% ── Figure 8 : Steady-State Temperature Map (now uses analytical Tj) ─────
s_max = nS;
fig8 = figure('Name', 'Steady-State Temperature Plot (Analytical)', ...
    'Color', [1 1 1], 'Position', [100, 100, 820, 480], 'Visible', 'off');
hold on;
for cfg = 1:nConfigs
    plot(torque_sweep, maps(cfg).Peak_Tj(:,s_max),    '-',  'Color', cfg_colors(cfg,:), ...
        'LineWidth', 2,   'DisplayName', sprintf('T_j — %s',       motorConfigs{cfg}.label));
    plot(torque_sweep, maps(cfg).T_winding(:,s_max), '--',  'Color', cfg_colors(cfg,:), ...
        'LineWidth', 1.5, 'DisplayName', sprintf('T_{wind} — %s',  motorConfigs{cfg}.label));
    plot(torque_sweep, maps(cfg).T_housing(:,s_max),  ':',  'Color', cfg_colors(cfg,:), ...
        'LineWidth', 1.5, 'DisplayName', sprintf('T_{hous} — %s',  motorConfigs{cfg}.label));
end
yline(150, 'r--', 'LineWidth', 2, 'DisplayName', '150°C Limit');
xlabel('Electromagnetic Torque (N·m)', 'FontWeight', 'bold', 'FontSize', 11);
ylabel('Temperature (°C)',             'FontWeight', 'bold', 'FontSize', 11);
title(sprintf('Steady-State Temperature vs. Load at Peak Speed (%.0f RPM) — Analytical', ...
    speed_sweep_RPM(s_max)), 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 8, 'NumColumns', 2); grid on; hold off;
fprintf('Figure 8 rendered: Steady-State Temperature Map\n');
if savePlots; saveFigure(fig8, plotFolder, 'steady_state_temperature_map'); end

%% ── Figure 9 : Temperature Rise vs. Time (Gait Profile) ─────────────────
if exist('gait_time','var') && exist('gait_T_junction','var')
    fig9 = figure('Name', 'Temperature Rise vs Time - Gait Profile', ...
        'Color', [1 1 1], 'Position', [100, 100, 820, 480], 'Visible', 'off');
    hold on;
    plot(gait_time, gait_T_junction, '-o',  'Color', [0.85 0.33 0.1], ...
        'LineWidth', 2, 'MarkerSize', 7, 'DisplayName', 'T_{junction} (FET)');
    plot(gait_time, gait_T_winding,  '--s', 'Color', [0.47 0.67 0.19], ...
        'LineWidth', 2, 'MarkerSize', 7, 'DisplayName', 'T_{winding}');
    plot(gait_time, gait_T_housing,   ':^', 'Color', [0.49 0.18 0.56], ...
        'LineWidth', 2, 'MarkerSize', 7, 'DisplayName', 'T_{housing}');
    yline(150, 'r--', 'LineWidth', 2, 'DisplayName', '150°C Limit');
    xlabel('Gait Profile Time (s)', 'FontWeight', 'bold', 'FontSize', 11);
    ylabel('Temperature (°C)',      'FontWeight', 'bold', 'FontSize', 11);
    title('Temperature Rise Over Representative Gait Cycle (48 V Baseline)', ...
        'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'northeast', 'FontSize', 10); grid on; hold off;
    fprintf('Figure 9 rendered: Gait Temperature Rise\n');
    if savePlots; saveFigure(fig9, plotFolder, 'gait_temperature_rise'); end
else
    fprintf('[SKIP] Figure 9: gait temperature data not found in .mat\n');
end

%% ── Figure 10 : Continuous Torque Capability (analytical Tj) ─────────────
if exist('cont_torque_sweep','var') && exist('cont_T_j','var')
    fig10 = figure('Name', 'Continuous Torque Capability - Thermal Limit', ...
        'Color', [1 1 1], 'Position', [100, 100, 820, 480], 'Visible', 'off');
    hold on;
    cont_cap_labels = {'48 V Baseline', '96 V Baseline Motor'};
    for ccfg = 1:min(2, nConfigs)
        plot(cont_torque_sweep, cont_T_j(ccfg,:), '-o', 'Color', cfg_colors(ccfg,:), ...
            'LineWidth', 2, 'MarkerSize', 7, 'DisplayName', cont_cap_labels{ccfg});
        idx_over = find(cont_T_j(ccfg,:) >= T_LIMIT_DEG, 1);
        if ~isempty(idx_over) && idx_over > 1
            T_cap = interp1(cont_T_j(ccfg,idx_over-1:idx_over), ...
                cont_torque_sweep(idx_over-1:idx_over), T_LIMIT_DEG);
            xline(T_cap, '--', 'Color', cfg_colors(ccfg,:), 'LineWidth', 1.5, ...
                'Label', sprintf('%.2f Nm (%s)', T_cap, cont_cap_labels{ccfg}), 'FontSize', 9);
        end
    end
    yline(T_LIMIT_DEG, 'r--', 'LineWidth', 2, 'DisplayName', '150°C Thermal Limit');
    xlabel('Commanded Torque (N·m)',         'FontWeight', 'bold', 'FontSize', 11);
    ylabel('Peak Junction Temperature (°C)', 'FontWeight', 'bold', 'FontSize', 11);
    title(sprintf('Continuous Torque Capability (ω = %.0f rad/s) — 48 V vs. 96 V', cont_speed_fixed), ...
        'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'northwest', 'FontSize', 10); grid on; hold off;
    fprintf('Figure 10 rendered: Continuous Torque Capability\n');
    if savePlots; saveFigure(fig10, plotFolder, 'continuous_torque_capability'); end
else
    fprintf('[SKIP] Figure 10: cont_T_j data not found in .mat\n');
end

%% ── Figure 11 : ΔT Rise vs. RMS Phase Current — Mid Speed, Torque Sweep ─────
% Fixed at mid-speed (15 rad/s, index 3), sweeping across all torque levels.
% One clean curve per config — matches the reference chart style.
T_ambient_degC = 25;
s_mid = 3;  % speed index for mid-speed (15 rad/s / ~143 RPM)

if isfield(maps(1), 'I_rms') && any(maps(1).I_rms(:) ~= 0)
    fig11 = figure('Name', 'Temperature Rise vs RMS Phase Current (Mid Speed)', ...
        'Color', [1 1 1], 'Position', [100, 100, 820, 520], 'Visible', 'off');
    hold on;
    for cfg = 1:nConfigs
        % Extract the mid-speed column across all torques
        I_rms_vec = maps(cfg).I_rms(:, s_mid);          % [nT × 1]
        dT_vec    = maps(cfg).Peak_Tj(:, s_mid) - T_ambient_degC;
        % Sort by ascending current for a clean line
        [I_sorted, srt] = sort(I_rms_vec);
        dT_sorted = dT_vec(srt);
        plot(I_sorted, dT_sorted, '-o', ...
            'Color', cfg_colors(cfg,:), 'LineWidth', 2, 'MarkerSize', 7, ...
            'DisplayName', motorConfigs{cfg}.label);
    end
    xlabel('RMS Phase Current (A)', 'FontWeight', 'bold', 'FontSize', 11);
    ylabel('\DeltaT_{junction} = T_j - T_{amb} (°C)', 'FontWeight', 'bold', 'FontSize', 11);
    title(sprintf('Junction Temperature Rise vs. RMS Phase Current\n(Fixed Speed: %.0f rad/s / %.0f RPM, torque sweep)', ...
        speed_sweep(s_mid), speed_sweep_RPM(s_mid)), 'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'northwest', 'FontSize', 10); grid on; hold off;
    fprintf('Figure 11 rendered: Temperature Rise vs I_RMS (mid speed, torque sweep)\n');
    if savePlots; saveFigure(fig11, plotFolder, 'temperature_rise_vs_Irms'); end
else
    fprintf('[SKIP] Figure 11: I_rms data is all zeros (captured before current logging was added)\n');
end

%% ── Figure 12 : DC-Link Capacitor RMS Current ────────────────────────────
% Checklist Section A5: DC-link capacitor RMS current vs torque for each
% config (curves at mid-speed) + 2-D heatmap for 48 V vs 96 V.

if isfield(maps(1), 'I_dc_rms') && any(maps(1).I_dc_rms(:) ~= 0)
    s_mid12 = 3;   % mid-speed index (15 rad/s)

    fig12 = figure('Name', 'DC-Link Capacitor RMS Current', ...
        'Color', [1 1 1], 'Position', [100, 100, 1300, 520], 'Visible', 'off');
    tl12 = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

    % ── Left: Idc_rms vs Torque curves at mid-speed ─────────────────────
    nexttile;
    hold on;
    mk12 = {'-o','--s',':^'};
    for cfg = 1:nConfigs
        plot(torque_sweep, maps(cfg).I_dc_rms(:, s_mid12), mk12{cfg}, ...
            'Color', cfg_colors(cfg,:), 'LineWidth', 2, 'MarkerSize', 7, ...
            'DisplayName', motorConfigs{cfg}.label);
    end
    % Overlay theoretical 0.5–0.7 × I_phase_rms band for reference
    if isfield(maps(1), 'I_rms') && any(maps(1).I_rms(:) ~= 0)
        I_ph_ref = maps(1).I_rms(:, s_mid12);
        fill([torque_sweep, fliplr(torque_sweep)], ...
            [0.5*I_ph_ref; flipud(0.7*I_ph_ref)]', ...
            [0.85 0.85 0.85], 'FaceAlpha', 0.35, 'EdgeColor', 'none', ...
            'DisplayName', '0.5–0.7 × I_{phase} (theory, 48V)');
    end
    xlabel('Torque (N·m)',            'FontWeight', 'bold', 'FontSize', 10);
    ylabel('I_{DC,rms} (A)',          'FontWeight', 'bold', 'FontSize', 10);
    title(sprintf('DC-Link RMS Current vs Torque\n(ω = %.0f rad/s)', speed_sweep(s_mid12)), ...
        'FontSize', 10, 'FontWeight', 'bold');
    legend('Location', 'northwest', 'FontSize', 8); grid on; hold off;

    % Compute shared color and contour scale so both heatmaps can be visually compared side-by-side
    max_Idc_fig12 = max([maps(1).I_dc_rms(:); maps(2).I_dc_rms(:)]);
    levels_fig12  = linspace(0, max_Idc_fig12, 12);

    % ── Middle: 2-D heatmap — 48 V baseline ────────────────────────────
    nexttile;
    contourf(X_Speed_RPM, Y_Torque, maps(1).I_dc_rms, levels_fig12);
    colormap(gca, parula); clim([0 max_Idc_fig12]); cb12a = colorbar;
    cb12a.Label.String = 'I_{DC,rms} (A)';
    cb12a.Label.FontWeight = 'bold';
    xlabel('Speed (RPM)',    'FontWeight', 'bold', 'FontSize', 10);
    ylabel('Torque (N·m)',  'FontWeight', 'bold', 'FontSize', 10);
    title('DC-Link RMS Current Map\n48 V Baseline', ...
        'FontSize', 10, 'FontWeight', 'bold');
    grid on; box on;

    % ── Right: 2-D heatmap — 96 V co-design ────────────────────────────
    nexttile;
    contourf(X_Speed_RPM, Y_Torque, maps(2).I_dc_rms, levels_fig12);
    colormap(gca, parula); clim([0 max_Idc_fig12]); cb12b = colorbar;
    cb12b.Label.String = 'I_{DC,rms} (A)';
    cb12b.Label.FontWeight = 'bold';
    xlabel('Speed (RPM)',   'FontWeight', 'bold', 'FontSize', 10);
    ylabel('Torque (N·m)', 'FontWeight', 'bold', 'FontSize', 10);
    title('DC-Link RMS Current Map\n96 V GaN (rewound double Kt)', ...
        'FontSize', 10, 'FontWeight', 'bold');
    grid on; box on;

    sgtitle('DC-Link Capacitor RMS Current — Sizing Input (Checklist §A5)', ...
        'FontSize', 12, 'FontWeight', 'bold');
    fprintf('Figure 12 rendered: DC-Link Capacitor RMS Current\n');
    if savePlots; saveFigure(fig12, plotFolder, 'dclink_rms_current'); end
else
    fprintf('[SKIP] Figure 12: I_dc_rms data is all zeros (re-run VirtualSpeedDynoParams.m)\n');
end

%% ── Figure 13 : Gait Mechanical Energy Budget ────────────────────────────
% Checklist Section C3: Energy per gait waypoint interval and cumulative
% energy over the full gait cycle.  Computed analytically from stored gait
% profile — no re-simulation required.  Since both 48 V and 96 V drive the
% SAME mechanical demand, the mechanical energy is identical; the efficiency
% maps are used to estimate the electrical (input) energy per segment.

if exist('gait_time','var') && exist('gait_torque','var') && exist('gait_speed','var')

    nGait13 = length(gait_time);

    % Mechanical power at each knot point [W]
    P_mech_gait = gait_torque .* gait_speed;   % T × ω

    % Mechanical energy per interval via trapezoid rule [J]
    dt_intervals = diff(gait_time);             % nGait-1 intervals
    E_mech_seg   = 0.5 * (P_mech_gait(1:end-1) + P_mech_gait(2:end)) .* dt_intervals;

    % Electrical energy per interval — use efficiency maps interpolated at
    % each knot point (average of segment endpoints).
    % Clamp speed to sweep range for safe interpolation.
    E_elec_seg = zeros(2, nGait13-1);   % rows: [48V, 96V]
    for cfgIdx = 1:2
        for seg = 1:nGait13-1
            T_mid = 0.5*(gait_torque(seg) + gait_torque(seg+1));
            w_mid = 0.5*(gait_speed(seg)  + gait_speed(seg+1));
            RPM_mid = w_mid * (60/(2*pi));
            T_mid_c = max(min(T_mid, max(torque_sweep)), min(torque_sweep));
            R_mid_c = max(min(RPM_mid, max(speed_sweep_RPM)), min(speed_sweep_RPM));
            eta_mid = interp2(X_Speed_RPM, Y_Torque, maps(cfgIdx).Efficiency, ...
                R_mid_c, T_mid_c, 'linear', NaN);
            if isnan(eta_mid) || eta_mid <= 0
                eta_mid = 70;   % conservative fallback [%]
            end
            E_elec_seg(cfgIdx, seg) = E_mech_seg(seg) / (eta_mid / 100);
        end
    end

    % Segment midpoint times for x-axis
    t_mid_seg = 0.5*(gait_time(1:end-1) + gait_time(2:end));
    seg_labels = arrayfun(@(a,b) sprintf('%.1f–%.1f s', a, b), ...
        gait_time(1:end-1), gait_time(2:end), 'UniformOutput', false);

    fig13 = figure('Name', 'Gait Mechanical Energy Budget', ...
        'Color', [1 1 1], 'Position', [100, 100, 1300, 520], 'Visible', 'off');
    tl13 = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

    % ── Left: Mechanical power profile over gait ─────────────────────
    nexttile;
    hold on;
    yyaxis left;
    plot(gait_time, P_mech_gait, '-o', 'Color', [0.2 0.6 0.2], ...
        'LineWidth', 2, 'MarkerSize', 7, 'DisplayName', 'P_{mech} (W)');
    ylabel('Mechanical Power (W)', 'FontWeight', 'bold');
    yyaxis right;
    plot(gait_time, gait_torque, '--s', 'Color', [0.85 0.33 0.1], ...
        'LineWidth', 1.5, 'MarkerSize', 6, 'DisplayName', 'Torque (N·m)');
    ylabel('Torque (N·m)', 'FontWeight', 'bold');
    xlabel('Gait Time (s)', 'FontWeight', 'bold');
    title('Mechanical Power Profile', 'FontWeight', 'bold', 'FontSize', 10);
    legend('Location', 'northeast', 'FontSize', 8); grid on; hold off;

    % ── Middle: Electrical energy per gait segment (bar chart) ────────
    nexttile;
    hold on;
    bar_x = 1:nGait13-1;
    b13 = bar(bar_x, [E_elec_seg(1,:); E_elec_seg(2,:)]', 'grouped');
    b13(1).FaceColor = cfg_colors(1,:);
    b13(2).FaceColor = cfg_colors(2,:);
    xticks(bar_x); xticklabels(seg_labels);
    xtickangle(35);
    ylabel('Electrical Energy (J)', 'FontWeight', 'bold');
    title('Input Energy per Gait Segment', 'FontWeight', 'bold', 'FontSize', 10);
    legend({'48 V Baseline','96 V Base Motor'}, 'Location', 'northwest', 'FontSize', 8);
    grid on; hold off;

    % ── Right: Cumulative electrical energy over gait ─────────────────
    nexttile;
    hold on;
    cum48 = [0, cumsum(E_elec_seg(1,:))];
    cum96 = [0, cumsum(E_elec_seg(2,:))];
    plot(gait_time, cum48, '-o', 'Color', cfg_colors(1,:), ...
        'LineWidth', 2, 'MarkerSize', 7, 'DisplayName', '48 V Baseline');
    plot(gait_time, cum96, '--s', 'Color', cfg_colors(2,:), ...
        'LineWidth', 2, 'MarkerSize', 7, 'DisplayName', '96 V Base Motor');
    xlabel('Gait Time (s)',          'FontWeight', 'bold');
    ylabel('Cumulative Energy (J)',  'FontWeight', 'bold');
    title('Cumulative Input Energy', 'FontWeight', 'bold', 'FontSize', 10);
    legend('Location', 'northwest', 'FontSize', 8);
    % Annotate total savings
    dE = cum48(end) - cum96(end);
    text(gait_time(1), cum96(end)*0.9, ...
        sprintf('Savings: %.2f J (%.1f%%)', dE, 100*dE/cum48(end)), ...
        'FontSize', 9, 'Color', [0.2 0.2 0.2]);
    grid on; hold off;

    sgtitle('Gait Cycle Energy Budget — 48 V vs 96 V (Checklist §C3)', ...
        'FontSize', 12, 'FontWeight', 'bold');
    fprintf('Figure 13 rendered: Gait Mechanical Energy Budget\n');
    if savePlots; saveFigure(fig13, plotFolder, 'gait_energy_budget'); end
else
    fprintf('[SKIP] Figure 13: gait profile data not found in .mat\n');
end

fprintf('\n===== All figures rendered from saved data. =====\n');
fprintf('Output folder: %s\n', plotFolder);

%% ── SAVE UPDATED .MAT (with corrected temperatures) ─────────────────────
save(matPath, '-append', 'maps', 'cont_T_j');
fprintf('Corrected temperature data written back to thesis_sim_data.mat\n');


%% ── LOCAL HELPER ─────────────────────────────────────────────────────────

function saveFigure(fig, folder, baseFilename)
if ~exist(folder, 'dir'); mkdir(folder); end
safeName = regexprep(baseFilename, '[^\w\-]', '_');

pngPath = fullfile(folder, [safeName, '.png']);
exportgraphics(fig, pngPath, 'Resolution', 300);

pdfPath = fullfile(folder, [safeName, '.pdf']);
exportgraphics(fig, pdfPath, 'ContentType', 'vector');

figPath = fullfile(folder, [safeName, '.fig']);
savefig(fig, figPath);

fprintf('  Saved: %s (.png / .pdf / .fig)\n', fullfile(folder, safeName));
end

function cmap = bwr_colormap(n)
% Blue-White-Red diverging colourmap for ΔEfficiency plots.
% Blue  = negative (96V worse than 48V)
% White = zero (break-even)
% Red   = positive (96V better than 48V)
if nargin < 1; n = 256; end
m1 = ceil(n/2);
m2 = floor(n/2);
blue_to_white = [linspace(0.1, 1, m1)', linspace(0.3, 1, m1)', ones(m1, 1)];
white_to_red  = [ones(m2, 1), linspace(1, 0.2, m2)', linspace(1, 0.1, m2)'];
cmap = [blue_to_white; white_to_red];
end
