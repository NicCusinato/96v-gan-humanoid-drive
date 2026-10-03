% 1. Load the telemetry CSV file
data = readtable('Motor_Telemetry_20260918_133824.csv');

% 2. Isolate the steady-state sinusoidal region (t >= 8 seconds)
idx = data.Time_s >= 8.0;
t             = data.Time_s(idx);
Ia            = data.Ia_Amps_Raw(idx);
Ib            = data.Ib_Amps_Raw(idx);
speed_smooth  = data.Speed_RPM_Smoothed(idx);

% 3. Initialize Professional Figure with two stacked subplots
fig = figure('Color', 'w', 'Position', [100, 100, 900, 650]);
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

% --- Subplot 1: Stator Currents ---
nexttile;
plot(t, Ia, 'LineWidth', 1.4, 'DisplayName', '$I_a$ (Phase A)');
hold on;
plot(t, Ib, 'LineWidth', 1.4, 'DisplayName', '$I_b$ (Phase B)');
hold off;

ax1 = gca;
ax1.FontSize = 11;
ax1.FontName = 'Times New Roman';
ax1.TickLabelInterpreter = 'latex';
ax1.XMinorTick = 'on';
ax1.YMinorTick = 'on';
grid on; grid minor;
ax1.GridAlpha = 0.2; ax1.MinorGridAlpha = 0.1;

ylabel('Stator Current (A)', 'Interpreter', 'latex', 'FontSize', 13);
title('\textbf{PMSM Telemetry: Stator Currents and Smoothed Rotor Speed}', ...
      'Interpreter', 'latex', 'FontSize', 14);

leg = legend('Location', 'best');
leg.Interpreter = 'latex';
leg.FontSize = 11;
leg.Box = 'off';

% --- Subplot 2: Smoothed Motor Speed ---
nexttile;
plot(t, speed_smooth, 'Color', [0, 0.45, 0.74], 'LineWidth', 1.6, 'DisplayName', 'Smoothed Speed');

ax2 = gca;
ax2.FontSize = 11;
ax2.FontName = 'Times New Roman';
ax2.TickLabelInterpreter = 'latex';
ax2.XMinorTick = 'on';
ax2.YMinorTick = 'on';
grid on; grid minor;
ax2.GridAlpha = 0.2; ax2.MinorGridAlpha = 0.1;

xlabel('Time $t$ (s)', 'Interpreter', 'latex', 'FontSize', 13);
ylabel('Speed (RPM)', 'Interpreter', 'latex', 'FontSize', 13);

% Link x-axes so zooming/panning is synchronized
linkaxes([ax1, ax2], 'x');

% 4. Export Vector PDF for Overleaf / Reports
exportgraphics(fig, 'PMSM_Currents_and_Speed.pdf', 'ContentType', 'vector', 'BackgroundColor', 'none');