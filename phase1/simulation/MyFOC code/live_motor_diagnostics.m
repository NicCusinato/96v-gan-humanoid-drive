% Live Motor Diagnostics GUI
delete(serialportfind);
s = serialport('COM7', 5625000, 'Timeout', 1);
flush(s);
fig = figure('Name', 'Live Motor Angle', 'Color', [0.15 0.15 0.15], 'Position', [150, 150, 900, 480]);
ax = axes('Parent', fig, 'Color', [0.2 0.2 0.2], 'XColor', 'w', 'YColor', 'w');
grid(ax, 'on'); ax.GridColor = [0.4 0.4 0.4];
ylim(ax, [0 360]); xlim(ax, [0 200]);
ylabel(ax, 'Angle (Degrees)', 'Color', 'w', 'FontSize', 12);
xlabel(ax, 'Samples', 'Color', 'w', 'FontSize', 12);
title(ax, 'Live AS5047P Mechanical Angle (0 - 360°)', 'Color', 'cyan', 'FontSize', 14);
hLine = animatedline(ax, 'Color', [0 0.9 1], 'LineWidth', 2);
txt = text(ax, 10, 320, 'Turn motor by hand...', 'FontSize', 13, 'Color', 'yellow');
frame = 0; pos_deg = 0; pos_cnt = 0;
cleanup = onCleanup(@() delete(s));
while isvalid(fig)
    n = s.NumBytesAvailable;
    if n >= 10
        raw = read(s, n, 'uint8');
        for k = length(raw)-9:-1:1
            if raw(k) == 170 && raw(k+1) == 85
                pos_cnt = double(raw(k+2))*256 + double(raw(k+3));
                pos_deg = (pos_cnt / 4096.0) * 360.0;
                break;
            end
        end
        frame = frame + 1;
        addpoints(hLine, frame, pos_deg);
        if frame > 200, xlim(ax, [frame-200, frame]); end
        set(txt, 'String', sprintf('Position: %4d counts  |  Angle: %5.1f°', pos_cnt, pos_deg), 'Position', [max(10, frame-190), 320, 0]);
        % Compute running average of raw counts for offset estimation
        if frame == 1
            samples = [];
        end
        samples(end+1) = pos_cnt;
        if numel(samples) > 200
            samples(1) = [];
        end
        offset_est = round(mean(samples));
        % Display offset estimate
        text(ax, frame-190, 350, sprintf('Offset Est: %d counts', offset_est), 'Color', 'magenta', 'FontSize', 12);
        drawnow;
    else
        pause(0.005);
    end
end
