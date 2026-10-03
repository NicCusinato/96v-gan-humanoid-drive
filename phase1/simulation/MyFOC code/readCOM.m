s = serialport('COM7', 5625000);
figure('Name', 'Live Motor Diagnostics');
hPos = animatedline('Color', 'b', 'LineWidth', 2);
title('Live Rotor Position (0 - 4096 counts)');
xlabel('Sample'); ylabel('Counts');
ylim([0 4096]); grid on;
t = 0;
while ishandle(hPos)
    if s.NumBytesAvailable >= 10
        raw = read(s, 10, 'uint8');
        if raw(1) == 170 && raw(2) == 85
            pos = double(raw(3))*256 + double(raw(4));
            t = t + 1;
            addpoints(hPos, t, pos);
            if mod(t, 20) == 0, drawnow limitrate; end
        end
    end
end
delete(s);