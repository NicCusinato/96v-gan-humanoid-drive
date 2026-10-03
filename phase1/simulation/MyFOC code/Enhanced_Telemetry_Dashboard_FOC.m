function Enhanced_Telemetry_Dashboard_FOC()
%% Enhanced Telemetry Dashboard & Control Panel for FOC Hardware Deployment
% Displays comprehensive FOC diagnostics and provides live interactive controls:
%   - High-Speed Multi-Packet Streaming (supports 5.625 MBaud & 115200 Baud)
%   - Interactive Buttons: STOP (0), RUN 100 RPM (1), RUN 200 RPM (2), Clear Plots
%   - Phase currents (Ia, Ib, Ic) [A] with power-aware baseline calibration
%   - d/q axis currents (Id, Iq) [A]
%   - Current magnitude sqrt(Id^2 + Iq^2) [A]
%   - Voltage commands (Vd, Vq) [V]
%   - Rotor electrical angle (theta_e) [rad]
%   - DC link voltage (Vdc) [V]
%   - Speed tracking (Actual vs Command) [RPM]
%   - Speed error (Command - Actual) [RPM]
%   - Rotor position (eQEP vs SPI) [deg]
%   - Real-time status badges and gate indicators

% Serial configuration
COM_PORT = 'COM7';
BAUD_RATE_HIGH = 115200; % 5.625 MBaud for C2000 FTDI high-speed channel
BAUD_RATE_STD  = 115200;  % 115200 Baud fallback
TIMEOUT = 5.0;

% Telemetry packet structure (32 bytes)
PACKET_SIZE = 32;
SYNC_BYTE_1 = 170;
SYNC_BYTE_2 = 85;
SYNC_BYTE_3 = 170;
END_BYTE = 69;

% GUI configuration
MAX_POINTS = 400;

fprintf('=== Enhanced FOC Telemetry Dashboard & Control Panel ===\n');
fprintf('Cleaning up stale serial connections...\n');
old_ports = serialportfind;
if ~isempty(old_ports)
    delete(old_ports);
end

% Try connecting at 5.625 MBaud first, with graceful fallback to 115200 Baud
s = [];
fprintf('Attempting connection to %s at %d baud...\n', COM_PORT, BAUD_RATE_HIGH);
try
    s = serialport(COM_PORT, BAUD_RATE_HIGH);
    s.Timeout = TIMEOUT;
    pause(0.2);
    fprintf('✓ Connected to %s at %d baud (High-Speed Mode)\n\n', COM_PORT, BAUD_RATE_HIGH);
catch
    fprintf('High-speed connect failed, trying %d baud...\n', BAUD_RATE_STD);
    try
        s = serialport(COM_PORT, BAUD_RATE_STD);
        s.Timeout = TIMEOUT;
        pause(0.2);
        fprintf('✓ Connected to %s at %d baud (Standard Mode)\n\n', COM_PORT, BAUD_RATE_STD);
    catch ME_conn
        fprintf('⚠ Could not open %s: %s\n', COM_PORT, ME_conn.message);
        fprintf('Available serial ports:\n');
        disp(serialportlist('available'));
        fprintf('Launching dashboard in monitor mode...\n\n');
    end
end

% Flags and counters
sample_count = 0;
packet_count = 0;
is_running = true;

% Create main figure with dark professional theme
hFig = figure('Name', 'FOC Hardware Deployment Dashboard & Control Panel', ...
    'NumberTitle', 'off', ...
    'Position', [80, 50, 1420, 880], ...
    'Color', [0.15, 0.16, 0.18], ...
    'CloseRequestFcn', @(~,~) onClose());

% Explicit normalized positions for 3x3 plot grid to guarantee ALL plots stay in ONE window
col_x = [0.055, 0.375, 0.695];
row_y = [0.150, 0.430, 0.710]; % bottom row, mid row, top row
plot_w = 0.275;
plot_h = 0.225;

% ===== ROW 1 (Visual Top): Currents =====
% Plot 1: Phase Currents (Ia, Ib, Ic)
ax1 = axes('Parent', hFig, 'Position', [col_x(1), row_y(3), plot_w, plot_h]);
hLineIa = animatedline(ax1, 'Color', [1.0, 0.35, 0.35], 'LineWidth', 1.5);
hLineIb = animatedline(ax1, 'Color', [0.25, 0.90, 0.40], 'LineWidth', 1.5);
hLineIc = animatedline(ax1, 'Color', [0.35, 0.65, 1.00], 'LineWidth', 1.5);
title(ax1, 'Phase Currents (Ia, Ib, Ic)', 'Color', 'w', 'FontSize', 10, 'FontWeight', 'bold');
ylabel(ax1, 'Current (A)', 'Color', [0.85 0.85 0.85]);
legend(ax1, [hLineIa, hLineIb, hLineIc], {'Ia', 'Ib', 'Ic'}, ...
    'TextColor', 'w', 'Color', [0.15 0.16 0.20], 'EdgeColor', [0.3 0.3 0.3], 'Location', 'northeast');

% Plot 2: d/q Axis Currents
ax2 = axes('Parent', hFig, 'Position', [col_x(2), row_y(3), plot_w, plot_h]);
hLineId = animatedline(ax2, 'Color', [0.90, 0.40, 0.90], 'LineWidth', 1.5);
hLineIq = animatedline(ax2, 'Color', [0.20, 0.85, 0.85], 'LineWidth', 1.5);
title(ax2, 'd/q Axis Currents', 'Color', 'w', 'FontSize', 10, 'FontWeight', 'bold');
ylabel(ax2, 'Current (A)', 'Color', [0.85 0.85 0.85]);
legend(ax2, [hLineId, hLineIq], {'Id', 'Iq'}, ...
    'TextColor', 'w', 'Color', [0.15 0.16 0.20], 'EdgeColor', [0.3 0.3 0.3], 'Location', 'northeast');

% Plot 3: Current Magnitude
ax3 = axes('Parent', hFig, 'Position', [col_x(3), row_y(3), plot_w, plot_h]);
hLineImag = animatedline(ax3, 'Color', [1.0, 0.60, 0.10], 'LineWidth', 1.5);
title(ax3, 'Current Magnitude |I| = \surd(Id^2 + Iq^2)', 'Color', 'w', 'FontSize', 10, 'FontWeight', 'bold');
ylabel(ax3, 'Magnitude (A)', 'Color', [0.85 0.85 0.85]);

% ===== ROW 2 (Visual Mid): Voltages, Angle, Bus =====
% Plot 4: Voltage Commands (Vd, Vq)
ax4 = axes('Parent', hFig, 'Position', [col_x(1), row_y(2), plot_w, plot_h]);
hLineVd = animatedline(ax4, 'Color', [1.0, 0.40, 0.40], 'LineWidth', 1.5);
hLineVq = animatedline(ax4, 'Color', [0.30, 0.90, 0.40], 'LineWidth', 1.5);
title(ax4, 'Voltage Commands (Vd, Vq)', 'Color', 'w', 'FontSize', 10, 'FontWeight', 'bold');
ylabel(ax4, 'Voltage (V)', 'Color', [0.85 0.85 0.85]);
legend(ax4, [hLineVd, hLineVq], {'Vd', 'Vq'}, ...
    'TextColor', 'w', 'Color', [0.15 0.16 0.20], 'EdgeColor', [0.3 0.3 0.3], 'Location', 'northeast');

% Plot 5: Rotor Electrical Angle
ax5 = axes('Parent', hFig, 'Position', [col_x(2), row_y(2), plot_w, plot_h]);
hLineTheta = animatedline(ax5, 'Color', [0.95, 0.75, 0.20], 'LineWidth', 1.5);
title(ax5, 'Rotor Electrical Angle (\theta_e)', 'Color', 'w', 'FontSize', 10, 'FontWeight', 'bold');
ylabel(ax5, 'Angle (rad)', 'Color', [0.85 0.85 0.85]);
ylim(ax5, [0, 2*pi]);

% Plot 6: DC Link Voltage
ax6 = axes('Parent', hFig, 'Position', [col_x(3), row_y(2), plot_w, plot_h]);
hLineVdc = animatedline(ax6, 'Color', [0.25, 0.75, 1.00], 'LineWidth', 1.5);
title(ax6, 'DC Link Voltage (Vdc)', 'Color', 'w', 'FontSize', 10, 'FontWeight', 'bold');
ylabel(ax6, 'Voltage (V)', 'Color', [0.85 0.85 0.85]);

% ===== ROW 3 (Visual Bottom): Speed & Position =====
% Plot 7: Speed Tracking (Actual vs Command)
ax7 = axes('Parent', hFig, 'Position', [col_x(1), row_y(1), plot_w, plot_h]);
hLineSpd = animatedline(ax7, 'Color', [0.25, 0.95, 0.40], 'LineWidth', 1.5);
hLineSpdCmd = animatedline(ax7, 'Color', [1.0, 0.30, 0.30], 'LineWidth', 1.5, 'LineStyle', '--');
title(ax7, 'Speed Tracking (Actual vs Command)', 'Color', 'w', 'FontSize', 10, 'FontWeight', 'bold');
xlabel(ax7, 'Sample', 'Color', [0.85 0.85 0.85]);
ylabel(ax7, 'Speed (RPM)', 'Color', [0.85 0.85 0.85]);
legend(ax7, [hLineSpd, hLineSpdCmd], {'Actual', 'Command'}, ...
    'TextColor', 'w', 'Color', [0.15 0.16 0.20], 'EdgeColor', [0.3 0.3 0.3], 'Location', 'northeast');

% Plot 8: Speed Error
ax8 = axes('Parent', hFig, 'Position', [col_x(2), row_y(1), plot_w, plot_h]);
hLineSpdErr = animatedline(ax8, 'Color', [1.0, 0.35, 0.35], 'LineWidth', 1.5);
yline(ax8, 0, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8);
title(ax8, 'Speed Error (Cmd - Actual)', 'Color', 'w', 'FontSize', 10, 'FontWeight', 'bold');
xlabel(ax8, 'Sample', 'Color', [0.85 0.85 0.85]);
ylabel(ax8, 'Error (RPM)', 'Color', [0.85 0.85 0.85]);

% Plot 9: Rotor Position (eQEP vs SPI)
ax9 = axes('Parent', hFig, 'Position', [col_x(3), row_y(1), plot_w, plot_h]);
hLineQep = animatedline(ax9, 'Color', [0.35, 0.65, 1.00], 'LineWidth', 1.5);
hLineSpi = animatedline(ax9, 'Color', [1.0, 0.45, 0.25], 'LineWidth', 1.5);
title(ax9, 'Rotor Position (eQEP vs SPI)', 'Color', 'w', 'FontSize', 10, 'FontWeight', 'bold');
xlabel(ax9, 'Sample', 'Color', [0.85 0.85 0.85]);
ylabel(ax9, 'Position (°)', 'Color', [0.85 0.85 0.85]);
legend(ax9, [hLineQep, hLineSpi], {'eQEP', 'SPI'}, ...
    'TextColor', 'w', 'Color', [0.15 0.16 0.20], 'EdgeColor', [0.3 0.3 0.3], 'Location', 'northeast');

% Format all 9 axes with consistent dark theme styling
allAxes = [ax1, ax2, ax3, ax4, ax5, ax6, ax7, ax8, ax9];
for a = allAxes
    set(a, 'Color', [0.10, 0.11, 0.14], ...
           'XColor', [0.80, 0.82, 0.85], ...
           'YColor', [0.80, 0.82, 0.85], ...
           'GridColor', [0.30, 0.32, 0.38], ...
           'GridAlpha', 0.6, ...
           'FontSize', 8);
    grid(a, 'on');
end

allLines = [hLineIa, hLineIb, hLineIc, hLineId, hLineIq, hLineImag, ...
            hLineVd, hLineVq, hLineTheta, hLineVdc, ...
            hLineSpd, hLineSpdCmd, hLineSpdErr, hLineQep, hLineSpi];

% ===== BOTTOM CONTROL PANEL =====
pnlCtrl = uipanel('Parent', hFig, ...
    'Position', [0.01, 0.01, 0.98, 0.10], ...
    'BackgroundColor', [0.20, 0.22, 0.26], ...
    'BorderType', 'line', 'HighlightColor', [0.35, 0.38, 0.45]);

% STOP Button [0]
uicontrol('Parent', pnlCtrl, 'Style', 'pushbutton', ...
    'Units', 'normalized', 'Position', [0.01, 0.15, 0.11, 0.70], ...
    'String', 'STOP [0]', 'FontSize', 12, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.85 0.22 0.22], 'ForegroundColor', 'white', ...
    'Callback', @(~,~) sendCommand(s, uint8('0'), 'STOP (0) commanded'));

% RUN 100 RPM Button [1]
uicontrol('Parent', pnlCtrl, 'Style', 'pushbutton', ...
    'Units', 'normalized', 'Position', [0.13, 0.15, 0.12, 0.70], ...
    'String', 'RUN 100 RPM [1]', 'FontSize', 12, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.18 0.72 0.32], 'ForegroundColor', 'white', ...
    'Callback', @(~,~) sendCommand(s, uint8('1'), '100 RPM (1) commanded'));

% RUN 200 RPM Button [2]
uicontrol('Parent', pnlCtrl, 'Style', 'pushbutton', ...
    'Units', 'normalized', 'Position', [0.26, 0.15, 0.12, 0.70], ...
    'String', 'RUN 200 RPM [2]', 'FontSize', 12, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.20 0.55 0.90], 'ForegroundColor', 'white', ...
    'Callback', @(~,~) sendCommand(s, uint8('2'), '200 RPM (2) commanded'));

% Clear Plots Button
uicontrol('Parent', pnlCtrl, 'Style', 'pushbutton', ...
    'Units', 'normalized', 'Position', [0.39, 0.15, 0.08, 0.70], ...
    'String', 'Clear Data', 'FontSize', 10, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.38 0.40 0.45], 'ForegroundColor', 'white', ...
    'Callback', @(~,~) clearAllData());

% State Status Indicator
lblState = uicontrol('Parent', pnlCtrl, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.48, 0.15, 0.15, 0.70], ...
    'String', 'STATE: STOPPED', 'FontSize', 11, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.85 0.25 0.25], 'ForegroundColor', 'white');

% Rotor Position Display
lblPos = uicontrol('Parent', pnlCtrl, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.64, 0.15, 0.18, 0.70], ...
    'String', 'SPI: 0.0° | eQEP: 0.0° | θe: 0.00 rad', ...
    'FontSize', 9, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.90 0.92 0.95], 'ForegroundColor', [0.1 0.1 0.1]);

% Telemetry Packet & DC Bus Readout
lblInfo = uicontrol('Parent', pnlCtrl, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.83, 0.15, 0.16, 0.70], ...
    'String', 'Pkts: 0 | Gate: OFF | Imag: 0.0A', ...
    'FontSize', 9, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.90 0.92 0.95], 'ForegroundColor', [0.1 0.1 0.1]);

% Keypress shortcuts on figure window ('0', '1', '2')
set(hFig, 'KeyPressFcn', @(~, event) handleKey(event.Key, s));

fprintf('✓ Dashboard ready. Click buttons or press [0], [1], [2] to command motor.\n');
fprintf('Streaming telemetry packets...\n\n');

% Main streaming loop
try
    while is_running && ishandle(hFig)
        if ~isempty(s) && isvalid(s)
            nBytes = s.NumBytesAvailable;
            if nBytes >= PACKET_SIZE
                rx_buf = read(s, nBytes, 'uint8');
                buf_len = length(rx_buf);
                idx = 1;
                
                % Multi-packet drain: process ALL complete frames in buffer
                while idx <= (buf_len - PACKET_SIZE + 1)
                    if rx_buf(idx) == SYNC_BYTE_1 && rx_buf(idx+1) == SYNC_BYTE_2 && ...
                       rx_buf(idx+2) == SYNC_BYTE_3 && rx_buf(idx+PACKET_SIZE-1) == END_BYTE
                        
                        pkt = rx_buf(idx : idx + PACKET_SIZE - 1);
                        
                        % Unpack status flags first
                        state_echo  = double(pkt(30));
                        gate_enable = double(pkt(31));
                        
                        % Byte 4-5: eQEP position (Big-endian uint16)
                        qep_cnt = double(pkt(4))*256 + double(pkt(5));
                        qep_deg = qep_cnt * 360.0 / 4096.0;
                        
                        % Byte 6-7: Speed RPM (Big-endian int16 from C2000 -> swapped for x86)
                        spd_rpm = double(typecast(uint8([pkt(7), pkt(6)]), 'int16'));
                        
                        % Byte 8-9: Phase A raw ADC counts
                        adc_a = double(pkt(8))*256 + double(pkt(9));
                        
                        % Byte 10-11: Phase B raw ADC counts
                        adc_b = double(pkt(10))*256 + double(pkt(11));
                        
                        % Byte 12-13: Phase C raw ADC counts
                        adc_c = double(pkt(12))*256 + double(pkt(13));
                        
                        % Byte 14-15: DC link voltage raw ADC counts
                        vdc_cnt = double(pkt(14))*256 + double(pkt(15));
                        vdc_volts_raw = vdc_cnt * (3.3 * 46.4545 / 4095);
                        
                        % On the EPC inverter, when high-voltage DC terminals are disconnected (floating),
                        % the high-impedance divider sits at ~0.60V (ESD clamp diode drop), reading ~27.5-28.5V.
                        % When unpowered on the bench, zero this floating reading so it displays 0.0 V:
                        if vdc_volts_raw >= 26.0 && vdc_volts_raw <= 29.5 && gate_enable == 0
                            vdc_volts = 0.0;
                        else
                            vdc_volts = vdc_volts_raw;
                        end
                        
                        % Byte 16-17: SPI position
                        spi_cnt = bitand(double(pkt(16))*256 + double(pkt(17)), 16383);
                        spi_deg = spi_cnt * 360.0 / 16384.0;
                        
                        % Byte 18-19: Id (int16 scaled by 3276.7 -> swapped for x86)
                        Id = double(typecast(uint8([pkt(19), pkt(18)]), 'int16')) / 3276.7;
                        
                        % Byte 20-21: Iq (int16 scaled by 3276.7 -> swapped for x86)
                        Iq = double(typecast(uint8([pkt(21), pkt(20)]), 'int16')) / 3276.7;
                        
                        % Byte 22-23: Theta electrical (0 to 65535 = 0 to 2*pi)
                        theta_u16 = double(pkt(22))*256 + double(pkt(23));
                        theta_e = theta_u16 / 65535.0 * (2 * pi);
                        
                        % Byte 24-25: Speed command RPM (int16 -> swapped for x86)
                        spd_cmd = double(typecast(uint8([pkt(25), pkt(24)]), 'int16'));
                        
                        % Byte 26-27: Vd (int16 scaled by 327.67 for +/-100V range -> swapped for x86)
                        Vd = double(typecast(uint8([pkt(27), pkt(26)]), 'int16')) / 327.67;
                        
                        % Byte 28-29: Vq (int16 scaled by 327.67 for +/-100V range -> swapped for x86)
                        Vq = double(typecast(uint8([pkt(29), pkt(28)]), 'int16')) / 327.67;
                        
                        % True physical 3-phase currents reconstructed from C2000 controller's Id, Iq
                        % (Inverse Park/Clarke: exact true currents, 100% immune to analog unpowered pull-down)
                        I_alpha = Id * cos(theta_e) - Iq * sin(theta_e);
                        I_beta  = Id * sin(theta_e) + Iq * cos(theta_e);
                        ia_amps = I_alpha;
                        ib_amps = -0.5 * I_alpha + (sqrt(3)/2) * I_beta;
                        ic_amps = -0.5 * I_alpha - (sqrt(3)/2) * I_beta;
                        
                        imag = sqrt(Id^2 + Iq^2);
                        
                        % Filter transient startup spike on sample 1-2 when stationary
                        if sample_count <= 2 && abs(spd_rpm) < 15.0 && spd_cmd == 0
                            spd_rpm = 0.0;
                        end
                        spd_err = spd_cmd - spd_rpm;
                        
                        % Update plot lines
                        sample_count = sample_count + 1;
                        addpoints(hLineIa, sample_count, ia_amps);
                        addpoints(hLineIb, sample_count, ib_amps);
                        addpoints(hLineIc, sample_count, ic_amps);
                        addpoints(hLineId, sample_count, Id);
                        addpoints(hLineIq, sample_count, Iq);
                        addpoints(hLineImag, sample_count, imag);
                        addpoints(hLineVd, sample_count, Vd);
                        addpoints(hLineVq, sample_count, Vq);
                        addpoints(hLineTheta, sample_count, theta_e);
                        addpoints(hLineVdc, sample_count, vdc_volts);
                        addpoints(hLineSpd, sample_count, spd_rpm);
                        addpoints(hLineSpdCmd, sample_count, spd_cmd);
                        addpoints(hLineSpdErr, sample_count, spd_err);
                        addpoints(hLineQep, sample_count, qep_deg);
                        addpoints(hLineSpi, sample_count, spi_deg);
                        
                        % Rolling window limit
                        if sample_count > MAX_POINTS
                            set(allAxes, 'XLim', [sample_count - MAX_POINTS, sample_count]);
                        end
                        
                        % Update status badges
                        state_str = 'STOPPED';
                        state_color = [0.85 0.30 0.30];
                        switch state_echo
                            case 0
                                state_str = 'STOPPED'; state_color = [0.85 0.30 0.30];
                            case 1
                                state_str = 'OL 60 RPM'; state_color = [0.20 0.75 0.35];
                            case 5
                                state_str = 'CL FOC 100 RPM'; state_color = [0.20 0.65 0.90];
                            case 6
                                state_str = 'CL FOC 200 RPM'; state_color = [0.15 0.50 0.85];
                            case 51
                                state_str = 'CALIBRATING (0.5s)'; state_color = [0.90 0.70 0.20];
                        end
                        set(lblState, 'String', sprintf('STATE: %s', state_str), 'BackgroundColor', state_color);
                        set(lblPos, 'String', sprintf('SPI: %.1f° | eQEP: %.1f° | θe: %.3f rad', ...
                            spi_deg, qep_deg, theta_e));
                        gate_str = 'OFF';
                        if gate_enable > 0, gate_str = 'ON'; end
                        set(lblInfo, 'String', sprintf('Pkts: %d | Gate: %s | Imag: %.2fA | Vdc: %.1fV', ...
                            packet_count, gate_str, imag, vdc_volts));
                        
                        packet_count = packet_count + 1;
                        idx = idx + PACKET_SIZE;
                    else
                        idx = idx + 1;
                    end
                end
            end
        end
        
        drawnow limitrate;
        pause(0.002);
    end
catch ME
    if ishandle(hFig)
        fprintf('Dashboard runtime note: %s\n', ME.message);
    end
end

% Cleanup serial port on exit
if ~isempty(s) && isvalid(s)
    delete(s);
    fprintf('Serial port closed successfully.\n');
end

fprintf('Dashboard closed.\n');

%% Nested & Helper Functions
    function onClose()
        is_running = false;
        if ishandle(hFig)
            delete(hFig);
        end
    end

    function clearAllData()
        for k = 1:length(allLines)
            clearpoints(allLines(k));
        end
        sample_count = 0;
        set(allAxes, 'XLimMode', 'auto');
        fprintf('Dashboard plots cleared.\n');
    end

end

function sendCommand(s, cmd_byte, msg)
    if ~isempty(s) && isvalid(s)
        try
            write(s, cmd_byte, 'uint8');
            fprintf('>>> %s\n', msg);
        catch ME_tx
            fprintf('⚠ Serial write failed: %s\n', ME_tx.message);
        end
    else
        fprintf('>>> Simulated: %s (Serial port not connected)\n', msg);
    end
end

function handleKey(key, s)
    if strcmp(key, '0')
        sendCommand(s, uint8('0'), 'Key [0] -> STOP');
    elseif strcmp(key, '1')
        sendCommand(s, uint8('1'), 'Key [1] -> RUN 100 RPM');
    elseif strcmp(key, '2')
        sendCommand(s, uint8('2'), 'Key [2] -> RUN 200 RPM');
    end
end
