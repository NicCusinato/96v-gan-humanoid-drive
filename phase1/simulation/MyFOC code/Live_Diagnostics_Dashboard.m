function Live_Diagnostics_Dashboard()
% Live_Diagnostics_Dashboard.m
% Interactive live diagnostic tool for LAUNCHXL-F28069M + EPC9147B inverter
% Reads the 33-byte expanded telemetry frame from Hardware_Diagnostics_Test
% (and retains compatibility with older 19/21-byte frames):
% Expanded frame: ['S', 170, 85, payload, State, reserved, 'E'].
% Displays board telemetry without changing control signs or angle direction.
% The board/controller is responsible for all FOC sign conventions.

delete(serialportfind);
close all;

PORT = 'COM7';
BAUD = 5625000;

fprintf('Connecting to %s at %d baud...\n', PORT, BAUD);
s = serialport(PORT, BAUD, 'Timeout', 1);
flush(s);

% Create GUI Figure
hFig = figure('Name', 'Live Motor & Inverter Hardware Diagnostics: Angle Alignment (SPI vs QEP vs θ_ol)', ...
              'NumberTitle', 'off', 'Color', [0.12 0.12 0.15], ...
              'Position', [40 40 1440 820]);

% UI - Title
uicontrol(hFig, 'Style', 'text', 'String', 'EPC9147B + F28069M Live Diagnostics: Angle Alignment Verification (AS5047P SPI vs eQEP vs \theta_{ol})', ...
    'Units', 'normalized', 'Position', [0.03 0.95 0.94 0.04], ...
    'BackgroundColor', [0.12 0.12 0.15], 'ForegroundColor', [0.9 0.9 0.9], ...
    'FontSize', 13, 'FontWeight', 'bold');

% Subplot 1: Board-reported and raw encoder angles
axPos = subplot(2, 3, 1, 'Parent', hFig);
hLineThOl  = animatedline(axPos, 'Color', [1.0 0.3 0.3], 'LineWidth', 1.5); % Red = \theta_ol
hLineThSpi = animatedline(axPos, 'Color', [1.0 0.85 0.2], 'LineWidth', 2);   % Yellow = \theta_spi
hLineThQep = animatedline(axPos, 'Color', [0.4 0.8 1.0], 'LineWidth', 1.5, 'LineStyle', '--'); % Cyan = \theta_qep
title(axPos, 'Board \theta_{ol} vs Raw SPI vs Raw QEP', 'Color', 'w');
xlabel(axPos, 'Time (samples)', 'Color', 'w');
ylabel(axPos, 'Angle (°)', 'Color', 'w');
set(axPos, 'Color', [0.08 0.08 0.1], 'XColor', 'w', 'YColor', 'w', 'YLim', [0 360]);
grid(axPos, 'on');
lgdPos = legend(axPos, {'Board \theta_{ol}', 'Raw SPI', 'Raw QEP'}, ...
    'Location', 'northeast', 'Interpreter', 'tex');
set(lgdPos, 'Color', [0.12 0.12 0.15], 'TextColor', 'w', ...
    'EdgeColor', [0.55 0.55 0.60], 'FontSize', 9);

% Subplot 2: Rotor Speed (RPM)
axSpd = subplot(2, 3, 2, 'Parent', hFig);
hLineSpd = animatedline(axSpd, 'Color', [0.3 1.0 0.8], 'LineWidth', 2);
title(axSpd, 'Rotor Speed (Mechanical RPM)', 'Color', 'w');
xlabel(axSpd, 'Time (samples)', 'Color', 'w');
ylabel(axSpd, 'Speed (RPM)', 'Color', 'w');
set(axSpd, 'Color', [0.08 0.08 0.1], 'XColor', 'w', 'YColor', 'w', 'YLim', [-150 150]);
grid(axSpd, 'on');

% Subplot 3: FOC Current Vectors (Id and Iq)
axFoc = subplot(2, 3, 3, 'Parent', hFig);
hLineId = animatedline(axFoc, 'Color', [0.3 0.7 1.0], 'LineWidth', 2); % Blue = Id
hLineIq = animatedline(axFoc, 'Color', [1.0 0.4 0.2], 'LineWidth', 2); % Orange = Iq
title(axFoc, 'FOC Vectors: Id (Blue) & Iq (Orange)', 'Color', 'w');
xlabel(axFoc, 'Time (samples)', 'Color', 'w');
ylabel(axFoc, 'Current (Amperes)', 'Color', 'w');
set(axFoc, 'Color', [0.08 0.08 0.1], 'XColor', 'w', 'YColor', 'w', 'YLim', [-5 5]);
grid(axFoc, 'on');
lgdFoc = legend(axFoc, {'Id (Field)', 'Iq (Torque)'}, ...
    'Location', 'northeast', 'Interpreter', 'tex');
set(lgdFoc, 'Color', [0.12 0.12 0.15], 'TextColor', 'w', ...
    'EdgeColor', [0.55 0.55 0.60], 'FontSize', 9);

% Subplot 4: Phase Currents (Ia & Ib)
axCur = subplot(2, 3, [4 5], 'Parent', hFig);
hLineIa = animatedline(axCur, 'Color', [0.2 1.0 0.4], 'LineWidth', 1.5);
hLineIb = animatedline(axCur, 'Color', [1.0 0.3 0.5], 'LineWidth', 1.5);
title(axCur, 'Phase Currents (Ia = Green, Ib = Pink)', 'Color', 'w');
xlabel(axCur, 'Time (samples)', 'Color', 'w');
ylabel(axCur, 'Current (Amperes)', 'Color', 'w');
set(axCur, 'Color', [0.08 0.08 0.1], 'XColor', 'w', 'YColor', 'w', 'YLim', [-5 5]);
grid(axCur, 'on');
lgdCur = legend(axCur, {'Ia (A)', 'Ib (A)'}, ...
    'Location', 'northeast', 'Interpreter', 'tex');
set(lgdCur, 'Color', [0.12 0.12 0.15], 'TextColor', 'w', ...
    'EdgeColor', [0.55 0.55 0.60], 'FontSize', 9);

% Interactive Control Panel (Right)
panel = uipanel(hFig, 'Title', 'Motor Control & Direction Diagnostic', ...
    'Units', 'normalized', 'Position', [0.68 0.02 0.30 0.46], ...
    'BackgroundColor', [0.18 0.18 0.22], 'ForegroundColor', 'w', 'FontWeight', 'bold');

lblState = uicontrol(panel, 'Style', 'text', 'String', 'STATE: STOPPED (Gates OFF)', ...
    'Units', 'normalized', 'Position', [0.05 0.90 0.90 0.08], ...
    'BackgroundColor', [0.18 0.18 0.22], 'ForegroundColor', [1.0 0.4 0.4], ...
    'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'left');

lblSpd = uicontrol(panel, 'Style', 'text', 'String', 'Speed: 0 RPM', ...
    'Units', 'normalized', 'Position', [0.05 0.82 0.45 0.07], ...
    'BackgroundColor', [0.18 0.18 0.22], 'ForegroundColor', [0.3 1.0 0.8], ...
    'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'left');

lblSyncDiag = uicontrol(panel, 'Style', 'text', 'String', 'Direction: IDLE', ...
    'Units', 'normalized', 'Position', [0.52 0.82 0.45 0.07], ...
    'BackgroundColor', [0.18 0.18 0.22], 'ForegroundColor', [1.0 0.8 0.2], ...
    'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'left');

lblAngles = uicontrol(panel, 'Style', 'text', 'String', '\theta_{ol}: 0° | SPI: 0° | QEP: 0°', ...
    'Units', 'normalized', 'Position', [0.05 0.74 0.90 0.07], ...
    'BackgroundColor', [0.18 0.18 0.22], 'ForegroundColor', [1.0 0.85 0.2], ...
    'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'left');

lblFocVec = uicontrol(panel, 'Style', 'text', 'String', 'Id: 0.00 A | Iq: 0.00 A', ...
    'Units', 'normalized', 'Position', [0.05 0.67 0.90 0.07], ...
    'BackgroundColor', [0.18 0.18 0.22], 'ForegroundColor', [1.0 0.6 0.2], ...
    'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'left');

    function sendCommand(cmd)
        if exist('s', 'var') && isvalid(s)
            try
                write(s, uint8(cmd), 'uint8');
            catch writeErr
                warning('Failed to send command ''%s'': %s', char(cmd), writeErr.message);
            end
        else
            warning('Cannot send command ''%s'': Serial port is closed or invalid.', char(cmd));
        end
    end

% Motor Control Buttons
% Row 1: Calibration monitor and clear
uicontrol(panel, 'Style', 'pushbutton', 'String', 'HAND-SPIN MONITOR', ...
    'Units', 'normalized', 'Position', [0.05 0.57 0.43 0.08], ...
    'BackgroundColor', [0.15 0.45 0.2], 'ForegroundColor', 'w', ...
    'FontSize', 9, 'FontWeight', 'bold', ...
    'Callback', @(src, evt) sendCommand('0'));

uicontrol(panel, 'Style', 'pushbutton', 'String', 'CLEAR CAL OFFSET', ...
    'Units', 'normalized', 'Position', [0.52 0.57 0.43 0.08], ...
    'BackgroundColor', [0.1 0.4 0.5], 'ForegroundColor', 'w', ...
    'FontSize', 9, 'FontWeight', 'bold', ...
    'Callback', @(src, evt) sendCommand('C'));

% Row 2: Board-side open-loop direction tests
uicontrol(panel, 'Style', 'pushbutton', 'String', 'OPEN LOOP + (P)', ...
    'Units', 'normalized', 'Position', [0.05 0.47 0.43 0.08], ...
    'BackgroundColor', [0.15 0.55 0.3], 'ForegroundColor', 'w', ...
    'FontSize', 9, 'FontWeight', 'bold', ...
    'Callback', @(src, evt) sendCommand('P'));

uicontrol(panel, 'Style', 'pushbutton', 'String', 'OPEN LOOP - (N)', ...
    'Units', 'normalized', 'Position', [0.52 0.47 0.43 0.08], ...
    'BackgroundColor', [0.1 0.5 0.6], 'ForegroundColor', 'w', ...
    'FontSize', 9, 'FontWeight', 'bold', ...
    'Callback', @(src, evt) sendCommand('N'));

% Row 3: Board-side alignment and positive open-loop test
uicontrol(panel, 'Style', 'pushbutton', 'String', 'ALIGN & CAPTURE (A)', ...
    'Units', 'normalized', 'Position', [0.05 0.37 0.43 0.08], ...
    'BackgroundColor', [0.6 0.35 0.1], 'ForegroundColor', 'w', ...
    'FontSize', 9, 'FontWeight', 'bold', ...
    'Callback', @(src, evt) sendCommand('A'));

uicontrol(panel, 'Style', 'pushbutton', 'String', 'RE-ALIGN (A)', ...
    'Units', 'normalized', 'Position', [0.52 0.37 0.43 0.08], ...
    'BackgroundColor', [0.5 0.4 0.1], 'ForegroundColor', 'w', ...
    'FontSize', 9, 'FontWeight', 'bold', ...
    'Callback', @(src, evt) sendCommand('A'));

% Row 4: Stop
uicontrol(panel, 'Style', 'pushbutton', 'String', 'STOP / TRIP PWM (0)', ...
    'Units', 'normalized', 'Position', [0.05 0.26 0.90 0.09], ...
    'BackgroundColor', [0.75 0.15 0.15], 'ForegroundColor', 'w', ...
    'FontSize', 10, 'FontWeight', 'bold', ...
    'Callback', @(src, evt) sendCommand('0'));

% Data Logging UI Controls
lblRecStatus = uicontrol(panel, 'Style', 'text', 'String', 'LOGGING: IDLE', ...
    'Units', 'normalized', 'Position', [0.05 0.14 0.90 0.09], ...
    'BackgroundColor', [0.18 0.18 0.22], 'ForegroundColor', [0.7 0.7 0.7], ...
    'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

% Global recording variables shared with callbacks
is_recording = false;
log_data = struct('t', [], 'spi_deg', [], 'qep_cnt', [], 'th_ol_deg', [], ...
                  'spd_rpm', [], 'ia_amps', [], 'ib_amps', [], ...
                  'id_amps', [], 'iq_amps', [], 'state', []);

btnRecord = uicontrol(panel, 'Style', 'pushbutton', 'String', '● START LOGGING', ...
    'Units', 'normalized', 'Position', [0.05 0.02 0.42 0.11], ...
    'BackgroundColor', [0.5 0.1 0.1], 'ForegroundColor', 'w', ...
    'FontSize', 9, 'FontWeight', 'bold');

btnExport = uicontrol(panel, 'Style', 'pushbutton', 'String', ' EXPORT LOG', ...
    'Units', 'normalized', 'Position', [0.52 0.02 0.43 0.11], ...
    'BackgroundColor', [0.15 0.35 0.55], 'ForegroundColor', 'w', ...
    'FontSize', 9, 'FontWeight', 'bold', 'Enable', 'off');

btnRecord.Callback = @toggleLogging;
btnExport.Callback = @exportTelemetry;

    function toggleLogging(src, ~)
        if ~is_recording
            is_recording = true;
            log_data.t         = [];
            log_data.spi_deg   = [];
            log_data.qep_cnt   = [];
            log_data.th_ol_deg = [];
            log_data.spd_rpm   = [];
            log_data.ia_amps   = [];
            log_data.ib_amps   = [];
            log_data.id_amps   = [];
            log_data.iq_amps   = [];
            log_data.state     = [];
            t_start = tic;
            set(src, 'String', '■ STOP LOGGING', 'BackgroundColor', [0.7 0.15 0.15]);
            set(lblRecStatus, 'String', 'LOGGING ACTIVE...', 'ForegroundColor', [1.0 0.3 0.3]);
            set(btnExport, 'Enable', 'off');
        else
            is_recording = false;
            set(src, 'String', '● START LOGGING', 'BackgroundColor', [0.5 0.1 0.1]);
            set(lblRecStatus, 'String', sprintf('LOGGED %d POINTS', length(log_data.t)), ...
                'ForegroundColor', [0.3 1.0 0.5]);
            if ~isempty(log_data.t)
                set(btnExport, 'Enable', 'on');
            end
        end
    end

    function exportTelemetry(~, ~)
        if isempty(log_data.t)
            warndlg('No telemetry points recorded!', 'Export Warning');
            return;
        end
        ts_str = datestr(now, 'yyyymmdd_HHMMSS');
        mat_file = sprintf('Motor_Telemetry_%s.mat', ts_str);
        csv_file = sprintf('Motor_Telemetry_%s.csv', ts_str);

        telemetry_raw = struct();
        telemetry_raw.time_sec   = log_data.t - log_data.t(1);
        telemetry_raw.speed_rpm  = log_data.spd_rpm;
        telemetry_raw.ia_amps    = log_data.ia_amps;
        telemetry_raw.ib_amps    = log_data.ib_amps;
        telemetry_raw.id_amps    = log_data.id_amps;
        telemetry_raw.iq_amps    = log_data.iq_amps;
        telemetry_raw.spi_deg    = log_data.spi_deg;
        telemetry_raw.qep_cnt    = log_data.qep_cnt;
        telemetry_raw.th_ol_deg  = log_data.th_ol_deg;
        telemetry_raw.state      = log_data.state;

        raw = telemetry_raw;
        save(mat_file, 'raw');

        T = table(telemetry_raw.time_sec', telemetry_raw.speed_rpm', ...
                  telemetry_raw.ia_amps', telemetry_raw.ib_amps', ...
                  telemetry_raw.id_amps', telemetry_raw.iq_amps', ...
                  telemetry_raw.spi_deg', telemetry_raw.qep_cnt', ...
                  telemetry_raw.th_ol_deg', telemetry_raw.state', ...
                  'VariableNames', {'Time_s', 'Speed_RPM', ...
                                    'Ia_Amps', 'Ib_Amps', ...
                                    'Id_Amps', 'Iq_Amps', ...
                                    'SPI_Deg', 'QEP_Cnt', ...
                                    'Theta_OL_Deg', 'State'});
        writetable(T, csv_file);

        fprintf('\n========================================\n');
        fprintf('  MOTOR TELEMETRY DATA EXPORTED!\n');
        fprintf('  Total Samples : %d (%.2f seconds)\n', length(telemetry_raw.time_sec), telemetry_raw.time_sec(end));
        fprintf('  MAT File      : %s\n', fullfile(pwd, mat_file));
        fprintf('  CSV File      : %s\n', fullfile(pwd, csv_file));
        fprintf('========================================\n\n');
        msgbox(sprintf('Exported %d samples successfully!\n\nMAT: %s\nCSV: %s', ...
            length(telemetry_raw.time_sec), mat_file, csv_file), 'Export Succeeded');
    end

% Running loop
sample_count = 0;
MAX_POINTS = 300;
t_start = tic;

% Alignment tracking
prev_spi_deg = 0;

try
    while ishandle(hFig)
        nBytes = s.NumBytesAvailable;
        % Accept the current 19-byte model frame, the legacy 21-byte
        % S/AA/55/.../E frame, and the expanded 33-byte QEP diagnostic frame.
        if nBytes >= 19
            rx_buf = read(s, nBytes, 'uint8');
            valid_idx = -1;
            frame_len = 0;
            % Prefer the expanded diagnostic frame when present.
            for i = 1:(length(rx_buf)-32)
                if rx_buf(i) == 83 && rx_buf(i+1) == 170 && rx_buf(i+2) == 85 && rx_buf(i+32) == 69
                    valid_idx = i;
                    frame_len = 33;
                end
            end
            % Legacy framed packet: 'S', 170, 85, payload, state, 'E'.
            if frame_len == 0
                for i = 1:(length(rx_buf)-20)
                    if rx_buf(i) == 83 && rx_buf(i+1) == 170 && rx_buf(i+2) == 85 && rx_buf(i+20) == 69
                        valid_idx = i;
                        frame_len = 21;
                    end
                end
            end
            % Current Hardware_Diagnostics_Test packet: 170, 85, payload, state.
            if frame_len == 0
                for i = 1:(length(rx_buf)-18)
                    if rx_buf(i) == 170 && rx_buf(i+1) == 85
                        valid_idx = i;
                        frame_len = 19;
                    end
                end
            end
            if valid_idx ~= -1
                frame = rx_buf(valid_idx:(valid_idx + frame_len - 1));
                has_qep_diag = (frame_len == 33);
                if frame_len == 19
                    % Current model frame has no leading 'S' or trailing 'E'.
                    idx_id_h = 3;  idx_id_l = 4;
                    idx_iq_h = 5;  idx_iq_l = 6;
                    idx_spd_h = 7; idx_spd_l = 8;
                    idx_aa_h = 9;  idx_aa_l = 10;
                    idx_ab_h = 11; idx_ab_l = 12;
                    idx_spi_h = 13; idx_spi_l = 14;
                    idx_qep_h = 15; idx_qep_l = 16;
                    idx_th_h = 17;  idx_th_l = 18;
                    idx_state = 19;
                elseif frame_len == 33
                    % Expanded diagnostic frame: S, AA, 55, payload, state, reserved, E.
                    idx_id_h = 4;  idx_id_l = 5;
                    idx_iq_h = 6;  idx_iq_l = 7;
                    idx_spd_h = 8; idx_spd_l = 9;
                    idx_aa_h = 10; idx_aa_l = 11;
                    idx_ab_h = 12; idx_ab_l = 13;
                    idx_spi_h = 14; idx_spi_l = 15;
                    idx_qep_h = 16; idx_qep_l = 17;
                    idx_th_h = 29;  idx_th_l = 30;
                    idx_state = 31;
                else
                    % 21-byte legacy frame: S, AA, 55, payload, state, E.
                    idx_id_h = 4;  idx_id_l = 5;
                    idx_iq_h = 6;  idx_iq_l = 7;
                    idx_spd_h = 8; idx_spd_l = 9;
                    idx_aa_h = 10; idx_aa_l = 11;
                    idx_ab_h = 12; idx_ab_l = 13;
                    idx_spi_h = 14; idx_spi_l = 15;
                    idx_qep_h = 16; idx_qep_l = 17;
                    idx_th_h = 18;  idx_th_l = 19;
                    idx_state = 20;
                end

                % Decode Id, Iq (centi-amps)
                id_raw = double(typecast(uint8([frame(idx_id_l), frame(idx_id_h)]), 'int16'));
                id_amps = id_raw / 100.0;
                iq_raw = double(typecast(uint8([frame(idx_iq_l), frame(idx_iq_h)]), 'int16'));
                iq_amps = iq_raw / 100.0;
                
                % Decode Speed (RPM)
                spd_rpm = double(typecast(uint8([frame(idx_spd_l), frame(idx_spd_h)]), 'int16'));
                
                % Decode ADC currents
                adca = double(frame(idx_aa_h))*256 + double(frame(idx_aa_l));
                adcb = double(frame(idx_ab_h))*256 + double(frame(idx_ab_l));
                
                % Decode SPI Angle (AS5047P: 14-bit)
                spi_cnt = bitand(double(frame(idx_spi_h))*256 + double(frame(idx_spi_l)), 16383);
                spi_deg = spi_cnt * 360.0 / 16384.0;
                
                % Decode QEP Counter (4096 counts/rev)
                qep_cnt = double(frame(idx_qep_h))*256 + double(frame(idx_qep_l));
                qep_deg = mod(qep_cnt * 360.0 / 4096.0, 360.0);
                
                % Decode Theta_OL (0-65535 -> 0-360 deg)
                th_ol_raw = double(frame(idx_th_h))*256 + double(frame(idx_th_l));
                th_ol_deg = th_ol_raw * 360.0 / 65535.0;

                % Decode State Echo
                st_echo = frame(idx_state);

                % Optional QEP diagnostics from the expanded frame.
                qep_ilat = NaN;
                qep_dir = NaN;
                qep_coef = NaN;
                qep_cdef = NaN;
                qep_qcprd = NaN;
                qep_qcprdlat = NaN;
                qep_qposlat = NaN;
                if has_qep_diag
                    qep_ilat = double(frame(18))*256 + double(frame(19));
                    qep_dir = double(frame(20));
                    qep_coef = double(frame(21));
                    qep_cdef = double(frame(22));
                    qep_qcprd = double(frame(23))*256 + double(frame(24));
                    qep_qcprdlat = double(frame(25))*256 + double(frame(26));
                    qep_qposlat = double(frame(27))*256 + double(frame(28));
                end

                % Phase currents calculation (Negative scale matches EPC9147B low-side shunt amp polarity)
                offset_a = 2041.0;
                offset_b = 2059.0;
                ia_amps = -(adca - offset_a) * (3.3 / 4095) / (0.002 * 20);
                ib_amps = -(adcb - offset_b) * (3.3 / 4095) / (0.002 * 20);

                % Observation-only display: do not apply pole-pair scaling,
                % alignment offsets, or sign inversions in the dashboard.
                % The board/controller must generate the correct control angle.
                th_spi_display = spi_deg;
                th_qep_display = qep_deg;

                % Direction check
                d_spi = spi_deg - prev_spi_deg;
                if d_spi > 180, d_spi = d_spi - 360; elseif d_spi < -180, d_spi = d_spi + 360; end
                prev_spi_deg = spi_deg;

                sample_count = sample_count + 1;
                addpoints(hLineThOl,  sample_count, th_ol_deg);
                addpoints(hLineThSpi, sample_count, th_spi_display);
                addpoints(hLineThQep, sample_count, th_qep_display);
                addpoints(hLineSpd,   sample_count, spd_rpm);
                addpoints(hLineId,    sample_count, id_amps);
                addpoints(hLineIq,    sample_count, iq_amps);
                addpoints(hLineIa,    sample_count, ia_amps);
                addpoints(hLineIb,    sample_count, ib_amps);

                if sample_count > MAX_POINTS
                    xlim(axPos, [sample_count - MAX_POINTS, sample_count]);
                    xlim(axSpd, [sample_count - MAX_POINTS, sample_count]);
                    xlim(axFoc, [sample_count - MAX_POINTS, sample_count]);
                    xlim(axCur, [sample_count - MAX_POINTS, sample_count]);
                end

                % Board-side gate command is in byte 32 of the expanded frame.
                gate_cmd = NaN;
                if has_qep_diag
                    gate_cmd = double(frame(32));
                end
                if isnan(gate_cmd)
                    gate_text = 'PWM: ?';
                elseif gate_cmd > 0.5
                    gate_text = 'PWM: RELEASED';
                else
                    gate_text = 'PWM: TRIPPED';
                end

                % Update state text
                switch st_echo
                    case 1
                        set(lblState, 'String', 'STATE: LEGACY OPEN-LOOP +', 'ForegroundColor', [0.2 1.0 0.4]);
                    case 2
                        set(lblState, 'String', 'STATE: LEGACY OPEN-LOOP', 'ForegroundColor', [0.2 0.8 1.0]);
                    case 3
                        set(lblState, 'String', 'STATE: LEGACY OPEN-LOOP -', 'ForegroundColor', [1.0 0.7 0.2]);
                    case 9
                        set(lblState, 'String', 'STATE: AUTO-ALIGNING ON BOOT (Vdc > 30V)...', 'ForegroundColor', [1.0 0.85 0.2]);
                    case 4
                        set(lblState, 'String', sprintf('STATE: CALIBRATING (SPI = %.1f°)', spi_deg), 'ForegroundColor', [1.0 0.85 0.2]);
                    case 10
                        set(lblState, 'String', sprintf('STATE: CALIBRATION VALID (QEP Offset = %.2f°)', th_ol_deg), 'ForegroundColor', [0.2 1.0 0.4]);
                    case 11
                        set(lblState, 'String', 'STATE: ALIGNMENT OVERCURRENT - GATES OFF', 'ForegroundColor', [1.0 0.2 0.2]);
                    case 12
                        set(lblState, 'String', 'STATE: ALIGNMENT SAMPLE FAILURE - GATES OFF', 'ForegroundColor', [1.0 0.4 0.2]);
                    case 5
                        set(lblState, 'String', 'STATE: BOARD OPEN-LOOP + (2 RPM)', 'ForegroundColor', [0.3 1.0 0.5]);
                    case 6
                        set(lblState, 'String', 'STATE: COMMUTATED VOLTAGE (+2.0V Vq)', 'ForegroundColor', [0.4 0.9 1.0]);
                    case 7
                        set(lblState, 'String', 'STATE: BOARD OPEN-LOOP - (2 RPM)', 'ForegroundColor', [1.0 0.6 0.2]);
                    case 8
                        set(lblState, 'String', 'STATE: ALIGN FIRST — OUTPUT TRIPPED', 'ForegroundColor', [1.0 0.7 0.2]);
                    case 13
                        set(lblState, 'String', 'STATE: REQUESTED TEST BLOCKED — VDC < 30 V', 'ForegroundColor', [1.0 0.7 0.2]);
                    otherwise
                        set(lblState, 'String', 'STATE: STOPPED (Gates OFF)', 'ForegroundColor', [1.0 0.4 0.4]);
                end

                % Direction verification status
                if st_echo == 10
                    set(lblSyncDiag, 'String', sprintf('%s | CAL OFFSET: %.2f° | QEP ILAT: %d', gate_text, th_ol_deg, round(qep_ilat)), ...
                        'ForegroundColor', [0.2 1.0 0.4]);
                elseif has_qep_diag
                    dir_text = 'QEP DIR: ?';
                    if qep_dir > 0.5, dir_text = 'QEP DIR: FWD';
                    elseif qep_dir <= 0.5, dir_text = 'QEP DIR: REV'; end
                    set(lblSyncDiag, 'String', sprintf('%s | %s | IDX LATCH: %d', gate_text, dir_text, round(qep_ilat)), ...
                        'ForegroundColor', [0.4 0.8 1.0]);
                elseif st_echo == 1 || st_echo == 2
                    if d_spi > 0.05
                        set(lblSyncDiag, 'String', 'RAW SPI: INCREASING', 'ForegroundColor', [0.2 1.0 0.4]);
                    elseif d_spi < -0.05
                        set(lblSyncDiag, 'String', 'RAW SPI: DECREASING', 'ForegroundColor', [1.0 0.2 0.2]);
                    else
                        set(lblSyncDiag, 'String', 'RAW SPI: ZERO SPEED', 'ForegroundColor', [0.7 0.7 0.7]);
                    end
                elseif st_echo == 3
                    if d_spi > 0.05
                        set(lblSyncDiag, 'String', 'RAW SPI: INCREASING', 'ForegroundColor', [0.2 1.0 0.4]);
                    elseif d_spi < -0.05
                        set(lblSyncDiag, 'String', 'RAW SPI: DECREASING', 'ForegroundColor', [1.0 0.2 0.2]);
                    else
                        set(lblSyncDiag, 'String', 'RAW SPI: ZERO SPEED', 'ForegroundColor', [0.7 0.7 0.7]);
                    end
                else
                    set(lblSyncDiag, 'String', sprintf('QEP: %d cnt', qep_cnt), 'ForegroundColor', [0.4 0.8 1.0]);
                end

                set(lblSpd, 'String', sprintf('Speed: %+d RPM', round(spd_rpm)));
                if st_echo == 10
                    set(lblAngles, 'String', sprintf('CAL OFFSET: %.2f° | SPI: %.0f° | QEP: %.1f°', th_ol_deg, spi_deg, qep_deg));
                elseif st_echo == 5 || st_echo == 7
                    set(lblAngles, 'String', sprintf('THETA CMD: %.0f° | SPI: %.0f° | QEP: %.1f°', th_ol_deg, th_spi_display, qep_deg));
                else
                    set(lblAngles, 'String', sprintf('\\theta_{ol}: %.0f° | SPI: %.0f° | QEP: %.1f°', th_ol_deg, th_spi_display, qep_deg));
                end
                if has_qep_diag
                    set(lblFocVec, 'String', sprintf('Id: %+.2f A | Iq: %+.2f A | QEP ILAT: %d', ...
                        id_amps, iq_amps, round(qep_ilat)));
                else
                    set(lblFocVec, 'String', sprintf('Id: %+.2f A | Iq: %+.2f A', id_amps, iq_amps));
                end

                % Log data if recording is active
                if is_recording
                    t_now = toc(t_start);
                    log_data.t(end+1)         = t_now;
                    log_data.spi_deg(end+1)   = spi_deg;
                    log_data.qep_cnt(end+1)   = qep_cnt;
                    log_data.th_ol_deg(end+1) = th_ol_deg;
                    log_data.spd_rpm(end+1)   = spd_rpm;
                    log_data.ia_amps(end+1)   = ia_amps;
                    log_data.ib_amps(end+1)   = ib_amps;
                    log_data.id_amps(end+1)   = id_amps;
                    log_data.iq_amps(end+1)   = iq_amps;
                    log_data.state(end+1)     = st_echo;
                    set(lblRecStatus, 'String', sprintf('LOGGING: %d pts (%.1fs)', ...
                        length(log_data.t), t_now - log_data.t(1)));
                end
            end
        end
        drawnow;
        pause(0.01);
    end
catch ME
    if ishandle(hFig)
        fprintf(2, 'Dashboard loop error: %s\n', ME.message);
        fprintf(2, '%s\n', getReport(ME, 'extended', 'hyperlinks', 'off'));
        if exist('lblState', 'var') && ishandle(lblState)
            set(lblState, 'String', 'STATE: DISCONNECTED / ERROR', 'ForegroundColor', [1.0 0.2 0.2]);
        end
    else
        disp('Dashboard closed or interrupted.');
    end
end

if exist('s', 'var') && isvalid(s)
    delete(s);
end
disp('Serial port closed.');
end
