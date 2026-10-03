function offset = calibrate_encoder(action)
% CALIBRATE_ENCODER - Run live encoder offset calibration on LAUNCHXL-F28069M
%
% Usage:
%   offset = calibrate_encoder()       - Run 8-second calibration and display live telemetry
%   calibrate_encoder('stop')          - Abort calibration immediately
%   calibrate_encoder('close')         - Release COM7
%

    port = 'COM7';
    baud = 115200;
    
    if nargin >= 1 && (strcmpi(action, 'stop') || strcmpi(action, '0'))
        try
            s = serialport(port, baud, 'Timeout', 0.5);
            write(s, '0', 'char');
            delete(s);
            disp('>>> Calibration ABORTED (Sent ''0'').');
        catch e
            disp(['Error sending stop: ' e.message]);
        end
        return;
    end
    
    if nargin >= 1 && strcmpi(action, 'close')
        delete(serialportfind('Port', port));
        disp(['Released ' port]);
        return;
    end
    
    % Clear any existing connections on port
    existing = serialportfind('Port', port);
    if ~isempty(existing)
        delete(existing);
    end
    
    fprintf('Connecting to %s at %d baud...\n', port, baud);
    try
        s = serialport(port, baud, 'Timeout', 0.2);
        pause(0.5); % Allow DTR/RTS lines to stabilize
        flush(s);
    catch e
        error('Failed to open %s: %s\nMake sure no other software is holding the port.', port, e.message);
    end
    
    % Send Start Command '1'
    fprintf('>>> Triggering Calibration: Sending ''1'' to LaunchPad...\n');
    write(s, '1', 'char');
    pause(0.05);
    write(s, '1', 'char'); % Send confirmation pulse
    
    fprintf('\n=======================================================================\n');
    fprintf('  LIVE ENCODER CALIBRATION RUNNING (CubeMars AKE80-8 KV30)\n');
    fprintf('  Phase 1: Rotor alignment to 0 deg elec (t = 0 - 2.5s)\n');
    fprintf('  Phase 2: Open-loop synchronous spin @ ~15 RPM (t = 2.5 - 7.5s)\n');
    fprintf('=======================================================================\n');
    fprintf('%-8s | %-12s | %-12s | %-12s | %-12s\n', 'Time (s)', 'eQEP Counts', 'Injected (rad)', 'eQEP (rad)', 'Offset (PU)');
    fprintf('-----------------------------------------------------------------------\n');
    
    tStart = tic;
    lastPrint = 0;
    final_offset_pu = 0;
    started = false;
    
    cleanupObj = onCleanup(@() cleanup_serial(s));
    
    while toc(tStart) < 10.0
        tNow = toc(tStart);
        
        % Re-trigger if not started after 1 second
        if ~started && tNow > 1.0 && tNow < 1.5
            write(s, '1', 'char');
        end
        
        if s.NumBytesAvailable >= 10
            b = read(s, 1, 'uint8');
            if b == uint8('S')
                b2 = read(s, 1, 'uint8');
                if b2 == uint8('S')
                    data = read(s, 4, 'uint16');
                    offset_pu_raw = double(data(1)) / 10000.0;
                    raw_counts    = double(data(2));
                    angle_inj     = double(data(3)) * (2 * pi) / 10000.0;
                    angle_qep     = double(data(4)) * (2 * pi) / 10000.0;
                    
                    final_offset_pu = offset_pu_raw;
                    if offset_pu_raw > 0 || angle_inj > 0 || raw_counts > 0
                        started = true;
                    end
                    
                    if (tNow - lastPrint) >= 0.25
                        lastPrint = tNow;
                        if tNow < 2.5
                            status = 'Aligning';
                        elseif tNow < 7.5
                            status = 'Spinning';
                        else
                            status = 'Done';
                        end
                        fprintf('%5.1f s [%s] | Counts: %5d | Inj: %5.2f rad | QEP: %5.2f rad | Offset: %0.4f PU\n', ...
                            tNow, status, raw_counts, angle_inj, angle_qep, offset_pu_raw);
                    end
                end
            end
        else
            pause(0.01);
        end
    end
    
    fprintf('-----------------------------------------------------------------------\n');
    fprintf('>>> CALIBRATION COMPLETE!\n');
    final_offset_rad = final_offset_pu * 2 * pi;
    final_offset_deg = final_offset_rad * 180 / pi;
    fprintf('>>> Calibrated Offset (PU)  : %0.4f\n', final_offset_pu);
    fprintf('>>> Calibrated Offset (rad) : %0.4f rad\n', final_offset_rad);
    fprintf('>>> Calibrated Offset (deg) : %0.2f deg\n', final_offset_deg);
    fprintf('=======================================================================\n\n');
    
    assignin('base', 'calib_offset_pu', final_offset_pu);
    assignin('base', 'calib_offset_rad', final_offset_rad);
    evalin('base', 'pmsm.PositionOffset = calib_offset_pu;');
    fprintf('Updated pmsm.PositionOffset = %0.4f in MATLAB base workspace.\n', final_offset_pu);
    
    if nargout > 0
        offset = final_offset_pu;
    end
end

function cleanup_serial(s)
    if ~isempty(s) && isvalid(s)
        try
            write(s, '0', 'char');
        catch
        end
        delete(s);
    end
end
