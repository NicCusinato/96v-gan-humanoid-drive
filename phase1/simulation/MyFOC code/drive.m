function drive(cmd)
% DRIVE - Speed control and start/stop for FOC_Hardware on COM7
%
% Usage:
%   drive(100)       - Run closed-loop FOC at 100 RPM
%   drive(1)         - Run closed-loop FOC at 100 RPM
%   drive(200)       - Run closed-loop FOC at 200 RPM
%   drive(0)         - Stop motor immediately
%   drive('stop')    - Stop motor immediately
%   drive('close')   - Close and release COM7
%
    persistent s
    
    N_base = 1130.0; % Base speed from mcb_pmsm_foc_qep_f28069LaunchPad_data.m
    
    if nargin < 1
        disp('Usage: drive(100), drive(200), drive(0), drive(''stop''), drive(''close'')');
        return;
    end
    
    if ischar(cmd) && strcmpi(cmd, 'close')
        if ~isempty(s) && isvalid(s)
            delete(s);
        end
        clear s;
        existing = serialportfind('Port', 'COM7');
        if ~isempty(existing), delete(existing); end
        disp('COM7 closed and released.');
        return;
    end
    
    if ischar(cmd) && strcmpi(cmd, 'stop')
        cmd = 0;
    end
    
    % Open port if needed
    if isempty(s) || ~isvalid(s)
        try
            s = serialport('COM7', 115200, 'Timeout', 0.2);
            pause(0.2);
            flush(s);
            disp('Connected to LAUNCHXL-F28069M on COM7 (115200 baud).');
        catch e
            disp(['Error opening COM7: ' e.message]);
            return;
        end
    end
    
    if isnumeric(cmd)
        if cmd == 0
            pkt = int16([0, 0]);
            write(s, pkt, 'int16');
            fprintf('>>> MOTOR STOPPED: Disabled inverter (0 RPM).\n');
        else
            target_rpm = cmd;
            if target_rpm == 1, target_rpm = 100; end
            if target_rpm == 2, target_rpm = 200; end
            speed_pu = target_rpm / N_base;
            speed_q12 = int16(round(speed_pu * 4096));
            enable_q12 = int16(4096);
            pkt = [speed_q12, enable_q12];
            write(s, pkt, 'int16');
            fprintf('>>> MOTOR ENABLED: Commanded %d RPM (PU = %.4f, Q12 = %d)\n', ...
                target_rpm, speed_pu, speed_q12);
        end
    end
end
