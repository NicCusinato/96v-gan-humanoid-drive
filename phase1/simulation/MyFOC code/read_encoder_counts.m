% read_encoder_counts.m
% Decodes the 10-byte telemetry packet from Custom_Encoder_Calib:
% Header: 'SS' (0x53, 0x53)
% uint16 #1: Scale_Offset
% uint16 #2: mech_cnt / Rate_Trans_QEP (Raw eQEP count: 0 - 4095)
% uint16 #3: Scale_AngleInj
% uint16 #4: Scale_AngleQEP

clear s;
s = serialport('COM7', 115200, 'Timeout', 0.5);
flush(s);
fprintf('===================================================\n');
fprintf('Reading encoder positions from COM7...\n');
fprintf('Rotate shaft by hand to observe mechanical counts.\n');
fprintf('===================================================\n\n');

for k = 1:40
    % Find header 0x53 0x53 ("SS")
    foundHeader = false;
    for attempt = 1:50
        if s.NumBytesAvailable < 10
            pause(0.02);
        end
        b = read(s, 1, 'uint8');
        if b == 83 % ASCII 'S'
            b2 = read(s, 1, 'uint8');
            if b2 == 83
                foundHeader = true;
                break;
            end
        end
    end

    if foundHeader
        data = read(s, 8, 'uint8');
        if numel(data) == 8
            pkt = typecast(uint8(data), 'uint16');
            offset_val  = pkt(1);
            mech_counts = pkt(2);
            ang_inj     = pkt(3);
            ang_qep     = pkt(4);
            fprintf('Sample %02d: eQEP Mechanical Count = %5d (Offset = %d)\n', k, mech_counts, offset_val);
        end
    else
        fprintf('Sample %02d: Waiting for header...\n', k);
    end
    pause(0.1);
end

delete(s);
clear s;
fprintf('\nDone.\n');