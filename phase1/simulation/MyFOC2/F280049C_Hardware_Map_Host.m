function result = F280049C_Hardware_Map_Host(port, action, varargin)
%F280049C_HARDWARE_MAP_HOST Send bench commands and read map telemetry.
%
% Target command frame, little-endian uint16 words:
%   [gate_enable, duty_u_percent, duty_v_percent, duty_w_percent]
% gate_enable = 0 keeps GPIO34/nEN high (disabled); 1 requests enable.
%
% Target telemetry frame, little-endian uint16 words:
%   [qep_position_count, qep_index_latch, spi_raw_word]
%
% Examples:
%   F280049C_Hardware_Map_Host('COM7', 'command', 0, 25, 50, 75)
%   F280049C_Hardware_Map_Host('COM7', 'command', 1, 25, 50, 75)
%   t = F280049C_Hardware_Map_Host('COM7', 'read')
%   F280049C_Hardware_Map_Host('COM7', 'watch', 10)
%   F280049C_Hardware_Map_Host('COM7', 'close')
%
% The serial settings match the imported F280049C host example: 1.5 Mbaud,
% 8 data bits, no parity, one stop bit. Change only the COM port here.

persistent serialDevice serialPort

if nargin < 2
    error('F280049C:HostArguments', 'Provide a COM port and an action.');
end

action = lower(char(action));
if strcmp(action, 'close')
    serialDevice = [];
    serialPort = '';
    result = [];
    return;
end

if isempty(serialDevice) || ~strcmpi(serialPort, char(port))
    serialDevice = serialport(char(port), 1.5e6, ...
        'DataBits', 8, 'Parity', 'none', 'StopBits', 1, 'Timeout', 0.2);
    serialPort = char(port);
    flush(serialDevice);
end

switch action
    case 'command'
        if numel(varargin) ~= 4
            error('F280049C:HostCommand', ...
                'command requires gate_enable, duty_u, duty_v, duty_w.');
        end
        payload = uint16([varargin{:}]);
        if payload(1) > 1 || any(payload(2:4) > 100)
            error('F280049C:HostCommandRange', ...
                'gate_enable must be 0/1 and duties must be 0..100 percent.');
        end
        sendFrame(serialDevice, payload);
        result = payload;

    case 'disable'
        sendFrame(serialDevice, uint16([0 0 0 0]));
        result = uint16([0 0 0 0]);

    case 'enable'
        sendFrame(serialDevice, uint16([1 0 0 0]));
        result = uint16([1 0 0 0]);

    case 'read'
        result = readTelemetry(serialDevice);

    case 'watch'
        if numel(varargin) ~= 1
            error('F280049C:HostWatch', 'watch requires a duration in seconds.');
        end
        stopTime = tic;
        result = struct('qep_position_count', {}, ...
            'qep_index_latch', {}, 'spi_raw_word', {}, 'time_s', {});
        k = 0;
        while toc(stopTime) < double(varargin{1})
            try
                sample = readTelemetry(serialDevice);
                k = k + 1;
                sample.time_s = toc(stopTime);
                result(k) = sample;
                fprintf('QEP=%6u  INDEX=%u  SPI=0x%04X\n', ...
                    sample.qep_position_count, sample.qep_index_latch, ...
                    sample.spi_raw_word);
            catch ME
                fprintf('Telemetry timeout: %s\n', ME.message);
            end
        end

    otherwise
        error('F280049C:HostAction', ...
            'Action must be command, disable, enable, read, watch, or close.');
end
end

function sendFrame(device, payload)
bytes = zeros(1, 2 * numel(payload), 'uint8');
bytes(1:2:end) = uint8(bitand(payload, 255));
bytes(2:2:end) = uint8(bitshift(payload, -8));
write(device, [uint8('S') bytes uint8('E')], 'uint8');
end

function sample = readTelemetry(device)
deadline = tic;
while toc(deadline) < 1
    if device.NumBytesAvailable == 0
        pause(0.002);
        continue;
    end
    if read(device, 1, 'uint8') ~= uint8('S')
        continue;
    end
    bytes = read(device, 6, 'uint8');
    if read(device, 1, 'uint8') ~= uint8('E')
        continue;
    end
    words = uint16(bytes(1:2:end)) + bitshift(uint16(bytes(2:2:end)), 8);
    sample = struct('qep_position_count', words(1), ...
        'qep_index_latch', words(2), 'spi_raw_word', words(3));
    return;
end
error('F280049C:TelemetryTimeout', 'No complete S...E telemetry frame received.');
end
