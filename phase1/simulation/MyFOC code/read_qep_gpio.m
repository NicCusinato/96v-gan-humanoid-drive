% read_qep_gpio.m
% Sends a 1‑byte command to the board that we will add to the firmware
% (see step 2). The board returns three bits: [A B I] in the low 3 bits
% of the response byte.

s = serialport('COM7',115200,'Timeout',0.5);
fprintf('Reading raw eQEP pins – turn the motor shaft now …\n');

for k = 1:30
    write(s, uint8('G'), 'uint8');   % custom command we will add below
    pause(0.2);
    resp = read(s,1,'uint8');        % one byte reply
    a = bitand(resp, 1);             % bit0 = A
    b = bitand(bitshift(resp,-1),1); % bit1 = B
    i = bitand(bitshift(resp,-2),1); % bit2 = Index
    fprintf('  A=%d  B=%d  I=%d\n', a, b, i);
end

delete(s);