% Fix PWM Stop State in Hardware_Diagnostics_Test.slx
% The Open_Loop_Controller was outputting 50% PWM even when stopped
% This script fixes it to output 0% PWM (neutral)

clear all; close all; clc;

model_path = 'Hardware_Diagnostics_Test.slx';
load_system(model_path);

% Find the Open_Loop_Controller block
block_path = [model_path(1:end-4) '/Open_Loop_Controller'];

% Get the MATLAB function code
mlfcn_block = get_param(block_path, 'Object');
code = mlfcn_block.UserData.msfcnDeepCopy.Script;

% Show current code (find the STOP state section)
fprintf('=== Current Open_Loop_Controller Code ===\n');
lines = split(code, newline);
for i = 1:length(lines)
    if contains(lines{i}, 'state == uint8(0)')
        % Print context around STOP state
        start_idx = max(1, i-2);
        end_idx = min(length(lines), i+8);
        for j = start_idx:end_idx
            fprintf('[%3d] %s\n', j, lines{j});
        end
        break
    end
end

fprintf('\nTo fix: Change line containing "pwm_counts = single([1125.0; 1125.0; 1125.0])"\n');
fprintf('in the STOP state (state == 0) to: pwm_counts = single([0.0; 0.0; 0.0])\n\n');

% Try to fix it
try
    new_code = strrep(code, ...
        'if state == uint8(0)' + newline + '       pwm_counts = single([1125.0; 1125.0; 1125.0]);', ...
        'if state == uint8(0)' + newline + '       pwm_counts = single([0.0; 0.0; 0.0]);');
    
    if ~strcmp(code, new_code)
        mlfcn_block.UserData.msfcnDeepCopy.Script = new_code;
        save_system(model_path);
        fprintf('✓ Fix applied successfully!\n');
    else
        fprintf('Could not find exact match. Manual edit required.\n');
    end
catch ME
    fprintf('Error: %s\n', ME.message);
    fprintf('Please manually edit the Open_Loop_Controller MATLAB function block:\n');
    fprintf('1. Double-click on Open_Loop_Controller block\n');
    fprintf('2. Find the section: if state == uint8(0)\n');
    fprintf('3. Change: pwm_counts = single([1125.0; 1125.0; 1125.0]);\n');
    fprintf('   To:     pwm_counts = single([0.0; 0.0; 0.0]);\n');
    fprintf('4. Click OK and re-deploy to hardware\n');
end

close_system(model_path, 0);
