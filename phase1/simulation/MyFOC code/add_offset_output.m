%% Add offset_foc as 5th output to Open_Loop_Controller
% This script modifies Hardware_Diagnostics_Test.slx to output the true FOC offset

model_path = 'Hardware_Diagnostics_Test.slx';

% Load model
load_system(model_path);

% Access the Open_Loop_Controller block
block_path = [model_path(1:end-4) '/Open_Loop_Controller'];

try
    % Get the block handle
    h_block = get_param(block_path, 'Handle');
    
    % Get current MATLAB function code
    code = get_param(block_path, 'MatlabCode');
    
    % Modify function signature: add offset_foc_out as 5th output
    code = strrep(code, ...
        'function [pwm_counts, en_gate, speed_rpm, state_echo] = Open_Loop_Controller', ...
        'function [pwm_counts, en_gate, speed_rpm, state_echo, offset_foc_out] = Open_Loop_Controller');
    
    % Preallocate offset_foc_out at the top
    code = strrep(code, ...
        ['    pwm_counts = single([1125.0; 1125.0; 1125.0]);' newline ...
         '    en_gate = false;' newline ...
         '    speed_rpm = single(0.0);' newline ...
         '    state_echo = uint8(0);'], ...
        ['    pwm_counts = single([1125.0; 1125.0; 1125.0]);' newline ...
         '    en_gate = false;' newline ...
         '    speed_rpm = single(0.0);' newline ...
         '    state_echo = uint8(0);' newline ...
         '    offset_foc_out = single(0.0);']);
    
    % Output offset_foc at the end (before end statement)
    % Insert before the final "end"
    [tokens] = regexp(code, '(.*?)(^\s*end\s*$)', 'tokens', 'lineanchors');
    if ~isempty(tokens)
        % Add offset output at end of function
        code = strrep(code, 'theta_ol = theta_ol + w_e_ramp * single(0.001);', ...
            ['theta_ol = theta_ol + w_e_ramp * single(0.001);' newline ...
             '           offset_foc_out = offset_foc;  % Output to telemetry']);
    end
    
    % Set the modified code back
    set_param(block_path, 'MatlabCode', code);
    
    fprintf('✓ Successfully added offset_foc_out to Open_Loop_Controller\n');
    fprintf('  Now add output port to block: Right-click Open_Loop_Controller > Edit\n');
    
    % Save model
    save_system(model_path);
    fprintf('✓ Model saved\n');
    
catch ME
    fprintf('Error: %s\n', ME.message);
    fprintf('\nManual steps:\n');
    fprintf('1. Open Hardware_Diagnostics_Test.slx\n');
    fprintf('2. Double-click Open_Loop_Controller block\n');
    fprintf('3. Change line 1: function [pwm_counts, en_gate, speed_rpm, state_echo] = Open_Loop_Controller\n');
    fprintf('   To: function [pwm_counts, en_gate, speed_rpm, state_echo, offset_foc_out] = Open_Loop_Controller\n');
    fprintf('4. Add line after preallocate:\n');
    fprintf('   offset_foc_out = single(0.0);\n');
    fprintf('5. Add line after "offset_foc = theta_ol - POLE_PAIRS * th_m;"\n');
    fprintf('   offset_foc_out = offset_foc;\n');
    fprintf('6. Click OK\n');
    fprintf('7. Connect new output port to Format_Telemetry block\n');
end

close_system(model_path, 0);
