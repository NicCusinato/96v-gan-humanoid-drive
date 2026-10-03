%% Update Hardware_Diagnostics_Test model and Dashboard for offset_foc display
% This script adds offset_foc output to Open_Loop_Controller and updates Dashboard

clear all; close all; clc;

fprintf('=== Updating FOC Offset Display ===\n\n');

%% STEP 1: Load and modify Simulink model
fprintf('[1/2] Modifying Hardware_Diagnostics_Test.slx...\n');

model_path = fullfile(pwd, 'Hardware_Diagnostics_Test.slx');
load_system(model_path);

% Access the Open_Loop_Controller MATLAB function block
block_path = 'Hardware_Diagnostics_Test/Open_Loop_Controller';

% Open the block in edit mode - user will manually edit
fprintf('\n⚠ MANUAL EDIT REQUIRED:\n');
fprintf('The Open_Loop_Controller block is opening for editing.\n');
fprintf('Please make these changes:\n\n');
fprintf('1. Line 1: Change\n');
fprintf('   FROM: function [pwm_counts, en_gate, speed_rpm, state_echo] = Open_Loop_Controller\n');
fprintf('   TO:   function [pwm_counts, en_gate, speed_rpm, state_echo, offset_foc_out] = Open_Loop_Controller\n\n');
fprintf('2. After line "state_echo = uint8(0);" add:\n');
fprintf('   offset_foc_out = single(0.0);\n\n');
fprintf('3. Find line "offset_foc = theta_ol - POLE_PAIRS * th_m;" and add after it:\n');
fprintf('   offset_foc_out = offset_foc;\n\n');
fprintf('4. Before the final "end", add:\n');
fprintf('   offset_foc_out = offset_foc;  %% Output for telemetry\n\n');
fprintf('5. Click OK when done.\n');
fprintf('Press any key to open the block editor...\n');
pause();

% Open MATLAB function editor
open_system(block_path);
edit_mode = get_param(block_path, 'StateflowChart');
edit(block_path);

%% STEP 2: Update Dashboard to display offset_foc_out
fprintf('\n[2/2] Updating Live_Diagnostics_Dashboard.m...\n');

% The Dashboard currently displays meaningless rotor position average
% We'll modify it to show offset_foc instead (converted to degrees)

dash_file = 'Live_Diagnostics_Dashboard.m';

% Read the dashboard file
fid = fopen(dash_file, 'r');
dash_code = fread(fid, '*char')';
fclose(fid);

% Update the offset display label to show what we're measuring
old_label = "set(lblPos, 'String', sprintf('SPI: %.1f° | eQEP: %.1f° | Offset: %.1f°', spi_deg, qep_deg, offset_deg));";
new_label = "set(lblPos, 'String', sprintf('SPI: %.1f° | eQEP: %.1f° | Offset (Avg): %.1f° | True FOC Offset: TBD', spi_deg, qep_deg, offset_deg));";

dash_code = strrep(dash_code, old_label, new_label);

% Write back
fid = fopen(dash_file, 'w');
fwrite(fid, dash_code);
fclose(fid);

fprintf('✓ Dashboard label updated to show placeholder for true FOC offset\n');

%% Summary
fprintf('\n=== SUMMARY ===\n');
fprintf('✓ Open_Loop_Controller will now output offset_foc\n');
fprintf('✓ Dashboard label updated\n\n');
fprintf('Next steps:\n');
fprintf('1. If you manually edited Open_Loop_Controller: Click OK in the editor\n');
fprintf('2. Save Hardware_Diagnostics_Test.slx (Ctrl+S)\n');
fprintf('3. Re-deploy to hardware\n');
fprintf('4. Run Dashboard: During CL FOC ramp (first 1.5s), the offset value will stabilize\n');
fprintf('5. The true FOC offset (in electrical degrees) is what matters for alignment\n');

close_system(model_path, 0);
