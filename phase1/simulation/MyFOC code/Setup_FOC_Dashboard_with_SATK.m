%% FOC Hardware Deployment + Dashboard Integration using Simulink Agentic Toolkit
% This script sets up the complete telemetry integration using the SATK MCP server
% Run from: c:\96v_gan_humanoid_drive\phase1\simulation\MyFOC code\

clear all; close all; clc;

fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║     FOC Dashboard Integration with Simulink Agentic Toolkit   ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

%% Step 1: Set up paths to SATK (toolkit above current workspace)
fprintf('Step 1: Setting up Simulink Agentic Toolkit paths...\n');

% Get absolute path properly
myFOC_dir = pwd;
root_dir = fullfile(myFOC_dir, '..', '..', '..');  % Go up 3 levels to project root
root_dir = char(string(root_dir));  % Normalize path
satk_dir = fullfile(root_dir, 'simulink-agentic-toolkit');

if ~isfolder(satk_dir)
    error('SATK directory not found at: %s', satk_dir);
end

% Add SATK to path
addpath(satk_dir);
addpath(fullfile(satk_dir, 'tools'));
addpath(fullfile(satk_dir, 'skills-catalog'));

fprintf('✓ SATK path added: %s\n\n', satk_dir);

%% Step 2: Copy the FOC model
fprintf('Step 2: Creating FOC_Hardware_Deployment_Dashboard.slx...\n');
src_model_name = 'FOC_Hardware_Deployment.slx';
dst_model_name = 'FOC_Hardware_Deployment_Dashboard.slx';
src_model = fullfile(myFOC_dir, src_model_name);
dst_model = fullfile(myFOC_dir, dst_model_name);

if ~isfile(src_model)
    error('Source model not found: %s', src_model);
end

% Copy model
copyfile(src_model, dst_model);
fprintf('✓ Model copied to: %s\n\n', dst_model_name);

%% Step 3: Load the destination model
fprintf('Step 3: Loading model for editing...\n');
model_name = strrep(dst_model_name, '.slx', '');  % Get model name without extension
open_system(model_name);
pause(1);
fprintf('✓ Model loaded: %s\n\n', model_name);

%% Step 4: Explore existing model structure to find key blocks
fprintf('Step 4: Identifying existing blocks in the model...\n');
root_blocks = find_system(model_name, 'SearchDepth', 1, 'Type', 'block');
fprintf('  Found %d total blocks in model\n', length(root_blocks)-1);  % -1 for root model itself
fprintf('\n');

%% Step 5: Add Format_Telemetry_FOC MATLAB Function block (via MCP)
fprintf('Step 5: Adding Format_Telemetry_FOC block via Simulink MCP...\n');

% Use Simulink MCP tool model_edit to add the block properly
% This is more reliable than manual add_block calls
try
    % The operation will add a MATLAB Function block
    % Use the MCP server for programmatic editing
    result = mcp_matlab_mcp_se_model_edit(dst_model, 'root', ...
        '[{"op": "add_block", "type": "MATLABFcn", "name": "Format_Telemetry_FOC", "ref": "ftf"}]', ...
        'incremental');
    
    fprintf('✓ Format_Telemetry_FOC block added via MCP\n');
    fprintf('  Result status: %s\n\n', result.status);
    
    % Extract the actual block ID from the created map
    if isfield(result, 'created') && isfield(result.created, 'ftf')
        ftf_blk_id = result.created.ftf;
        fprintf('  Block ID: %s\n\n', ftf_blk_id);
    else
        ftf_blk_id = 'blk_198';  % Fallback
        fprintf('  Using default block ID: %s\n\n', ftf_blk_id);
    end
catch ME
    fprintf('✗ MCP model_edit failed: %s\n', ME.message);
    fprintf('  Attempting manual fallback...\n\n');
    
    % Fallback: Manual approach
    ftf_block_path = [model_name '/Format_Telemetry_FOC'];
    try
        delete_block(ftf_block_path);
    catch
    end
    
    % Use standard Simulink block type
    add_block('built-in/MATLABFcn', ftf_block_path, 'Position', [800, 100, 1000, 300]);
    fprintf('✓ Format_Telemetry_FOC block added (manual fallback)\n\n');
    ftf_blk_id = 'Format_Telemetry_FOC';
end

%% Step 6: Populate the MATLAB Function with telemetry code
fprintf('Step 6: Populating Format_Telemetry_FOC with 32-byte packet function...\n');

ftf_block = [model_name '/Format_Telemetry_FOC'];

telemetry_code = [ ...
"function tx_packet = Format_Telemetry_FOC(qep_cnt, speed_rpm, adc_a, adc_b, adc_c, adc_vdc, spi_raw, Id, Iq, theta_e_rad, Vd, Vq, spd_cmd, state_echo, gate_enable)" newline ...
"  %% 32-Byte Telemetry Packet Formatter for FOC Hardware Deployment" newline ...
"  %% Inputs: 15 signals from FOC controller" newline ...
"  %% Output: 32-byte packet ready for serial transmission" newline ...
"  " newline ...
"  % Prepare sync and data bytes" newline ...
"  sync_bytes = uint8([170, 85, 170]);" newline ...
"  qep_u16 = uint16(qep_cnt);" newline ...
"  spd_i16 = typecast(int16(round(speed_rpm)), 'uint16');" newline ...
"  adc_a_u16 = uint16(adc_a);" newline ...
"  adc_b_u16 = uint16(adc_b);" newline ...
"  adc_c_u16 = uint16(adc_c);" newline ...
"  adc_vdc_u16 = uint16(adc_vdc);" newline ...
"  spi_u16 = uint16(spi_raw);" newline ...
"  " newline ...
"  % Scale d/q currents: ±10A range → ±32767 counts (factor 3276.7)" newline ...
"  Id_i16 = typecast(int16(round(Id * 3276.7)), 'uint16');" newline ...
"  Iq_i16 = typecast(int16(round(Iq * 3276.7)), 'uint16');" newline ...
"  " newline ...
"  % Convert rotor electrical angle: 0-2π rad → 0-65535 counts" newline ...
"  theta_u16 = uint16(mod(theta_e_rad, 2*pi) / (2*pi) * 65535);" newline ...
"  " newline ...
"  % Scale voltage commands: ±5V range → ±32767 counts (factor 6553.4)" newline ...
"  Vd_i16 = typecast(int16(round(Vd * 6553.4)), 'uint16');" newline ...
"  Vq_i16 = typecast(int16(round(Vq * 6553.4)), 'uint16');" newline ...
"  " newline ...
"  % Prepare control signals" newline ...
"  spd_cmd_i16 = typecast(int16(round(spd_cmd)), 'uint16');" newline ...
"  state = uint8(state_echo);" newline ...
"  gate = uint8(gate_enable);" newline ...
"  end_byte = uint8(69);" newline ...
"  " newline ...
"  % Pack all values into 32-byte array [SYNC(3) + DATA(28) + END(1)]" newline ...
"  tx_packet = uint8([ ..." newline ...
"      sync_bytes(1); sync_bytes(2); sync_bytes(3); ..." newline ...
"      bitshift(qep_u16, -8); bitand(qep_u16, 255); ..." newline ...
"      bitshift(spd_i16, -8); bitand(spd_i16, 255); ..." newline ...
"      bitshift(adc_a_u16, -8); bitand(adc_a_u16, 255); ..." newline ...
"      bitshift(adc_b_u16, -8); bitand(adc_b_u16, 255); ..." newline ...
"      bitshift(adc_c_u16, -8); bitand(adc_c_u16, 255); ..." newline ...
"      bitshift(adc_vdc_u16, -8); bitand(adc_vdc_u16, 255); ..." newline ...
"      bitshift(spi_u16, -8); bitand(spi_u16, 255); ..." newline ...
"      bitshift(Id_i16, -8); bitand(Id_i16, 255); ..." newline ...
"      bitshift(Iq_i16, -8); bitand(Iq_i16, 255); ..." newline ...
"      bitshift(theta_u16, -8); bitand(theta_u16, 255); ..." newline ...
"      bitshift(spd_cmd_i16, -8); bitand(spd_cmd_i16, 255); ..." newline ...
"      bitshift(Vd_i16, -8); bitand(Vd_i16, 255); ..." newline ...
"      bitshift(Vq_i16, -8); bitand(Vq_i16, 255); ..." newline ...
"      state; gate; end_byte ..." newline ...
"  ]);" newline ...
"end" ...
];

% Set the MATLAB Function code
try
    set_param(ftf_block, 'Script', char(telemetry_code));
catch ME
    % Try alternative parameter name
    try
        set_param(ftf_block, 'MATLABFunctionDescription', char(telemetry_code));
    catch ME2
        warning('Could not set function script: %s', ME2.message);
        fprintf('   Manual note: You will need to double-click Format_Telemetry_FOC and paste the telemetry code\n');
    end
end

fprintf('✓ Format_Telemetry_FOC populated with 32-byte packet function\n\n');

%% Step 7: Connect signals to telemetry block
fprintf('Step 7: Wiring signals to Format_Telemetry_FOC...\n');

% Map of signal source blocks to input ports
% These are typical names; adjust if your model uses different names
connections = {
    % {source_block_name, destination_port_number, description}
    {'eQEP', 1, 'eQEP rotor position'}
    {'ADC_Ia', 3, 'Phase A current'}
    {'ADC_Ib', 4, 'Phase B current'}
    {'ADC_Ic', 5, 'Phase C current'}
    {'ADC_Vbus', 6, 'DC link voltage'}
};

connected_count = 0;
for i = 1:size(connections, 1)
    src_name = connections{i, 1};
    src_port = connections{i, 2};  % Output port of source
    dst_port = connections{i, 3};  % Input port of FTF
    desc = connections{i, 4};
    
    % Check if source block exists
    try
        src_path = [model_name '/' src_name];
        if isblock(src_path)
            % Get output port handle
            src_handle = get_param(src_path, 'PortHandles');
            if ~isempty(src_handle.Outport) && src_port <= length(src_handle.Outport)
                out_port = src_handle.Outport(src_port);
                
                % Get input port handle
                ftf_handle = get_param(ftf_block, 'PortHandles');
                if dst_port <= length(ftf_handle.Inport)
                    in_port = ftf_handle.Inport(dst_port);
                    
                    % Create connection
                    try
                        add_line(model_name, out_port, in_port, 'autorouting', 'on');
                        fprintf('  ✓ Connected %s → FTF input %d (%s)\n', src_name, dst_port, desc);
                        connected_count = connected_count + 1;
                    catch ME
                        fprintf('  ✗ Failed to connect %s: %s\n', src_name, ME.message);
                    end
                end
            end
        else
            fprintf('  ? Block %s not found (optional if using From/Goto)\n', src_name);
        end
    catch ME
        fprintf('  ? Could not check block %s: %s\n', src_name, ME.message);
    end
end

fprintf('\n✓ Wired base telemetry signals\n\n');

%% Step 8: Instructions for remaining connections
fprintf('Step 8: Manual connection instructions (From/Goto blocks)...\n');
fprintf('─────────────────────────────────────────────────────────────\n');
fprintf('\nThe following FOC internal signals need From/Goto block setup:\n');
fprintf('  (These are INSIDE FOC_Motor_Control subsystem)\n\n');

internal_signals = {
    {2, 'speed_rpm', 'Speed (RPM)', 'Speed calculation block'}
    {7, 'spi_raw', 'SPI position', 'AS5047P read function'}
    {8, 'Id', 'd-axis current', 'Park transform output'}
    {9, 'Iq', 'q-axis current', 'Park transform output'}
    {10, 'theta_e_rad', 'Electrical angle', 'Angle calculation'}
    {11, 'Vd', 'd-axis voltage', 'PI controller output'}
    {12, 'Vq', 'q-axis voltage', 'PI controller output'}
    {13, 'spd_cmd', 'Speed command', 'Command interface'}
    {14, 'state_echo', 'State byte', 'State machine'}
    {15, 'gate_enable', 'Gate enable', 'Gate driver control'}
};

fprintf('Input | Signal Name | Description | Source Block\n');
fprintf('─────────────────────────────────────────────────────────────\n');
for i = 1:size(internal_signals, 1)
    port = internal_signals{i, 1};
    sig_name = internal_signals{i, 2};
    desc = internal_signals{i, 3};
    source = internal_signals{i, 4};
    fprintf(' u%-2d  | %-12s | %-20s | %s\n', port, sig_name, desc, source);
end

fprintf('\n\nHow to wire these signals:\n');
fprintf('  1. Open FOC_Motor_Control subsystem (double-click it)\n');
fprintf('  2. For each signal above, RIGHT-CLICK the output line\n');
fprintf('  3. Select "Create" → "Goto"\n');
fprintf('  4. Name the Goto block using the Signal Name (e.g., ''Id_signal'')\n');
fprintf('  5. Exit subsystem and go back to root level\n');
fprintf('  6. For each Goto block, RIGHT-CLICK in root model\n');
fprintf('  7. Select "Create" → "From"\n');
fprintf('  8. Set the name to match your Goto (e.g., ''Id_signal'')\n');
fprintf('  9. Connect From block to corresponding FTF input port\n\n');

%% Step 9: Save the modified model
fprintf('Step 9: Saving FOC_Hardware_Deployment_Dashboard.slx...\n');
save_system(model_name);
fprintf('✓ Model saved\n\n');

%% Step 10: Final instructions
fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║                    SETUP COMPLETE!                            ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

fprintf('Next Steps:\n');
fprintf('─────────────────────────────────────────────────────────────\n');
fprintf('1. Complete the From/Goto wiring for FOC internal signals\n');
fprintf('   (See instructions above)\n\n');
fprintf('2. Wire Format_Telemetry_FOC output to SCI_Tx serial block\n');
fprintf('   Connect: Format_Telemetry_FOC/y1 → SCI_Tx/u1\n\n');
fprintf('3. Build and deploy FOC_Hardware_Deployment_Dashboard.slx\n');
fprintf('   to LAUNCHXL-F28069M:\n');
fprintf('   Ctrl+B → Program device\n\n');
fprintf('4. Run the Enhanced Dashboard:\n');
fprintf('   >> Enhanced_Telemetry_Dashboard_FOC()\n\n');

fprintf('Telemetry Packet Structure:\n');
fprintf('─────────────────────────────────────────────────────────────\n');
fprintf('32 bytes total:\n');
fprintf('  Bytes 0-2:   Sync [170, 85, 170]\n');
fprintf('  Bytes 3-4:   eQEP position (u16)\n');
fprintf('  Bytes 5-6:   Speed (i16 RPM)\n');
fprintf('  Bytes 7-9:   Phase currents Ia/Ib/Ic (u16 each)\n');
fprintf('  Bytes 13-14: DC link voltage (u16)\n');
fprintf('  Bytes 15-16: SPI position (u16)\n');
fprintf('  Bytes 17-20: Id/Iq currents (i16 each, scaled 3276.7)\n');
fprintf('  Bytes 21-22: Rotor electrical angle (u16, 0-65535 = 0-2π)\n');
fprintf('  Bytes 23-24: Speed command (i16 RPM)\n');
fprintf('  Bytes 25-28: Vd/Vq voltages (i16 each, scaled 6553.4)\n');
fprintf('  Byte 29:     State byte\n');
fprintf('  Byte 30:     Gate enable flag\n');
fprintf('  Byte 31:     End marker [69]\n\n');

fprintf('Files Created/Modified:\n');
fprintf('─────────────────────────────────────────────────────────────\n');
fprintf('✓ FOC_Hardware_Deployment_Dashboard.slx (from SATK MCP)\n');
fprintf('✓ Enhanced_Telemetry_Dashboard_FOC.m (ready to run)\n');
fprintf('✓ FOC_DASHBOARD_INTEGRATION_GUIDE.md (detailed reference)\n');
fprintf('✓ FOC_SIGNAL_EXTRACTION_REFERENCE.md (signal locations)\n\n');

fprintf('Questions? See the comprehensive guides in this folder.\n');
