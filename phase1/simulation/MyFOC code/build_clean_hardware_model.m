%% build_clean_hardware_model.m
% Builds FOC_Clean_Hardware.slx from VirtualSpeedDyno with pristine hardware interfaces.

model = 'FOC_Clean_Hardware';
if bdIsLoaded(model), close_system(model, 0); end

% 1. Load VirtualSpeedDyno and clone
load_system('VirtualSpeedDyno');
save_system('VirtualSpeedDyno', model);

% 2. Configure target hardware
set_param(model, 'HardwareBoard', 'TI Piccolo F28069M LaunchPad');
set_param(model, 'SystemTargetFile', 'ert.tlc');
set_param(model, 'SolverType', 'Fixed-step');
set_param(model, 'Solver', 'FixedStepDiscrete');
set_param(model, 'FixedStep', '50e-6');
set_param(model, 'StopTime', 'inf');

% 3. Delete all Simscape plant blocks and unused scopes
s_blks = find_system(model, 'SearchDepth', 1, 'BlockType', 'SimscapeBlock');
for i=1:length(s_blks), delete_block(s_blks{i}); end

extra = {'Solver Configuration', 'Gate Driver', 'PWM Generator', ...
         'PS-Simulink Converter', 'PS-Simulink Converter1', ...
         'PS-Simulink Converter2', 'PS-Simulink Converter3', ...
         'Simulink-PS Converter', 'Scope', 'Scope1', 'Scope2', ...
         'Scope3', 'Scope4', 'Scope5', 'i-Tracking'};
for i=1:length(extra)
    b = [model '/' extra{i}];
    if ~isempty(find_system(model, 'SearchDepth', 1, 'Name', extra{i}))
        delete_block(b);
    end
end

% 4. Copy verified hardware blocks from Official_Encoder_Calib_Site2
load_system('Official_Encoder_Calib_Site2');
add_block('Official_Encoder_Calib_Site2/Heartbeat LED', [model '/Heartbeat LED'], 'Position', [50 50 150 100]);
add_block('Official_Encoder_Calib_Site2/Offset Calculation/PWM Output', [model '/PWM Output'], 'Position', [1150 250 1300 350]);

% Configure SCI_A on target
cs = getActiveConfigSet(model);
data = codertarget.data.getData(cs);
data.SCI_A.UserBaudRate = '115200';
data.SCI_A.BaudRate = 114796;
data.SCI_A.BaudRatePrescaler = 97;
codertarget.data.setData(cs, data);

save_system(model);
fprintf('Base clean model successfully saved.\n');
