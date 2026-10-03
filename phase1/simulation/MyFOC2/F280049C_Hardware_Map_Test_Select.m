function active = F280049C_Hardware_Map_Test_Select(mode)
%F280049C_HARDWARE_MAP_TEST_SELECT Select one safe peripheral test.
%
% The model contains six independent test subsystems. This helper comments
% out every subsystem except the requested test and then updates the model.
% GPIO and SCI command receive are always kept active. GPIO34/nEN is driven
% high while the SCI command frame is zero (gate disabled).
%
% Valid modes: PWM, ADC, QEP, SPI, QEP_SPI, SCI, GPIO, ALL.
% ADC also keeps PWM active because ADC SOCs are triggered by ePWM1_SOCA.

if nargin < 1 || isempty(mode)
    mode = 'ALL';
end

mode = upper(char(mode));
validModes = {'PWM', 'ADC', 'QEP', 'SPI', 'QEP_SPI', 'SCI', 'GPIO', 'ALL'};
if ~ismember(mode, validModes)
    error('F280049C:InvalidTestMode', ...
        'Mode must be one of: %s.', strjoin(validModes, ', '));
end

cfg = F280049C_Hardware_Map_Test_Config();
if ~isfile(cfg.model)
    error('F280049C:MissingModel', 'Model not found: %s', cfg.model);
end

[~, modelName] = fileparts(cfg.model);
if ~bdIsLoaded(modelName)
    open_system(cfg.model);
end

requiredByMode = struct( ...
    'PWM', {{'Test_PWM', 'Test_GPIO', 'Test_SCI'}}, ...
    'ADC', {{'Test_PWM', 'Test_ADC', 'Test_GPIO', 'Test_SCI'}}, ...
    'QEP', {{'Test_QEP', 'Test_GPIO', 'Test_SCI'}}, ...
    'SPI', {{'Test_SPI', 'Test_GPIO', 'Test_SCI'}}, ...
    'QEP_SPI', {{'Test_QEP', 'Test_SPI', 'Test_GPIO', 'Test_SCI'}}, ...
    'SCI', {{'Test_SCI', 'Test_GPIO'}}, ...
    'GPIO', {{'Test_GPIO', 'Test_SCI'}}, ...
    'ALL', {{'Test_PWM', 'Test_ADC', 'Test_QEP', 'Test_SPI', 'Test_SCI', 'Test_GPIO'}});

allNames = {'Test_PWM', 'Test_ADC', 'Test_QEP', 'Test_SPI', 'Test_SCI', 'Test_GPIO'};
activeNames = requiredByMode.(mode);
topLevelBlocks = get_param(modelName, 'Blocks');

for k = 1:numel(allNames)
    if sum(strcmp(topLevelBlocks, allNames{k})) ~= 1
        error('F280049C:ModelStructure', ...
            'Expected one top-level subsystem named %s.', allNames{k});
    end
    block = [modelName '/' allNames{k}];
    if ismember(allNames{k}, activeNames)
        set_param(block, 'Commented', 'off');
    else
        set_param(block, 'Commented', 'on');
    end
end

set_param(modelName, 'SimulationCommand', 'update');
save_system(modelName);

active = activeNames;
fprintf('F280049C hardware-map test selected: %s\n', mode);
fprintf('Active subsystems: %s\n', strjoin(activeNames, ', '));
fprintf('GPIO34/nEN remains driven high: gate driver disabled.\n');
end
