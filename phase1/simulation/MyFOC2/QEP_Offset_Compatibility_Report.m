function report = QEP_Offset_Compatibility_Report
%QEP_OFFSET_COMPATIBILITY_REPORT Compare the imported QEP examples with our hardware model.
%
% This is a read-only audit. It loads models for inspection, but does not
% edit or save any model. The report deliberately separates algorithm
% reuse from target-peripheral and board-wiring compatibility.

root = fileparts(mfilename('fullpath'));
referenceRoot = fullfile(root, 'reference_examples');

models = struct( ...
    'f28069', fullfile(referenceRoot, 'EncoderOffsetCalibrationF28069mLaunchPad.slx'), ...
    'f280049C', fullfile(referenceRoot, 'mcb_pmsm_qep_offset_f280049C.slx'), ...
    'hardware', fullfile(root, 'SAFECOPYFOC_Motor_Control_Hardware.slx'));

loadedBefore = cellfun(@(name) bdIsLoaded(name), fieldnames(models));
modelNames = fieldnames(models);
cleanupObj = onCleanup(@() closeNewModels(models, modelNames, loadedBefore));

for k = 1:numel(modelNames)
    load_system(models.(modelNames{k}));
end

report = struct();
report.models = models;
report.f28069_reference = collectModelInfo('EncoderOffsetCalibrationF28069mLaunchPad', ...
    'Offset Calculation');
report.f280049C_reference = collectModelInfo('mcb_pmsm_qep_offset_f280049C', ...
    'Offset Calculation');
report.hardware_model = collectHardwareInfo('SAFECOPYFOC_Motor_Control_Hardware');
report.checks = evaluateChecks(report);

printReport(report);
end

function info = collectModelInfo(modelName, offsetRoot)
info = collectCommonInfo(modelName);
info.qep = readQEP([modelName '/' offsetRoot '/eQEP']);
info.pwm = readPWM(modelName, offsetRoot, {'ePWM4', 'ePWM5', 'ePWM6'});
info.adcMasks = readADCMasks(modelName);
info.hasOffsetControlSystem = hasNamedBlock(modelName, 'Control_System');
info.hasQEPDecoder = hasNamedBlock(modelName, 'eQEP_Decoder');
info.hasHostModel = hasNamedBlock(modelName, 'SCI Transmit');
end

function info = collectHardwareInfo(modelName)
info = collectCommonInfo(modelName);
info.qep = readQEP([modelName '/Feedback/eQEP']);
info.pwm = readPWM(modelName, 'PWM Output', {'ePWM4', 'ePWM5', 'ePWM6'});
info.adcMasks = readADCMasks(modelName);
info.hasOffsetControlSystem = hasNamedBlock(modelName, 'Control_System');
info.hasQEPDecoder = hasNamedBlock(modelName, 'eQEP_Decoder');
info.hasHostModel = hasNamedBlock(modelName, 'SCI_Send') || ...
    hasNamedBlock(modelName, 'Serial_Send');

feedbackPath = [modelName '/Feedback/Feedback_Processing'];
configuration = get_param(feedbackPath, 'MATLABFunctionConfiguration');
info.feedbackFunction = configuration.FunctionScript;
info.feedbackHasHardcodedOffset = contains(info.feedbackFunction, 'TH_OFFSET');
info.feedbackHasHardcodedCounts = contains(info.feedbackFunction, '4096.0');
info.feedbackHasHardcodedPolePairs = contains(info.feedbackFunction, '21.0');
info.hasSPI = hasFunctionBlock(modelName, 'c28xspi');
end

function info = collectCommonInfo(modelName)
configSet = getActiveConfigSet(modelName);
info = struct();
info.hardwareBoard = get_param(modelName, 'HardwareBoard');
info.systemTargetFile = get_param(configSet, 'SystemTargetFile');
info.fixedStep = get_param(configSet, 'FixedStep');
info.stopTime = get_param(modelName, 'StopTime');
end

function qep = readQEP(blockPath)
names = {'useModule', 'pcInputmode', 'externalClockrate', ...
    'pcMaximumvalue', 'pcResetmode', 'qapPolarityselection', ...
    'qbpPolarityselection', 'qipPolarityselection', 'indexEventlatch'};
qep = struct();
for k = 1:numel(names)
    qep.(names{k}) = get_param(blockPath, names{k});
end
end

function pwm = readPWM(modelName, subsystemName, blockNames)
pwm = struct();
for k = 1:numel(blockNames)
    matches = find_system([modelName '/' subsystemName], ...
        'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
        'Type', 'Block', 'Name', blockNames{k});
    if ~isempty(matches)
        blockPath = matches{1};
        pwm.(blockNames{k}) = struct( ...
            'useModule', get_param(blockPath, 'useModule'), ...
            'countingMode', get_param(blockPath, 'countingMode'), ...
            'deadBandPolarity', get_param(blockPath, 'DBpolarity'), ...
            'deadBandEdge', get_param(blockPath, 'DBedgeDelay'));
    end
end
end

function masks = readADCMasks(modelName)
blocks = find_system(modelName, 'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', 'Type', 'Block', 'FunctionName', 'c2802xadc');
masks = cell(size(blocks));
for k = 1:numel(blocks)
    masks{k} = struct('path', blocks{k}, ...
        'maskValueString', get_param(blocks{k}, 'MaskValueString'));
end
end

function checks = evaluateChecks(report)
f69 = report.f28069_reference;
f49 = report.f280049C_reference;
hw = report.hardware_model;

checks = struct();
checks.referenceAlgorithmsMatch = ...
    f69.hasOffsetControlSystem && f49.hasOffsetControlSystem && ...
    f69.hasQEPDecoder && f49.hasQEPDecoder;
checks.f280049CIsF28069Target = strcmp(f49.hardwareBoard, f69.hardwareBoard);
checks.hardwareBoardMatchesF28069 = strcmp(hw.hardwareBoard, f69.hardwareBoard);
checks.qepModuleMatchesF28069 = strcmp(hw.qep.useModule, f69.qep.useModule);
checks.qepModeMatchesF28069 = strcmp(hw.qep.pcInputmode, f69.qep.pcInputmode) && ...
    strcmp(hw.qep.externalClockrate, f69.qep.externalClockrate) && ...
    strcmp(hw.qep.pcResetmode, f69.qep.pcResetmode);
checks.qepPolarityMatchesF28069 = strcmp(hw.qep.qapPolarityselection, f69.qep.qapPolarityselection) && ...
    strcmp(hw.qep.qbpPolarityselection, f69.qep.qbpPolarityselection) && ...
    strcmp(hw.qep.qipPolarityselection, f69.qep.qipPolarityselection);
checks.pwmModulesMatchF28069Reference = samePWMModuleSet(hw.pwm, f69.pwm);
checks.hardwareHasImportedOffsetAlgorithm = hw.hasOffsetControlSystem && hw.hasQEPDecoder;
checks.hardwareHasSPI = hw.hasSPI;
checks.hardwareUsesHardcodedOffset = hw.feedbackHasHardcodedOffset;
checks.hardwareUsesHardcodedQEPScale = hw.feedbackHasHardcodedCounts;
checks.encoderResolutionRequiresConfirmation = true;
checks.epcAdapterPinMapRequiresConfirmation = true;
end

function tf = samePWMModuleSet(left, right)
leftNames = fieldnames(left);
rightNames = fieldnames(right);
leftModules = sort(cellfun(@(name) left.(name).useModule, leftNames, ...
    'UniformOutput', false));
rightModules = sort(cellfun(@(name) right.(name).useModule, rightNames, ...
    'UniformOutput', false));
tf = isequal(leftModules, rightModules);
end

function tf = hasNamedBlock(modelName, blockName)
tf = ~isempty(find_system(modelName, 'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', 'Type', 'Block', 'Name', blockName));
end

function tf = hasFunctionBlock(modelName, functionName)
blocks = find_system(modelName, 'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', 'Type', 'Block');
tf = false;
for k = 1:numel(blocks)
    try
        if strcmp(get_param(blocks{k}, 'FunctionName'), functionName)
            tf = true;
            return;
        end
    catch
        % Most Simulink blocks do not expose FunctionName.
    end
end
end

function printReport(report)
f69 = report.f28069_reference;
f49 = report.f280049C_reference;
hw = report.hardware_model;
c = report.checks;

fprintf('\nQEP offset compatibility report\n');
fprintf('  F28069M reference board: %s\n', f69.hardwareBoard);
fprintf('  F280049C reference board: %s\n', f49.hardwareBoard);
fprintf('  Current hardware board: %s\n', hw.hardwareBoard);
fprintf('  F28069M reference ePWM modules: %s, %s, %s\n', ...
    f69.pwm.ePWM4.useModule, f69.pwm.ePWM5.useModule, f69.pwm.ePWM6.useModule);
fprintf('  Current hardware ePWM modules: %s, %s, %s\n', ...
    hw.pwm.ePWM4.useModule, hw.pwm.ePWM5.useModule, hw.pwm.ePWM6.useModule);
fprintf('  F28069M QEP: %s, %s, max=%s\n', f69.qep.useModule, ...
    f69.qep.externalClockrate, f69.qep.pcMaximumvalue);
fprintf('  Current QEP: %s, %s, max=%s\n', hw.qep.useModule, ...
    hw.qep.externalClockrate, hw.qep.pcMaximumvalue);

printCheck('Reference algorithm structure is shared', c.referenceAlgorithmsMatch);
printCheck('F280049C model is a drop-in F28069M target', c.f280049CIsF28069Target);
printCheck('Current model selects the F28069M board', c.hardwareBoardMatchesF28069);
printCheck('Current QEP module and operating mode match', ...
    c.qepModuleMatchesF28069 && c.qepModeMatchesF28069);
printCheck('Current QEP input polarity matches reference', c.qepPolarityMatchesF28069);
printCheck('Current PWM module map matches F28069M reference', c.pwmModulesMatchF28069Reference);
printCheck('Offset algorithm is already integrated in current model', ...
    c.hardwareHasImportedOffsetAlgorithm);
printCheck('SPI absolute-position driver is present', c.hardwareHasSPI);
printCheck('Current model still contains a hard-coded position offset', ...
    c.hardwareUsesHardcodedOffset);
fprintf('  REVIEW: encoder line count/index wiring must be confirmed before changing QEP scale.\n');
fprintf('  REVIEW: EPC9147B/EPC91200 adapter pin mapping must be confirmed against the actual harness.\n');
fprintf('  RESULT: reuse the F28069M algorithm, but do not deploy the F280049C target model unchanged.\n\n');
end

function printCheck(label, passed)
if passed
    fprintf('  PASS: %s\n', label);
else
    fprintf('  REVIEW: %s\n', label);
end
end

function closeNewModels(models, modelNames, loadedBefore)
for k = 1:numel(modelNames)
    modelName = modelNames{k};
    if ~loadedBefore(k) && isfield(models, modelName) && bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
end
end
