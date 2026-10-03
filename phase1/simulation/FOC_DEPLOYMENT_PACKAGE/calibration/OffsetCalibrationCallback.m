% Model         :   Offset calibration for QEP based 3 phase motor
% Description   :   Set Parameters for offset calibration for QEP

% Copyright 2020-2024 The MathWorks, Inc.

%% Set parameters from Dashboard selection

PWM_frequency       = eval(get_param([bdroot '/Offset Calculation/Parameters/PWM Frequency'], 'Value'));
motor.polePairs     = eval(get_param([bdroot '/Offset Calculation/Parameters/Number of pole pairs'], 'Value'));        % Pole Pairs for the motor
motor.calibSpeed    = 60;   % Motor speed during calibration process // RPM

%% Set Sample Times
T_pwm               = 1/PWM_frequency;  %s      // PWM switching time period
Ts          	    = T_pwm;        %sec        // Sample time for controller

selectedDataType    = eval(get_param([bdroot '/Offset Calculation/Parameters/Data Type'],'Value'));
if selectedDataType == 0
    dataType        = 'single';
else
    dataType        = fixdt(1,32,17);
end

clear selectedDataType;

%% Variables for algorithm Export / customized models 
% Target Hardware: TI LAUNCHXL-F28069M (BoosterPack Site 2: J5-J8)
% Inverter: EPC9147B Interface + EPC91200 GaN Inverter
% Motor: CubeMars AKE80-8 KV30 (21 Pole Pairs)
% Sensor: AS5047P (1024 slits / 4096 counts per rev ABI mode on eQEP1)
%
% PWM_frequency         = 20000;            % Hz (20 kHz, 2250 counts @ 90 MHz)
% motor.polePairs       = 21;               % Number of pole pairs (CubeMars AKE80-8)
% motor.QEPSlits        = 1024;             % Number of slits in QEP encoder (4096 counts in 4x mode)
% motor.calibSpeed      = 60;               % Motor speed during calibration [RPM]
% dataType              = 'single';         % Single precision floating point (FPU32)
% inverter.EnableLogic  = 1;                % Active high gate enable (GPIO-52)
% T_pwm                 = 1/PWM_frequency;  % [sec] PWM switching time period
% Ts                    = T_pwm;            % [sec] Sample time for controller
