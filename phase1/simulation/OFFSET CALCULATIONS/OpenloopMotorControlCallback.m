% Model         :   Open Loop Control of 3-phase motors
% Description   :   Set Parameters for Open Loop Control of 3-phase motors
% File name     :   mcb_open_loop_control_data.m

% Copyright 2020 The MathWorks, Inc.

%% Set parameters from Dashboard selection
PWM_frequency = eval(get_param([bdroot '/Open Loop Control/Parameters/PWM Frequency'], 'Value'));               %Hz   PWM frquency
motor.polePairs     = eval(get_param([bdroot '/Open Loop Control/Parameters/Number of pole pairs'], 'Value'));  %     Pole Pairs for the motor
motor.base_speed    = eval(get_param([bdroot '/Open Loop Control/Parameters/Base Speed'], 'Value'));            %rpm  Rated speed (Synchronous Speed)

selectedDataType = eval(get_param([bdroot '/Open Loop Control/Parameters/Data Type'],'Value'));
if selectedDataType == 0
    dataType = 'single';
else
    dataType = fixdt(1,32,17);
end
clear selectedDataType;

%% Derive paramters for the model
T_pwm           = 1/PWM_frequency;  %[sec] PWM switching time period
Ts          	= T_pwm;            %[sec] Sample time for controller
motor.base_freq     = motor.base_speed*motor.polePairs/60; % Derive motor base frequency


%% Variables for algorithm Export / customized models 
% Target Hardware: TI LAUNCHXL-F28069M (BoosterPack Site 2: J5-J8)
% Inverter: EPC9147B Interface + EPC91200 GaN Inverter
% Motor: CubeMars AKE80-8 KV30 (21 Pole Pairs)
%
% PWM_frequency         = 20000;            % Hz (20 kHz, 2250 counts @ 90 MHz)
% motor.polePairs       = 21;               % Number of pole pairs (CubeMars AKE80-8)
% motor.base_speed      = 1440;             % Base speed [RPM] @ 48V
% dataType              = 'single';         % Single precision floating point (FPU32)
% inverter.EnableLogic  = 1;                % Active high gate enable (GPIO-52)
% T_pwm                 = 1/PWM_frequency;  % [sec] PWM switching time period
% Ts                    = T_pwm;            % [sec] Sample time for controller
% motor.base_freq       = motor.base_speed*motor.polePairs/60; % Base electrical frequency (504 Hz)