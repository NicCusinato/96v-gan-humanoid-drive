% Model         :   Offset calibration for QEP based 3 phase motor
% Description   :   Set Parameters for offset calibration for QEP
% File name     :   mcb_c2000_pmsm_offset_data.m

% Copyright 2021 The MathWorks, Inc.

%% Set parameters from Dashboard selection

PWM_frequency = eval(get_param([bdroot '/Offset Calculation/Parameters/PWM Frequency'], 'Value'));
motor.polePairs     = eval(get_param([bdroot '/Offset Calculation/Parameters/Number of pole pairs'], 'Value'));        %           // Pole Pairs for the motor

%% Set Sample Times
T_pwm           = 1/PWM_frequency;  %s      // PWM switching time period
Ts          	= T_pwm;        %sec        // Sample time for controller
dataType = fixdt(1,32,17);


%% Variables for algorithm Export / customized models 

% PWM_frequency         = 20000;            % Hz
% motor.polePairs       = 4;                % Number of pole pairs
% motor.QEPSlits        = 1250;             % Number of slits in QEP encoder (necessary only for QEP offset)                                                                                         
% dataType              = 'single';         % Data type used in algorithm is floating point
% % dataType            = fixdt(1,32,17);   % Data type used in algorithm is fixed point
% inverter.EnableLogic  = 1;                % Active high enable inverter: 1, Active low enable: 0
% T_pwm                 = 1/PWM_frequency;  %[sec] PWM switching time period
% Ts                    = T_pwm;            %[sec] Sample time for controller
