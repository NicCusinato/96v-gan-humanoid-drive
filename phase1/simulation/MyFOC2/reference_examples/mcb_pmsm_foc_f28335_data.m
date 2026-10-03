% Model         :   PMSM Field Oriented Control
% Description   :   Set Parameters for PMSM Field Oriented Control
% File name     :   mcb_pmsm_foc_f28335_data.m
% Copyright 2020 - 2024 The MathWorks, Inc.

%% Simulation Parameters 

%% Set PWM Switching frequency
PWM_frequency 	= 20e3;    %Hz          // converter s/w freq
T_pwm           = 1/PWM_frequency;  %s  // PWM switching time period

%% Set Sample Times
Ts          	= T_pwm;        %sec        // simulation time step for controller
Ts_simulink     = T_pwm/2;      %sec        // simulation time step for model simulation
Ts_motor        = T_pwm/2;      %Sec        // Simulation sample time
Ts_inverter     = T_pwm/2;      %sec        // simulation time step for average value inverter
Ts_speed        = 10*Ts;        %Sec        // Sample time for speed controller
% Ts_speed        = 10e-3;      %Sec        // Sample time for speed controller

%% Set data type for controller & code-gen
% Uncomment the required data type and comment the other
%dataType = fixdt(1,32,17);    % Fixed point code-generation
dataType = 'single';            % Floating point code-generation

%% System Parameters // Hardware parameters

pmsm = mcb.getPMSMParameters('BLY171D');
switch position_decoder 
    case 'hall'
        pmsm.PositionOffset = 0.24; 
    case 'qep'
        pmsm.PositionOffset = 0.0664;
        pmsm.QEPSlits = 2000;
end
%% Target & Inverter Parameters

target = mcb.getProcessorParameters('F28335',PWM_frequency);  

inverter = mcb.getInverterParameters('DRV8312-C2-KIT'); 
%Manual Calibration of ADC offsets
inverter.CtSensAOffset = 2275;        		            %Counts // ADC Offset for phase-A 
inverter.CtSensBOffset = 2275;        		            %Counts // ADC Offset for phase-B 

%% Derive Characteristics
pmsm.N_base = mcb.getMotorBaseSpeed(pmsm,inverter); %rpm // Base speed of motor at given Vdc
% mcb.PMSMCharacteristics(pmsm,inverter);

%% PU System details // Set base values for pu conversion

PU_System = mcb.getPUSystemParameters(pmsm,inverter);

%% Controller design // Get ballpark values!

PI_params = mcb.getPIControllerParameters(pmsm,inverter,PU_System,T_pwm,Ts,Ts_speed);

%Updating delays for simulation
PI_params.delay_Currents    = int32(Ts/Ts_simulink);
PI_params.delay_Position    = int32(Ts/Ts_simulink);
PI_params.delay_Speed       = int32(Ts_speed/Ts_simulink);
PI_params.delay_Speed1       = (PI_params.delay_IIR + 0.5*Ts)/Ts_speed;

% mcb.getMotorControlAnalysis(pmsm,inverter,PU_System,PI_params,Ts,Ts_speed);
%% Displaying model variables
disp(pmsm);
disp(inverter);
disp(target);
disp(PU_System);