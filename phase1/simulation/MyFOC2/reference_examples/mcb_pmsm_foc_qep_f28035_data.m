% Model         :   PMSM Field Oriented Control
% Description   :   Set Parameters for PMSM Field Oriented Control
% File name     :   mcb_pmsm_foc_qep_f28035_data.m

% Copyright 2020 - 2024 The MathWorks, Inc.

%% Set PWM Switching frequency
PWM_frequency 	= 20e3;    %Hz          // converter s/w freq
T_pwm           = 1/PWM_frequency;  %s  // PWM switching time period

%% Set Sample Times
Ts          	= T_pwm;        %sec        // Sample time for control system
Ts_simulink     = T_pwm/2;      %sec        // Simulation time step for model simulation
Ts_motor        = T_pwm/2;      %Sec        // Simulation time step for pmsm
Ts_inverter     = T_pwm/2;      %sec        // Simulation time step for inverter
Ts_speed        = 10*Ts;        %Sec        // Sample time for speed controller

%% Set data type for controller & code-gen
dataType = fixdt(1,32,17);    % Fixed point code-generation

%% System Parameters // Hardware parameters 

pmsm = mcb.getPMSMParameters('BLY171D');
pmsm.PositionOffset = 0.0662;     % Per-Unit position offset
pmsm.QEPSlits       = 2000;       % QEP Encoder Slits

%% Target & Inverter Parameters

target = mcb.getProcessorParameters('F28035',PWM_frequency);

inverter = mcb.getInverterParameters('DRV8312-C2-KIT');

%% Calibration section 
% Enable automatic calibration of ADC offset for current measurement
inverter.ADCOffsetCalibEnable = 1;  % Enable: 1, Disable: 0

% If automatic ADC offset calibration is disabled, uncomment and update the 
% offset values below manually
% inverter.CtSensAOffset = 2295;      % ADC Offset for phase current A 
% inverter.CtSensBOffset = 2286;      % ADC Offset for phase current B

% Max and min ADC counts for current sense offsets
inverter.CtSensOffsetMax = 2500; % Maximum permitted ADC counts for current sense offset
inverter.CtSensOffsetMin = 1500; % Minimum permitted ADC counts for current sense offset

%% Derive Characteristics
pmsm.N_base = mcb.getMotorBaseSpeed(pmsm,inverter); %rpm // Base speed of motor at given Vdc

% mcb_getCharacteristics(pmsm,inverter);

%% PU System details // Set base values for pu conversion

PU_System = mcb.getPUSystemParameters(pmsm,inverter);

%% Controller design // Get ballpark values!

PI_params = mcb.getPIControllerParameters(pmsm,inverter,PU_System,T_pwm,Ts,Ts_speed);

%Updating delays for simulation
PI_params.delay_Currents    = int32(Ts/Ts_simulink);
PI_params.delay_Speed       = int32(Ts_speed/Ts_simulink);

% mcb.getMotorControlAnalysis(pmsm,inverter,PU_System,PI_params,Ts,Ts_speed); 

%% Displaying model variables
disp(pmsm);
disp(inverter);
disp(target);
disp(PU_System);
