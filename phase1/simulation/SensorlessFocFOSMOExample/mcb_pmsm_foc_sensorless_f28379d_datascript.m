% Model         :   PMSM Field Oriented Control
% Description   :   Set Parameters for PMSM Field Oriented Control
% File name     :   mcb_pmsm_foc_sensorless_f28379d_datascript.m

% Copyright 2021-2024 The MathWorks, Inc.

%% Simulation Parameters

%% Set PWM Switching frequency
PWM_frequency   = 20e3;             %Hz     // Converter s/w freq
T_pwm           = 1/PWM_frequency;  %s      // PWM switching time period

%% Set Sample Times
Ts          	= T_pwm;            %sec    // Sample time for control system
Ts_simulink     = T_pwm/2;          %sec    // Simulation time step for model simulation
Ts_motor        = T_pwm/2;          %sec    // Simulation sample time for pmsm
Ts_inverter     = T_pwm/2;          %sec    // Simulation time step for inverter
Ts_speed        = 10*Ts;            %sec    // Sample time for speed controller

%% Set data type for controller & code-gen
% dataType = fixdt(1,32,24);    % Fixed point code-generation  
dataType = 'single';

%% System Parameters 
% Set motor parameters
% pmsm = mcb.getPMSMParameters('BLY172S');
  pmsm = mcb.getPMSMParameters('Teknic2310P');

%% Target & Inverter Parameters
target = mcb.getProcessorParameters('F28379D',PWM_frequency);
target.comport = '<Select a port...>';
% target.comport = 'COM9';       % Uncomment and update the appropriate serial port

% Set inverter parameters
%inverter = mcb.getInverterParameters('BoostXL-DRV8305');
inverter = mcb.getInverterParameters('BoostXL-DRV8305');

%% Calibration section 
% Enable automatic calibration of ADC offset for current measurement
inverter.ADCOffsetCalibEnable = 1;  % Enable: 1, Disable: 0

% If automatic ADC offset calibration is disabled, uncomment and update the 
% offset values below manually
% inverter.CtSensAOffset = 2295;      % ADC Offset for phase current A 
% inverter.CtSensBOffset = 2286;      % ADC Offset for phase current B

% Update inverter.ISenseMax based for the chosen motor and target
inverter = mcb.updateInverterParameters(pmsm,inverter,target);

% Max and min ADC counts for current sense offsets
inverter.CtSensOffsetMax = 2500; % Maximum permitted ADC counts for current sense offset
inverter.CtSensOffsetMin = 1500; % Minimum permitted ADC counts for current sense offset

%% Derive Characteristics
pmsm.N_base = mcb.getMotorBaseSpeed(pmsm,inverter); % rpm // Base speed of motor at given Vdc
% mcb_getCharacteristics(pmsm,inverter);       % Uncomment for motor characteristics

%% PU System details // Set base values for pu conversion
PU_System = mcb.getPUSystemParameters(pmsm,inverter);


%% Set Acceleration
acceleration = 10000/PU_System.N_base;                  %  P.U/Sec // Maximum allowable acceleraton

%% Open loop reference values
% T_Ref_openLoop          = 1;                    % Sec // Time for open-loop start-up
Speed_openLoop_PU       = 0.1;                  % PU  // Per-Unit speed referene for open-loop start-up
% Vd_Ref_openLoop_PU      = Speed_openLoop_PU;    % Use 1.2x for Dyno setup and 1x for others

%% State-machine constants
one_sec_tick = uint16(1/Ts_speed);   % one sec delay
two_sec_tick = uint16(2/Ts_speed);   % two sec delay
RAMP_STEP_SIZE = 0.001;

MAX_OL_POS_SPD = 0.2; % speed limit to switch from open-loop to closed-loop
MAX_OL_NEG_SPD = -0.2;
MIN_CL_POS_SPD = 0.15; % speed limit to switch from closed-loop to open-loop
MIN_CL_NEG_SPD = -0.15;

MAX_OL_VD_LIMIT = 0.6; % Max Vd for open-loop run
MIN_OL_VD_LIMIT = 0.15; % Min Vd for open-loop run.
                        % Due to inertia if motor does not start initially,
                        % increase the Vd minimum limit


%% Controller design // Get ballpark values!
PI_params = mcb.getPIControllerParameters(pmsm,inverter,PU_System,T_pwm,2*Ts,2*Ts_speed);

% Set SMO parameters
smo = mcb.computeSMOParameters(pmsm,Ts,PU_System);

%Updating delays for simulation
PI_params.delay_Currents    = 1; % No of samples delayed for current sensing

% %Uncomment for frequency domain analysis
% mcb.getMotorControlAnalysis(pmsm,inverter,PU_System,PI_params,Ts,Ts_speed);

%% Displaying model variables
disp(pmsm);
disp(inverter);
disp(target);
disp(PU_System);
