% Model         :   PMSM Field Oriented Control
% Description   :   Set Parameters for PMSM Field Oriented Control
% File name     :   mcb_pmsm_foc_f280049C_data.m

% Copyright 2021 - 2024 The MathWorks, Inc.

%% Set PWM Switching frequency
PWM_frequency 	= 20e3;             %Hz     // converter s/w freq
T_pwm           = 1/PWM_frequency;  %s      // PWM switching time period

%% Set Sample Times
Ts          	= T_pwm;            %sec    // Sample time for control system
Ts_simulink     = T_pwm/2;          %sec    // Simulation time step for model simulation
Ts_motor        = T_pwm/2;          %Sec    // Simulation time step for pmsm
Ts_inverter     = T_pwm/2;          %sec    // Simulation time step for inverter
Ts_speed        = 10*Ts;            %Sec    // Sample time for speed controller

%% Set data type for controller & code-gen
%Uncomment the required dataType and comment out the other
% dataType = fixdt(1,32,17);        % Fixed point code-generation
dataType = 'single';                % Floating point code-generation

%% System Parameters 
% Motor parameters
pmsm = mcb.getPMSMParameters('BLY171D');

switch position_decoder 
    case 'hall'
        pmsm.PositionOffset = 0.24; 
    case 'qep'
        pmsm.PositionOffset = 0.0664;
        pmsm.QEPSlits = 2000;
end

%% Target & Inverter Parameters
target = mcb.getProcessorParameters('F280049C',PWM_frequency);

inverter = mcb.getInverterParameters('BoostXL-DRV8305');
%% Callibration Section

% Enable automatic calibration of ADC offset for current measurement
inverter.ADCOffsetCalibEnable = 1;  % Enable: 1, Disable:0 

% If automatic ADC offset calibration is disabled, uncomment and update the 
% offset values below manually
% inverter.CtSensAOffset = 2295;      % ADC Offset for phase current A 
% inverter.CtSensBOffset = 2286;      % ADC Offset for phase current B

% Update ADC Gain for DRV8305
if pmsm.I_rated < 5
    inverter.ADCGain = 4;   % ADC Range = +- 4.825A wrt 0-4095 counts
    inverter.SPI_Gain_Setting = 0x502A;
    
elseif pmsm.I_rated < 7
    inverter.ADCGain = 2;   % ADC Range = +- 9.650A wrt 0-4095 counts
    inverter.SPI_Gain_Setting = 0x5015;

else     
    inverter.ADCGain = 1;   % ADC Range = +- 19.300A wrt 0-4095 counts       
    inverter.SPI_Gain_Setting = 0x5000;        
    
end

% Voltage output of inverter current sense circuit
inverter.ISenseVoltPerAmp = inverter.ISenseVoltPerAmp * inverter.ADCGain; 

% Update ISenseMax according to set ADC gain
inverter.ISenseMax = inverter.ISenseMax /inverter.ADCGain;

% Max and min ADC counts for current sense offsets
inverter.CtSensOffsetMax = 2500;    % Maximum permitted ADC counts for current sense offset
inverter.CtSensOffsetMin = 1500;    % Minimum permitted ADC counts for current sense offset

%% Derive Characteristics
pmsm.N_base = mcb.getMotorBaseSpeed(pmsm,inverter); %rpm // Base speed of motor at given Vdc
% Uncomment the line below to get motor charecteristics
% mcb.PMSMCharacteristics(pmsm,inverter);

%% PU System details // Set base values for pu conversion

PU_System = mcb.getPUSystemParameters(pmsm,inverter);

%% Controller design // Get ballpark values!

PI_params = mcb.getPIControllerParameters(pmsm,inverter,PU_System,T_pwm,Ts,Ts_speed);

%Updating delays for simulation
PI_params.delay_Currents    = int32(Ts/Ts_simulink);
PI_params.delay_Position    = int32(Ts/Ts_simulink);
PI_params.delay_Speed       = int32(Ts_speed/Ts_simulink);
PI_params.delay_Speed1      = (PI_params.delay_IIR + 0.5*Ts)/Ts_speed;

% mcb.getMotorControlAnalysis(pmsm,inverter,PU_System,PI_params,Ts,Ts_speed); 

%% Displaying model variables
disp(pmsm);
disp(inverter);
disp(target);
disp(PU_System);
