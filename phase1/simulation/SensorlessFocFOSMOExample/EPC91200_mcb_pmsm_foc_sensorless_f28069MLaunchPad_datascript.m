% Model         :   PMSM Field Oriented Control
% Description   :   Set Parameters for PMSM Field Oriented Control
% File name     :   mcb_pmsm_foc_sensorless_f28069MLaunchPad_datascript.m

% Copyright 2021-2024 The MathWorks, Inc.

%% Simulation Parameters

%% Set PWM Switching frequency
PWM_frequency= 20e3;    %Hz          // converter s/w freq
T_pwm           = 1/PWM_frequency;  %s  // PWM switching time period

%% Set Sample Times
Ts          	= T_pwm;        %sec        // Sample time for control system
Ts_simulink     = T_pwm/2;      %sec        // Simulation time step for model simulation
Ts_motor        = T_pwm/2;      %Sec        // Simulation sample time for pmsm
Ts_inverter     = T_pwm/2;      %sec        // Simulation time step for inverter
Ts_speed        = 10*Ts;            %sec    // Sample time for speed controller

%% Set data type for controller & code-gen
% dataType = fixdt(1,32,24);  % Fixed point code-generation  
dataType = 'single';

%% System Parameters // Hardware parameters
% Set motor parameters
% pmsm = mcb.getPMSMParameters('Teknic2310P');
pmsm = mcb.getPMSMParameters('Teknic2310P'); % Load default struct to satisfy mathworks

% Override with custom AKE80 parameters
pmsm.p = 21;                    % Pole pairs
pmsm.Rs = 0.435;                % Stator resistance (Ohms)
pmsm.Ld = 0.000495;             % d-axis inductance (H)
pmsm.Lq = 0.000495;             % q-axis inductance (H)
pmsm.J = 0.001;                 % Realistic inertia 
pmsm.B = 0.002;                 % Friction coefficient
pmsm.I_rated = 4.8;             % Rated current (A)
pmsm.Ke = 1000 / 30;            % Back-EMF constant in [Vpk_LL/krpm] (Derived from 30 KV)
pmsm.FluxPM = (pmsm.Ke * 60) / (1000 * sqrt(3) * 2 * pi * pmsm.p);
pmsm.N_max = 1440;              % Max theoretical speed at 48V (30 KV * 48V = 1440 RPM)
pmsm.T_rated = 0.5;             % 0.5 Nm

% Set inverter parameters
% inverter = mcb.getInverterParameters('BoostXL-DRV8305');
inverter = mcb.getInverterParameters('BoostXL-DRV8305');

% Override with custom EPC91200 parameters
inverter.model = 'EPC91200';
inverter.V_dc = 30;             
inverter.I_max = 12;            
inverter.ISenseVoltPerAmp = 0.012; 
inverter.V_base = 153.3;        
inverter.ISenseVref = 3.3;      
inverter.invertingAmp = 1;      
inverter.R_board = 0;           
inverter.ISenseMax = 15; % Set manually to 15A for AKE80

% Set target hardware parameters
target = mcb.getProcessorParameters('F28069M',PWM_frequency);
target.comport = '<Select a port...>';
target.comport = 'COM7';       % Uncomment and update the appropriate serial port

%% Calibration section // Uncomment and update relevant parameters

% %Update ADC offsets with manually calibrated values below
% inverter.CtSensAOffset = 2087;
% inverter.CtSensBOffset = 2082;

% %Update ADC offsets with auto-calibrate feature
inverter.ADCOffsetCalibEnable = 1; % Enable: 1, Disable: 0

% -------------------------------------------------------------------------
% VERY IMPORTANT EPC91200 MODIFICATION!
% The DRV8305 has a Programmable Gain Amplifier (PGA) via SPI.
% The EPC91200 GaN board DOES NOT! It is a fixed analog hall sensor.
% If we allow the script below to multiply our ISenseVoltPerAmp by 4, 
% the math will be completely wrong and the physical motor will draw 4x current!
% I have commented out this DRV8305-specific scaling.
% -------------------------------------------------------------------------
% % Update ADC Gain for DRV8305
% if pmsm.I_rated < 5
%     inverter.ADCGain = 4;   % ADC Range = +- 4.825A wrt 0-4095 counts
%     inverter.SPI_Gain_Setting = 0x502A;
%     
% elseif pmsm.I_rated < 7
%     inverter.ADCGain = 2;   % ADC Range = +- 9.650A wrt 0-4095 counts
%     inverter.SPI_Gain_Setting = 0x5015;
% 
% else     
%     inverter.ADCGain = 1;   % ADC Range = +- 19.300A wrt 0-4095 counts       
%     inverter.SPI_Gain_Setting = 0x5000;        
%     
% end
% 
% % Voltage output of inverter current sense circuit
% inverter.ISenseVoltPerAmp = inverter.ISenseVoltPerAmp * inverter.ADCGain; 
% 
% % Update ISenseMax according to set ADC gain
% inverter.ISenseMax = inverter.ISenseMax /inverter.ADCGain;
% -------------------------------------------------------------------------

inverter.SPI_Gain_Setting = 0x5000; % Leave a dummy value to satisfy the DRV SPI block

% Max and min ADC counts for current sense offsets
inverter.CtSensOffsetMax = 2500; % Maximum permitted ADC counts for current sense offset
inverter.CtSensOffsetMin = 1500; % Minimum permitted ADC counts for current sense offset

%% Derive Characteristics
pmsm.N_base = mcb.getMotorBaseSpeed(pmsm,inverter); %rpm // Base speed of motor at given Vdc
% mcb_getCharacteristics(pmsm,inverter);

%% PU System details // Set base values for pu conversion
PU_System = mcb.getPUSystemParameters(pmsm,inverter);

%% Set Acceleration
acceleration = 10000/PU_System.N_base;                  % P.U/Sec // Maximum allowable acceleraton

%% Open loop reference values
T_Ref_openLoop          = 1;                    % Sec // Time for open-loop start-up
Speed_openLoop_PU       = 0.10;                 % PU  // Per-Unit speed reference for open-loop start-up (10% base speed)
% Vd_Ref_openLoop_PU      = Speed_openLoop_PU;    % Use 1.2x for Dyno setup and 1x for others


%% State-machine constants
one_sec_tick = uint16(1/Ts_speed);   % one sec delay
two_sec_tick = uint16(2/Ts_speed);   % two sec delay
RAMP_STEP_SIZE = 0.001;

MAX_OL_POS_SPD = 0.10; % speed limit to switch from open-loop to closed-loop (10% base speed)
MAX_OL_NEG_SPD = -0.10;
MIN_CL_POS_SPD = 0.05; % speed limit to switch from closed-loop to open-loop
MIN_CL_NEG_SPD = -0.05;

MAX_OL_VD_LIMIT = 0.35; % Max Vd for open-loop run (~35% of Vdc)
MIN_OL_VD_LIMIT = 0.15; % Min Vd for open-loop run (~15% of Vdc)
                        % Provides sufficient torque/current to overcome rotor inertia and lock observer

%% Controller design // Get ballpark values!
PI_params = mcb.getPIControllerParameters(pmsm,inverter,PU_System,T_pwm,2*Ts,2*Ts_speed);

% Detune the current controllers!
PI_params.Kp_id = PI_params.Kp_id * 0.25; % Quarter the d-axis gain to ensure stability
PI_params.Kp_i  = PI_params.Kp_i  * 0.25; % Quarter the q-axis gain to ensure stability

% Soften the speed controller to ignore raw derivative noise
PI_params.Kp_speed = 0.01; 
PI_params.Ki_speed = 0.5;  

% CLAMP the speed controller output so it never exceeds your 4.8A rated limit!
PI_params.iq_max = pmsm.I_rated / PU_System.I_base;
PI_params.iq_min = -PI_params.iq_max;

% Set SMO parameters
smo = mcb.computeSMOParameters(pmsm,Ts,PU_System);

%Updating delays for simulation
PI_params.delay_Currents    = 1; %No of samples delayed for current sensing

% %Uncomment for frequency domain analysis
% mcb.getMotorControlAnalysis(pmsm,inverter,PU_System,PI_params,Ts,Ts_speed);

%% Displaying model variables
disp(pmsm);
disp(inverter);
disp(target);
disp(PU_System);
