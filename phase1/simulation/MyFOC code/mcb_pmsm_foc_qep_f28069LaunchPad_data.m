% Model         :   PMSM Field Oriented Control with Quadrature Encoder
% Description   :   Configured for EPC91200 GaN Inverter + LAUNCHXL-F28069M + AKE80-8 KV30 Motor
% File name     :   mcb_pmsm_foc_qep_f28069LaunchPad_data.m

%% Set PWM Switching frequency
PWM_frequency   = 20e3;             % Hz (20 kHz: 2250 counts @ 90 MHz up-down)
T_pwm           = 1/PWM_frequency;  % s

%% Set Sample Times
Ts              = T_pwm;            % sec // Sample time step for current controller (20 kHz)
Ts_simulink     = T_pwm/2;
Ts_motor        = T_pwm/2;
Ts_inverter     = T_pwm/2;
Ts_speed        = 1e-3;             % sec // Sample time for speed controller (1 kHz)
dataType        = 'single';

%% Target Controller: LAUNCHXL-F28069M
target = mcb.getProcessorParameters('F28069M', PWM_frequency);
target.comport = 'COM7';
target.SCI_baud_rate = 115200;

%% Motor: AKE80-8 KV30
pmsm = struct();
pmsm.p               = 21;          % Pole pairs
pmsm.Rs              = 0.435;       % Stator resistance
pmsm.Ld              = 0.000495;    % d-inductance (H)
pmsm.Lq              = 0.000495;    % q-inductance (H)
pmsm.Ke              = 19.24;
pmsm.FluxPM          = 0.002785;    % PM flux (Wb)
pmsm.I_rated         = 10.0;        % Rated phase current (A)
pmsm.I_max           = 3.0;         % Safe bench current limit
pmsm.N_max           = 960;
pmsm.N_rated         = 960;
pmsm.T_rated         = 1.5 * pmsm.p * pmsm.FluxPM * pmsm.I_rated;
pmsm.T_max           = pmsm.T_rated;
pmsm.Power           = pmsm.T_rated * (pmsm.N_rated * 2 * pi / 60);
pmsm.QEPSlits        = 1024;
pmsm.PositionOffset  = 0.2850;      % Calibrated PU offset
pmsm.J               = 0.0005;
pmsm.B               = 0.0005;
pmsm.Efficiency      = 90;
LPFltCoeff           = 0.02;

%% Inverter: EPC91200 GaN on BoosterPack Site 2
inverter = struct();
inverter.V_dc                = 32.0;
inverter.V_max               = 152.3;
inverter.I_max               = 137.5;
inverter.I_trip              = 10.0;
inverter.ISenseVoltPerAmp     = 0.012;
inverter.ISenseVref          = 1.65;
inverter.ISenseMax           = 275.0;
inverter.ISenseOffset        = 0.50;
inverter.VSenseMax           = 152.3;
inverter.VSenseOffset        = 0.0;
inverter.R_board             = 0.001;
inverter.DeadTime            = 0.05e-6;
inverter.ADCGain             = 1;
inverter.SPI_Gain_Setting    = 0x5000;
inverter.invertingAmp        = 1;
inverter.CtSensAOffset       = 2048;
inverter.CtSensBOffset       = 2048;
inverter.CtSensCOffset       = 2048;
inverter.ADCOffsetCalibEnable= 1;
inverter.CtSensOffsetMax     = 2500;
inverter.CtSensOffsetMin     = 1500;

%% Characteristics & Base Values
pmsm.N_base = mcb.getMotorBaseSpeed(pmsm, inverter);
PU_System = mcb.getPUSystemParameters(pmsm, inverter);
PI_params = mcb.getPIControllerParameters(pmsm, inverter, PU_System, T_pwm, Ts, Ts_speed);
PI_params.Kp_speed = PI_params.Kp_speed * 0.3;
PI_params.Ki_speed = PI_params.Ki_speed * 0.3;
PI_params.delay_Currents = int32(Ts/Ts_simulink);
PI_params.delay_Position = int32(Ts/Ts_simulink);
PI_params.delay_Speed    = int32(Ts_speed/Ts_simulink);
if isfield(PI_params, 'delay_IIR'), PI_params.delay_Speed1 = (PI_params.delay_IIR + 0.5*Ts)/Ts_speed; end
