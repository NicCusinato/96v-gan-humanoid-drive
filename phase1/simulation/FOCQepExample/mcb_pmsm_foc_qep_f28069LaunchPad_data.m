% Model         :   PMSM Field Oriented Control with Quadrature Encoder
% Description   :   Configured for EPC91200 GaN Inverter + LAUNCHXL-F28069M + AKE80-8 KV30 Motor
% File name     :   mcb_pmsm_foc_qep_f28069LaunchPad_data.m

%% Simulation & PWM Parameters 

%% Set PWM Switching frequency
PWM_frequency 	= 30e3;             % Hz (30 kHz optimal for GaN & F28069M)
T_pwm           = 1/PWM_frequency;  % s  // PWM switching time period

%% Set Sample Times
Ts          	= T_pwm;            % sec // Sample time step for current controller (30 kHz)
Ts_simulink     = T_pwm/2;          % sec // Simulation time step for model simulation
Ts_motor        = T_pwm/2;          % sec // Simulation sample time
Ts_inverter     = T_pwm/2;          % sec // Simulation time step for average value inverter
Ts_speed        = 1e-3;             % sec // Sample time for speed controller (1 kHz standard)

%% Set data type for controller & code-gen
dataType = 'single';                % Floating point code-generation for F28069M (FPU32)

%% =========================================================================
%% System Parameters // Hardware parameters 
%% =========================================================================

% 1. Target Controller: LAUNCHXL-F28069M
target = mcb.getProcessorParameters('F28069M', PWM_frequency);
target.comport = 'COM7';            % LaunchPad virtual COM port (XDS100 Class USB Serial Port)

% 2. Motor: AKE80-8 KV30
pmsm = struct();
pmsm.p               = 21;          % Number of pole pairs (42 poles)
pmsm.Rs              = 0.435;       % Stator phase resistance (Ohms, phase-to-neutral)
pmsm.Ld              = 0.000495;    % d-axis inductance (H) -> 495 uH
pmsm.Lq              = 0.000495;    % q-axis inductance (H) -> 495 uH
pmsm.GearRatio       = 8;           % AKE80-8 reduction; plant model is motor-shaft only
pmsm.Kt              = 0.32;        % Motor-side torque constant (N*m/A), CubeMars data
pmsm.Ke              = 33.0;        % Published back-EMF constant (V/krpm)
% Motor Control Blockset uses T = 3/2*p*FluxPM*Iq.  Derive FluxPM from
% the published motor-side Kt so the torque and back-EMF models agree.
pmsm.FluxPM          = pmsm.Kt / (1.5 * pmsm.p); % 0.0101587 Wb
pmsm.I_rated         = 4.8;         % Published rated current (A peak)
pmsm.I_max           = 12.0;        % Published peak current (A peak)
% The PMSM block has no gearbox.  Convert the published output speeds to
% motor-shaft speeds for the electrical plant and PU reference generator.
pmsm.N_max           = 195 * pmsm.GearRatio; % Published no-load output speed -> motor shaft
pmsm.N_rated         = 150 * pmsm.GearRatio; % Published rated output speed -> motor shaft
pmsm.T_rated         = 1.5 * pmsm.p * pmsm.FluxPM * pmsm.I_rated; % ~1.536 N*m motor-side
pmsm.T_max           = 1.5 * pmsm.p * pmsm.FluxPM * pmsm.I_max;    % ~3.84 N*m motor-side
pmsm.Power           = pmsm.T_rated * (pmsm.N_rated * 2 * pi / 60); % Motor-side mechanical power
pmsm.QEPSlits        = 1024;        % AS5047P ABI Default: 1024 PPR (4096 CPR in 4x quadrature mode)
pmsm.PositionOffset  = 0.2850;         % Per-Unit encoder index offset [0.0 - 1.0] (Set via calibration)
pmsm.J               = 0.0005;      % Rotor + Hub inertia (kg*m^2) for 80mm outrunner motor
pmsm.B               = 0.0005;      % Viscous friction coefficient (N*m*s)
pmsm.Efficiency      = 90;          % %

% Low-pass filter coefficient used by the speed-feedback IIR block.
% 0.001 suppresses the QEP speed quantization seen at the 32 V operating
% point while retaining sub-second response in the closed-loop test.
LPFltCoeff           = 0.001;

% 3. Inverter: EPC91200 GaN Inverter + EPC9147B Interface
inverter = struct();
inverter.V_dc                = 32.0;    % Nominal DC bus voltage (30V - 35V range)
inverter.V_max               = 152.3;   % Full-scale ADC voltage (V) [Calibrated]
inverter.I_max               = 137.5;   % Peak measurable phase current (+/- 137.5 A)
inverter.I_trip              = 20.0;    % Overcurrent protection trip limit (A)

% Current Sensing (Allegro ACS37003LLUTR-050B3: 12 mV/A, 1.65V zero-current bias)
inverter.ISenseVoltPerAmp     = 0.012;   % Sensor sensitivity (V/A)
inverter.ISenseVref          = 1.65;    % Midpoint voltage reference (V)
inverter.ISenseMax           = 137.5;   % MCB PU peak-current base; ADC range is +/-137.5 A
inverter.ISenseSpan          = 275.0;   % Sensor full-scale span (3.3V / 0.012 V/A = 275 A pk-pk)
inverter.ISenseOffset        = 0.50;    % Zero-current ADC offset (1.65V / 3.3V = 0.50 pu -> 2048 counts)

% Voltage Sensing
inverter.VSenseMax           = 152.3;   % Full-scale voltage corresponding to 3.3V on ADC
inverter.VSenseOffset        = 0.0;     % DC voltage offset (0V = 0 counts)

% Board Parasitics & Deadtime
inverter.R_board             = 0.001;   % Inverter trace + switch resistance (Ohms)
inverter.DeadTime            = 0.05e-6; % GaN deadtime: 50 ns
inverter.ADCGain             = 1;       % Fixed analog gain
inverter.SPI_Gain_Setting    = 0x5000;  % SPI Gain setting word (dummy for simulation block compatibility)
inverter.invertingAmp        = 1;       % 1: Non-inverting amplifier, -1: Inverting amplifier

% Current Sense Initial ADC Offsets (1.65V zero-current bias on 12-bit ADC = 2048 counts)
inverter.CtSensAOffset       = 2048;    % Phase A current offset ADC counts
inverter.CtSensBOffset       = 2048;    % Phase B current offset ADC counts
inverter.CtSensCOffset       = 2048;    % Phase C current offset ADC counts

% Automatic ADC zero-current offset calibration at startup
inverter.ADCOffsetCalibEnable= 1;       % 1: Enabled, 0: Disabled
inverter.CtSensOffsetMax     = 2500;    % Max permitted ADC offset counts (2048 +/- tolerance)
inverter.CtSensOffsetMin     = 1500;    % Min permitted ADC offset counts

%% =========================================================================
%% Derive Characteristics & Per-Unit Base Values
%% =========================================================================
pmsm.N_base = mcb.getMotorBaseSpeed(pmsm, inverter); % rpm // Base speed of motor at given Vdc

PU_System = mcb.getPUSystemParameters(pmsm, inverter);

%% =========================================================================
%% Controller Design (PI Gains Calculation)
%% =========================================================================
PI_params = mcb.getPIControllerParameters(pmsm, inverter, PU_System, T_pwm, Ts, Ts_speed);

% The speed PI output is in current PU.  Scale its gains and saturation
% to the motor peak-current limit; +/-1 PU would otherwise command the
% 137.5 A ADC base into a 12 A motor.
Iq_ref_max_PU              = pmsm.I_max / PU_System.I_base;
SpeedLoopDampingScale      = 0.3;
PI_params.Kp_speed          = PI_params.Kp_speed * SpeedLoopDampingScale * Iq_ref_max_PU;
PI_params.Ki_speed          = PI_params.Ki_speed * SpeedLoopDampingScale * Iq_ref_max_PU;

% Updating delays for simulation
PI_params.delay_Currents    = int32(Ts/Ts_simulink);
PI_params.delay_Position    = int32(Ts/Ts_simulink);
PI_params.delay_Speed       = int32(Ts_speed/Ts_simulink);
if isfield(PI_params, 'delay_IIR')
    PI_params.delay_Speed1  = (PI_params.delay_IIR + 0.5*Ts)/Ts_speed;
end

%% Displaying model variables
fprintf('\n================== SYSTEM CONFIGURATION ==================\n');
fprintf('Target DSP   : TMS320F28069M (LAUNCHXL-F28069M)\n');
fprintf('Inverter     : EPC91200 GaN (152.3V Full Scale, 275A Span)\n');
fprintf('Motor        : AKE80-8 KV30 (p=21, Rs=0.435 Ohm, L=495 uH)\n');
fprintf('PWM Freq     : %.1f kHz (Deadtime: 50 ns)\n', PWM_frequency/1e3);
fprintf('Encoder PPR  : %d slits (%d counts/rev)\n', pmsm.QEPSlits, pmsm.QEPSlits*4);
fprintf('Base Speed   : %.1f RPM\n', pmsm.N_base);
fprintf('Current Kp/Ki: Kp = %.4f, Ki = %.4f\n', PI_params.Kp_i, PI_params.Ki_i);
fprintf('Speed Kp/Ki  : Kp = %.4f, Ki = %.4f\n', PI_params.Kp_speed, PI_params.Ki_speed);
fprintf('==========================================================\n\n');
