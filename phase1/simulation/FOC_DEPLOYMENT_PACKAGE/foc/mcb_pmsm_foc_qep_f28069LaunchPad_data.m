% Model         :   PMSM Field Oriented Control with Quadrature Encoder
% Description   :   Configured for EPC91200 GaN Inverter + LAUNCHXL-F28069M + AKE80-8 KV30 Motor
% File name     :   mcb_pmsm_foc_qep_f28069LaunchPad_data.m
% Notes         :   Master parameter and dynamic PI gain synthesis script.
%                   Automatically scales all per-unit bases, base speeds, and PI gains
%                   when V_dc_supply is changed.

%% 0. Master Operating Bus Voltage Configuration
% Set this single variable to whatever voltage your power supply is set to.
% Examples: 40.0 (bench test), 48.0 (nominal), 96.0 (full humanoid rating)
V_dc_supply     = 40.0;             % V // DC Link Voltage of the Inverter

%% 1. Simulation & PWM Parameters 
PWM_frequency 	= 20e3;             % Hz (20 kHz PWM for GaN & F28069M, 2250 ticks @ 90 MHz)
T_pwm           = 1/PWM_frequency;  % s  // PWM switching time period

%% 2. Set Sample Times
Ts          	= T_pwm;            % sec // Sample time step for current controller (20 kHz / 50 us)
Ts_simulink     = T_pwm/2;          % sec // Simulation time step for model simulation
Ts_motor        = T_pwm/2;          % sec // Simulation sample time
Ts_inverter     = T_pwm/2;          % sec // Simulation time step for average value inverter
Ts_speed        = 1e-3;             % sec // Sample time for speed controller (1 kHz standard)

%% 3. Set Data Type for Controller & Code-Gen
dataType        = 'single';         % Floating point code-generation for F28069M (FPU32)

%% =========================================================================
%% 4. System Parameters // Hardware Parameters 
%% =========================================================================

% 4.1 Target Controller: LAUNCHXL-F28069M
target = mcb.getProcessorParameters('F28069M', PWM_frequency);
target.comport = 'COM7';            % LaunchPad virtual COM port (XDS100 Class USB Serial Port)

% 4.2 Motor: CubeMars AKE80-8 KV30
pmsm = struct();
pmsm.p               = 21;          % Number of pole pairs (42 poles)
pmsm.Rs              = 0.435;       % Stator phase resistance (Ohms, phase-to-neutral)
pmsm.Ld              = 0.000495;    % d-axis inductance (H) -> 495 uH
pmsm.Lq              = 0.000495;    % q-axis inductance (H) -> 495 uH
pmsm.GearRatio       = 8;           % AKE80-8 reduction; plant model is motor-shaft only
pmsm.Kt              = 0.32;        % Motor-side torque constant (N*m/A), CubeMars data
pmsm.Ke              = 33.0;        % Published back-EMF constant (V/krpm)
% Derive FluxPM from Kt: T = 1.5 * p * FluxPM * Iq
pmsm.FluxPM          = pmsm.Kt / (1.5 * pmsm.p); % 0.0101587 Wb
pmsm.I_rated         = 1.0;         % Published rated current is 4.8A, clamped to 1.0A for bench supply
pmsm.I_max           = 1.5;         % Published peak current is 12.0A, clamped to 1.5A for bench supply
pmsm.N_max           = 195 * pmsm.GearRatio; % Published no-load output speed -> motor shaft
pmsm.N_rated         = 150 * pmsm.GearRatio; % Published rated output speed -> motor shaft
pmsm.T_rated         = 1.5 * pmsm.p * pmsm.FluxPM * pmsm.I_rated; % ~1.536 N*m motor-side
pmsm.T_max           = 1.5 * pmsm.p * pmsm.FluxPM * pmsm.I_max;    % ~3.84 N*m motor-side
pmsm.Power           = pmsm.T_rated * (pmsm.N_rated * 2 * pi / 60); % Motor-side mechanical power
pmsm.QEPSlits        = 1024;        % AS5047P ABI Default: 1024 PPR (4096 CPR in 4x quadrature mode)
pmsm.PositionOffset  = 0.025635;    % Calibrated with AS5047P SPI & QEP (0.025635 PU = 9.23 deg mech)
pmsm.J               = 0.0005;      % Rotor + Hub inertia (kg*m^2) for 80mm outrunner motor
pmsm.B               = 0.0005;      % Viscous friction coefficient (N*m*s)
pmsm.Efficiency      = 90;          % %

% Low-pass filter coefficient for speed feedback
LPFltCoeff           = 0.001;

% 4.3 Inverter: EPC91200 GaN Inverter + EPC9147B Interface Board
inverter = struct();
inverter.V_dc                = V_dc_supply; % Dynamic bus voltage from master config
inverter.V_max               = 152.3;       % Full-scale ADC voltage (V) [Calibrated]
inverter.I_max               = 137.5;       % Peak measurable phase current (+/- 137.5 A)
inverter.I_trip              = 1.5;         % Overcurrent protection trip limit (A) clamped to bench supply limit

% Current Sensing (Allegro ACS37003LLUTR-050B3: 12 mV/A, 1.65V zero-current bias)
inverter.ISenseVoltPerAmp     = 0.012;       % Sensor sensitivity (V/A)
inverter.ISenseVref          = 3.30;        % Full-scale current sense amplifier rail / ADC Vref (V)
inverter.ISenseMax           = 137.5;       % MCB PU peak-current base; ADC range is +/-137.5 A
inverter.ISenseSpan          = 275.0;       % Sensor full-scale span (3.3V / 0.012 V/A = 275 A pk-pk)
inverter.ISenseOffset        = 0.50;        % Zero-current ADC offset (1.65V / 3.3V = 0.50 pu -> 2048 counts)

% Voltage Sensing
inverter.VSenseMax           = 152.3;       % Full-scale voltage corresponding to 3.3V on ADC
inverter.VSenseOffset        = 0.0;         % DC voltage offset (0V = 0 counts)

% Board Parasitics & Deadtime
inverter.R_board             = 0.001;       % Inverter trace + switch resistance (Ohms)
inverter.DeadTime            = 0.05e-6;     % GaN deadtime: 50 ns (5 clock cycles @ 90 MHz)
inverter.ADCGain             = 1;           % Fixed analog gain
inverter.SPI_Gain_Setting    = 0x5000;      % SPI Gain setting word (dummy for simulation block compatibility)
inverter.invertingAmp        = 1;           % 1: Non-inverting amplifier, -1: Inverting amplifier

% Current Sense Initial ADC Offsets (Bench Measured)
inverter.CtSensAOffset       = 2058;        % Phase A current offset ADC counts (1.658V)
inverter.CtSensBOffset       = 2057;        % Phase B current offset ADC counts (1.657V)
inverter.CtSensCOffset       = 2048;        % Phase C current offset ADC counts

% Automatic ADC zero-current offset calibration at startup
inverter.ADCOffsetCalibEnable= 1;           % 1: Enabled, 0: Disabled
inverter.CtSensOffsetMax     = 2500;        % Max permitted ADC offset counts (2048 +/- tolerance)
inverter.CtSensOffsetMin     = 1500;        % Min permitted ADC offset counts

%% =========================================================================
%% 5. Derive Characteristics & Per-Unit Base Values (Dynamic with V_dc_supply)
%% =========================================================================
pmsm.N_base = mcb.getMotorBaseSpeed(pmsm, inverter); % rpm // Base mechanical speed dynamically calculated
PU_System   = mcb.getPUSystemParameters(pmsm, inverter);

%% =========================================================================
%% 6. Controller Design (Bench-Safe PI Gain Synthesis: 15*Ts Tuning)
%% =========================================================================
% Uses 15*Ts response time for current loop on low-current bench supply to prevent limit-cycle oscillation
PI_params = mcb.getPIControllerParameters(pmsm, inverter, PU_System, T_pwm, 15*Ts, Ts_speed);

% Scale speed PI output to motor peak-current limit (1.5 A clamp on bench)
Iq_ref_max_PU              = pmsm.I_max / PU_System.I_base;
PI_params.Kp_speed          = 0.0119;
PI_params.Ki_speed          = 0.0707;

% Updating delays for simulation
PI_params.delay_Currents    = int32(Ts/Ts_simulink);
PI_params.delay_Position    = int32(Ts/Ts_simulink);
PI_params.delay_Speed       = int32(Ts_speed/Ts_simulink);
if isfield(PI_params, 'delay_IIR')
    PI_params.delay_Speed1  = (PI_params.delay_IIR + 0.5*Ts)/Ts_speed;
end

%% =========================================================================
%% 7. Display Configured Values in MATLAB Command Window
%% =========================================================================
fprintf('\n================== SYSTEM CONFIGURATION ==================\n');
fprintf('Target DSP   : TMS320F28069M (LAUNCHXL-F28069M)\n');
fprintf('Inverter     : EPC91200 GaN (DC Bus: %.1f V, Deadtime: 50 ns)\n', inverter.V_dc);
fprintf('Motor        : AKE80-8 KV30 (p=21, Rs=%.3f Ohm, L=%d uH)\n', pmsm.Rs, round(pmsm.Ld*1e6));
fprintf('PWM Freq     : %.1f kHz (Period: %d ticks @ 90 MHz)\n', PWM_frequency/1e3, target.PWM_Counter_Period);
fprintf('Encoder PPR  : %d slits (%d counts/rev, Offset: %.4f PU)\n', pmsm.QEPSlits, pmsm.QEPSlits*4, pmsm.PositionOffset);
fprintf('Base Speed   : %.1f RPM (at %.1f V DC Bus)\n', pmsm.N_base, inverter.V_dc);
fprintf('Current Kp/Ki: Kp = %.4f, Ki = %.4f (5*Ts Bench-Safe Tuning)\n', PI_params.Kp_i, PI_params.Ki_i);
fprintf('Speed Kp/Ki  : Kp = %.4f, Ki = %.4f\n', PI_params.Kp_speed, PI_params.Ki_speed);
fprintf('==========================================================\n\n');
