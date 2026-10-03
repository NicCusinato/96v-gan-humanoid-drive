%% Model initialization callback for the models:
%   > speed_control_algorithm.slx
%   > current_control_algorithm.slx
%   > foc_qep_sim.slx
% Hardware: TI TMS320F28069M LaunchPad + EPC9147B/EPC91200 GaN Inverter
% Motor   : CubeMars AKE80-8 KV30 (21 Pole Pairs, 42 Poles)
% Sensor  : AS5047P ABI mode (1024 slits -> 4096 counts/rev on eQEP1)

%% 1. Set PWM Switching frequency & Sample Times
PWM_frequency   = 20e3;             % Hz        // Converter switching frequency (20 kHz)
T_pwm           = 1/PWM_frequency;  % s         // PWM switching period (50 us)

Ts              = T_pwm;            % sec       // Sample time for current controller (50 us)
Ts_simulink     = T_pwm/2;          % sec       // Simulation time step
Ts_motor        = T_pwm/2;          % sec       // Motor simulation time step
Ts_inverter     = T_pwm/2;          % sec       // Inverter simulation time step
Ts_speed        = 10*Ts;            % sec       // Sample time for speed controller (500 us / 2 kHz)

%% 2. Set Data Type for Controller & Code-Gen
dataType        = 'single';         % Floating-point code generation (TI C2000 FPU32)

%% 3. Target Parameters (TI TMS320F28069M LaunchPad)
target.model              = 'LAUNCHXL-F28069M';
target.CPU_frequency      = 90e6;                 % Hz    // 90 MHz SYSCLK
target.PWM_frequency      = PWM_frequency;        % Hz    // 20 kHz PWM
target.PWM_Counter_Period = round(target.CPU_frequency / (2 * target.PWM_frequency)); % 2250 counts up-down
target.ADC_Vref           = 3.3;                  % V     // 3.3V ADC reference voltage
target.ADC_MaxCount       = 4095;                 %       // Max count for 12-bit ADC
target.SCI_baud_rate      = 1.25e6;               % Hz    // Baud rate for serial communication
target.comport            = 'COM7';

%% 4. Motor Parameters (CubeMars AKE80-8 KV30)
pmsm.model          = 'CubeMars-AKE80-8-KV30';
pmsm.sn             = '001';
pmsm.p              = 21;                   % 21 Pole Pairs (42 poles)
pmsm.Rs             = 0.435;                % Ohms  // Phase resistance (0.870 ohm line-to-line / 2)
pmsm.Ld             = 495e-6;               % H     // Direct-axis inductance (495 uH)
pmsm.Lq             = 495e-6;               % H     // Quadrature-axis inductance (495 uH)
pmsm.J              = 1.47175e-4;           % kg*m^2// Rotor inertia
pmsm.B              = 1e-5;                 % N*m*s // Viscous damping
pmsm.FluxPM         = 0.01016;              % Wb    // Permanent magnet flux linkage
pmsm.Kt             = 0.32;                 % N*m/A // Torque constant
pmsm.Ke             = (pmsm.FluxPM * pmsm.p * 1000 * 2 * pi / 60) * sqrt(3); % Line-line Vpk / krpm
pmsm.I_rated        = 4.8;                  % Amps  // Rated continuous RMS current
pmsm.I_max          = 12.0;                 % Amps  // Maximum peak current
pmsm.N_max          = 2000;                 % RPM   // Maximum allowable mechanical speed
pmsm.QEPSlits       = 1024;                 % Slits // 1024 lines -> 4096 counts/rev in 4x mode
pmsm.PositionOffset = 0.025635;             % PU    // Calibrated on bench with AS5047P SPI & QEP (0.025635 PU = 9.23 deg mech)
pmsm.T_rated        = pmsm.Kt * pmsm.I_rated;

%% 5. Inverter Details (EPC9147B + EPC91200 3-Phase GaN Inverter)
inverter.model            = 'EPC91200_EPC2305';
inverter.sn               = 'INV_001';
inverter.V_dc             = 48.0;                 % V     // DC Link Voltage of the Inverter (Bench Supply)
inverter.I_trip           = 25.0;                 % Amps  // Overcurrent trip threshold
inverter.DeadTime         = 0.05e-6;              % sec   // 50 ns deadtime for EPC2305 GaN FETs
inverter.Rds_on           = 2.2e-3;               % Ohms  // EPC2305 GaN typical RDS(on)
inverter.Rshunt           = 0.005;                % Ohms  // Current shunt resistance
inverter.ISenseVoltPerAmp = 0.012;                % V/A   // Allegro ACS37003 Hall-effect current sensor sensitivity
inverter.ISenseVref       = 3.3;                  % V     // Voltage ref of current sense circuit
inverter.ISenseMax        = inverter.ISenseVref / (2 * inverter.ISenseVoltPerAmp); % 137.5 Amps peak
inverter.CtSensAOffset    = 2058;                 % Counts// Calibrated ADC Offset for Phase A (1.658V)
inverter.CtSensBOffset    = 2057;                 % Counts// Calibrated ADC Offset for Phase B (1.657V)
inverter.CtSensCOffset    = 2048;
inverter.ADCOffsetCalibEnable = 1;
inverter.ADCGain          = 1;                    % ADC Gain factor
inverter.EnableLogic      = 1;                    % Active High Gate Driver Enable (GPIO-52)
inverter.invertingAmp     = -1;                   % Current polarity compensation
inverter.R_board          = inverter.Rds_on + inverter.Rshunt/3;

%% 6. Derive Characteristics & Base Values (Per-Unit System)
pmsm.N_base = mcb.getMotorBaseSpeed(pmsm, inverter); % RPM // Base mechanical speed @ 48V Vdc
PU_System   = mcb.getPUSystemParameters(pmsm, inverter);

%% 7. Controller Design (Automated PI Gain Synthesis)
PI_params = mcb.getPIControllerParameters(pmsm, inverter, PU_System, T_pwm, 5*Ts, Ts_speed);

% Updating delays for simulation
PI_params.delay_Currents = int32(Ts/Ts_simulink);
PI_params.delay_Position = int32(Ts/Ts_simulink);
PI_params.delay_Speed    = int32(Ts_speed/Ts_simulink);
PI_params.delay_Speed1   = (PI_params.delay_IIR + 0.5*Ts)/Ts_speed;

%% 8. Display Configured Values
disp('=== Hardware Configuration Loaded Successfully ===');
fprintf('  MCU       : %s (SYSCLK: %.1f MHz, PWM: %.1f kHz, TBPRD: %d)\n', ...
    target.model, target.CPU_frequency/1e6, target.PWM_frequency/1e3, target.PWM_Counter_Period);
fprintf('  Motor     : %s (Poles: %d, Rs: %.3f Ohm, L: %d uH)\n', ...
    pmsm.model, 2*pmsm.p, pmsm.Rs, round(pmsm.Ld*1e6));
fprintf('  Inverter  : %s (Vdc: %.1f V, DeadTime: %.1f ns, ISense: %.3f V/A)\n', ...
    inverter.model, inverter.V_dc, inverter.DeadTime*1e9, inverter.ISenseVoltPerAmp);
fprintf('  PU System : V_base=%.2f V, I_base=%.2f A, N_base=%d RPM\n', ...
    PU_System.V_base, PU_System.I_base, PU_System.N_base);
fprintf('  PI Gains  : Kp_current=%.4f, Ki_current=%.2f | Kp_speed=%.4f, Ki_speed=%.4f\n', ...
    PI_params.Kp_i, PI_params.Ki_i, PI_params.Kp_speed, PI_params.Ki_speed);
disp('==================================================');
