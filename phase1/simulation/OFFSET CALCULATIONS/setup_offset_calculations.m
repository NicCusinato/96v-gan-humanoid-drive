%% setup_offset_calculations.m
% =========================================================================
% Master Setup Script for Offset Calculations & Hardware Verification
% Project : 96V GaN Humanoid Drive (Phase 1)
% Hardware:
%   - MCU      : TI TMS320F28069M LaunchPad (LAUNCHXL-F28069M, 90 MHz FPU32)
%   - Inverter : EPC9147B Interface Board + EPC91200 GaN Inverter
%   - Motor    : CubeMars AKE80-8 KV30 (21 Pole Pairs, 42 Poles)
%   - Sensor   : AS5047P ABI mode (1024 slits -> 4096 counts/rev on eQEP1)
%   - Site     : BoosterPack Site 2 (J5, J6, J7, J8)
%   - PWM      : ePWM4, ePWM5, ePWM6 (20 kHz, 50 ns deadtime)
%   - ADC      : Phase U (ADCINA3), Phase V (ADCINB3), Trigger ePWM4 ADCSOCA
%   - Gate En  : GPIO-52 (Active HIGH)
%   - Serial   : COM7 (GPIO-28 RX, GPIO-29 TX)
% =========================================================================

clear all; clc;
fprintf('=================================================================\n');
fprintf('  96V GaN Humanoid Drive: Offset Calculations Workspace Setup\n');
fprintf('=================================================================\n\n');

%% 1. Controller & PWM Timing
PWM_frequency = 20000;                 % [Hz] 20 kHz PWM
T_pwm         = 1 / PWM_frequency;     % [s]  50 µs
Ts            = T_pwm;                 % [s]  Sample time for controller ISR
dataType      = 'single';

target = mcb.getProcessorParameters('F28069M', PWM_frequency);
target.comport = 'COM7';

%% 2. Inverter Parameters (EPC9147B + EPC91200 GaN)
inverter = struct();
inverter.EnableLogic    = 1;           % Active high gate enable (GPIO-52)
inverter.DeadTime       = 0.05e-6;     % 50 ns deadband (5 clock cycles @ 90 MHz)
inverter.ISenseVoltPerAmp = 0.012;     % 12 mV/A ACS37003 sensitivity
inverter.CtSensAOffset  = 2058;        % Calibrated zero-current counts (1.658V)
inverter.CtSensBOffset  = 2057;        % Calibrated zero-current counts (1.657V)
inverter.CtSensCOffset  = 2048;
inverter.ADCOffsetCalibEnable = 1;

%% 3. Motor Parameters (CubeMars AKE80-8 KV30)
motor = struct();
motor.Name          = 'CubeMars AKE80-8 KV30';
motor.polePairs     = 21;              % 21 Pole pairs
motor.base_speed    = 1440;            % [RPM] Base mechanical speed @ 48V bus
motor.base_freq     = motor.base_speed * motor.polePairs / 60; % 504 Hz electrical
motor.calibSpeed    = 60;              % [RPM] Motor speed during QEP index search
motor.QEPSlits      = 1024;            % 1024 pulses/rev -> 4096 counts/rev in 4x mode
motor.Rs            = 0.435;           % Phase resistance [Ohm]
motor.L_d           = 495e-6;          % d-axis inductance [H]
motor.L_q           = 495e-6;          % q-axis inductance [H]
motor.I_rated       = 4.8;             % Rated RMS current [A]
motor.I_peak        = 12.0;            % Peak current [A]
motor.PositionOffset = 0.025635;       % [PU] Calibrated with AS5047P SPI & QEP (0.025635 PU = 9.23 deg mech)

fprintf('✓ Parameters loaded into base workspace:\n');
fprintf('  - Motor      : %s (%d pole pairs, %d slits / %d counts)\n', ...
    motor.Name, motor.polePairs, motor.QEPSlits, 4*motor.QEPSlits);
fprintf('  - PWM Timing : %.1f kHz (Period: %d ticks @ 90 MHz)\n', ...
    PWM_frequency/1000, target.PWM_Counter_Period);
fprintf('  - Gate Drive : ePWM4, ePWM5, ePWM6 (BoosterPack Site 2: J5-J8)\n');
fprintf('  - Deadband   : 5 clock cycles (~55.5 ns for EPC GaN FETs)\n');
fprintf('  - Current ADC: Phase A = ADCINA3, Phase B = ADCINB3 (Trigger: ePWM4 SOCA)\n');
fprintf('  - Gate Enable: GPIO-52 (Active High)\n');
fprintf('  - Serial Port: COM7\n\n');

fprintf('Models available in this directory:\n');
fprintf('  [1] Open-Loop & ADC Offset Verification:\n');
fprintf('      - Target : OpenloopMotorControlF28069mLaunchPad.slx\n');
fprintf('      - Host   : OpenloopMotorControlHost.slx\n');
fprintf('  [2] QEP Encoder Position Offset Calibration:\n');
fprintf('      - Target : EncoderOffsetCalibrationF28069mLaunchPad.slx\n');
fprintf('      - Host   : OffsetCalibrationF28069mHost.slx\n');
fprintf('=================================================================\n');
