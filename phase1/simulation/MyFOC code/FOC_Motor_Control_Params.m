%% =========================================================================
%  Field-Oriented Control (FOC) - Master Configuration & Parameters
%  Project     : 96V GaN Humanoid Drive (Phase 1)
%  Model Name  : FOC_Motor_Control
%  Hardware    : EPC91200 GaN Inverter + EPC9147B Interface Board
%                Texas Instruments LAUNCHXL-F28069M (C2000 Piccolo MCU)
%                AKE80-8 KV30 / AKE90-KV35 PMSM Outrunner Motors
% =========================================================================

% Retain overrides if set before calling this script
if ~exist('SELECTED_MOTOR', 'var')
    SELECTED_MOTOR = 'AKE80_KV30'; 
end

if ~exist('EXECUTION_MODE', 'var')
    EXECUTION_MODE = 'SINGLE_RUN';
end

if ~exist('PWM_frequency', 'var')
    PWM_frequency = 20e3;  % [Hz] (2250 period counts at 90 MHz clock)
end

fprintf('=================================================================\n');
fprintf('  FOC MOTOR CONTROL INITIALIZATION\n');
fprintf('  Motor Profile  : %s\n', SELECTED_MOTOR);
fprintf('  Execution Mode : %s\n', EXECUTION_MODE);
fprintf('  PWM Frequency  : %.1f kHz\n', PWM_frequency/1e3);
fprintf('=================================================================\n\n');

%% ── 2. TARGET CONTROLLER: TI TMS320F28069M (LAUNCHXL-F28069M) ───────────
% 90 MHz C28x 32-bit CPU with single-precision Floating-Point Unit (FPU32)
% Configured for BoosterPack Site 2: Headers J5, J6, J7, J8
target = struct();
target.Name               = 'TMS320F28069M';
target.ClockFreq_Hz       = 90e6;       % 90 MHz system clock
target.dataType           = 'single';   % Single-precision floating point
target.comport            = 'COM7';     % LaunchPad virtual COM port for external mode / SCI
target.HeaderSite         = 'J5-8';     % BoosterPack Site 2 (J5, J6, J7, J8)

% ── TI LaunchPad BoosterPack Site 2 (J5-J8) & EPC9147B Pin Mapping ──
%
% EPWM Gate Drives (ePWM4, ePWM5, ePWM6):
%   Phase U High : EPWM4A (GPIO-06 | Header J5-1)
%   Phase U Low  : EPWM4B (GPIO-07 | Header J5-2)
%   Phase V High : EPWM5A (GPIO-08 | Header J5-3)
%   Phase V Low  : EPWM5B (GPIO-09 | Header J5-4)
%   Phase W High : EPWM6A (GPIO-10 | Header J5-5)
%   Phase W Low  : EPWM6B (GPIO-11 | Header J5-6)
%
% Gate Driver Control, SPI & Protection:
%   Gate Enable          : GPIO-52 (Header J5-Site2)
%   SPI Module / AS5047P : SPI-A (SIMO GPIO-16/J2-15, SOMI GPIO-17/J2-14,
%                           CLK GPIO-18/J1-7, CSn GPIO-19/J2-19)
%   nEN  (Gate Driver)   : Pin J2-39 (Active LOW, physically grounded on EPC9147B adapter)
%   OCPn (Over-Current)  : Pin J2-37 (TripZone TZ1/TZ2 input)
%
% Analog Feedback (ADC SOCs triggered by EPWM4 SOCA on J5 site):
%   Phase U Current (Ia_fb) : ADC-A3 (SOC0 & SOC1)
%   Phase V Current (Ib_fb) : ADC-B3 (SOC2)
%   Phase W Current (Ic_fb) : ADC-A4 (SOC3)
%   Phase U Voltage (Vhb1)  : ADC-B4 (SOC4)
%   Phase V Voltage (Vhb2)  : ADC-A5 (SOC5)
%   Phase W Voltage (Vhb3)  : ADC-B5 (SOC6)
%   DC Bus Voltage  (Vdcbus): ADC-B7 (SOC7)
%   ADC SOC Trigger Source  : ADC_SocTrigSrc_EPWM4_ADCSOCA
%   Interrupt Source        : PIE_enablePwmInt(PWM_Number_4)
%
% Position Feedback (eQEP Modules on LaunchPad):
%   eQEP1: QEP1A (GPIO-20), QEP1B (GPIO-21), QEP1I (GPIO-23)
%   eQEP2: QEP2A (GPIO-54), QEP2B (GPIO-55), QEP2I (GPIO-56)
target.PWM_channels       = [4, 5, 6];   % ePWM4, ePWM5, ePWM6 for J5-J8 site
target.PWM_int_number     = 4;           % EPWM4 triggers ADC SOC & FOC ISR
target.ADC_IA_channel     = 'ADC-A3';
target.ADC_IB_channel     = 'ADC-B3';
target.ADC_IC_channel     = 'ADC-A4';
target.ADC_VDC_channel    = 'ADC-B7';
target.ADC_resolution_bits= 12;
target.ADC_Vref           = 3.3;         % 3.3V full-scale ADC reference
target.ADC_max_counts     = 2^target.ADC_resolution_bits - 1; % 4095

%% ── 3. INVERTER: EPC91200 GaN INVERTER + EPC9147B INTERFACE ──────────────
% 3-Phase GaN Bridge using EPC2305 eGaN FETs + EPC23102 Gate Drivers
inverter = struct();
inverter.Name               = 'EPC91200';
inverter.V_min_operating    = 30.0;     % Min 30V recommended (UVLO is at 20V)
inverter.V_max_rated        = 130.0;    % Max rated DC voltage (V)
inverter.I_max_cont         = 60.0;     % Max rated continuous current (A RMS)
inverter.I_trip             = 75.0;     % Hardware overcurrent protection threshold (A)

% ── GaN Switching & Deadtime Characteristics ──
inverter.DeadTime           = 0.05e-6;  % 50 ns deadtime (GaN ultrafast switching)
inverter.V_th               = 1.4;      % EPC2305 Gate threshold voltage (V)
inverter.R_DS_on            = 2.2e-3;   % Typical drain-source on-resistance: 2.2 mΩ (3.0 mΩ max)
inverter.G_off              = 1e-6;     % Off-state conductance (1/Ω)
inverter.E_on               = 90e-9;    % 90 nJ turn-on switching energy @ 48V, 10A
inverter.E_off              = 20e-9;    % 20 nJ turn-off switching energy @ 48V, 10A (no reverse recovery)
inverter.V_off_sw           = 48.0;     % Reference voltage for E_on/E_off scaling
inverter.I_on_sw            = 10.0;     % Reference current for E_on/E_off scaling

% ── Current Sensing (Allegro ACS37003LLUTR-050B3 / INA240) ──
% 12 mV/A sensitivity, 1.65V zero-current bias midpoint
inverter.ISenseVoltPerAmp   = 0.012;    % Sensor sensitivity (12 mV/A)
inverter.ISenseVref         = 1.65;     % Midpoint voltage reference (1.65 V)
inverter.ISenseMax          = target.ADC_Vref / inverter.ISenseVoltPerAmp; % 275.0 A peak-to-peak span
inverter.I_max              = inverter.ISenseMax / 2; % +/- 137.5 A peak single-ended limit
inverter.CtSensAOffset      = 2048;     % 1.65V / 3.3V * 4096 = 2048 counts nominal
inverter.CtSensBOffset      = 2048;     % Phase B offset counts
inverter.CtSensCOffset      = 2048;     % Phase C offset counts
inverter.ADCOffsetCalibEnable = 1;      % CRITICAL: 1 to auto-calibrate at boot (prevents phantom current)

% ── Voltage Sensing (Resistor Divider: R2 = 100kΩ, R1 = 2.2kΩ) ──
% Ratio = (100k + 2.2k) / 2.2k = 46.4545
inverter.VBusDividerR1      = 2.2e3;    % [Ω]
inverter.VBusDividerR2      = 100.0e3;  % [Ω]
inverter.VBusDividerRatio   = (inverter.VBusDividerR1 + inverter.VBusDividerR2) / inverter.VBusDividerR1;
inverter.V_max              = target.ADC_Vref * inverter.VBusDividerRatio; % 153.3 V full-scale (152.3V calibrated)
inverter.VSenseMax          = inverter.V_max;
inverter.VSenseOffset       = 0.0;

% ── Thermal Foster/Cauer Network ──
inverter.R_th_jc_ca         = [0.2, 6.5];       % Junction-to-case, case-to-ambient [K/W] per FET
inverter.M_th_jc            = [1.5e-4, 112.1];  % Thermal masses [J/K]
inverter.R_conv             = 6.504;            % Shared convection resistance [K/W]
inverter.R_CA_eff           = 0.9286;           % Effective case-to-ambient resistance [K/W]
inverter.T_ambient          = 25.0;             % Ambient temperature [°C]
inverter.T_j_max            = 150.0;            % Max allowable junction temperature [°C]

%% ── 4. MOTOR DATABASE ────────────────────────────────────────────────────

% --- Motor 1: AKE80-8 KV30 Exoskeleton Actuator ---
motor_AKE80.Name          = 'AKE80-8 KV30';
motor_AKE80.p             = 21;          % Number of pole pairs (42 poles)
motor_AKE80.KV            = 30.0;        % [RPM/V]
motor_AKE80.Rs            = 0.435;       % Stator phase resistance [Ω] (phase-to-neutral)
motor_AKE80.L_d           = 495.0e-6;    % d-axis inductance [H] (495 µH)
motor_AKE80.L_q           = 495.0e-6;    % q-axis inductance [H] (495 µH)
motor_AKE80.lambda        = 0.002785;    % PM Flux linkage [Wb] = 60 / (2*pi*KV*p*sqrt(3))
motor_AKE80.Kt            = 1.5 * motor_AKE80.p * motor_AKE80.lambda; % Torque constant [N·m/A] ~0.0877
motor_AKE80.I_rated       = 4.8;         % Rated RMS current [A]
motor_AKE80.I_peak        = 12.0;        % Peak current [A]
motor_AKE80.V_bat         = 48.0;        % Nominal DC bus voltage [V] (32V - 48V range)
motor_AKE80.N_rated       = 1440;        % Base speed @ 48V [RPM]
motor_AKE80.T_rated       = motor_AKE80.Kt * motor_AKE80.I_rated; % ~0.42 N·m
motor_AKE80.T_peak        = motor_AKE80.Kt * motor_AKE80.I_peak;  % ~1.05 N·m
motor_AKE80.QEPSlits      = 1024;        % AS5047P ABI: 1024 PPR (4096 CPR in 4x mode)
motor_AKE80.PositionOffset= 0.2592;      % Measured encoder offset (93.3 deg electrical from open-loop 60 RPM spin)
motor_AKE80.J             = 0.0005;      % Rotor + hub inertia [kg·m²]
motor_AKE80.B             = 0.0005;      % Viscous damping [N·m·s/rad]
motor_AKE80.R_wh          = 1.2;         % Winding-to-housing thermal resistance [K/W]
motor_AKE80.R_ha          = 2.5;         % Housing-to-ambient thermal resistance [K/W]
motor_AKE80.ironLoss.P_oc_iron = [4.0, 8.0, 1.0];   % [Hyst, Eddy, Excess] OC [W]
motor_AKE80.ironLoss.P_sc_iron = [5.0, 10.0, 1.5];  % SC [W]
motor_AKE80.ironLoss.f_iron    = 504.0;              % Rated electrical freq [Hz] @ 1440 RPM
motor_AKE80.ironLoss.I_sc_iron = 12.0;               % [A]

% --- Motor 2: AKE90-KV35 Humanoid Drive Baseline (48V) ---
motor_AKE90_48V.Name      = 'AKE90-KV35 48V Baseline';
motor_AKE90_48V.p         = 21;          % 21 pole pairs
motor_AKE90_48V.KV        = 35.0;        % [RPM/V]
motor_AKE90_48V.Kt        = 0.272;       % [N·m/A] (peak line-to-line)
motor_AKE90_48V.lambda    = 0.00863;     % Peak phase flux linkage [Wb]
motor_AKE90_48V.Rs        = 0.082;       % Stator phase resistance [Ω] (82 mΩ per star phase)
motor_AKE90_48V.L_d       = 117.5e-6;    % [H] (117.5 µH SPMSM)
motor_AKE90_48V.L_q       = 117.5e-6;    % [H]
motor_AKE90_48V.I_rated   = 25.0;        % Rated RMS current [A]
motor_AKE90_48V.I_peak    = 40.0;        % Peak current [A]
motor_AKE90_48V.V_bat     = 48.0;        % 48V DC bus
motor_AKE90_48V.N_rated   = 1680;        % Rated speed @ 48V [RPM] (175.9 rad/s)
motor_AKE90_48V.T_rated   = 6.8;         % Rated torque [N·m]
motor_AKE90_48V.T_peak    = 10.9;        % Peak torque [N·m]
motor_AKE90_48V.QEPSlits  = 1024;        % 1024 PPR encoder
motor_AKE90_48V.PositionOffset = 0.0;
motor_AKE90_48V.J         = 0.0012;      % [kg·m²]
motor_AKE90_48V.B         = 0.0008;      % [N·m·s/rad]
motor_AKE90_48V.R_wh      = 0.8;         % [K/W]
motor_AKE90_48V.R_ha      = 2.0;         % [K/W]
motor_AKE90_48V.ironLoss.P_oc_iron = [8.0, 15.0, 2.0];  % [W]
motor_AKE90_48V.ironLoss.P_sc_iron = [10.0, 18.0, 2.5]; % [W]
motor_AKE90_48V.ironLoss.f_iron    = 588.0;             % [Hz]
motor_AKE90_48V.ironLoss.I_sc_iron = 25.0;              % [A]

% --- Motor 3: AKE90-KV17.5 Rewound Humanoid Drive (96V GaN Advanced) ---
motor_AKE90_96V.Name      = 'AKE90-KV17.5 96V Rewound';
motor_AKE90_96V.p         = 21;          % 21 pole pairs
motor_AKE90_96V.KV        = 17.5;        % Half KV (double turns)
motor_AKE90_96V.Kt        = 0.272 * 2;   % 0.544 N·m/A (double Kt)
motor_AKE90_96V.lambda    = 0.00863 * 2; % 0.01726 Wb (double flux)
motor_AKE90_96V.Rs        = 0.082 * 4;   % 0.328 Ω (4x resistance with double turns)
motor_AKE90_96V.L_d       = 117.5e-6 * 4;% 470 µH (4x inductance)
motor_AKE90_96V.L_q       = 117.5e-6 * 4;% 470 µH
motor_AKE90_96V.I_rated   = 12.5;        % Halved current for identical rated torque
motor_AKE90_96V.I_peak    = 20.0;        % Peak current [A]
motor_AKE90_96V.V_bat     = 96.0;        % 96V GaN bus
motor_AKE90_96V.N_rated   = 1680;        % Rated speed @ 96V [RPM] (175.9 rad/s)
motor_AKE90_96V.T_rated   = 6.8;         % [N·m]
motor_AKE90_96V.T_peak    = 10.9;        % [N·m]
motor_AKE90_96V.QEPSlits  = 1024;
motor_AKE90_96V.PositionOffset = 0.0;
motor_AKE90_96V.J         = 0.0012;
motor_AKE90_96V.B         = 0.0008;
motor_AKE90_96V.R_wh      = 0.8;
motor_AKE90_96V.R_ha      = 2.0;
motor_AKE90_96V.ironLoss  = motor_AKE90_48V.ironLoss;

% ── Select Active Motor Struct ──
switch SELECTED_MOTOR
    case 'AKE80_KV30'
        pmsm = motor_AKE80;
    case 'AKE90_48V'
        pmsm = motor_AKE90_48V;
    case 'AKE90_96V'
        pmsm = motor_AKE90_96V;
    otherwise
        error('Unknown motor profile: %s', SELECTED_MOTOR);
end

%% ── 5. TIMING, SAMPLE RATES & DISCRETIZATION ────────────────────────────
T_pwm           = 1 / PWM_frequency;    % PWM period [s] (e.g. 10 µs @ 100 kHz)
Ts              = T_pwm;                % Current control loop sample time [s]
Ts_simulink     = T_pwm / 2;            % Simulation solver step (2x oversampling)
Ts_speed        = 1e-3;                 % Speed control loop sample time (1 kHz)
R_cable_total   = 0.016;                % 8 mΩ DC rail + 8 mΩ ground return [Ω]

%% ── 6. FOC CONTROLLER GAINS DESIGN (POLE-ZERO CANCELLATION) ─────────────
% Design current loop bandwidth as 1/10th of PWM switching frequency
f_c_current     = PWM_frequency / 10;   % Current loop bandwidth [Hz] (e.g., 10 kHz @ 100 kHz PWM)
omega_c         = 2 * pi * f_c_current; % [rad/s]

% Decoupled PI Gains (Continuous / Standard FOC):
% G_pi(s) = Kp + Ki/s = Kp * (s + Ki/Kp) / s
% Choosing Ki/Kp = Rs/L cancels the stator pole:
Kp_d = pmsm.L_d * omega_c;
Ki_d = pmsm.Rs  * omega_c;
Kp_q = pmsm.L_q * omega_c;
Ki_q = pmsm.Rs  * omega_c;

% Speed Controller PI Design:
% Standard symmetrical optimum / bandwidth separation (10x slower than current loop)
f_c_speed       = f_c_current / 10;     % Speed loop bandwidth [Hz] (e.g., 100 Hz - 300 Hz)
omega_c_speed   = 2 * pi * min(f_c_speed, 100);
Kp_speed        = (pmsm.J * omega_c_speed) / (1.5 * pmsm.p * pmsm.lambda);
Ki_speed        = Kp_speed * (omega_c_speed / 5);

% Damping adjustment for high pole-count outrunner (prevents zero-load limit-cycle hunting)
Kp_speed        = Kp_speed * 0.4;
Ki_speed        = Ki_speed * 0.4;

% Pack PI parameters struct for Simulink blocks
PI_params = struct();
PI_params.Kp_d      = Kp_d;
PI_params.Ki_d      = Ki_d;
PI_params.Kp_q      = Kp_q;
PI_params.Ki_q      = Ki_q;
PI_params.Kp_speed  = Kp_speed;
PI_params.Ki_speed  = Ki_speed;

%% ── 7. DEFAULT SIMULATION OPERATING POINT ────────────────────────────────
w_ref   = 100.0;                        % Mechanical angular velocity [rad/s] (955 RPM)
T_e_ref = pmsm.T_rated * 0.75;          % 75% rated torque [N·m]
i_d_ref = 0.0;                          % MTPA zero d-axis current for SPMSM (Ld ≈ Lq)
i_q_ref = T_e_ref / (1.5 * pmsm.p * pmsm.lambda); % Required q-axis current [A]

% Default simulation commands
StopTime = '0.500';                     % 500 ms simulation duration
modelName = 'FOC_Motor_Control';

%% ── 8. SEED BASE WORKSPACE FOR SIMULINK / SIMSCAPE ───────────────────────
assignin('base', 'modelName',      modelName);
assignin('base', 'pmsm',           pmsm);
assignin('base', 'inverter',       inverter);
assignin('base', 'target',         target);
assignin('base', 'PI_params',      PI_params);

% Scalar block variables directly referenced in Simscape / Simulink diagrams:
assignin('base', 'p',              pmsm.p);
assignin('base', 'lambda',         pmsm.lambda);
assignin('base', 'L_d',            pmsm.L_d);
assignin('base', 'L_q',            pmsm.L_q);
assignin('base', 'R',              pmsm.Rs);
assignin('base', 'V_bat',          pmsm.V_bat);
assignin('base', 'fsw',            PWM_frequency);
assignin('base', 'T_e_ref',        T_e_ref);
assignin('base', 'w_ref',          w_ref);
assignin('base', 'i_d_ref',        i_d_ref);
assignin('base', 'i_q_ref',        i_q_ref);

% Inverter block parameters
assignin('base', 'V_th',           inverter.V_th);
assignin('base', 'R_DS_on',        inverter.R_DS_on);
assignin('base', 'G_off',          inverter.G_off);
assignin('base', 'E_on',           inverter.E_on);
assignin('base', 'E_off',          inverter.E_off);
assignin('base', 'V_off_sw',       inverter.V_off_sw);
assignin('base', 'I_on_sw',        inverter.I_on_sw);
assignin('base', 'R_th_jc_ca',     inverter.R_th_jc_ca);
assignin('base', 'M_th_jc',        inverter.M_th_jc);
assignin('base', 'R_cable_total',  R_cable_total);

% Iron loss parameters
assignin('base', 'P_oc_iron',      pmsm.ironLoss.P_oc_iron);
assignin('base', 'P_sc_iron',      pmsm.ironLoss.P_sc_iron);
assignin('base', 'f_iron',         pmsm.ironLoss.f_iron);
assignin('base', 'I_sc_iron',      pmsm.ironLoss.I_sc_iron);

% Hardware deployment scaling variables
ISenseGain = (target.ADC_Vref / 4096) / inverter.ISenseVoltPerAmp;
VSenseGain = (target.ADC_Vref * inverter.VBusDividerRatio) / 4096;
PWM_period_counts = target.ClockFreq_Hz / (2 * PWM_frequency);

assignin('base', 'ISenseGain',        ISenseGain);
assignin('base', 'VSenseGain',        VSenseGain);
assignin('base', 'PWM_period_counts', PWM_period_counts);

% PI controller gains
assignin('base', 'Kp_d',           Kp_d);
assignin('base', 'Ki_d',           Ki_d);
assignin('base', 'Kp_q',           Kp_q);
assignin('base', 'Ki_q',           Ki_q);
assignin('base', 'Kp_speed',       Kp_speed);
assignin('base', 'Ki_speed',       Ki_speed);

% Per-Unit Base definitions for Host Model
PU_System = struct();
PU_System.N_base = pmsm.N_rated;
PU_System.I_base = pmsm.I_rated;
assignin('base', 'PU_System', PU_System);

fprintf('Base workspace successfully populated with %s parameters.\n', pmsm.Name);
fprintf('Current PI Gains: Kp_d = %.5f, Ki_d = %.5f | Bandwidth = %.1f kHz\n', Kp_d, Ki_d, f_c_current/1e3);
fprintf('Nominal Bus: %.1f V | Rated Speed: %d RPM | i_q_ref: %.2f A for T=%.2f N·m\n\n', ...
    pmsm.V_bat, pmsm.N_rated, i_q_ref, T_e_ref);

%% ── 9. PREPARE SIMULINK MODEL SETTINGS ───────────────────────────────────
if bdIsLoaded(modelName) || exist([modelName '.slx'], 'file')
    if ~bdIsLoaded(modelName)
        load_system(modelName);
    end
    set_param(modelName, 'SimscapeLogType', 'all');
    set_param(modelName, 'SimscapeLogName', 'simlog');
    
    % Mitigate PMSM angular_velocity initial condition conflict with ideal speed source
    try
        blkSel = simscape.block.Selector([modelName '/PMSM']);
        blkSel.setVariablePriority('angular_velocity', 'Low');
    catch
        try
            set_param([modelName '/PMSM'], 'angular_velocity_priority', 'low');
        catch
            % Parameter fallback handled silently
        end
    end
    save_system(modelName);
    fprintf('Simulink model "%s" configured and ready for simulation.\n', modelName);
end

%% ── 10. CONDITIONAL EXECUTION BRANCHES ───────────────────────────────────
switch EXECUTION_MODE
    case 'SINGLE_RUN'
        fprintf('\n>>> Ready! Press Play in Simulink or run: out = sim(''%s'');\n', modelName);

    case 'DYNO_SWEEP'
        fprintf('\n>>> Initiating Multi-Point Torque-Speed Dyno Sweep...\n');
        runDynoSweep(modelName, pmsm, inverter, PWM_frequency);

    case 'FSW_SWEEP'
        fprintf('\n>>> Initiating GaN Frequency Sweep (20 kHz - 100 kHz)...\n');
        runFswSweep(modelName, pmsm, inverter);

    case 'GAIT_CYCLE'
        fprintf('\n>>> Initiating Humanoid Walking Gait Temperature Trace...\n');
        runGaitProfile(modelName, pmsm, inverter);
end

%% =========================================================================
%% LOCAL HELPER FUNCTIONS FOR PARAMETER SWEEPS & LOGGING
%% =========================================================================

function simIn = buildSimulationInput(modelName, pmsm, inv, T_cmd, w_cmd, fsw_val)
    f_c  = fsw_val / 10;
    Kp_d = pmsm.L_d * 2*pi*f_c;
    Ki_d = pmsm.Rs  * 2*pi*f_c;
    Kp_q = pmsm.L_q * 2*pi*f_c;
    Ki_q = pmsm.Rs  * 2*pi*f_c;

    simIn = Simulink.SimulationInput(modelName);
    simIn = simIn.setModelParameter('SimulationMode', 'rapid-accelerator');
    simIn = simIn.setModelParameter('StopTime', '0.500');
    simIn = simIn.setModelParameter('SimscapeLogType', 'all');
    simIn = simIn.setModelParameter('SimscapeLogName', 'simlog');
    
    simIn = simIn.setVariable('lambda',     pmsm.lambda);
    simIn = simIn.setVariable('p',          pmsm.p);
    simIn = simIn.setVariable('L_d',        pmsm.L_d);
    simIn = simIn.setVariable('L_q',        pmsm.L_q);
    simIn = simIn.setVariable('R',          pmsm.Rs);
    simIn = simIn.setVariable('V_bat',      pmsm.V_bat);
    simIn = simIn.setVariable('T_e_ref',    T_cmd);
    simIn = simIn.setVariable('w_ref',      w_cmd);
    simIn = simIn.setVariable('fsw',        fsw_val);
    simIn = simIn.setVariable('Kp_d',       Kp_d);
    simIn = simIn.setVariable('Ki_d',       Ki_d);
    simIn = simIn.setVariable('Kp_q',       Kp_q);
    simIn = simIn.setVariable('Ki_q',       Ki_q);
    simIn = simIn.setVariable('V_th',       inv.V_th);
    simIn = simIn.setVariable('R_DS_on',    inv.R_DS_on);
    simIn = simIn.setVariable('G_off',      inv.G_off);
    simIn = simIn.setVariable('E_on',       inv.E_on);
    simIn = simIn.setVariable('E_off',      inv.E_off);
    simIn = simIn.setVariable('V_off_sw',   inv.V_off_sw);
    simIn = simIn.setVariable('I_on_sw',    inv.I_on_sw);
    simIn = simIn.setVariable('R_th_jc_ca', inv.R_th_jc_ca);
    simIn = simIn.setVariable('M_th_jc',    inv.M_th_jc);
    simIn = simIn.setVariable('P_oc_iron',  pmsm.ironLoss.P_oc_iron);
    simIn = simIn.setVariable('P_sc_iron',  pmsm.ironLoss.P_sc_iron);
    simIn = simIn.setVariable('f_iron',     pmsm.ironLoss.f_iron);
    simIn = simIn.setVariable('I_sc_iron',  pmsm.ironLoss.I_sc_iron);
end

function runDynoSweep(modelName, pmsm, inv, fsw_val)
    torque_sweep = linspace(1.0, pmsm.T_rated * 1.2, 5);
    speed_sweep  = linspace(20, pmsm.N_rated * 2*pi/60, 5);
    nT = length(torque_sweep);
    nS = length(speed_sweep);
    nTotal = nT * nS;
    
    simInputs(1:nTotal) = Simulink.SimulationInput(modelName);
    k = 0;
    for t_idx = 1:nT
        for s_idx = 1:nS
            k = k + 1;
            simInputs(k) = buildSimulationInput(modelName, pmsm, inv, ...
                torque_sweep(t_idx), speed_sweep(s_idx), fsw_val);
        end
    end
    fprintf('Dispatching %d simulation points via parsim...\n', nTotal);
    outs = parsim(simInputs, 'ShowProgress', 'on');
    fprintf('Dyno sweep completed successfully.\n');
end

function runFswSweep(modelName, pmsm, inv)
    fsw_list = [20e3, 40e3, 60e3, 80e3, 100e3];
    T_fixed  = pmsm.T_rated * 0.7;
    w_fixed  = 100.0; % rad/s
    nFsw     = length(fsw_list);
    
    simInputs(1:nFsw) = Simulink.SimulationInput(modelName);
    for idx = 1:nFsw
        simInputs(idx) = buildSimulationInput(modelName, pmsm, inv, T_fixed, w_fixed, fsw_list(idx));
    end
    fprintf('Dispatching %d frequency points (20k - 100k) via parsim...\n', nFsw);
    outs = parsim(simInputs, 'ShowProgress', 'on');
    fprintf('Frequency sweep completed successfully.\n');
end

function runGaitProfile(modelName, pmsm, inv)
    gait_time   = [0, 0.4, 0.8, 1.2, 1.6, 2.0, 2.4, 2.8, 3.0];
    gait_torque = [3.5, 4.375, 2.5, 0.5, 2.5, 4.375, 3.5, 1.5, 3.5] * (pmsm.T_rated / 6.8);
    gait_speed  = [20, 25, 15, 5, 15, 25, 20, 10, 20];
    nGait       = length(gait_time);
    
    simInputs(1:nGait) = Simulink.SimulationInput(modelName);
    for g_idx = 1:nGait
        simInputs(g_idx) = buildSimulationInput(modelName, pmsm, inv, ...
            gait_torque(g_idx), gait_speed(g_idx), 100e3);
    end
    fprintf('Dispatching %d gait trajectory points...\n', nGait);
    outs = parsim(simInputs, 'ShowProgress', 'on');
    fprintf('Gait profile simulation completed successfully.\n');
end
