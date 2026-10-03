%% Model initialization callback for the model 'open_loop_algorithm.slx'
% Hardware: TI TMS320F28069M LaunchPad + EPC9147B/EPC91200 GaN Inverter
% Motor   : CubeMars AKE80-8 KV30 (21 Pole Pairs)

%% 1. Target Controller Parameters (TI TMS320F28069M)
target.model              = 'LAUNCHXL-F28069M';
target.CPU_frequency      = 90e6;                 % 90 MHz SYSCLK
PWM_frequency             = 20e3;                 % 20 kHz PWM
T_pwm                     = 1/PWM_frequency;      % 50 us switching period
Ts                        = T_pwm;                % Sample time for controller
target.PWM_frequency      = PWM_frequency;
target.PWM_Counter_Period = round(target.CPU_frequency / (2 * target.PWM_frequency)); % 2250 counts
target.ADC_Vref           = 3.3;                  % 3.3V ADC reference
target.ADC_MaxCount       = 4095;                 % 12-bit ADC full-scale
target.SCI_baud_rate      = 1.25e6;               % 1.25 MBaud
target.comport            = 'COM7';

dataType                  = 'single';

%% 2. Inverter Parameters (EPC9147B + EPC91200 GaN Inverter)
inverter.model            = 'EPC91200_GaN';
inverter.V_dc             = 48.0;                 % 48V bench supply
inverter.I_trip           = 25.0;                 % Overcurrent trip (A)
inverter.DeadTime         = 0.05e-6;              % 50 ns deadtime for GaN
inverter.ISenseVoltPerAmp = 0.012;                % Allegro ACS37003 (12 mV/A)
inverter.ISenseVref       = 3.3;                  % ADC reference
inverter.ISenseMax        = inverter.ISenseVref / (2 * inverter.ISenseVoltPerAmp); % 137.5 A
inverter.CtSensAOffset    = 2048;                 % 1.65V nominal midpoint
inverter.CtSensBOffset    = 2048;
inverter.EnableLogic      = 1;                    % Active high (GPIO-52)
inverter.invertingAmp     = -1;

%% 3. Motor Parameters (CubeMars AKE80-8 KV30)
motor.Name                = 'CubeMars AKE80-8 KV30';
motor.polePairs           = 21;                   % 21 Pole pairs (42 poles)
motor.base_speed          = 1440;                 % Mechanical base speed @ 48V
motor.base_freq           = motor.base_speed * motor.polePairs / 60; % 504 Hz electrical
motor.calibSpeed          = 60;                   % Test speed (RPM)
motor.QEPSlits            = 1024;                 % AS5047P ABI mode (4096 counts/rev)
motor.Rs                  = 0.435;                % Phase resistance (Ohms)
motor.L_d                 = 495e-6;               % 495 uH
motor.L_q                 = 495e-6;               % 495 uH
motor.I_rated             = 4.8;                  % 4.8 A RMS
motor.I_peak              = 12.0;                 % 12.0 A peak
