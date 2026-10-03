function [pwm_counts, en_gate, speed_rpm, state_echo, id_meas, iq_meas, th_ol_out] = Open_Loop_Controller(rx_cmd, spi_pos, vdc_cnt, adc_a, adc_b, qep_pos)
%#codegen
% BOARD-SIDE COMMISSIONING CONTROLLER
%
% Commands:
%   0/S/s : stop and latch the ePWM software trip
%   A/a/4 : apply a small static alignment vector and capture both encoders
%   P/p/5 : open-loop electrical rotation in the board-positive direction
%   N/n/7 : open-loop electrical rotation in the board-negative direction
%   C/c   : clear the captured QEP electrical offset and stop
%
% SPI is sampled during alignment as an independent absolute reference.
% QEP is the runtime mechanical angle/speed source after calibration.
% The dashboard does not alter any of these signs.

pwm_counts = single([1125.0;1125.0;1125.0]);
en_gate = false;
speed_rpm = single(0.0);
state_echo = uint8(0);
id_meas = single(0.0);
iq_meas = single(0.0);
th_ol_out = single(0.0);

persistent state align_timer sample_count sum_spi_sin sum_spi_cos ...
    sum_qep_sin sum_qep_cos qep_elec_offset cal_valid spd_timer ...
    last_qep filt_speed qep_speed_initialized theta_ol last_cmd

if isempty(state)
    state = uint8(0);
    align_timer = uint32(0);
    sample_count = uint32(0);
    sum_spi_sin = single(0.0);
    sum_spi_cos = single(0.0);
    sum_qep_sin = single(0.0);
    sum_qep_cos = single(0.0);
    qep_elec_offset = single(0.0);
    cal_valid = false;
    spd_timer = uint16(0);
    last_qep = uint16(0);
    filt_speed = single(0.0);
    qep_speed_initialized = false;
    theta_ol = single(0.0);
    last_cmd = uint8(0);
end

TWO_PI = single(6.283185307);
TWO_PI_OVER_3 = single(2.094395102);
POLE_PAIRS = single(21.0);
SPI_COUNTS_PER_REV = single(16384.0);
QEP_COUNTS_PER_REV = int32(4096);
QEP_HALF_REV = int32(2048);
QEP_ANGLE_SCALE = TWO_PI / single(4096.0);
QEP_RPM_SCALE = single(14.6484375);       % 4096 counts/rev, 1 ms update
TS_FAST = single(0.00005);                 % 20 kHz controller step
PWM_PERIOD = single(2250.0);

ALIGN_HOLD_TICKS = uint32(30000);          % 1.5 s
ALIGN_SETTLE_TICKS = uint32(20000);        % collect after first 1.0 s
ALIGN_MIN_SAMPLES = uint32(500);
ALIGN_MODULATION = single(0.01);            % intentionally conservative
OPEN_LOOP_MODULATION = single(0.02);        % first powered test only
OPEN_LOOP_MECH_RPM = single(2.0);            % deliberately slow bench test
OPEN_LOOP_ELEC_SPEED = OPEN_LOOP_MECH_RPM * POLE_PAIRS * TWO_PI / single(60.0);
ALIGN_CURRENT_LIMIT = single(1.0);          % board ADC units must be verified
MIN_VDC = single(30.0);

% Convert board ADC channels to the existing diagnostic current estimate.
vdc_volts = single(vdc_cnt) * single(3.3 * 46.4545 / 4095.0);
offset_a = single(2041.0);
offset_b = single(2059.0);
I_SCALE = single(-0.02014652);
ia = (single(adc_a) - offset_a) * I_SCALE;
ib = (single(adc_b) - offset_b) * I_SCALE;
i_alpha = ia;
i_beta = (ia + single(2.0) * ib) * single(0.577350269);
i_mag = sqrt(i_alpha * i_alpha + i_beta * i_beta);

% Raw encoder angles. QEP polarity was configured in the model so the
% hand-spin-positive direction is the board-positive direction.
spi_i32 = mod(int32(spi_pos), int32(16384));
if spi_i32 < int32(0), spi_i32 = spi_i32 + int32(16384); end
theta_spi = single(spi_i32) / SPI_COUNTS_PER_REV * TWO_PI;

qep_i32 = mod(int32(qep_pos), QEP_COUNTS_PER_REV);
if qep_i32 < int32(0), qep_i32 = qep_i32 + QEP_COUNTS_PER_REV; end
theta_qep = single(qep_i32) * QEP_ANGLE_SCALE;

% QEP speed with a first-sample guard so startup cannot create false RPM.
if ~qep_speed_initialized
    last_qep = uint16(qep_i32);
    qep_speed_initialized = true;
end
spd_timer = spd_timer + uint16(1);
if spd_timer >= uint16(20)
    spd_timer = uint16(0);
    delta_qep = qep_i32 - int32(last_qep);
    if delta_qep > QEP_HALF_REV
        delta_qep = delta_qep - QEP_COUNTS_PER_REV;
    elseif delta_qep < -QEP_HALF_REV
        delta_qep = delta_qep + QEP_COUNTS_PER_REV;
    end
    last_qep = uint16(qep_i32);
    filt_speed = single(0.85) * filt_speed + ...
        single(0.15) * single(delta_qep) * QEP_RPM_SCALE;
end
speed_rpm = filt_speed;

% Process command bytes. A zero/no-byte result is ignored so a command
% remains latched until the next explicit command or STOP.
last_state = state;
% Execute each recognized serial command once. This prevents a held SCI byte
% from restarting alignment on every 50 us step.
if rx_cmd ~= last_cmd
    if rx_cmd == uint8(48) || rx_cmd == uint8(83) || rx_cmd == uint8(115)
        state = uint8(0);
        last_cmd = rx_cmd;
    elseif rx_cmd == uint8(65) || rx_cmd == uint8(97) || rx_cmd == uint8(52)
        state = uint8(4);
        last_cmd = rx_cmd;
    elseif rx_cmd == uint8(80) || rx_cmd == uint8(112) || rx_cmd == uint8(53)
        state = uint8(5);
        last_cmd = rx_cmd;
    elseif rx_cmd == uint8(78) || rx_cmd == uint8(110) || rx_cmd == uint8(55)
        state = uint8(7);
        last_cmd = rx_cmd;
    elseif rx_cmd == uint8(67) || rx_cmd == uint8(99)
        state = uint8(0);
        cal_valid = false;
        qep_elec_offset = single(0.0);
        theta_ol = single(0.0);
        last_cmd = rx_cmd;
    end
end

if state ~= last_state
    if state == uint8(4)
        align_timer = uint32(0);
        sample_count = uint32(0);
        sum_spi_sin = single(0.0);
        sum_spi_cos = single(0.0);
        sum_qep_sin = single(0.0);
        sum_qep_cos = single(0.0);
    elseif state == uint8(5) || state == uint8(7)
        % Start at the measured QEP electrical position so a re-run after
        % a hand movement is deterministic.
        theta_qep_e = mod(POLE_PAIRS * theta_qep + qep_elec_offset + TWO_PI, TWO_PI);
        theta_ol = theta_qep_e;
    end
end

% Runtime electrical angle from calibrated QEP. This is intentionally not
% derived from SPI during the open-loop test.
theta_qep_e = mod(POLE_PAIRS * theta_qep + qep_elec_offset + TWO_PI, TWO_PI);
cos_qep_e = cos(theta_qep_e);
sin_qep_e = sin(theta_qep_e);
id_meas = i_alpha * cos_qep_e + i_beta * sin_qep_e;
iq_meas = -i_alpha * sin_qep_e + i_beta * cos_qep_e;

if state == uint8(4)
    state_echo = uint8(4);
    if vdc_volts < MIN_VDC
        state_echo = uint8(13);             % requested alignment, no HV bus
        en_gate = false;
        pwm_counts = single([1125.0;1125.0;1125.0]);
    elseif i_mag > ALIGN_CURRENT_LIMIT
        state = uint8(11);
        state_echo = uint8(11);
        en_gate = false;
        pwm_counts = single([1125.0;1125.0;1125.0]);
    else
        align_timer = align_timer + uint32(1);

        % Static positive phase-A vector: electrical zero for this test.
        da = single(0.5) + ALIGN_MODULATION;
        db = single(0.5) - ALIGN_MODULATION * single(0.5);
        dc = db;
        pwm_counts = [da * PWM_PERIOD; db * PWM_PERIOD; dc * PWM_PERIOD];
        en_gate = true;

        if align_timer >= ALIGN_SETTLE_TICKS
            sum_spi_sin = sum_spi_sin + sin(theta_spi);
            sum_spi_cos = sum_spi_cos + cos(theta_spi);
            sum_qep_sin = sum_qep_sin + sin(theta_qep);
            sum_qep_cos = sum_qep_cos + cos(theta_qep);
            sample_count = sample_count + uint32(1);
        end

        if align_timer >= ALIGN_HOLD_TICKS
            if sample_count >= ALIGN_MIN_SAMPLES && ...
                    abs(sum_qep_sin) + abs(sum_qep_cos) > single(1.0) && ...
                    abs(sum_spi_sin) + abs(sum_spi_cos) > single(1.0)
                qep_mean = atan2(sum_qep_sin, sum_qep_cos);
                if qep_mean < single(0.0), qep_mean = qep_mean + TWO_PI; end
                % At alignment, the commanded phase-A vector is electrical 0.
                qep_elec_offset = mod(-POLE_PAIRS * qep_mean + TWO_PI, TWO_PI);
                cal_valid = true;
                state = uint8(10);
                state_echo = uint8(10);
                en_gate = false;
                pwm_counts = single([1125.0;1125.0;1125.0]);
            else
                state = uint8(12);
                state_echo = uint8(12);
                en_gate = false;
                pwm_counts = single([1125.0;1125.0;1125.0]);
            end
        end
    end
elseif state == uint8(5) || state == uint8(7)
    if ~cal_valid
        state_echo = uint8(8);                % align first
        en_gate = false;
        pwm_counts = single([1125.0;1125.0;1125.0]);
    elseif vdc_volts < MIN_VDC
        state_echo = uint8(13);               % no HV bus
        en_gate = false;
        pwm_counts = single([1125.0;1125.0;1125.0]);
    elseif i_mag > ALIGN_CURRENT_LIMIT
        state = uint8(11);
        state_echo = uint8(11);
        en_gate = false;
        pwm_counts = single([1125.0;1125.0;1125.0]);
    else
        if state == uint8(5)
            theta_ol = theta_ol + OPEN_LOOP_ELEC_SPEED * TS_FAST;
        else
            theta_ol = theta_ol - OPEN_LOOP_ELEC_SPEED * TS_FAST;
        end
        if theta_ol >= TWO_PI
            theta_ol = theta_ol - TWO_PI;
        elseif theta_ol < single(0.0)
            theta_ol = theta_ol + TWO_PI;
        end

        da = single(0.5) + OPEN_LOOP_MODULATION * cos(theta_ol);
        db = single(0.5) + OPEN_LOOP_MODULATION * cos(theta_ol - TWO_PI_OVER_3);
        dc = single(0.5) + OPEN_LOOP_MODULATION * cos(theta_ol + TWO_PI_OVER_3);
        pwm_counts = [da * PWM_PERIOD; db * PWM_PERIOD; dc * PWM_PERIOD];
        en_gate = true;
        state_echo = state;
    end
else
    % Stopped, valid calibration, or latched fault: software ePWM trip is
    % the actual gate-off mechanism. Neutral compare values are retained
    % because CMPA=0 is not a safe disable for complementary PWM.
    en_gate = false;
    pwm_counts = single([1125.0;1125.0;1125.0]);
    if state == uint8(10) || (state == uint8(0) && cal_valid)
        state_echo = uint8(10);
    elseif state == uint8(11)
        state_echo = uint8(11);
    elseif state == uint8(12)
        state_echo = uint8(12);
    else
        state_echo = uint8(0);
    end
end

if state == uint8(10) || (state == uint8(0) && cal_valid)
    th_ol_out = qep_elec_offset;
elseif state == uint8(5) || state == uint8(7)
    th_ol_out = theta_ol;
else
    th_ol_out = theta_spi;
end

% EPC91200 has no usable nEN gate-disable path in this wiring. Force the
% ePWM trip from the controller itself; GPIO52 is only a legacy diagnostic.
if coder.target('Rtw')
    coder.cinclude('pwm_gate_control.h');
    coder.updateBuildInfo('addSourceFiles', 'pwm_gate_control.c');
    coder.updateBuildInfo('addIncludePaths', 'C:\\96v_gan_humanoid_drive\\phase1\\simulation\\MyFOC code');
    gate_cmd = uint16(en_gate);
    coder.ceval('pwm_gate_control', gate_cmd);
end
end
