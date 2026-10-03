function [pwm_counts, en_gate, speed_rpm, state_echo, id_meas, iq_meas, th_ol_out] = Open_Loop_Controller(rx_cmd, spi_pos, vdc_cnt, adc_a, adc_b)
%#codegen
    % Preallocate outputs
    pwm_counts = single([1125.0; 1125.0; 1125.0]);
    en_gate = false;
    speed_rpm = single(0.0);
    state_echo = uint8(0);
    id_meas = single(0.0);
    iq_meas = single(0.0);
    th_ol_out = single(0.0);

    persistent state theta_ol last_spi_pos_1ms spd_timer filt_speed th_align align_done align_timer ...
               spd_int id_int iq_int
    if isempty(state)
        state = uint8(0);
        theta_ol = single(0.0);
        last_spi_pos_1ms = uint16(0);
        spd_timer = uint16(0);
        filt_speed = single(0.0);
        th_align = single(5.4437); % Default 311.9 deg
        align_done = false;
        align_timer = uint32(0);
        spd_int = single(0.0);
        id_int  = single(0.0);
        iq_int  = single(0.0);
    end

    % Constants
    TWO_PI = single(6.283185307);
    TWO_PI_OVER_3 = single(2.094395102);
    POLE_PAIRS = single(21.0);
    ONE_OVER_SQRT3 = single(0.577350269);
    SQRT3_OVER_2   = single(0.866025404);
    TS_FAST = single(0.00005); % 50 us (20 kHz) base loop time

    % 1. Vdc Calculation
    vdc_raw_volts = single(vdc_cnt) * single(3.3 * 46.4545 / 4095.0);
    if vdc_raw_volts > single(10.0)
        offset_a = single(2041.0);
        offset_b = single(2059.0);
        vdc_volts = vdc_raw_volts;
    else
        offset_a = single(2041.0);
        offset_b = single(2059.0);
        vdc_volts = single(40.0);
    end

    % 2. Speed Calculation from AS5047P SPI (Decimated to 1 ms / 1 kHz window)
    spd_timer = spd_timer + uint16(1);
    if spd_timer >= uint16(20)
        spd_timer = uint16(0);
        cur_pos = int32(spi_pos);
        prev_pos = int32(last_spi_pos_1ms);
        delta_pos = cur_pos - prev_pos;
        if delta_pos > 8192
            delta_pos = delta_pos - 16384;
        elseif delta_pos < -8192
            delta_pos = delta_pos + 16384;
        end
        last_spi_pos_1ms = uint16(spi_pos);
        raw_rpm = single(delta_pos) * single(3.662109375);
        filt_speed = single(0.85) * filt_speed + single(0.15) * raw_rpm;
    end
    speed_rpm = filt_speed;

    % 3. Mechanical & Electrical Angle from AS5047P Absolute SPI (20 kHz instantaneous)
    th_m = (single(spi_pos) / single(16384.0)) * TWO_PI;
    th_e_unmod = POLE_PAIRS * (th_m - th_align);
    th_e = mod(th_e_unmod, TWO_PI);
    if th_e < single(0.0)
        th_e = th_e + TWO_PI;
    end

    % 4. Current Measurement & Transforms (Clarke & Park)
    I_SCALE = single(-0.02014652); % -(3.3/4095)/(0.002*20)
    ia = (single(adc_a) - offset_a) * I_SCALE;
    ib = (single(adc_b) - offset_b) * I_SCALE;
    i_alpha = ia;
    i_beta  = (ia + single(2.0) * ib) * ONE_OVER_SQRT3;

    cos_e = cos(th_e);
    sin_e = sin(th_e);
    id_meas =  i_alpha * cos_e + i_beta * sin_e;
    iq_meas = -i_alpha * sin_e + i_beta * cos_e;

    % 5. Automatic Startup Alignment (When Vdc > 30V on boot)
    if ~align_done && vdc_volts >= single(30.0)
        align_timer = align_timer + uint32(1);
        m_align = single(0.04); % Safe 4% modulation at 40V = 1.6V (~3.6A peak during pull-in)
        da = single(0.5) + m_align;
        db = single(0.5) - m_align * single(0.5);
        dc = single(0.5) - m_align * single(0.5);
        pwm_counts = [da * single(2250.0); db * single(2250.0); dc * single(2250.0)];
        en_gate = true;
        state_echo = uint8(9); % State 9 = AUTO-ALIGNING
        if align_timer >= uint32(30000) % 30,000 * 50 us = 1.50 seconds settling
            th_align = th_m;           % Latch absolute mechanical angle at 0 deg elec
            align_done = true;
            state = uint8(0);          % Enter Idle/Stopped
            en_gate = false;
        end
        th_ol_out = single(0.0);
        return;
    end

    % 6. Command Decoding
    last_state = state;
    if rx_cmd == uint8('0') || rx_cmd == uint8(48) || rx_cmd == uint8('s') || rx_cmd == uint8('S')
        state = uint8(0);
    elseif rx_cmd == uint8(1) || rx_cmd == uint8('1') || rx_cmd == uint8('r') || rx_cmd == uint8('R')
        state = uint8(1);
    elseif rx_cmd == uint8(2) || rx_cmd == uint8('2')
        state = uint8(2);
    elseif rx_cmd == uint8(3) || rx_cmd == uint8('3')
        state = uint8(3);
    elseif rx_cmd == uint8(4) || rx_cmd == uint8('4') || rx_cmd == uint8('a') || rx_cmd == uint8('A')
        state = uint8(4);
    elseif rx_cmd == uint8(5) || rx_cmd == uint8('5')
        state = uint8(5); % Pure Torque Mode: +0.5 A Iq
    elseif rx_cmd == uint8(6) || rx_cmd == uint8('6')
        state = uint8(6); % Pure Torque Mode: +1.0 A Iq
    elseif rx_cmd == uint8(7) || rx_cmd == uint8('7')
        state = uint8(7); % Pure Torque Mode: -0.5 A Iq (Reverse)
    end

    if state ~= last_state
        spd_int = single(0.0);
        id_int  = single(0.0);
        iq_int  = single(0.0);
    end

    % 7. Mode Execution
    if state == uint8(0)
        % IDLE / STOPPED
        pwm_counts = single([1125.0; 1125.0; 1125.0]);
        en_gate = false;
        spd_int = single(0.0);
        id_int  = single(0.0);
        iq_int  = single(0.0);
        state_echo = uint8(0);

    elseif state == uint8(1) || state == uint8(2) || state == uint8(3)
        % Open Loop Mode (Corrected for 21 Pole Pairs @ 50 us step)
        if state == uint8(1)
            w_e = single(131.94689); % 60 RPM for 21 pole pairs
        elseif state == uint8(2)
            w_e = single(263.89378); % 120 RPM for 21 pole pairs
        else
            w_e = single(-131.94689); % -60 RPM for 21 pole pairs
        end
        theta_ol = theta_ol + w_e * TS_FAST;
        if theta_ol >= TWO_PI
            theta_ol = theta_ol - TWO_PI;
        elseif theta_ol < single(0.0)
            theta_ol = theta_ol + TWO_PI;
        end
        m_ol = single(0.06);
        da = single(0.5) + m_ol * cos(theta_ol);
        db = single(0.5) + m_ol * cos(theta_ol - TWO_PI_OVER_3);
        dc = single(0.5) + m_ol * cos(theta_ol + TWO_PI_OVER_3);
        pwm_counts = [da * single(2250.0); db * single(2250.0); dc * single(2250.0)];
        en_gate = true;
        state_echo = state;

    elseif state == uint8(4)
        % Manual Alignment Mode (0 deg Elec)
        m_align = single(0.04);
        da = single(0.5) + m_align;
        db = single(0.5) - m_align * single(0.5);
        dc = single(0.5) - m_align * single(0.5);
        pwm_counts = [da * single(2250.0); db * single(2250.0); dc * single(2250.0)];
        en_gate = true;
        th_align = th_m;
        align_done = true;
        state_echo = uint8(4);

    else
        % PURE TORQUE MODE FOC (Speed Loop Bypassed - Direct Iq Injection @ 20 kHz)
        if state == uint8(5)
            iq_ref = single(0.5);  % Gentle +0.5 A constant torque
        elseif state == uint8(6)
            iq_ref = single(1.0);  % +1.0 A torque
        else
            iq_ref = single(-0.5); % -0.5 A reverse torque
        end
        id_ref = single(0.0); % Pure torque mode (Id = 0)

        % Inner Current PI Regulators (Id and Iq) @ 20 kHz
        err_id = id_ref - id_meas;
        err_iq = iq_ref - iq_meas;
        
        KP_CURR = single(1.5);   % V / A
        KI_CURR_STEP = single(0.015); % V / A per 50 us step (300 V/(A*s))
        V_MAX_LINEAR = single(0.577) * vdc_volts; % Max SVPWM linear voltage (~23V at 40V)

        % Id PI controller
        id_int = id_int + KI_CURR_STEP * err_id;
        if id_int > V_MAX_LINEAR,  id_int = V_MAX_LINEAR;  end
        if id_int < -V_MAX_LINEAR, id_int = -V_MAX_LINEAR; end
        vd_cmd = KP_CURR * err_id + id_int;

        % Iq PI controller + Back-EMF Feedforward
        bemf_ff = speed_rpm * single(0.033); % Ke = 33 V/krpm = 0.033 V/RPM
        iq_int = iq_int + KI_CURR_STEP * err_iq;
        if iq_int > V_MAX_LINEAR,  iq_int = V_MAX_LINEAR;  end
        if iq_int < -V_MAX_LINEAR, iq_int = -V_MAX_LINEAR; end
        vq_cmd = KP_CURR * err_iq + iq_int + bemf_ff;

        % Voltage Vector Saturation
        v_mag = sqrt(vd_cmd * vd_cmd + vq_cmd * vq_cmd);
        if v_mag > V_MAX_LINEAR
            v_scale = V_MAX_LINEAR / v_mag;
            vd_cmd = vd_cmd * v_scale;
            vq_cmd = vq_cmd * v_scale;
        end

        % Inverse Park Transform
        v_alpha = vd_cmd * cos_e - vq_cmd * sin_e;
        v_beta  = vd_cmd * sin_e + vq_cmd * cos_e;

        % Inverse Clarke Transform
        va = v_alpha;
        vb = single(-0.5) * v_alpha + SQRT3_OVER_2 * v_beta;
        vc = single(-0.5) * v_alpha - SQRT3_OVER_2 * v_beta;

        % Space Vector Neutral-Point Modulation (Center-aligned)
        v_min = min(min(va, vb), vc);
        v_max = max(max(va, vb), vc);
        v_com = single(0.5) * (v_min + v_max);
        da = single(0.5) + (va - v_com) / vdc_volts;
        db = single(0.5) + (vb - v_com) / vdc_volts;
        dc = single(0.5) + (vc - v_com) / vdc_volts;

        % Duty Cycle Clamping (2% to 98%)
        if da < single(0.02), da = single(0.02); elseif da > single(0.98), da = single(0.98); end
        if db < single(0.02), db = single(0.02); elseif db > single(0.98), db = single(0.98); end
        if dc < single(0.02), dc = single(0.02); elseif dc > single(0.98), dc = single(0.98); end

        pwm_counts = [da * single(2250.0); db * single(2250.0); dc * single(2250.0)];
        en_gate = true;
        state_echo = state;
    end
    th_ol_out = theta_ol;
end
