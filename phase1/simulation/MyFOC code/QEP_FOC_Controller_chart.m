function [pwm_counts, en_gate, speed_rpm, state_echo, id_meas, iq_meas, th_ol_out] = Open_Loop_Controller(rx_cmd, spi_pos, vdc_cnt, adc_a, adc_b, qep_pos)
%#codegen
% QEP is the runtime rotor-position/speed source. SPI is used once at startup
% for reference alignment and remains in diagnostics.
pwm_counts = single([1125.0;1125.0;1125.0]);
en_gate = false;
speed_rpm = single(0.0);
state_echo = uint8(0);
id_meas = single(0.0);
iq_meas = single(0.0);
th_ol_out = single(0.0);

persistent state theta_ol last_qep_pos spd_timer filt_speed th_align align_done align_timer qep_phase_offset qep_ref_initialized spd_int id_int iq_int
if isempty(state)
    state=uint8(0); theta_ol=single(0.0); last_qep_pos=uint16(0); spd_timer=uint16(0); filt_speed=single(0.0); th_align=single(0.0); align_done=false; align_timer=uint32(0); qep_phase_offset=single(0.0); qep_ref_initialized=false; spd_int=single(0.0); id_int=single(0.0); iq_int=single(0.0);
end

TWO_PI=single(6.283185307); TWO_PI_OVER_3=single(2.094395102); PI_OVER_2=single(1.570796327); POLE_PAIRS=single(21.0); ONE_OVER_SQRT3=single(0.577350269); SQRT3_OVER_2=single(0.866025404); TS_FAST=single(0.00005); QEP_COUNTS_PER_REV=int32(4096); QEP_HALF_REV=int32(2048); QEP_ANGLE_SCALE=TWO_PI/single(4096.0); QEP_RPM_SCALE=single(14.6484375); SPI_COUNTS_PER_REV=single(16384.0);

vdc_volts=single(vdc_cnt)*single(3.3*46.4545/4095.0);
offset_a=single(2041.0); offset_b=single(2059.0);

qep_count_i32=int32(qep_pos);
if qep_count_i32<int32(0), qep_count_i32=int32(0); end
qep_count_i32=mod(qep_count_i32,QEP_COUNTS_PER_REV);
qep_raw_angle=single(qep_count_i32)*QEP_ANGLE_SCALE;

spi_count_i32=int32(spi_pos);
if spi_count_i32<int32(0), spi_count_i32=int32(0); end
spi_count_i32=mod(spi_count_i32,int32(16384));
spi_angle=single(spi_count_i32)/SPI_COUNTS_PER_REV*TWO_PI;

if ~qep_ref_initialized
    qep_phase_offset=mod(spi_angle-qep_raw_angle+TWO_PI,TWO_PI);
    qep_ref_initialized=true;
    last_qep_pos=uint16(qep_count_i32);
end
qep_mech_angle=mod(qep_raw_angle+qep_phase_offset+TWO_PI,TWO_PI);

spd_timer=spd_timer+uint16(1);
if spd_timer>=uint16(20)
    spd_timer=uint16(0);
    delta_qep=qep_count_i32-int32(last_qep_pos);
    if delta_qep>QEP_HALF_REV, delta_qep=delta_qep-QEP_COUNTS_PER_REV; elseif delta_qep<-QEP_HALF_REV, delta_qep=delta_qep+QEP_COUNTS_PER_REV; end
    last_qep_pos=uint16(qep_count_i32);
    raw_rpm=single(delta_qep)*QEP_RPM_SCALE;
    filt_speed=single(0.85)*filt_speed+single(0.15)*raw_rpm;
end
speed_rpm=filt_speed;

th_e=mod(POLE_PAIRS*(qep_mech_angle-th_align)+PI_OVER_2+TWO_PI,TWO_PI);
I_SCALE=single(-0.02014652);
ia=(single(adc_a)-offset_a)*I_SCALE; ib=(single(adc_b)-offset_b)*I_SCALE;
i_alpha=ia; i_beta=(ia+single(2.0)*ib)*ONE_OVER_SQRT3;
cos_e=cos(th_e); sin_e=sin(th_e);
id_meas=i_alpha*cos_e+i_beta*sin_e; iq_meas=-i_alpha*sin_e+i_beta*cos_e;

last_state=state;
if rx_cmd==uint8(48)||rx_cmd==uint8(83)||rx_cmd==uint8(115), state=uint8(0); elseif rx_cmd==uint8(49)||rx_cmd==uint8(82)||rx_cmd==uint8(114), state=uint8(1); elseif rx_cmd==uint8(50), state=uint8(2); elseif rx_cmd==uint8(51), state=uint8(3); elseif rx_cmd==uint8(52)||rx_cmd==uint8(65)||rx_cmd==uint8(97), state=uint8(4); elseif rx_cmd==uint8(53), state=uint8(5); elseif rx_cmd==uint8(54), state=uint8(6); elseif rx_cmd==uint8(55), state=uint8(7); end
if state~=last_state
    spd_int=single(0.0); id_int=single(0.0); iq_int=single(0.0);
    if state==uint8(4), align_timer=uint32(0); end
end

auto_align=(~align_done)&&(state==uint8(0))&&(vdc_volts>=single(30.0));
manual_align=(state==uint8(4))&&(vdc_volts>=single(30.0));
if auto_align||manual_align
    align_timer=align_timer+uint32(1);
    m_align=single(0.06); da=single(0.5)+m_align; db=single(0.5)-m_align*single(0.5); dc=single(0.5)-m_align*single(0.5);
    pwm_counts=[da*single(2250.0);db*single(2250.0);dc*single(2250.0)]; en_gate=true;
    if auto_align, state_echo=uint8(9); else, state_echo=uint8(4); end
    if align_timer>=uint32(30000)
        th_align=qep_mech_angle; align_done=true; align_timer=uint32(0); state=uint8(0); en_gate=false; pwm_counts=single([1125.0;1125.0;1125.0]); state_echo=uint8(0);
    end
    th_ol_out=theta_ol; return;
elseif state==uint8(4)
    pwm_counts=single([1125.0;1125.0;1125.0]); en_gate=false; state_echo=uint8(4); th_ol_out=theta_ol; return;
end

if state==uint8(0)
    pwm_counts=single([1125.0;1125.0;1125.0]); en_gate=false; spd_int=single(0.0); id_int=single(0.0); iq_int=single(0.0); state_echo=uint8(0);
elseif state==uint8(1)||state==uint8(2)||state==uint8(3)
    if state==uint8(1), w_e=single(131.94689); elseif state==uint8(2), w_e=single(263.89378); else, w_e=single(-131.94689); end
    theta_ol=theta_ol+w_e*TS_FAST;
    if theta_ol>=TWO_PI, theta_ol=theta_ol-TWO_PI; elseif theta_ol<single(0.0), theta_ol=theta_ol+TWO_PI; end
    m_ol=single(0.06); da=single(0.5)+m_ol*cos(theta_ol); db=single(0.5)+m_ol*cos(theta_ol-TWO_PI_OVER_3); dc=single(0.5)+m_ol*cos(theta_ol+TWO_PI_OVER_3);
    pwm_counts=[da*single(2250.0);db*single(2250.0);dc*single(2250.0)]; en_gate=true; state_echo=state;
elseif state==uint8(5)||state==uint8(6)||state==uint8(7)
    if ~align_done
        pwm_counts=single([1125.0;1125.0;1125.0]); en_gate=false; state_echo=uint8(8);
    else
        if state==uint8(5), vd_cmd=single(0.0); vq_cmd=single(2.5); elseif state==uint8(6), vd_cmd=single(0.0); vq_cmd=single(4.0); else, vd_cmd=single(0.0); vq_cmd=single(-2.5); end
        v_alpha=vd_cmd*cos_e+vq_cmd*sin_e; v_beta=vd_cmd*sin_e-vq_cmd*cos_e; va=v_alpha; vb=single(-0.5)*v_alpha+SQRT3_OVER_2*v_beta; vc=single(-0.5)*v_alpha-SQRT3_OVER_2*v_beta;
        v_min=min(min(va,vb),vc); v_max=max(max(va,vb),vc); v_com=single(0.5)*(v_min+v_max); vdc_for_mod=vdc_volts; if vdc_for_mod<single(1.0), vdc_for_mod=single(1.0); end
        da=single(0.5)+(va-v_com)/vdc_for_mod; db=single(0.5)+(vb-v_com)/vdc_for_mod; dc=single(0.5)+(vc-v_com)/vdc_for_mod;
        if da<single(0.02), da=single(0.02); elseif da>single(0.98), da=single(0.98); end; if db<single(0.02), db=single(0.02); elseif db>single(0.98), db=single(0.98); end; if dc<single(0.02), dc=single(0.02); elseif dc>single(0.98), dc=single(0.98); end
        pwm_counts=[da*single(2250.0);db*single(2250.0);dc*single(2250.0)]; en_gate=true; state_echo=state;
    end
else
    pwm_counts=single([1125.0;1125.0;1125.0]); en_gate=false; state_echo=uint8(8);
end
th_ol_out=theta_ol;
end
