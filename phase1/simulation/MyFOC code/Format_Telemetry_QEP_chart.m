function [tx_packet, send_flag] = Format_Telemetry(id_meas, iq_meas, speed_rpm, adc_a, adc_b, qep_raw, spi_raw, th_ol, state_echo, qep_dir, qep_ilat, qep_qcprd, qep_coef, qep_cdef, qep_qcprdlat, qep_qposlat, en_gate)
%#codegen
persistent dec_cnt
if isempty(dec_cnt), dec_cnt=uint16(0); end
dec_cnt=dec_cnt+uint16(1);
if dec_cnt>=uint16(20), dec_cnt=uint16(0); send_flag=true; else, send_flag=false; end

tx_packet=zeros(33,1,'uint8');
id_u16=typecast(int16(round(id_meas*single(100.0))),'uint16');
iq_u16=typecast(int16(round(iq_meas*single(100.0))),'uint16');
spd_u16=typecast(int16(round(speed_rpm)),'uint16');
aa=uint16(adc_a); ab=uint16(adc_b); spi_val=uint16(spi_raw); qep_val=uint16(qep_raw); ilat_val=uint16(qep_ilat);
th_norm=th_ol*single(10430.37835); if th_norm<single(0.0), th_norm=single(0.0); end; if th_norm>single(65535.0), th_norm=single(65535.0); end; th_u16=uint16(th_norm);
qcprd_val=uint16(qep_qcprd); coef_val=uint16(qep_coef); cdef_val=uint16(qep_cdef); qcprdlat_val=uint16(qep_qcprdlat); qposlat_val=uint16(qep_qposlat);

tx_packet(1)=uint8(83); tx_packet(2)=uint8(170); tx_packet(3)=uint8(85);
tx_packet(4)=uint8(bitshift(id_u16,-8)); tx_packet(5)=uint8(bitand(id_u16,255)); tx_packet(6)=uint8(bitshift(iq_u16,-8)); tx_packet(7)=uint8(bitand(iq_u16,255)); tx_packet(8)=uint8(bitshift(spd_u16,-8)); tx_packet(9)=uint8(bitand(spd_u16,255));
tx_packet(10)=uint8(bitshift(aa,-8)); tx_packet(11)=uint8(bitand(aa,255)); tx_packet(12)=uint8(bitshift(ab,-8)); tx_packet(13)=uint8(bitand(ab,255)); tx_packet(14)=uint8(bitshift(spi_val,-8)); tx_packet(15)=uint8(bitand(spi_val,255)); tx_packet(16)=uint8(bitshift(qep_val,-8)); tx_packet(17)=uint8(bitand(qep_val,255));
tx_packet(18)=uint8(bitshift(ilat_val,-8)); tx_packet(19)=uint8(bitand(ilat_val,255)); tx_packet(20)=uint8(qep_dir~=0); tx_packet(21)=uint8(qep_coef~=0); tx_packet(22)=uint8(qep_cdef~=0);
tx_packet(23)=uint8(bitshift(qcprd_val,-8)); tx_packet(24)=uint8(bitand(qcprd_val,255)); tx_packet(25)=uint8(bitshift(qcprdlat_val,-8)); tx_packet(26)=uint8(bitand(qcprdlat_val,255)); tx_packet(27)=uint8(bitshift(qposlat_val,-8)); tx_packet(28)=uint8(bitand(qposlat_val,255));
tx_packet(29)=uint8(bitshift(th_u16,-8)); tx_packet(30)=uint8(bitand(th_u16,255)); tx_packet(31)=uint8(state_echo); tx_packet(32)=uint8(en_gate); tx_packet(33)=uint8(69);
end
