function spi_raw = Read_AS5047P()
%#codegen
% SPI-A AS5047P read on GPIO16/17/18 with GPIO19 chip select.
% Force the EPC91200 PWM trip before the first controller update.
persistent init_done
spi_raw = uint16(0);
if coder.target('Rtw')
    coder.cinclude('as5047p_spi.h');
    coder.cinclude('pwm_gate_control.h');
    coder.updateBuildInfo('addSourceFiles', 'as5047p_spi.c');
    coder.updateBuildInfo('addSourceFiles', 'pwm_gate_control.c');
    coder.updateBuildInfo('addIncludePaths', 'C:\\96v_gan_humanoid_drive\\phase1\\simulation\\MyFOC code');
    if isempty(init_done)
        coder.ceval('pwm_gate_control', uint16(0));
        coder.ceval('as5047p_spi_init');
        init_done = true;
    end
    coder.ceval('as5047p_spi_read', coder.ref(spi_raw));
end
end
