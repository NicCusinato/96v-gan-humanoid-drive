#include <stdint.h>
#include "as5047p_spi.h"
#include "qep_spi_sync.h"

static unsigned short spi_transfer_word(unsigned short tx_cmd);

void as5047p_spi_init(void)
{
#ifndef MATLAB_MEX_FILE
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.SPIAENCLK = 1;

    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;

    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1; /* SIMO / MOSI */
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1; /* SOMI / MISO */
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1; /* CLK         */
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0; /* GPIO CSn, LaunchXL J2 pin 19 */

    GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3;

    SpiaRegs.SPICCR.bit.SPISWRESET = 0;
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0; /* Clock idles low */
    SpiaRegs.SPICCR.bit.SPICHAR = 15;    /* 16-bit word */
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0;   /* Normal phase: output on rising edge, input on falling edge */
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;
    SpiaRegs.SPICTL.bit.TALK = 1;
    SpiaRegs.SPIBRR = 19;                /* ~1.125 - 2.25 MHz */
    SpiaRegs.SPIFFTX.all = 0xE040;
    SpiaRegs.SPIFFRX.all = 0x2044;
    SpiaRegs.SPIFFCT.all = 0x0;
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;
    SpiaRegs.SPIPRI.bit.FREE = 1;
    EDIS;

    /* Prime pipeline and clear any power-on error flags */
    spi_transfer_word(0x4001); /* Read ERRFL */
    spi_transfer_word(0xFFFF); /* Read ANGLECOM */

    /* Ensure inverter GaN gates start tri-stated (forced low) */
    epc_inverter_gate_enable(0);
#endif
}

static unsigned short spi_transfer_word(unsigned short tx_cmd)
{
    unsigned short rx_val = 0;
#ifndef MATLAB_MEX_FILE
    unsigned short timeout = 5000;
    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    asm(" RPT #50 || NOP");
    SpiaRegs.SPITXBUF = tx_cmd;
    while ((SpiaRegs.SPIFFRX.bit.RXFFST == 0) && (timeout > 0)) { timeout--; }
    if (timeout > 0) { rx_val = SpiaRegs.SPIRXBUF; }
    asm(" RPT #50 || NOP");
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    asm(" RPT #80 || NOP");
#endif
    return rx_val;
}

void as5047p_spi_read(unsigned short *pVal)
{
#ifndef MATLAB_MEX_FILE
    unsigned short raw = spi_transfer_word(0xFFFF);
    if (raw & 0x4000) {
        /* Error flag set: clear ERRFL on next cycle */
        spi_transfer_word(0x4001);
    }
    if (pVal) {
        *pVal = (raw & 0x3FFF);
    }
#else
    if (pVal) *pVal = 0;
#endif
}

