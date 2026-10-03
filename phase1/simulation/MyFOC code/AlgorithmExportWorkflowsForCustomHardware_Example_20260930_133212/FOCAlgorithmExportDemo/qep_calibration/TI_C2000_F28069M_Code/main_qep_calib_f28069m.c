/**
 * ============================================================================
 * @file    main_qep_calib_f28069m.c
 * @brief   Workflow 2: Quadrature Encoder (QEP) Offset Calibration
 * @target  TI TMS320F28069M LaunchPad (BoosterPack Site 2: J5-J8)
 * @inverter EPC9147B Interface + EPC91200 GaN Inverter
 * @motor   CubeMars AKE80-8 KV30 + AS5047P (1024 slits -> 4096 counts/rev)
 * ============================================================================
 */

#include "F2806x_Device.h"
#include "F2806x_Examples.h"
#include "qep_calibration_algorithm.h"
#include <string.h>

// --- Global Variables ---
volatile uint16_t ENABLE_INV = 0;              // Inverter enable sent from host
real32_T duty_cycles[3] = {0.0f, 0.0f, 0.0f};  // 3-phase PWM duty cycles [0.0, 1.0]
real32_T calib_mode = 0.0f;                    // State machine mode: 0 (lock), 1 (ramp)
real32_T pos_meas_pu = 0.0f;                   // Position measured in PU [0.0, 1.0]

// Telemetry Buffers (Host Communication at 1.25 MBaud)
volatile uint8_t rx_buffer[8];
volatile uint16_t sample_count = 0;

// Function Prototypes
void Init_EPWM_GaN(void);
void Init_ADC_CurrentSense(void);
void Init_EQEP1(void);
void Init_SCIA_Host(void);
interrupt void adc_isr(void);

int main(void)
{
    InitSysCtrl();
    InitGpio();
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    EALLOW;
    PieVectTable.ADCINT1 = &adc_isr;
    // Diagnostic Pin (GPIO12) & Gate Enable Pin (GPIO52)
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;
    GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO52 = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
    EDIS;

    // Initialize Embedded Coder QEP Calibration Algorithm
    qep_calibration_algorithm_initialize();

    Init_EPWM_GaN();
    Init_ADC_CurrentSense();
    Init_EQEP1();
    Init_SCIA_Host();

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    IER |= M_INT1;
    EINT;
    ERTM;

    while (1)
    {
        // Receive host command (Enable flag float)
        if (SciaRegs.SCIFFRX.bit.RXFFST >= 4)
        {
            for (int i = 0; i < 4; i++) {
                rx_buffer[i] = SciaRegs.SCIRXBUF.all;
            }
            float en_f = *(float *)rx_buffer;
            ENABLE_INV = (uint16_t)en_f;

            if (ENABLE_INV) {
                GpioDataRegs.GPBSET.bit.GPIO52 = 1;
            } else {
                GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
            }
        }
    }
}

/**
 * ADCINT1 ISR (20 kHz / 50 us Period)
 */
interrupt void adc_isr(void)
{
    GpioDataRegs.GPASET.bit.GPIO12 = 1;

    // 1. Read eQEP1 Counter & Latched Index Values
    uint16_t qep_count = (uint16_t)EQep1Regs.QPOSCNT;
    uint16_t qep_index = (uint16_t)EQep1Regs.QPOSILAT;

    // 2. Call Exported QEP Calibration Algorithm Step Function
    qep_calibration_algorithm_step(
        (boolean_T)ENABLE_INV,
        qep_index,
        qep_count,
        duty_cycles,
        &calib_mode,
        &pos_meas_pu
    );

    // 3. Update ePWM Compare Registers (TBPRD = 2250)
    EPwm4Regs.CMPA.half.CMPA = (uint16_t)(duty_cycles[0] * EPwm4Regs.TBPRD);
    EPwm5Regs.CMPA.half.CMPA = (uint16_t)(duty_cycles[1] * EPwm5Regs.TBPRD);
    EPwm6Regs.CMPA.half.CMPA = (uint16_t)(duty_cycles[2] * EPwm6Regs.TBPRD);

    // 4. Stream Telemetry to Host PC (position_meas and mode)
    uint16_t debug1 = (uint16_t)(pos_meas_pu * 65535.0f);
    uint16_t debug2 = (uint16_t)calib_mode;

    if (sample_count == 0) {
        while (SciaRegs.SCIFFTX.bit.TXFFST > 10);
        SciaRegs.SCITXBUF = 's';
        SciaRegs.SCITXBUF = 's';
        SciaRegs.SCITXBUF = (debug1 & 0xFF);
        SciaRegs.SCITXBUF = ((debug1 >> 8) & 0xFF);
        SciaRegs.SCITXBUF = (debug2 & 0xFF);
        SciaRegs.SCITXBUF = ((debug2 >> 8) & 0xFF);
        sample_count++;
    } else if (sample_count == 199) {
        while (SciaRegs.SCIFFTX.bit.TXFFST > 10);
        SciaRegs.SCITXBUF = (debug1 & 0xFF);
        SciaRegs.SCITXBUF = ((debug1 >> 8) & 0xFF);
        SciaRegs.SCITXBUF = (debug2 & 0xFF);
        SciaRegs.SCITXBUF = ((debug2 >> 8) & 0xFF);
        SciaRegs.SCITXBUF = 'e';
        SciaRegs.SCITXBUF = 'e';
        sample_count = 0;
    } else {
        if (SciaRegs.SCIFFTX.bit.TXFFST <= 12) {
            SciaRegs.SCITXBUF = (debug1 & 0xFF);
            SciaRegs.SCITXBUF = ((debug1 >> 8) & 0xFF);
            SciaRegs.SCITXBUF = (debug2 & 0xFF);
            SciaRegs.SCITXBUF = ((debug2 >> 8) & 0xFF);
            sample_count++;
        }
    }

    GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

/**
 * Configure eQEP1 Module for Quadrature Decoding and Index Latch
 */
void Init_EQEP1(void)
{
    EALLOW;
    SysCtrlRegs.PCLKCR1.bit.EQEP1ENCLK = 1;

    // GPIO20: EQEP1A, GPIO21: EQEP1B, GPIO23: EQEP1I (Index)
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 1;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 0; // Sync to SYSCLK
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 0;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 0;
    EDIS;

    // Quadrature 4x count mode
    EQep1Regs.QDECCTL.bit.QSRC = 0;      // Quadrature count mode
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;  // Unaffected by emulation suspend
    EQep1Regs.QEPCTL.bit.PCRM = 0;       // Position counter reset on max position
    EQep1Regs.QPOSMAX = 4095;            // 1024 lines * 4 counts/line - 1 = 4095

    // Latch position counter into QPOSILAT upon Index Event (IEL = 1)
    EQep1Regs.QEPCTL.bit.IEL = 1;        // Latch on rising edge of index
    EQep1Regs.QEPCTL.bit.QPEN = 1;       // Enable QEP position counter
}

void Init_EPWM_GaN(void)
{
    EALLOW;
    SysCtrlRegs.PCLKCR1.bit.EPWM4ENCLK = 1;
    SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK = 1;
    SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK = 1;

    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;
    EDIS;

    volatile struct EPWM_REGS *epwm_list[3] = {&EPwm4Regs, &EPwm5Regs, &EPwm6Regs};

    for (int i = 0; i < 3; i++) {
        volatile struct EPWM_REGS *p = epwm_list[i];
        p->TBPRD = 2250;
        p->TBPHS.half.TBPHS = 0x0000;
        p->TBCTR = 0x0000;
        p->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
        p->TBCTL.bit.PHSEN = (i == 0) ? TB_DISABLE : TB_ENABLE;
        p->TBCTL.bit.PRDLD = TB_SHADOW;
        p->TBCTL.bit.SYNCOSEL = (i == 0) ? TB_CTR_ZERO : TB_SYNC_IN;
        p->TBCTL.bit.HSPCLKDIV = TB_DIV1;
        p->TBCTL.bit.CLKDIV = TB_DIV1;

        p->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
        p->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

        p->AQCTLA.bit.CAU = AQ_SET;
        p->AQCTLA.bit.CAD = AQ_CLEAR;

        p->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
        p->DBCTL.bit.POLSEL = DB_ACTV_HIC;
        p->DBCTL.bit.IN_MODE = DBA_ALL;
        p->DBRED = 5;                        // 55.5 ns for EPC GaN
        p->DBFED = 5;
    }

    EPwm4Regs.ETSEL.bit.SOCAEN = 1;
    EPwm4Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;
    EPwm4Regs.ETPS.bit.SOCAPRD = ET_1ST;
}

void Init_ADC_CurrentSense(void)
{
    InitAdc();
    AdcOffsetSelfCal();

    EALLOW;
    AdcRegs.ADCCTL1.bit.ADCREFSEL = 0;
    AdcRegs.ADCCTL1.bit.ADCBGPWD = 1;
    AdcRegs.ADCCTL1.bit.ADCPWDN = 1;
    AdcRegs.ADCCTL1.bit.ADCCLKPS = 0;
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    AdcRegs.ADCSOC0CTL.bit.CHSEL = 3;    // ADCINA3
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 11; // ePWM4 SOCA
    AdcRegs.ADCSOC0CTL.bit.ACQPS = 6;

    AdcRegs.ADCSOC1CTL.bit.CHSEL = 11;   // ADCINB3
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 11;
    AdcRegs.ADCSOC1CTL.bit.ACQPS = 6;

    AdcRegs.INTSEL1N2.bit.INT1SEL = 1;
    AdcRegs.INTSEL1N2.bit.INT1E = 1;
    AdcRegs.INTSEL1N2.bit.INT1CONT = 0;
    EDIS;
}

void Init_SCIA_Host(void)
{
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;
    EDIS;

    SciaRegs.SCICCR.all = 0x0007;
    SciaRegs.SCICTL1.all = 0x0003;
    SciaRegs.SCIHBAUD = 0x0000;
    SciaRegs.SCILBAUD = 0x0008;          // 1.25 MBaud

    SciaRegs.SCIFFTX.all = 0xC020;
    SciaRegs.SCIFFRX.all = 0x0028;
    SciaRegs.SCICTL1.all = 0x0023;
}
