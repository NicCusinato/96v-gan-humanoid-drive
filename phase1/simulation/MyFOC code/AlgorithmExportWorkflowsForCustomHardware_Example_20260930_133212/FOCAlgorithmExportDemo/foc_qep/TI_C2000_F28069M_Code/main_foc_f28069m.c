/**
 * ============================================================================
 * @file    main_foc_f28069m.c
 * @brief   Workflow 3: Dual-Rate Closed-Loop Field-Oriented Control (FOC)
 * @target  TI TMS320F28069M LaunchPad (BoosterPack Site 2: J5-J8)
 * @inverter EPC9147B Interface + EPC91200 GaN Inverter
 * @motor   CubeMars AKE80-8 KV30 + AS5047P (1024 slits -> 4096 counts/rev)
 * ============================================================================
 */

#include "F2806x_Device.h"
#include "F2806x_Examples.h"
#include "current_control_algorithm.h"
#include "speed_control_algorithm.h"
#include <string.h>

// --- Global Control Variables ---
volatile uint16_t ENABLE_INV     = 0;          // Inverter enable (host switch)
volatile uint16_t ENABLE_CL_HOST = 0;          // Closed-loop request (host switch)
volatile float    SPEED_REF_RPM  = 0.0f;       // Commanded speed in mechanical RPM
volatile float    SPEED_MEAS_PU  = 0.0f;       // Measured speed in per-unit
volatile boolean_T CL_ENABLE     = 0;          // Internal closed-loop active flag
volatile float    IDQ_REF[2]     = {0.0f, 0.0f};// D-Q current commands from speed loop

real32_T duty_cycles[3]   = {0.0f, 0.0f, 0.0f};// 3-phase PWM duty cycles [0.0, 1.0]
real32_T Iab_meas_pu[2]   = {0.0f, 0.0f};      // Filtered phase currents in PU
real32_T pos_meas         = 0.0f;              // Electrical angle in PU [0.0, 1.0]

// Telemetry Buffers (Host Communication at 1.25 MBaud)
volatile uint8_t rx_buffer[12];
volatile uint16_t sample_count = 0;

// Function Prototypes
void Init_EPWM_GaN(void);
void Init_ADC_CurrentSense(void);
void Init_EQEP1(void);
void Init_CpuTimer0_SpeedLoop(void);
void Init_SCIA_Host(void);
interrupt void adc_isr(void);
interrupt void cpu_timer0_isr(void);

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
    PieVectTable.TINT0   = &cpu_timer0_isr;

    // Diagnostic Pin (GPIO12) & Gate Driver Enable Pin (GPIO52)
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO12  = 1;
    GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO52  = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
    EDIS;

    // 1. Initialize Embedded Coder FOC Algorithm Modules
    current_control_algorithm_initialize();
    speed_control_algorithm_initialize();

    // 2. Initialize Peripherals
    Init_EPWM_GaN();
    Init_ADC_CurrentSense();
    Init_EQEP1();
    Init_CpuTimer0_SpeedLoop();
    Init_SCIA_Host();

    // 3. Enable Interrupts
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // ADCINT1 in PIE Group 1
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1; // TINT0 in PIE Group 1
    IER |= M_INT1;                     // Enable CPU INT1
    EINT;
    ERTM;

    // Background Host Communication Loop
    while (1)
    {
        // Receive 3 floats (12 bytes): [ENABLE_INV, ENABLE_CL_HOST, SPEED_REF_RPM]
        if (SciaRegs.SCIFFRX.bit.RXFFST >= 12)
        {
            for (int i = 0; i < 12; i++) {
                rx_buffer[i] = SciaRegs.SCIRXBUF.all;
            }
            float en_inv_f = *(float *)(rx_buffer);
            float en_cl_f  = *(float *)(rx_buffer + 4);
            SPEED_REF_RPM  = *(float *)(rx_buffer + 8);

            ENABLE_INV     = (uint16_t)en_inv_f;
            ENABLE_CL_HOST = (uint16_t)en_cl_f;

            if (ENABLE_INV) {
                GpioDataRegs.GPBSET.bit.GPIO52 = 1;
            } else {
                GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
            }
        }
    }
}

/**
 * ============================================================================
 * FAST TASK: High-Frequency Current Loop ISR (20 kHz / 50 us Period)
 * Triggered by ADCINT1 when SOC0 (Phase U) and SOC1 (Phase V) complete.
 * ============================================================================
 */
interrupt void adc_isr(void)
{
    GpioDataRegs.GPASET.bit.GPIO12 = 1; // Profiling pulse start

    // 1. Read Raw 12-Bit ADC Conversions
    uint16_t Ia_raw = AdcResult.ADCRESULT0; // ADCINA3
    uint16_t Ib_raw = AdcResult.ADCRESULT1; // ADCINB3

    // 2. Read eQEP1 Counter and Hardware Index Latch
    uint16_t qep_count = (uint16_t)EQep1Regs.QPOSCNT;
    uint16_t qep_index = (uint16_t)EQep1Regs.QPOSILAT;

    // 3. Execute Current Control Step Function
    current_control_algorithm_step(
        (boolean_T)ENABLE_INV,
        (boolean_T)ENABLE_CL_HOST,
        (uint32_T)Ia_raw,
        (uint32_T)Ib_raw,
        qep_index,
        qep_count,
        SPEED_REF_RPM,
        (real32_T *)IDQ_REF,
        (real32_T *)&SPEED_MEAS_PU,
        duty_cycles,
        (boolean_T *)&CL_ENABLE,
        &pos_meas,
        Iab_meas_pu
    );

    // 4. Update ePWM Compare Registers (TBPRD = 2250)
    EPwm4Regs.CMPA.half.CMPA = (uint16_t)(duty_cycles[0] * EPwm4Regs.TBPRD);
    EPwm5Regs.CMPA.half.CMPA = (uint16_t)(duty_cycles[1] * EPwm5Regs.TBPRD);
    EPwm6Regs.CMPA.half.CMPA = (uint16_t)(duty_cycles[2] * EPwm6Regs.TBPRD);

    // 5. High-Speed Telemetry Streaming to Host PC
    uint16_t debug1 = (uint16_t)(((Iab_meas_pu[0] + 1.0f) * 0.5f) * 65535.0f);
    uint16_t debug2 = (uint16_t)(((SPEED_MEAS_PU  + 1.0f) * 0.5f) * 65535.0f);

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

    GpioDataRegs.GPACLEAR.bit.GPIO12 = 1; // Profiling pulse end
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

/**
 * ============================================================================
 * SLOW TASK: Speed Loop ISR (2 kHz / 500 us Period)
 * Triggered by CPUTimer0.
 * ============================================================================
 */
interrupt void cpu_timer0_isr(void)
{
    // Compute speed reference in PU (normalized against base mechanical speed)
    // CubeMars AKE80-8 KV30 N_base = 1116 RPM
    real32_T speed_ref_pu = SPEED_REF_RPM / 1116.0f;

    // Call Exported Speed Controller Step Function
    speed_control_algorithm_step(
        (boolean_T)ENABLE_INV,
        (boolean_T)CL_ENABLE,
        speed_ref_pu,
        SPEED_MEAS_PU,
        (real32_T *)IDQ_REF
    );

    CpuTimer0Regs.TCR.bit.TIF = 1; // Clear timer flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void Init_CpuTimer0_SpeedLoop(void)
{
    InitCpuTimers();
    // 90 MHz SYSCLK, 500 us period (2 kHz)
    ConfigCpuTimer(&CpuTimer0, 90, 500);
    CpuTimer0Regs.TCR.all = 0x4001; // Enable interrupt and start timer
}

void Init_EQEP1(void)
{
    EALLOW;
    SysCtrlRegs.PCLKCR1.bit.EQEP1ENCLK = 1;

    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 1; // EQEP1A
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 1; // EQEP1B
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 1; // EQEP1I (Index)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 0;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 0;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 0;
    EDIS;

    EQep1Regs.QDECCTL.bit.QSRC = 0;      // 4x Quadrature count mode
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;
    EQep1Regs.QEPCTL.bit.PCRM = 0;
    EQep1Regs.QPOSMAX = 4095;            // 1024 slits * 4 counts/slit - 1 = 4095
    EQep1Regs.QEPCTL.bit.IEL = 1;        // Hardware index latch into QPOSILAT
    EQep1Regs.QEPCTL.bit.QPEN = 1;       // Enable counter
}

void Init_EPWM_GaN(void)
{
    EALLOW;
    SysCtrlRegs.PCLKCR1.bit.EPWM4ENCLK = 1;
    SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK = 1;
    SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK = 1;

    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;  // EPWM4A
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;  // EPWM4B
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;  // EPWM5A
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;  // EPWM5B
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1; // EPWM6A
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1; // EPWM6B
    EDIS;

    volatile struct EPWM_REGS *epwm_list[3] = {&EPwm4Regs, &EPwm5Regs, &EPwm6Regs};

    for (int i = 0; i < 3; i++) {
        volatile struct EPWM_REGS *p = epwm_list[i];
        p->TBPRD = 2250;                     // 20 kHz carrier @ 90 MHz
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

        // Active High Complementary (AHC) deadband for EPC GaN FETs
        p->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
        p->DBCTL.bit.POLSEL = DB_ACTV_HIC;
        p->DBCTL.bit.IN_MODE = DBA_ALL;
        p->DBRED = 5;                        // 55.5 ns for EPC GaN
        p->DBFED = 5;
    }

    EPwm4Regs.ETSEL.bit.SOCAEN = 1;
    EPwm4Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO; // Trigger ADC on valley of carrier
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

    AdcRegs.ADCSOC0CTL.bit.CHSEL = 3;    // ADCINA3 (Phase U)
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 11; // ePWM4 SOCA
    AdcRegs.ADCSOC0CTL.bit.ACQPS = 6;

    AdcRegs.ADCSOC1CTL.bit.CHSEL = 11;   // ADCINB3 (Phase V)
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
    SciaRegs.SCIFFRX.all = 0x002C;       // FIFO depth = 12 bytes
    SciaRegs.SCICTL1.all = 0x0023;
}
