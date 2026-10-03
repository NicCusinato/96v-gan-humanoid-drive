/**
 * ============================================================================
 * @file    main_open_loop_f28069m.c
 * @brief   Workflow 1: Open-Loop Scalar V/f & ADC Offset Calibration
 * @target  TI TMS320F28069M LaunchPad (BoosterPack Site 2: J5-J8)
 * @inverter EPC9147B Interface + EPC91200 GaN Inverter
 * @motor   CubeMars AKE80-8 KV30 (21 Pole Pairs)
 * ============================================================================
 */

#include "F2806x_Device.h"
#include "F2806x_Examples.h"
#include "open_loop_algorithm.h"
#include <string.h>

// --- Global Control Variables ---
volatile uint16_t ENABLE_INV = 0;              // Commanded by Host over SCI
volatile float SPEED_REF_RPM = 60.0f;          // Initial safe test speed (60 RPM)
const float VOLTAGE_AMP_LOW_LIMIT = 0.15f;     // Minimum V/f boost voltage in PU

// Telemetry Buffers (Host Communication at 1.25 MBaud)
volatile uint8_t tx_buffer[6];
volatile uint8_t rx_buffer[8];
volatile uint16_t sample_count = 0;

// Duty cycle outputs from algorithm in PU [0.0, 1.0]
real32_T duty_cycles[3] = {0.0f, 0.0f, 0.0f};

// --- Function Prototypes ---
void Device_Init(void);
void Init_EPWM_GaN(void);
void Init_ADC_CurrentSense(void);
void Init_SCIA_Host(void);
interrupt void adc_isr(void);

int main(void)
{
    // 1. Initialize System Control (90 MHz SYSCLK, PLL, Clocks)
    InitSysCtrl();

    // 2. Initialize GPIOs
    InitGpio();
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    // Remap ADCINT1 to local ISR
    EALLOW;
    PieVectTable.ADCINT1 = &adc_isr;
    EDIS;

    // Diagnostic Profiling Pin (GPIO12)
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;  // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;   // Output
    // Gate Driver Enable Pin (GPIO52: Active High on EPC9147B)
    GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;  // GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO52 = 1;   // Output
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1; // Disabled initially
    EDIS;

    // 3. Initialize Embedded Coder Open-Loop Algorithm
    open_loop_algorithm_initialize();

    // 4. Initialize Peripherals
    Init_EPWM_GaN();
    Init_ADC_CurrentSense();
    Init_SCIA_Host();

    // 5. Enable PIE & CPU Interrupts
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;   // Enable ADCINT1 in PIE Group 1
    IER |= M_INT1;                       // Enable CPU INT1
    EINT;                                // Global interrupt enable
    ERTM;

    // Background loop
    while (1)
    {
        // Handle incoming SCI host commands
        if (SciaRegs.SCIFFRX.bit.RXFFST >= 8)
        {
            for (int i = 0; i < 8; i++) {
                rx_buffer[i] = SciaRegs.SCIRXBUF.all;
            }
            float enable_f = *(float *)(rx_buffer);
            ENABLE_INV = (uint16_t)enable_f;
            SPEED_REF_RPM = *(float *)(rx_buffer + 4);

            // Update physical Gate Driver Enable line (GPIO52)
            if (ENABLE_INV) {
                GpioDataRegs.GPBSET.bit.GPIO52 = 1;
            } else {
                GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
            }
        }
    }
}

/**
 * High-Frequency Current Loop ISR (20 kHz / 50 us Period)
 * Triggered by ADCINT1 when SOC0 (Phase U) and SOC1 (Phase V) complete conversion.
 */
interrupt void adc_isr(void)
{
    // Profile execution latency using oscilloscope on GPIO12
    GpioDataRegs.GPASET.bit.GPIO12 = 1;

    // 1. Read Raw 12-Bit ADC Conversion Values
    uint16_t Ia_raw = AdcResult.ADCRESULT0; // ADCINA3
    uint16_t Ib_raw = AdcResult.ADCRESULT1; // ADCINB3

    // 2. Call Exported Open-Loop Control Algorithm Step Function
    open_loop_algorithm_step(
        (boolean_T)ENABLE_INV,
        VOLTAGE_AMP_LOW_LIMIT,
        SPEED_REF_RPM,
        duty_cycles
    );

    // 3. Update ePWM Compare Registers (Scale PU [0.0, 1.0] to TBPRD = 2250)
    EPwm4Regs.CMPA.half.CMPA = (uint16_t)(duty_cycles[0] * EPwm4Regs.TBPRD);
    EPwm5Regs.CMPA.half.CMPA = (uint16_t)(duty_cycles[1] * EPwm5Regs.TBPRD);
    EPwm6Regs.CMPA.half.CMPA = (uint16_t)(duty_cycles[2] * EPwm6Regs.TBPRD);

    // 4. High-Speed Serial Monitoring Stream to Host PC
    if (sample_count == 0) {
        // Frame Header ('ss' = 0x73 0x73)
        while (SciaRegs.SCIFFTX.bit.TXFFST > 10);
        SciaRegs.SCITXBUF = 's';
        SciaRegs.SCITXBUF = 's';
        SciaRegs.SCITXBUF = (Ia_raw & 0xFF);
        SciaRegs.SCITXBUF = ((Ia_raw >> 8) & 0xFF);
        SciaRegs.SCITXBUF = (Ib_raw & 0xFF);
        SciaRegs.SCITXBUF = ((Ib_raw >> 8) & 0xFF);
        sample_count++;
    } else if (sample_count == 199) {
        // Frame Footer ('ee' = 0x65 0x65)
        while (SciaRegs.SCIFFTX.bit.TXFFST > 10);
        SciaRegs.SCITXBUF = (Ia_raw & 0xFF);
        SciaRegs.SCITXBUF = ((Ia_raw >> 8) & 0xFF);
        SciaRegs.SCITXBUF = (Ib_raw & 0xFF);
        SciaRegs.SCITXBUF = ((Ib_raw >> 8) & 0xFF);
        SciaRegs.SCITXBUF = 'e';
        SciaRegs.SCITXBUF = 'e';
        sample_count = 0;
    } else {
        if (SciaRegs.SCIFFTX.bit.TXFFST <= 12) {
            SciaRegs.SCITXBUF = (Ia_raw & 0xFF);
            SciaRegs.SCITXBUF = ((Ia_raw >> 8) & 0xFF);
            SciaRegs.SCITXBUF = (Ib_raw & 0xFF);
            SciaRegs.SCITXBUF = ((Ib_raw >> 8) & 0xFF);
            sample_count++;
        }
    }

    // Clear flags & acknowledge interrupt
    GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

/**
 * Configure ePWM4, ePWM5, ePWM6 for 20 kHz Center-Aligned PWM with GaN Dead-Band
 */
void Init_EPWM_GaN(void)
{
    EALLOW;
    SysCtrlRegs.PCLKCR1.bit.EPWM4ENCLK = 1;
    SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK = 1;
    SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK = 1;

    // GPIO muxing for Site 2 (GPIO6..GPIO11)
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
        p->TBPRD = 2250;                     // 90 MHz / (2 * 20 kHz) = 2250
        p->TBPHS.half.TBPHS = 0x0000;
        p->TBCTR = 0x0000;
        p->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Center-aligned
        p->TBCTL.bit.PHSEN = (i == 0) ? TB_DISABLE : TB_ENABLE;
        p->TBCTL.bit.PRDLD = TB_SHADOW;
        p->TBCTL.bit.SYNCOSEL = (i == 0) ? TB_CTR_ZERO : TB_SYNC_IN;
        p->TBCTL.bit.HSPCLKDIV = TB_DIV1;
        p->TBCTL.bit.CLKDIV = TB_DIV1;

        p->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
        p->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

        // Action Qualifier: PWM Mode 1
        p->AQCTLA.bit.CAU = AQ_SET;
        p->AQCTLA.bit.CAD = AQ_CLEAR;

        // Active High Complementary (AHC) Dead-Band for EPC GaN FETs
        p->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
        p->DBCTL.bit.POLSEL = DB_ACTV_HIC;  // EPWMxB inverted
        p->DBCTL.bit.IN_MODE = DBA_ALL;
        p->DBRED = 5;                        // 5 cycles * 11.1 ns = 55.5 ns
        p->DBFED = 5;
    }

    // Trigger ADC SOCA on ePWM4 Counter Underflow (TBCTR = 0, valley of PWM)
    EPwm4Regs.ETSEL.bit.SOCAEN = 1;
    EPwm4Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;
    EPwm4Regs.ETPS.bit.SOCAPRD = ET_1ST;
}

/**
 * Configure ADC-A & ADC-B for Injected Current Sensing on Phase A and Phase B
 */
void Init_ADC_CurrentSense(void)
{
    InitAdc();
    AdcOffsetSelfCal();

    EALLOW;
    AdcRegs.ADCCTL1.bit.ADCREFSEL = 0;   // Internal reference 3.3V
    AdcRegs.ADCCTL1.bit.ADCBGPWD = 1;
    AdcRegs.ADCCTL1.bit.ADCPWDN = 1;
    AdcRegs.ADCCTL1.bit.ADCCLKPS = 0;
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1; // Late interrupt pulse

    // SOC0: Phase U on ADCINA3
    AdcRegs.ADCSOC0CTL.bit.CHSEL = 3;    // ADCINA3
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 11; // ePWM4 SOCA
    AdcRegs.ADCSOC0CTL.bit.ACQPS = 6;    // 7 ADC clock cycles acquisition

    // SOC1: Phase V on ADCINB3
    AdcRegs.ADCSOC1CTL.bit.CHSEL = 11;   // ADCINB3
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 11; // ePWM4 SOCA
    AdcRegs.ADCSOC1CTL.bit.ACQPS = 6;

    // Trigger ADCINT1 on EOC1
    AdcRegs.INTSEL1N2.bit.INT1SEL = 1;   // EOC1 triggers ADCINT1
    AdcRegs.INTSEL1N2.bit.INT1E = 1;     // Enable ADCINT1
    AdcRegs.INTSEL1N2.bit.INT1CONT = 0;
    EDIS;
}

/**
 * Configure SCI-A for Host Telemetry at 1.25 MBaud
 */
void Init_SCIA_Host(void)
{
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1; // SCIRXDA
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1; // SCITXDA
    EDIS;

    SciaRegs.SCICCR.all = 0x0007;        // 1 stop bit, No parity, 8 char bits
    SciaRegs.SCICTL1.all = 0x0003;       // Enable TX, RX, internal SCICLK
    // 90 MHz LSPCLK / (8 * 1.25M) - 1 = 8 counts for BRR
    SciaRegs.SCIHBAUD = 0x0000;
    SciaRegs.SCILBAUD = 0x0008;

    SciaRegs.SCIFFTX.all = 0xC020;       // Enable FIFO
    SciaRegs.SCIFFRX.all = 0x0028;       // FIFO depth = 8
    SciaRegs.SCICTL1.all = 0x0023;       // Relinquish SCI from Reset
}
