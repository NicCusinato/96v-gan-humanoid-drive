#include "qep_spi_sync.h"

#ifndef MATLAB_MEX_FILE
#include "F2806x_Device.h"
#include "F2806x_Examples.h"
#endif

#define SCI_WATCHDOG_TIMEOUT_TICKS  2000U /* 2000 cycles @ 20 kHz = 100 ms */

static uint16_t s_spi_lock_val = 0;
static uint16_t s_spi_index_val = 0;
static uint16_t s_qep_seeded = 0;
static volatile uint16_t s_watchdog_ticks = 0;
static volatile uint16_t s_gate_active = 0;

static void force_all_gates_low(void)
{
#ifndef MATLAB_MEX_FILE
    EALLOW;
    /* 
     * Continuous Software Force (AQCSFRC):
     * Bits 1:0 (CSFA) = 01b (Force Low)
     * Bits 3:2 (CSFB) = 01b (Force Low)
     * Total value = 0x0005
     * Apply to both ePWM1..3 and ePWM4..6 to cover all hardware pin assignments.
     */
    EPwm1Regs.AQCSFRC.all = 0x0005;
    EPwm2Regs.AQCSFRC.all = 0x0005;
    EPwm3Regs.AQCSFRC.all = 0x0005;
    EPwm4Regs.AQCSFRC.all = 0x0005;
    EPwm5Regs.AQCSFRC.all = 0x0005;
    EPwm6Regs.AQCSFRC.all = 0x0005;

    /* Also configure Trip-Zone to force low and assert One-Shot Trip */
    EPwm1Regs.TZCTL.bit.TZA = 2U; /* 2 = Force Low */
    EPwm1Regs.TZCTL.bit.TZB = 2U;
    EPwm2Regs.TZCTL.bit.TZA = 2U;
    EPwm2Regs.TZCTL.bit.TZB = 2U;
    EPwm3Regs.TZCTL.bit.TZA = 2U;
    EPwm3Regs.TZCTL.bit.TZB = 2U;
    EPwm4Regs.TZCTL.bit.TZA = 2U;
    EPwm4Regs.TZCTL.bit.TZB = 2U;
    EPwm5Regs.TZCTL.bit.TZA = 2U;
    EPwm5Regs.TZCTL.bit.TZB = 2U;
    EPwm6Regs.TZCTL.bit.TZA = 2U;
    EPwm6Regs.TZCTL.bit.TZB = 2U;

    EPwm1Regs.TZFRC.bit.OST = 1U;
    EPwm2Regs.TZFRC.bit.OST = 1U;
    EPwm3Regs.TZFRC.bit.OST = 1U;
    EPwm4Regs.TZFRC.bit.OST = 1U;
    EPwm5Regs.TZFRC.bit.OST = 1U;
    EPwm6Regs.TZFRC.bit.OST = 1U;
    EDIS;
#endif
    s_gate_active = 0;
}

static void release_all_gates(void)
{
#ifndef MATLAB_MEX_FILE
    EALLOW;
    /* Clear One-Shot Trip flags */
    EPwm1Regs.TZCLR.bit.OST = 1U;
    EPwm2Regs.TZCLR.bit.OST = 1U;
    EPwm3Regs.TZCLR.bit.OST = 1U;
    EPwm4Regs.TZCLR.bit.OST = 1U;
    EPwm5Regs.TZCLR.bit.OST = 1U;
    EPwm6Regs.TZCLR.bit.OST = 1U;

    /* Release Continuous Software Force back to normal PWM duty cycle output */
    EPwm1Regs.AQCSFRC.all = 0x0000;
    EPwm2Regs.AQCSFRC.all = 0x0000;
    EPwm3Regs.AQCSFRC.all = 0x0000;
    EPwm4Regs.AQCSFRC.all = 0x0000;
    EPwm5Regs.AQCSFRC.all = 0x0000;
    EPwm6Regs.AQCSFRC.all = 0x0000;
    EDIS;
#endif
    s_gate_active = 1;
}

void seed_qep_from_spi(uint16_t spi_raw)
{
#ifndef MATLAB_MEX_FILE
    static uint16_t s_valid_count = 0;
    static uint16_t s_prev_spi = 0;

    /* 
     * Wait for AS5047P sensor power-on and valid SPI frames.
     * Angle must be non-zero, within 14-bit range (0..16383), and stable for 5 cycles.
     */
    if (!s_qep_seeded) {
        if (spi_raw > 0U && spi_raw <= 0x3FFFU) {
            if (s_prev_spi > 0U) {
                int16_t diff = (int16_t)spi_raw - (int16_t)s_prev_spi;
                if (diff < 0) { diff = -diff; }
                /* If stationary or nearly stationary on boot (< 50 counts change) */
                if (diff < 50) {
                    s_valid_count++;
                } else {
                    s_valid_count = 0;
                }
            }
            s_prev_spi = spi_raw;

            if (s_valid_count >= 5U) {
                /* Confirmed valid absolute shaft angle: seed eQEP hardware counter */
                uint32_t qep_count = (uint32_t)(spi_raw >> 2);
                EALLOW;
                EQep1Regs.QPOSCNT = qep_count;
                EDIS;
                s_qep_seeded = 1U;
            }
        }
    }
#else
    (void)spi_raw;
#endif
}

void record_spi_at_lock(uint16_t spi_raw)   { s_spi_lock_val = spi_raw; }
void record_spi_at_index(uint16_t spi_raw)  { s_spi_index_val = spi_raw; }
uint16_t get_spi_at_lock(void)              { return s_spi_lock_val; }
uint16_t get_spi_at_index(void)             { return s_spi_index_val; }

/*
 * Called at 20 kHz inside Current Control ISR.
 * 'enable' is passed directly from the Enable signal.
 */
void epc_inverter_gate_update(uint16_t enable)
{
#ifndef MATLAB_MEX_FILE
    if (enable == 0U) {
        /* Commanded OFF: immediately force all gates LOW */
        s_watchdog_ticks = 0U;
        if (s_gate_active) {
            force_all_gates_low();
        }
    } else {
        /* Commanded ON: release gates */
        if (!s_gate_active) {
            release_all_gates();
        }
    }
#else
    (void)enable;
#endif
}

/* Backward-compatible alias */
void epc_inverter_gate_enable(uint16_t enable)
{
    epc_inverter_gate_update(enable);
}

/*
 * Called at completion of DC alignment holding stage.
 * Resets QEP counter to 0 at the physical electrical zero alignment position,
 * and records the absolute SPI angle for reference.
 */
void align_lock_zero_qep(uint16_t spi_raw)
{
#ifndef MATLAB_MEX_FILE
    EALLOW;
    EQep1Regs.QPOSCNT = 0U;
    EDIS;
    s_spi_lock_val = spi_raw;
#else
    (void)spi_raw;
#endif
}