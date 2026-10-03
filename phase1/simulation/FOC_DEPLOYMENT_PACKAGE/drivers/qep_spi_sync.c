#include "qep_spi_sync.h"

#ifndef MATLAB_MEX_FILE
#include "F2806x_Device.h"
#include "F2806x_Examples.h"
#endif

#define SCI_WATCHDOG_TIMEOUT_TICKS  2000U /* 2000 cycles @ 20 kHz = 100 ms */

static uint16_t s_spi_lock_val = 0;
static uint16_t s_spi_index_val = 0;
static uint16_t s_qep_seeded = 0;
static uint16_t s_watchdog_ticks = 0;
static uint16_t s_gate_active = 0;

void seed_qep_from_spi(uint16_t spi_raw)
{
#ifndef MATLAB_MEX_FILE
    /* Only seed once at power-up / boot */
    if (!s_qep_seeded) {
        /* Map 14-bit absolute SPI (0..16383) to 12-bit eQEP (0..4095) */
        /* Ratio = 16384 / 4096 = 4. Shift right by 2. */
        uint32_t qep_count = (uint32_t)(spi_raw >> 2);
        
        EALLOW;
        EQep1Regs.QPOSCNT = qep_count;
        EDIS;
        
        s_qep_seeded = 1;
    }
#else
    (void)spi_raw;
#endif
}

void record_spi_at_lock(uint16_t spi_raw)
{
    s_spi_lock_val = spi_raw;
}

void record_spi_at_index(uint16_t spi_raw)
{
    s_spi_index_val = spi_raw;
}

uint16_t get_spi_at_lock(void)
{
    return s_spi_lock_val;
}

uint16_t get_spi_at_index(void)
{
    return s_spi_index_val;
}

void epc_inverter_gate_enable(uint16_t enable)
{
#ifndef MATLAB_MEX_FILE
    if (enable == 1U) {
        /* Active enable commanded from host: reset watchdog timer */
        s_watchdog_ticks = 0;
        if (!s_gate_active) {
            s_gate_active = 1;
            EALLOW;
            EPwm4Regs.TZCLR.bit.OST = 1U;
            EPwm5Regs.TZCLR.bit.OST = 1U;
            EPwm6Regs.TZCLR.bit.OST = 1U;
            EDIS;
        }
    } else {
        /* Commanded OFF: Trip immediately and mark watchdog expired */
        s_watchdog_ticks = SCI_WATCHDOG_TIMEOUT_TICKS;
        if (s_gate_active) {
            s_gate_active = 0;
            EALLOW;
            EPwm4Regs.TZCTL.bit.TZA = 2U; /* Force Low */
            EPwm4Regs.TZCTL.bit.TZB = 2U; /* Force Low */
            EPwm5Regs.TZCTL.bit.TZA = 2U;
            EPwm5Regs.TZCTL.bit.TZB = 2U;
            EPwm6Regs.TZCTL.bit.TZA = 2U;
            EPwm6Regs.TZCTL.bit.TZB = 2U;
            EPwm4Regs.TZFRC.bit.OST = 1U;
            EPwm5Regs.TZFRC.bit.OST = 1U;
            EPwm6Regs.TZFRC.bit.OST = 1U;
            EDIS;
        }
    }

    /* Communication Watchdog: check if host stopped sending */
    if (s_gate_active) {
        if (++s_watchdog_ticks >= SCI_WATCHDOG_TIMEOUT_TICKS) {
            /* 100 ms elapsed with no fresh enable packet -> clamp gates OFF */
            s_gate_active = 0;
            EALLOW;
            EPwm4Regs.TZCTL.bit.TZA = 2U;
            EPwm4Regs.TZCTL.bit.TZB = 2U;
            EPwm5Regs.TZCTL.bit.TZA = 2U;
            EPwm5Regs.TZCTL.bit.TZB = 2U;
            EPwm6Regs.TZCTL.bit.TZA = 2U;
            EPwm6Regs.TZCTL.bit.TZB = 2U;
            EPwm4Regs.TZFRC.bit.OST = 1U;
            EPwm5Regs.TZFRC.bit.OST = 1U;
            EPwm6Regs.TZFRC.bit.OST = 1U;
            EDIS;
        }
    }
#else
    (void)enable;
#endif
}