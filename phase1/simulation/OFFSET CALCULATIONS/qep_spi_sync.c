#include "qep_spi_sync.h"

#ifndef MATLAB_MEX_FILE
#include "F2806x_Device.h"
#include "F2806x_Examples.h"
#endif

static uint16_t s_spi_lock_val = 0;
static uint16_t s_spi_index_val = 0;
static uint16_t s_qep_seeded = 0;

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
