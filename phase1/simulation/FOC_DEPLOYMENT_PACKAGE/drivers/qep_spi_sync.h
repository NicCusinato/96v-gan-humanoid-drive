#ifndef QEP_SPI_SYNC_H
#define QEP_SPI_SYNC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize QEP counter from SPI absolute reading on bootup */
void seed_qep_from_spi(uint16_t spi_raw);

/* Latches captured during calibration */
void record_spi_at_lock(uint16_t spi_raw);
void record_spi_at_index(uint16_t spi_raw);
uint16_t get_spi_at_lock(void);
uint16_t get_spi_at_index(void);

/* EPC GaN Inverter Trip-Zone gate driver interlock */
void epc_inverter_gate_enable(uint16_t enable);

#ifdef __cplusplus
}
#endif

#endif /* QEP_SPI_SYNC_H */
