#ifndef AS5047P_SPI_H
#define AS5047P_SPI_H

#ifndef MATLAB_MEX_FILE
#include "F2806x_Device.h"
#include "F2806x_Examples.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

void as5047p_spi_init(void);
void as5047p_spi_read(unsigned short *pVal);

#ifdef __cplusplus
}
#endif

#endif
