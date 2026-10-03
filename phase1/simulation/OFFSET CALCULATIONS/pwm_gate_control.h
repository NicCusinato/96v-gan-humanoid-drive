#ifndef PWM_GATE_CONTROL_H
#define PWM_GATE_CONTROL_H

#ifndef MATLAB_MEX_FILE
#include "F2806x_Device.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* enable == 0 latches ePWM4/5/6 outputs low; nonzero releases the trip. */
void pwm_gate_control(unsigned short enable);

#ifdef __cplusplus
}
#endif

#endif
