#include "pwm_gate_control.h"

void pwm_gate_control(unsigned short enable)
{
#ifndef MATLAB_MEX_FILE
    EALLOW;

    /* Both complementary PWM inputs must be inactive during a stop. */
    EPwm4Regs.TZCTL.bit.TZA = 2U; /* force EPWMxA low */
    EPwm4Regs.TZCTL.bit.TZB = 2U; /* force EPWMxB low */
    EPwm5Regs.TZCTL.bit.TZA = 2U;
    EPwm5Regs.TZCTL.bit.TZB = 2U;
    EPwm6Regs.TZCTL.bit.TZA = 2U;
    EPwm6Regs.TZCTL.bit.TZB = 2U;

    if (enable != 0U) {
        EPwm4Regs.TZCLR.bit.OST = 1U;
        EPwm5Regs.TZCLR.bit.OST = 1U;
        EPwm6Regs.TZCLR.bit.OST = 1U;
    } else {
        EPwm4Regs.TZFRC.bit.OST = 1U;
        EPwm5Regs.TZFRC.bit.OST = 1U;
        EPwm6Regs.TZFRC.bit.OST = 1U;
    }

    EDIS;
#else
    (void)enable;
#endif
}
