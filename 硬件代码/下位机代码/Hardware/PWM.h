#ifndef __PWM_H
#define __PWM_H

void PWM_Init(void);
void PWM_SetPulseWidth_CH3(uint16_t pulse_us);
void PWM_SetPulseWidth_CH4(uint16_t pulse_us);
void Motor_Stop(void);

#endif
