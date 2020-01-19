#ifndef _TIMER_CONF_H_
#define _TIMER_CONF_H_

#include "stm8s.h"

#define PWM_CH1         1
#define PWM_CH2         2

void timer2_pwm_init(void);
void state_pwm_channel(uint8_t channel, uint8_t state);
void timer4_counter_init(void);

#endif /*_TIMER_CONF_H_*/