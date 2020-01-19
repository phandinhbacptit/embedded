#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "stm8s.h"

#define  ON_OF_CTRL_PORT        (GPIOC)
#define  ON_OFF_CTRL_PIN        (GPIO_PIN_3)

#define  THRESHOLD_PORT         (GPIOC)
#define  THRESHOLD_PIN          (GPIO_PIN_5)

#define  CTRL_PIN               0
#define  THRESH_PIN             1

void gpio_control_init(void);
void gpio_on(uint8_t ctrl_pin);
void gpio_off(uint8_t ctrl_pin);

void threshold_on(void);
void threshold_off(void);

#endif /*_CONTROL_H_*/