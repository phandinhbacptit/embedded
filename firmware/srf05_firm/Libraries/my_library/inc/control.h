#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "stm8s.h"

#define  ON_OF_CTRL_PORT        (GPIOC)
#define  ON_OFF_CTRL_PIN        (GPIO_PIN_3)

#define  THRESHOLD_PORT         (GPIOC)
#define  THRESHOLD_PIN          (GPIO_PIN_5)

#define SIGNAL_PORT             (GPIOA)
#define SIGNAL_PIN              (GPIO_PIN_3)

#define TRIG_PORT               (GPIOD)
#define TRIG_PIN                (GPIO_PIN_5)

#define ECHO_PORT               (GPIOD)
#define ECHO_PIN                (GPIO_PIN_6)

#define  CTRL_PIN               0
#define  THRESH_PIN             1

void gpio_control_init(void);
void signal_pin_init(void);
void enable_threshold(void);
void disable_threshold(void);
void poweron_max232(void);
void poweroff_max232(void);

#endif /*_CONTROL_H_*/