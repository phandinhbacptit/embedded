#include "control.h"

void gpio_control_init(void)
{
    /* Initialize I/Os in Output Mode */
    GPIO_Init(ON_OF_CTRL_PORT, (GPIO_Pin_TypeDef)ON_OFF_CTRL_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_Init(THRESHOLD_PORT, (GPIO_Pin_TypeDef)THRESHOLD_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
}

void gpio_on(uint8_t ctrl_pin)
{
    if (ctrl_pin == CTRL_PIN)
      GPIO_WriteHigh(ON_OF_CTRL_PORT, ON_OFF_CTRL_PIN);
    else if (ctrl_pin == THRESH_PIN)
       GPIO_WriteHigh(THRESHOLD_PORT, THRESHOLD_PIN);
}

void gpio_off(uint8_t ctrl_pin)
{
    if (ctrl_pin == CTRL_PIN)
      GPIO_WriteLow(ON_OF_CTRL_PORT, ON_OFF_CTRL_PIN);
    else if (ctrl_pin == THRESH_PIN)
      GPIO_WriteLow(THRESHOLD_PORT, THRESHOLD_PIN);
}