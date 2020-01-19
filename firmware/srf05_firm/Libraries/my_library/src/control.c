#include "control.h"

void gpio_control_init(void)
{
    /* Initialize I/Os in Output Mode */
    GPIO_Init(ON_OF_CTRL_PORT, (GPIO_Pin_TypeDef)ON_OFF_CTRL_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_Init(THRESHOLD_PORT, (GPIO_Pin_TypeDef)THRESHOLD_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_Init(TRIG_PORT, (GPIO_Pin_TypeDef)TRIG_PIN, GPIO_MODE_IN_PU_NO_IT);
    GPIO_Init(ECHO_PORT, (GPIO_Pin_TypeDef)ECHO_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
}

void signal_pin_init(void)
{
    GPIO_Init(SIGNAL_PORT, (GPIO_Pin_TypeDef)SIGNAL_PIN, GPIO_MODE_IN_PU_NO_IT);
}
void enable_threshold(void)
{
    GPIO_WriteLow(THRESHOLD_PORT, THRESHOLD_PIN);
}

void disable_threshold(void)
{
    GPIO_WriteHigh(THRESHOLD_PORT, THRESHOLD_PIN);
}

void poweron_max232(void)
{
    GPIO_WriteLow(ON_OF_CTRL_PORT, ON_OFF_CTRL_PIN);
}

void poweroff_max232(void)
{
    GPIO_WriteHigh(ON_OF_CTRL_PORT, ON_OFF_CTRL_PIN);
}