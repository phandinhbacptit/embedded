#include "timer_conf.h"
#include "usart_conf.h"
#include "control.h"
#include "stdio.h"

extern uint32_t mTime; // Increase every 1 ms
extern uint8_t n_pulse;

static void clk_config(void)
{
    /* Initialization of the clock , Clock divider to HSI/1 */
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
}
void delay(uint32_t n_time)
{
  while(n_time --);
}
int main(void )
{
    clk_config();
    usart_init();
    timer2_pwm_init();
    timer4_counter_init();
    gpio_control_init();
      
    while(1) {
      if (mTime > 1) {
        mTime = 0;
        printf(" \n %d", TIM2_GetCounter());
      }
    }
}
