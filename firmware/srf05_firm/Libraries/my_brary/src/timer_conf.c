#include "stm8s.h"
#include "timer_conf.h"

uint16_t CCR1_Val = 25;
uint16_t CCR2_Val = 25;

/*
  * @brief  Configure TIM2 peripheral in PWM mode,The TIM2CLK frequency is 2MHz, the Prescaler is 1 so the TIM2 counter clock is 2MHz. 
    TIM2 Frequency = TIM2 counter clock / (ARR + 1)
    ARR = 49 => TIM2 Frequency = 2000000 / 50 = 40  khz
    We need generate 40 khz pwm to put in srf05 sensor, duty cycle of 2 channel í 50%
  * @param  None
  * @retval None
*/
void timer2_pwm_init(void)
{
    TIM2_DeInit();
    /* Time base configuration */
    TIM2_TimeBaseInit(TIM2_PRESCALER_1, 49);
    
    /* PWM1 Mode configuration: Channel1 */ 
    TIM2_OC1Init(TIM2_OCMODE_ACTIVE, TIM2_OUTPUTSTATE_ENABLE, CCR1_Val, TIM2_OCPOLARITY_HIGH);
    //TIM2_OC1PreloadConfig(ENABLE);
    
    /* PWM1 Mode configuration: Channel2 */ 
    TIM2_OC2Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE, CCR2_Val, TIM2_OCPOLARITY_HIGH);
    //TIM2_OC2PreloadConfig(ENABLE);
    
    TIM2_ARRPreloadConfig(ENABLE);
    
    TIM2_ITConfig(TIM2_IT_CC1, ENABLE);
    /* TIM2 enable counter */
    TIM2_Cmd(ENABLE);
  
  
}
/*
  * @brief  Generate pwm in channel
  * @param  channel - Channel want to generate pulse
            state - Enable/Disable channel generate pulse
  * @retval None
*/
void state_pwm_channel(uint8_t channel, uint8_t state)
{
    uint8_t cmd = channel;
    
    switch (cmd) {
        case PWM_CH1: {
            if (state == 1) 
              TIM2_OC1PreloadConfig(ENABLE);
            else 
              TIM2_OC1PreloadConfig(DISABLE);
        }
        
        case PWM_CH2: {
            if (state == 1) 
              TIM2_OC2PreloadConfig(ENABLE);
            else 
              TIM2_OC2PreloadConfig(DISABLE);
        }
      
    }
}
/*
  * @brief: TIM4 configuration:
            TIM4CLK is set to 16 MHz, the TIM4 Prescaler is equal to 128 so the TIM1 counter
            clock used is 16 MHz / 128 = 125 000 Hz. With 125 000 Hz we can generate time base:
            TIM4_PERIOD = 124   --> (124 + 1) / 125000 = 1 ms
            So TIM4 generates an Update Interrupt each 1/1000 =  1ms
  * @param  channel - Channel want to generate pulse
            state - Enable/Disable channel generate pulse
  * @retval None
*/
#define TIM4_PERIOD       124
void timer4_counter_init(void)
{
  /* Time base configuration */
  TIM4_TimeBaseInit(TIM4_PRESCALER_128, TIM4_PERIOD);
  /* Clear TIM4 update flag */
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);
  /* Enable update interrupt */
  TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
  
  /* enable interrupts */
  enableInterrupts();

  /* Enable TIM4 */
  TIM4_Cmd(ENABLE);
}

#define TIM1_PERIOD       124
void timer1_counter_init(void)
{
//  /* Time base configuration */
//  TIM1_TimeBaseInit(TIM1_PRESCALER_128, TIM1_PERIOD);
//  /* Clear TIM4 update flag */
//  TIM1_ClearFlag(TIM1_FLAG_UPDATE);
//  /* Enable update interrupt */
//  TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE);
//  
//  /* enable interrupts */
//  enableInterrupts();
//
//  /* Enable TIM4 */
//  TIM1_Cmd(ENABLE);
}