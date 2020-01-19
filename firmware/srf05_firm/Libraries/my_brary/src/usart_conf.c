#include "stm8s.h"
#include "usart_conf.h"
#include "stdio.h"

#ifdef _RAISONANCE_
#define PUTCHAR_PROTOTYPE int putchar (char c)
#define GETCHAR_PROTOTYPE int getchar (void)
#elif defined (_COSMIC_)
#define PUTCHAR_PROTOTYPE char putchar (char c)
#define GETCHAR_PROTOTYPE char getchar (void)
#else /* _IAR_ */
#define PUTCHAR_PROTOTYPE int putchar (int c)
#define GETCHAR_PROTOTYPE int getchar (void)
#endif /* _RAISONANCE_ */


/**
  * @Brief : USART init
             BaudRate = 115200 baud
             Word Length = 8 Bits
             One stop bit
             No parity
             Receive and transmit enable
             UART1 Clock disable
  * @Param : None
  * @Ret   : None
**/
void usart_init(void)
{
    UART1_DeInit();  
    UART1_Init((uint32_t) 115200, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, 
               UART1_PARITY_NO, UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);
}

/**
  * @Brief : Retargets the C library printf function to the UART.
  * @Param : c Character to send
  * @Ret   : Char Character sent
  */
PUTCHAR_PROTOTYPE
{
  /* Write a character to the UART1 */
  UART1_SendData8(c);
  /* Loop until the end of transmission */
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);

  return (c);
}

/**
  *@brief : Retagets the C library scanf fucntion to the USART
  *@Param : None
  *@Ret   : Char character to read
*/
GETCHAR_PROTOTYPE
{
#ifdef _COSMIC_
    char c = 0;
#else
    int c = 0;
#endif
    
    /*Loop until data register flag is SET*/
    while (UART1_GetFlagStatus(UART1_FLAG_RXNE) == RESET);
    c = UART1_ReceiveData8();
    return (c);
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif