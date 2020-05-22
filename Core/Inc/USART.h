#ifndef STM32L4xx_LL_USART_H
#define STM32L4xx_LL_USART_H

//#ifdef __GNUC__
///* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
// set to 'Yes') calls __io_putchar() */
//#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif /* __GNUC__ */
#include "stm32l433xx.h"

#define BufferSize 32

uint8_t   USART_Read(USART_TypeDef * USARTx);
void USART_Write(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t nBytes);
void USART_Write_String(USART_TypeDef* USARTx, uint8_t* str, uint32_t nBytes);
void USART_IRQHandler(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t * pRx_counter);

#endif /* __STM32L476G_DISCOVERY_UART_H */
