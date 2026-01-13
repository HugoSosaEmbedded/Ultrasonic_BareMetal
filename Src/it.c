#include "main.h"

volatile uint32_t CCR1, CCR2, i;
volatile uint32_t count_1 = 0;

extern char* tx_buffer;

void TIM4_IRQHandler(void)
{
	if(TIM4->SR & (1<<0))
	{
		count_1++;
		TIM4->SR &= ~(1<<0); //Clean UIF
	}
}

void USART2_IRQHandler()
{
    if(USART2->SR & (1<<7)) // TXE
	{
    	if(tx_buffer[i] != '\0')       // solo enviar mientras no sea fin de string
    	{
    		USART2->DR = tx_buffer[i++];
    	}
    	else
    	{
	     	USART2->CR1 &= ~(1<<7);  // deshabilita TXEIE
    	}
	}
}

void TIM2_IRQHandler(void)
{
	if(TIM2->SR & (1<<1)) CCR1 = TIM2->CCR1;

	if(TIM2->SR & (1<<2)) CCR2 = TIM2->CCR2;

}

