#include "main.h"

void USART2_Init(void)
{
	RCC->APB1ENR |= (1<<17); //Enable clock for USART2
	USART2->CR1 &= ~(1<<12); // 8 data bits as word length
	USART2->CR2 &= ~((1<<12) | (1<<13)); // USART configured with 1 stop bit
	USART2->CR1 &= ~(1<<15); // Oversamplig  by 16
	USART2->BRR = (8<<4) | 11; // Baud rate desired 115200
	USART2->CR1 |= (1<<3); //Transmitter is enabled
	USART2->CR1 |= (1<<13); //USART enable

	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn,1);
	USART2->CR1 |= (1<<7); //Enable TXEIE interrupt*/

}
