#include "main.h"

void GPIO_TIM3_PWM_Init(void)
{
	RCC->AHB1ENR |= (1<<0); // Enable GPIOA clock
	GPIOA->MODER &= ~(3<<12);
	GPIOA->MODER |= (2<<12); // PA6 AF
	GPIOA->AFR[0] &= ~(0xF<<24);
	GPIOA->AFR[0] |= (2<<24); // Set up AF2 to TIM3_CH1

}

void GPIO_TIM2_IC(void)
{
	RCC->AHB1ENR |= (1<<0); // Enable GPIOA clock
	GPIOA->MODER &= ~(3<<0);
	GPIOA->MODER |= (2<<0); // PA0 as ALtarnate Function
	GPIOA->AFR[0] &= ~(0xF<<0);
	GPIOA->AFR[0] |= (1<<0); // Set up AF1 to TIM2_CH1
}

void GPIO_USART2(void)
{
	//PA2 Tx PA3 Rx
	RCC->AHB1ENR |= (1<<0);
	GPIOA->MODER &= ~((3<<4) | (3<<6)); // Clean ports for moder
	GPIOA->MODER |= ((2<<4) | (2<<6)); // Set up port A2 and A3 as alternate function
	GPIOA->OTYPER &= ~((1<<2) | (1<<3)); //Output push-pull
	GPIOA->OSPEEDR |= ((2<<4) | (2<<6)); // High speed for PA2 and PA3
	GPIOA->AFR[0] &= ~((0xF<<8) | (0xF<<12));
	GPIOA->AFR[0] |= ((7<<8) | (7<<12));
}




