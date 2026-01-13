#include "main.h"

void TIM2_Init_IC(void)
{
	RCC->APB1ENR |= (1<<0); // TIM2 clock enabled
	TIM2->ARR = 25000 - 1;
	TIM2->PSC = 15;
	//--------------------- CH1-----------------------------------------
	TIM2->CCMR1 &= ~(3<<0);
	TIM2->CCMR1 |= (1<<0); // CC1 channel is configured as input, IC1 is mapped on TI1
	TIM2->CCMR1 &= ~(0xF<<4); // No filter
	TIM2->CCMR1 &= ~(3<<2); // No prescaler
	TIM2->CCER &= ~((1<<1) | (1<<3)); // Rising edge
	TIM2->CCER |= (1<<0); // Enable CC1

	//-----------------------CH2-------------------------------------------
	TIM2->CCMR1 &= ~(3<<8);
	TIM2->CCMR1 |= (2<<8); // CC2 channel is configured as input, IC2 is mapped on TI1
	TIM2->CCMR1 &= ~(0xF<<12); // No filter
	TIM2->CCMR1 &= ~(3<<10); // No prescaler
	TIM2->CCER |= (1<<5); //Falling edge
	TIM2->CCER &= ~(1<<7); // Falling edge
	TIM2->CCER |= (1<<4); // Enable CC2

	TIM2->DIER |= (1<<1); //Enable CC1 interrupt
	TIM2->DIER |= (1<<2); //Enable CC2 interrupt
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn,0);
}

void TIM3_PWM_Init(void)
{

	RCC->APB1ENR |= (1<<1); // TIM3 clock enabled
	TIM3->PSC = 15;
	TIM3->ARR = 60010 - 1; // This PSC and ARR is configured to get a time_base = 0.060012 because we need to have 12 us + 60 ms,
						   // 12 us of pulse to activate trigger of ultrasonic sensor and 60 ms disable to wait a response of echo

	TIM3->CCMR1 &= ~(7<<4);
	TIM3->CCMR1 |= (6<<4);  // PWM mode1- Upcounting, channel 1 is active as long as TIM_CNT < TIM2_CCR1 else inactive.
	TIM3->CCMR1 |= (1<<3); // Enable preload register on TIM3_CCR1

	TIM3->CCER &= ~(1<<1); // OC1 polarity CH1 - HIGH
	TIM3->CCER |= (1<<0) ; // OC1 output enable CH1

	TIM3->CR1 |= (1<<7); // auto-reload preload register enable

}

void TIM4_Init(void)
{
	RCC->APB1ENR |= (1<<2); //TIM4 clock enabled
	TIM4->PSC = 15;
	TIM4->ARR = 1000 - 1;
	TIM4->CR1 &= ~(1<<1);

	TIM4->DIER |= (1<<0); // Enable Update interrupt enable
	NVIC_EnableIRQ(TIM4_IRQn);
	NVIC_SetPriority(TIM4_IRQn, 2);
}


