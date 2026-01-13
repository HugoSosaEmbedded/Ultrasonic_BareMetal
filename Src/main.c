#include "main.h"
#include <stdio.h>
#include <string.h>

char buffer1[64];
char buffer2[64];

char* tx_buffer = buffer1;   // buffer actual a transmitir
char* fill_buffer = buffer2; // buffer donde escribimos nuevos datos

extern volatile uint32_t CCR1, CCR2, i, count_1;

#define N 5
volatile uint16_t buffer[N];
volatile uint8_t llenado = 0;
volatile uint8_t index_1 = 0;

int main()
{
	uint16_t distance;
	uint32_t pulse_width;

	GPIO_TIM2_IC();
	TIM2_Init_IC();
	GPIO_USART2();
	USART2_Init();
	GPIO_TIM3_PWM_Init();
	TIM3_PWM_Init();
	TIM4_Init();

	TIM2->EGR |= (1<<0) ; // Initializing all register
	TIM2->CR1 |= (1<<0);// Enable CNT TIM2
	TIM3->EGR |= (1<<0) ; // Initializing all register
	TIM3->CR1 |= (1<<0); // Enable CNT TIM3
	TIM4->CR1 |= (1<<0); //Enable CNT TIM4
	i = 0;

	while(1)
	{
		TIM3->CCR1 = 12; // PWM

		if(CCR2 != CCR1)
		{
			if (CCR2 > CCR1)
			{
				pulse_width = CCR2 - CCR1;
			}

			else
			{
				pulse_width = (TIM2->ARR + 1 - CCR1) + CCR2;
			}

			distance = pulse_width / 58;
			Add_Sample(distance);
		}

		else
		{
			sprintf(fill_buffer, 64, "Error\r\n");
		}

		float distance_av = Average();
		if(count_1 >= 100)
		{
			sprintf(fill_buffer, 64, "Distance: %.1f cm\r\n", distance_av);
			count_1 = 0;
			if(USART2->SR & (1<<6)) // Check if transmission is complete
			{
				Swap_Buffer();
				i = 0;               // reinicia índice para ISR
				USART2->CR1 |= (1<<7); // reactivar TXEIE para que la ISR continúe transmitiendo
			}
		}

	}

	return 0;
}

void Add_Sample(uint16_t sample)
{
	buffer[index_1] = sample;
	index_1 = (index_1 + 1) % N;
	if(llenado < N) llenado++;
}

float Average(void)
{
	uint16_t sum;
	for(int j = 0; j < llenado; j++) sum +=buffer[index_1];

	return (float)(sum/llenado);
}

void Swap_Buffer(void)
{
	char* tmp = tx_buffer;
	tx_buffer = fill_buffer;
	fill_buffer = tmp;
}
