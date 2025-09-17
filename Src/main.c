#include "main.h"
#include <stdio.h>
#include <string.h>

void TIM2_Init_IC(void);
void GPIO_TIM2_IC(void);
void USART2_Init(void);
void GPIO_USART2(void);
void Swap_Buffer(void);
void TIM3_PWM_Init(void);
void GPIO_TIM3_PWM_Init(void);
void TIM4_Init(void);
void Add_Sample(uint16_t sample);
float Average(void);

volatile uint32_t CCR1, CCR2, CNT_anterior, i;
volatile uint32_t count_1 = 0;

char buffer1[64];
char buffer2[64];

char* tx_buffer = buffer1;   // buffer actual a transmitir
char* fill_buffer = buffer2; // buffer donde escribimos nuevos datos

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
			snprintf(fill_buffer, 64, "Error\r\n");
		}

		float distance_av = Average();
		if(count_1 >= 100)
		{
			snprintf(fill_buffer, 64, "Distance: %.1f cm\r\n", distance_av);
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

void TIM2_IRQHandler(void)
{
	if(TIM2->SR & (1<<1)) CCR1 = TIM2->CCR1;

	if(TIM2->SR & (1<<2)) CCR2 = TIM2->CCR2;

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

void GPIO_TIM2_IC(void)
{
	RCC->AHB1ENR |= (1<<0); // Enable GPIOA clock
	GPIOA->MODER &= ~(3<<0);
	GPIOA->MODER |= (2<<0); // PA0 as ALtarnate Function
	GPIOA->AFR[0] &= ~(0xF<<0);
	GPIOA->AFR[0] |= (1<<0); // Set up AF1 to TIM2_CH1
}

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

void GPIO_TIM3_PWM_Init(void)
{
	RCC->AHB1ENR |= (1<<0); // Enable GPIOA clock
	GPIOA->MODER &= ~(3<<12);
	GPIOA->MODER |= (2<<12); // PA6 AF
	GPIOA->AFR[0] &= ~(0xF<<24);
	GPIOA->AFR[0] |= (2<<24); // Set up AF2 to TIM3_CH1

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

void TIM4_IRQHandler(void)
{
	if(TIM4->SR & (1<<0))
	{
		count_1++;
		TIM4->SR &= ~(1<<0); //Clean UIF
	}
}
