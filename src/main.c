#include "main.h"
#include <string.h>
int main(void);
void init_LED(void);
void init_TIM2(void);
void init_USART1(void);
char USART1_putc(const char ch);
int USART1_puts(const char * str);
/*------------------------ main ------------------------*/
int main()
{
	init_USART1();
	init_LED();
	init_TIM2();
	USART1_puts(" Start program...");

	for(;;) {
	}

	return 0;
}/*main*/
/*------------------------ init_LED ------------------------*/
void init_LED()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = LED_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(LED_PORT, &GPIO_InitStructure);
	//
}/*init_LED*/
/*------------------------ init_TIM2 ------------------------*/
void init_TIM2()
{

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

	// Прерывания
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel 			= TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 		= 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd 			= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Таймер2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseInitStructure.TIM_ClockDivision 		= TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode 		= TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter 	= 0x0000;
	TIM_TimeBaseInitStructure.TIM_Prescaler 		= TIMER_PRESCALER;
	TIM_TimeBaseInitStructure.TIM_Period 			= TIMER_MAX_PERIOD;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}/*init_TIM2*/
/*------------------------- init_USART1 ---------------------------*/
void init_USART1()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1, ENABLE);
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	USART_InitStructure.USART_BaudRate            = 115200;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = (USART_Mode_Rx | USART_Mode_Tx);
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
}/*init_USART1*/
/*----------------------------- USART1_putc ------------------------------*/
char USART1_putc(const char ch)
{
	uint32_t cnt = 100000;
	USART_SendData(USART1, (uint8_t) ch );

	while ( (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) && cnt-- );

	if (0 == cnt)
		return -1;

	return ch;
}/*USART1_putc*/
/*------------------------------ USART1_puts -----------------------------*/
int USART1_puts(const char * str)
{
	int r = -1;
	uint8_t i;
	int size;

	if (str == NULL)
		return r;

	size = strlen(str);

	for (i = 0; i < size; i++) {
		if ( USART1_putc( *(str + i) ) == -1 )
			return r;
	}

	r = 0;
	return r;
}/* USART1_puts */
/*------------------------ assert_failed ----------------------------*/
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
	while (1) {
	}
}/* assert_failed */
#endif/*USE_FULL_ASSERT*/

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        	static uint8_t led_state = 0;

		if (0 == led_state) {
			led_off;
			led_state = 1;

		} else {
			led_on;
			led_state = 0;
		}
    }
}
