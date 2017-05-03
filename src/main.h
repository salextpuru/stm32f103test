#ifndef _MAIN_H
#define _MAIN_H
#include <stdint.h>
#include "stm32f10x.h"
#ifdef USE_STM32H_103
#define LED_PORT	GPIOC
#define LED_PIN		GPIO_Pin_13
#define led_off		GPIO_SetBits( LED_PORT, LED_PIN )
#define led_on		GPIO_ResetBits( LED_PORT, LED_PIN )/* -------|<|----*/
#elif defined(USE_OLIMEXINO_STM32)
#define LED_PORT	GPIOA
#define LED_PIN		GPIO_Pin_5
#define led_on		GPIO_SetBits( LED_PORT, LED_PIN )/* -------|>|----*/
#define led_off		GPIO_ResetBits( LED_PORT, LED_PIN )
#endif/*USE_STM32H_103*/
#define TIMER_PRESCALER				(F_CPU / 10000 - 1) /* предделитель для счетчика микросекунд */
#define TIMER_MAX_PERIOD			(10000)		/* период срабатывания */
#ifndef _MAIN_C_SRC
extern void init_LED(void);
extern void init_TIM2(void);
extern void init_USART1(void);
extern char USART1_putc(const char ch);
extern int USART1_puts(const char * str);
#endif/*_MAIN_C_SRC*/
#endif/*_MAIN_H*/
