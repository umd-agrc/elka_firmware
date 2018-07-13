/* Library includes. */
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_lis302dl.h"

#ifndef HW_CONFIG_H
#define HW_CONFIG_H

extern GPIO_InitTypeDef GPIO_InitStructure;
extern NVIC_InitTypeDef NVIC_InitStructure;

/* TIM4 Autoreload and Capture Compare register values */
#define TIM_ARR                          ((uint16_t)1900)
#define TIM_CCR                          ((uint16_t)100)

/* definitions used by prvLED_Config() to reconfigure LED pins GPIO<->TIM4 */
#define GPIO	0
#define TIMER	1

void prvSetupHardware( void );
void prvTIM4_Config(void);
void prvMEMS_Config(void);
void prvLED_Config(char state);
uint32_t LIS302DL_TIMEOUT_UserCallback(void);
void USART3Init( uint32_t baudrate );
void USART2Init( uint32_t baudrate );
void USART1Init(uint32_t baudrate);
void init_tim();


#endif /*HW_CONFIG_H*/
