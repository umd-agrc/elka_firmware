/*
 * adc.h
 *
 *  Created on: Sep 10, 2014
 *      Author: Vik
 */

#ifndef ADC_H_
#define ADC_H_

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32f4_discovery.h"

#define NBR_OF_ADC_CHANNELS   4


typedef struct
{
	uint16_t data[NBR_OF_ADC_CHANNELS];
} adcstruct;

void adc_init();
void adc_init_multi();
portTASK_FUNCTION_PROTO( adcTask, pvParameters );

#endif /* ADC_H_ */
