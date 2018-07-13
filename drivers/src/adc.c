/*
 * adc.c
 *
 *  Created on: Sep 10, 2014
 *      Author: Vik
 */

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "adc.h"


volatile uint16_t ADCConvertedValues[NBR_OF_ADC_CHANNELS];
xQueueHandle      adcQueue, adctransferQueue;

int k = 1;

void adc_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* Configure ADC1 Channel3 & 5 pins as analog input ******************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	ADC_InitTypeDef  ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef  DMA_InitStructure;

	/* Enable peripheral clocks *************************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* DMA2_Stream0 channel0 configuration **************************************/
	DMA_DeInit(DMA2_Stream0);
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADCConvertedValues[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 2;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	/* DMA2_Stream0 enable */
	DMA_Cmd(DMA2_Stream0, ENABLE);

	/* ADC Common Init **********************************************************/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC1 Init ****************************************************************/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 2;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channel configuration ******************************/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_480Cycles); // PA3
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_480Cycles); // PA5

	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 **************************************************************/
	ADC_Cmd(ADC1, ENABLE);

	/* Start ADC1 Software Conversion */
	ADC_SoftwareStartConv(ADC1);

	xTaskCreate(adcTask, (const signed char * const)"ADC",
	                  configMINIMAL_STACK_SIZE, NULL, /*priority*/3, NULL);
}

void adc_init_multi()
{
	k = 1;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* Configure ADC1 Channel1,2,3 & 5 pins as analog input ******************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_3 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	ADC_InitTypeDef  ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef  DMA_InitStructure;

	/* Enable peripheral clocks *************************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* DMA2_Stream0 channel0 configuration **************************************/
	DMA_DeInit(DMA2_Stream0);
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADCConvertedValues[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = NBR_OF_ADC_CHANNELS*2;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	/* DMA2_Stream0 enable */
	DMA_Cmd(DMA2_Stream0, ENABLE);

	/* ADC Common Init **********************************************************/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC1 Init ****************************************************************/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = NBR_OF_ADC_CHANNELS;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channel configuration ******************************/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_480Cycles); // PA3
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_480Cycles); // PA5
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, ADC_SampleTime_480Cycles); // PA6
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 4, ADC_SampleTime_480Cycles); // PA7

	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 **************************************************************/
	ADC_Cmd(ADC1, ENABLE);

	/* Start ADC1 Software Conversion */
	ADC_SoftwareStartConv(ADC1);
	adctransferQueue = xQueueCreate(1, sizeof(adcstruct));
	xTaskCreate(adcTask, (const signed char * const)"ADC",
	                  configMINIMAL_STACK_SIZE, NULL, /*priority*/3, NULL);
}

void adcTask(void *param)
{
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();


  adcstruct dummy;
  //adcDmaStart();

  while(1)
  {
    //xQueueReceive(adcQueue, &adcRawValues, portMAX_DELAY);
    //adcDecimate(adcRawValues, &adcValues);  // 10% CPU


		dummy.data[0] = ADCConvertedValues[0];//adcRawValues[0].vbat.val;
	    dummy.data[1] = ADCConvertedValues[1];
	    dummy.data[2] = ADCConvertedValues[2];
	    dummy.data[3] = ADCConvertedValues[3];
		//vDebugPrintf("%i %i %i %i\r\n", dummy.data[0], dummy.data[1], dummy.data[2],dummy.data[3]);


    //dummy++;
    xQueueSend(adctransferQueue,&dummy,0);
    //pmBatteryUpdate(&adcValues);

    STM_EVAL_LEDToggle(LED4);
    vTaskDelayUntil( &xLastWakeTime, 20 );
    //dummy++;

  }
}
