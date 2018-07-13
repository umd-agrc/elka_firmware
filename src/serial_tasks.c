/*
 * serial_tasks.c
 *
 *  Created on: Apr 18, 2016
 *      Author: Vik
 */

#include <string.h>

#include <serial_tasks.h>
#include "stm32f4xx_conf.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "math.h"
#include "semphr.h"
#include "stm32f4_discovery.h"
#include "stabilizer.h"

static void vSerialWrite( void *pvParameters );
void USART_puts1(USART_TypeDef* USARTx, uint8_t *data, int length);

//ROS_Type PrimaryROSState = {0,{0}};
//extern cb le_snap;

void serialInit()
{
	//txQueue_xbee = xQueueCreate(1,sizeof(rx_xbee));
	xTaskCreate( vSerialWrite, NULL, configMINIMAL_STACK_SIZE+100, NULL,tskIDLE_PRIORITY+1, NULL );

}

void vSerialWrite( void *pvParameters )
{
  //portTickType xLastWakeTime;
  //xLastWakeTime = xTaskGetTickCount();
	uint8_t i=0;

	for( ;; )
	{

		// XBee
		while( !(USART1->SR & 0x00000040) );
		USART_SendData(USART1, '#');
		USART_puts1(USART1, &imudata,30);
		while( !(USART1->SR & 0x00000040) );
		USART_SendData(USART1, '\r');
		while( !(USART1->SR & 0x00000040) );
		USART_SendData(USART1, '\n');

		// Snapdragon
    //USART_puts1(USART3,snap_buf_tx.data,snap_buf_tx_len);

		//index +=sprintf(&buffer[index],"%d",imudata[i]);
		i++;
		if (i>100)
		{
			i=0;
		}
		// USART_puts(j);

		STM_EVAL_LEDToggle(LED4);
		vTaskDelay(40);

	}
}

void USART1_IRQHandler(void)
{
	static int rx_buffer = 0;
	static portBASE_TYPE xHigherPriorityTaskWoken;

	int txlen;

	xHigherPriorityTaskWoken = pdFALSE;

	if (USART_GetITStatus(USART1, USART_IT_RXNE)) // Received characters modify string
	{

		rx_buf.data[rx_buffer++] = USART_ReceiveData(USART1);

		txlen = rx_buf.data[0];
		if (rx_buffer>txlen)
		{
			rx_buffer = 0;
		}

		portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );

	}
}

void USART3_IRQHandler(void)
{
	static int rx_buffer = 0;
	static portBASE_TYPE xHigherPriorityTaskWoken;
	int txlen;

	xHigherPriorityTaskWoken = pdFALSE;

	if (USART_GetITStatus(USART3, USART_IT_RXNE)) // Received characters modify string
	{
				snap_buf_rx.data[rx_buffer++] = USART_ReceiveData(USART3);

				txlen = snap_buf_rx.data[0];
				if (rx_buffer>txlen)
				{
					rx_buffer = 0;
				}

		portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
	}
}

void USART_puts(USART_TypeDef* USARTx, volatile char *s){

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, *s);
		*s++;
	}
}

void USART_puts1(USART_TypeDef* USARTx, uint8_t *data, int length){

	int i;
	for (i=0;i<length;i++){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, data[i]);

	}
}

char* format_uint_array(char *b, uint8_t* data, int length, char* delim, char* fmt)
{
	int i;
	char s[50];
	b[0] = 0;
	for( i = 0; i < length; i++ )
	{
		s[0]=0;
		sprintf( s, fmt, data[i], (i<length-1)?delim : "");
		strcat(b, s);
	}
	return b;
}

int ROS_parser(uint8_t* c, ROS_Type* ROS_state)
{
	int k;
	int txlen;
	int i;
	txlen = c[0];
	if (c[1]==0)
	{
		ROS_state->Sync = 1;
	}
	else if((c[2]==255) &  (ROS_state->Sync==1))
	{
		ROS_state->Sync = 2;
	}
	//	else if(c==0xFF & ROS_state->Sync==2)
	//	{
	//		ROS_state->Sync = 3;
	//	}
	else if(ROS_state->Sync==2)
	{
		for (i=0;i<txlen-3;i++)
		{

			ROS_state->values[ROS_state->ChannelCnt] = c[i+3];
			ROS_state->ChannelCnt++;
		}
		if(ROS_state->ChannelCnt==8)
		{
			ROS_state->Sync = 3;
			ROS_state->ChannelCnt = 0;
		}
	}
	k = ROS_state->Sync;
	return k;
}

