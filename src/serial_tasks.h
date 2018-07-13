
/*
 * serial_tasks.h
 *
 *  Created on: Apr 18, 2016
 *      Author: Vik
 */

#ifndef SERIAL_TASKS_H_
#define SERIAL_TASKS_H_

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "math.h"
#include "semphr.h"

#include "comm.h"

void serialInit(void);

uint16_t roschannel[4];

struct Pilot_ROS
{
	uint8_t Sync;
	uint8_t ChannelCnt;
	uint8_t values[8];
};
typedef struct Pilot_ROS ROS_Type;
int ROS_parser(uint8_t* c, ROS_Type* ROS_state);
//uint8_t rx_buf[32];
xQueueHandle txQueue_xbee;

typedef struct
{
	uint8_t gyrobytes[6];
	//uint8_t accelbytes[6];
	uint8_t eulerangles[4];
	uint8_t commanded[24];
}imubytes;

imubytes imudata;

#endif /* SERIAL_TASKS_H_ */
