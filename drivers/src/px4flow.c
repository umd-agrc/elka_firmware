/*
 * px4flow.c
 *
 *  Created on: Mar 10, 2016
 *      Author: Vik
 */
#include "px4flow.h"
#include "i2croutines.h"
#include "i2cdev.h"

void init_I2C2()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2 , ENABLE);
	RCC->APB1RSTR &= 0xFFBFFFFF;
	GPIO_StructInit(&GPIO_InitStruct);

	//setup SCL and SDA pins
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;			// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	// Connect I2C1 pins to AF
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);	// SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2); // SDA

	// configure I2C1
	I2C_DeInit(I2C2);
	I2C_InitStruct.I2C_ClockSpeed = 400000; 		// 100kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2C2, &I2C_InitStruct);				// init I2C2

	// enable I2C1
	I2C_Cmd(I2C2, ENABLE);
}

//Enable the ACK and send the start signal and address the slave
uint8_t I2C_startslave1(I2C_TypeDef* I2Cx, uint8_t address){
	uint32_t Timeout = 10000;
	uint8_t status=1;
	I2C_AcknowledgeConfig(I2Cx, ENABLE);

	/* While the bus is busy */
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)){
		if(Timeout--==0){
			Timeout = 10000;
			status = 0;
			return ERROR;

		}
		else
			status=1;

	}


	/* Send START condition */
	I2C_GenerateSTART(I2Cx, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){
		{
				if(Timeout--==0){
					Timeout = 10000;
					status = 0;
					return ERROR;

				}
				else
					status=1;

			}
	}

	// Send slave Address for write
	I2C_Send7bitAddress(I2Cx, address, I2C_Direction_Transmitter);
	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
		{
				if(Timeout--==0){
					Timeout = 10000;
					status = 0;
					return ERROR;


				}
				else
					status = 1;

			}
	}
	//GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_SET);
	//I2C_Cmd(I2Cx, ENABLE);
	return status;

}

void I2C_startslave(I2C_TypeDef* I2Cx, uint8_t address){
	I2C_AcknowledgeConfig(I2Cx, ENABLE);

	/* While the bus is busy */
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));


	/* Send START condition */
	I2C_GenerateSTART(I2Cx, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	// Send slave Address for write
	I2C_Send7bitAddress(I2Cx, address, I2C_Direction_Transmitter);
	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	//GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_SET);
	I2C_Cmd(I2Cx, ENABLE);

}

void I2C_send(I2C_TypeDef* I2Cx, uint8_t data){
	I2C_SendData(I2Cx, data);
	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

//send the byte to the slave (it need I2C_startslave for the select the address)
uint8_t I2C_read(I2C_TypeDef* I2Cx){
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

//generate pure start signal
void I2C_startsign(I2C_TypeDef* I2Cx){
	// Send I2C1 STOP Condition
	I2C_GenerateSTART(I2Cx, ENABLE);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
}

void I2C_setreceiver(I2C_TypeDef* I2Cx, uint8_t address){
I2C_Send7bitAddress(I2Cx, address, I2C_Direction_Receiver);

   while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

}

void I2C_stop(I2C_TypeDef* I2Cx){
    I2C_AcknowledgeConfig(I2Cx, DISABLE);
    I2C_GenerateSTOP(I2Cx, ENABLE);
}

void readx1(I2C_TypeDef* I2Cx, i2c_frame* px4data, uint8_t address,uint8_t status)
{
	static uint8_t id[22];
	    //I2C_startslave(I2Cx,address);
//if(status==1) {
//		I2C_send(I2Cx, 0x00); //send the byte to the slave.
//		I2C_startsign(I2Cx);
//		I2C_setreceiver(I2Cx, address); //set the I2C in mode Receiver to selected address
//		id[0] = I2C_read(I2Cx); id[1] = I2C_read(I2Cx);
//		id[2] = I2C_read(I2Cx); id[3] = I2C_read(I2Cx);
//		id[4] = I2C_read(I2Cx); id[5] = I2C_read(I2Cx);
//		id[6] = I2C_read(I2Cx); id[7] = I2C_read(I2Cx);
//		id[8] = I2C_read(I2Cx); id[9] = I2C_read(I2Cx);
//		id[10] = I2C_read(I2Cx); id[11] = I2C_read(I2Cx);
//		id[12] = I2C_read(I2Cx); id[13] = I2C_read(I2Cx);
//		id[14] = I2C_read(I2Cx); id[15] = I2C_read(I2Cx);
//		id[16] = I2C_read(I2Cx); id[17] = I2C_read(I2Cx);
//		id[18] = I2C_read(I2Cx); id[19] = I2C_read(I2Cx);
//		id[20] = I2C_read(I2Cx); id[21] = I2C_read(I2Cx);
//		//id[22] = I2C_read(I2Cx); id[23] = I2C_read(I2Cx);
//		//id[24] = I2C_read(I2Cx);
//		I2C_stop(I2Cx);
//}
	i2cdevRead(I2C2, address, 0x00, 22, id);
	px4data->frame_count = (id[1]<<8)|id[0];
	px4data->pixel_flow_x_sum = (id[3]<<8)|id[2];
	px4data->pixel_flow_y_sum = (id[5]<<8)|id[4];
	px4data->flow_comp_m_x = (id[7]<<8)|id[6];
	px4data->flow_comp_m_y = (id[9]<<8)|id[8];
	px4data->qual = (id[11]<<8)|id[10];
	px4data->gyro_x_rate = (id[13]<<8)|id[12];
	px4data->gyro_y_rate = (id[15]<<8)|id[14];
	px4data->gyro_z_rate = (id[17]<<8)|id[16];
	px4data->gyro_range = (id[18]);
	px4data->sonar_timestamp = (id[19]);
	px4data->ground_distance = (id[21]<<8)|id[20];

}

void readx(I2C_TypeDef* I2Cx, i2c_frame* px4data, uint8_t address)
{
	static uint8_t id[22];
	I2C_startslave(I2Cx,address);
	I2C_send(I2Cx, 0x00); //send the byte to the slave.
	I2C_startsign(I2Cx);
	I2C_setreceiver(I2Cx, address); //set the I2C in mode Receiver to selected address
	id[0] = I2C_read(I2Cx); id[1] = I2C_read(I2Cx);
	id[2] = I2C_read(I2Cx); id[3] = I2C_read(I2Cx);
	id[4] = I2C_read(I2Cx); id[5] = I2C_read(I2Cx);
	id[6] = I2C_read(I2Cx); id[7] = I2C_read(I2Cx);
	id[8] = I2C_read(I2Cx); id[9] = I2C_read(I2Cx);
	id[10] = I2C_read(I2Cx); id[11] = I2C_read(I2Cx);
	id[12] = I2C_read(I2Cx); id[13] = I2C_read(I2Cx);
	id[14] = I2C_read(I2Cx); id[15] = I2C_read(I2Cx);
	id[16] = I2C_read(I2Cx); id[17] = I2C_read(I2Cx);
	id[18] = I2C_read(I2Cx); id[19] = I2C_read(I2Cx);
	id[20] = I2C_read(I2Cx); id[21] = I2C_read(I2Cx);
	//id[22] = I2C_read(I2Cx); id[23] = I2C_read(I2Cx);
	//id[24] = I2C_read(I2Cx);
	I2C_stop(I2Cx);
	px4data->frame_count = (id[1]<<8)|id[0];
	px4data->pixel_flow_x_sum = (id[3]<<8)|id[2];
	px4data->pixel_flow_y_sum = (id[5]<<8)|id[4];
	px4data->flow_comp_m_x = (id[7]<<8)|id[6];
	px4data->flow_comp_m_y = (id[9]<<8)|id[8];
	px4data->qual = (id[11]<<8)|id[10];
	px4data->gyro_x_rate = (id[13]<<8)|id[12];
	px4data->gyro_y_rate = (id[15]<<8)|id[14];
	px4data->gyro_z_rate = (id[17]<<8)|id[16];
	px4data->gyro_range = (id[18]);
	px4data->sonar_timestamp = (id[19]);
	px4data->ground_distance = (id[21]<<8)|id[20];

}

void init_I2C1(void){

GPIO_InitTypeDef GPIO_InitStruct;
I2C_InitTypeDef I2C_InitStruct;
NVIC_InitTypeDef NVIC_InitStructure;

// enable APB1 peripheral clock for I2C1

// enable clock for SCL and SDA pins
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

/* setup SCL and SDA pins
 * You can connect I2C1 to two different
 * pairs of pins:
 * 1. SCL on PB6 and SDA on PB7
 * 2. SCL on PB8 and SDA on PB9
 */
GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // we are going to use PB6 and PB7
GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// set GPIO speed
GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;			// set output to open drain --> the line has to be only pulled low, not driven high
GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
GPIO_Init(GPIOB, &GPIO_InitStruct);					// init GPIOB

// Connect I2C1 pins to AF
GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL
GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA
RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_I2C_PRI;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);
NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_I2C_PRI + 1;
NVIC_Init(&NVIC_InitStructure);

I2C_DeInit(I2C1);
I2C_InitStruct.I2C_ClockSpeed = 100000; 		// 100kHz
I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;		// disable acknowledge when reading (can be changed later on)
I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
I2C_Init(I2C1, &I2C_InitStruct);				// init I2C1

// enable I2C1
//I2C_Cmd(I2C1, ENABLE);
#define I2C_BUSY 0x20
  if (I2C1->SR2 & I2C_BUSY)
  {
    /* Reset the I2C block */
    I2C_SoftwareResetCmd(I2C1, ENABLE);
    I2C_SoftwareResetCmd(I2C1, DISABLE);
  }
}

void init_I2C1_new()
{
	i2cdevInit(I2C1);
}

void init_I2C2_new()
{
	i2cdevInit(I2C2);
}



