/*
 * px4flow.h
 *
 *  Created on: Mar 10, 2016
 *      Author: Vik
 */

/* Library includes. */
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

#ifndef PX4FLOW_H_
#define PX4FLOW_H_

void init_I2C2();
void init_I2C1();
void init_I2C1_new();
void init_I2C2_new();
#define PX4_ADDRESS 0x42 // the slave address (example). Actual address is 0x42 bit shifted by 1 due to 7 bit addressing
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);
uint8_t I2C_startslave1(I2C_TypeDef* I2Cx, uint8_t address);
void I2C_startslave(I2C_TypeDef* I2Cx, uint8_t address);
void I2C_send(I2C_TypeDef* I2Cx, uint8_t data);
uint8_t I2C_read(I2C_TypeDef* I2Cx);
void I2C_startsign(I2C_TypeDef* I2Cx);
void I2C_setreceiver(I2C_TypeDef* I2Cx, uint8_t address);
void I2C_stop(I2C_TypeDef* I2Cx);
#define NVIC_I2C_PRI  5






typedef struct i2c_frame
{
    uint16_t frame_count;// counts created I2C frames [#frames]
    int16_t pixel_flow_x_sum;// latest x flow measurement in pixels*10 [pixels]
    int16_t pixel_flow_y_sum;// latest y flow measurement in pixels*10 [pixels]
    int16_t flow_comp_m_x;// x velocity*1000 [meters/sec]
    int16_t flow_comp_m_y;// y velocity*1000 [meters/sec]
    int16_t qual;// Optical flow quality / confidence [0: bad, 255: maximum quality]
    int16_t gyro_x_rate; // latest gyro x rate [rad/sec]
    int16_t gyro_y_rate; // latest gyro y rate [rad/sec]
    int16_t gyro_z_rate; // latest gyro z rate [rad/sec]
    uint8_t gyro_range; // gyro range [0 .. 7] equals [50 deg/sec .. 2000 deg/sec]
    uint8_t sonar_timestamp;// time since last sonar update [milliseconds]
    int16_t ground_distance;// Ground distance in meters*1000 [meters]. Positive value: distance known. Negative value: Unknown distance
} i2c_frame;

typedef struct i2c_integral_frame
{
    uint16_t frame_count_since_last_readout;//number of flow measurements since last I2C readout [#frames]
    int16_t pixel_flow_x_integral;//accumulated flow in radians*10000 around x axis since last I2C readout [rad*10000]
    int16_t pixel_flow_y_integral;//accumulated flow in radians*10000 around y axis since last I2C readout [rad*10000]
    int16_t gyro_x_rate_integral;//accumulated gyro x rates in radians*10000 since last I2C readout [rad*10000]
    int16_t gyro_y_rate_integral;//accumulated gyro y rates in radians*10000 since last I2C readout [rad*10000]
    int16_t gyro_z_rate_integral;//accumulated gyro z rates in radians*10000 since last I2C readout [rad*10000]
    uint32_t integration_timespan;//accumulation timespan in microseconds since last I2C readout [microseconds]
    uint32_t sonar_timestamp;// time since last sonar update [microseconds]
    int16_t ground_distance;// Ground distance in meters*1000 [meters*1000]
    int16_t gyro_temperature;// Temperature * 100 in centi-degrees Celsius [degcelsius*100]
    uint8_t quality;// averaged quality of accumulated flow values [0:bad quality;255: max quality]
} __attribute__((packed)) i2c_integral_frame;

void readx1(I2C_TypeDef* I2Cx, i2c_frame* px4data, uint8_t address,uint8_t status);
void readx(I2C_TypeDef* I2Cx, i2c_frame* px4data, uint8_t address);

#endif /* PX4FLOW_H_ */
