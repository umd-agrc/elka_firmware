/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * imu.c - inertial measurement unit
 */

#include <math.h>

#include "stm32f4xx_conf.h"
#include "FreeRTOS.h"
#include "task.h"

//#include "debug.h"
//#include "cfassert.h"
#include "imu.h"
#include "i2cdev.h"
#include "mpu6050.h"

#define TRUE 1
#define FALSE 0




#define IMU_GYRO_FS_CFG       0x03//MPU6050_GYRO_FS_2000
#define IMU_DEG_PER_LSB_CFG   (float)((2 * 2000.0) / 65536.0)
#define IMU_ACCEL_FS_CFG      0x02
#define IMU_G_PER_LSB_CFG     (float)((2 * 8) / 65536.0)
#define IMU_1G_RAW            (int16_t)(1.0 / (float)((2 * 8) / 65536.0))

#define IMU_STARTUP_TIME_MS   1000

#define GYRO_NBR_OF_AXES 3
#define GYRO_X_SIGN      (-1)
#define GYRO_Y_SIGN      (-1)
#define GYRO_Z_SIGN      (-1)
#define GYRO_NBR_OF_AXES            3
#define GYRO_MIN_BIAS_TIMEOUT_MS    1*1000

#define IMU_NBR_OF_BIAS_SAMPLES  128

#define GYRO_VARIANCE_BASE        4000
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)

typedef struct
{
  Axis3i16   bias;
  bool       isBiasValueFound;
  bool       isBufferFilled;
  Axis3i16*  bufHead;
  Axis3i16   buffer[IMU_NBR_OF_BIAS_SAMPLES];
} BiasObj;

BiasObj    gyroBias;
BiasObj    accelBias;
int32_t    varianceSampleTime;
Axis3i16   gyroMpu;
Axis3i16   accelMpu;
Axis3i16   accelLPF;
Axis3i16   accelLPFAligned;
Axis3i16   mag;
Axis3i32   accelStoredFilterValues;
uint8_t    imuAccLpfAttFactor;
static bool isHmc5883lPresent;
static bool isMs5611Present;



/**
 * MPU6050 selt test function. If the chip is moved to much during the self test
 * it will cause the test to fail.
 */
static void imuBiasInit(BiasObj* bias);
static void imuCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut);
static void imuCalculateVarianceAndMean(BiasObj* bias, Axis3i32* varOut, Axis3i32* meanOut);
static bool imuFindBiasValue(BiasObj* bias);
static void imuAddBiasValue(BiasObj* bias, Axis3i16* dVal);
static void imuAccIIRLPFilter(Axis3i16* in, Axis3i16* out,
                              Axis3i32* storedValues, int32_t attenuation);


int __errno;

static bool isInit;

void imu6Init(void)
{
 // if(isInit)
   // return;


  // Wait for sensors to startup
  //while (xTaskGetTickCount() < M2T(IMU_STARTUP_TIME_MS));
 int i;
 for (i=0; i<10000; i++);

  i2cdevInit(I2C1);
  mpu6050Init(I2C1);

  if (mpu6050TestConnection() == TRUE)
  {
    //DEBUG_PRINT("MPU6050 I2C connection [OK].\n");
  }
  else
  {
   // DEBUG_PRINT("MPU6050 I2C connection [FAIL].\n");
  }

  //mpu6050Reset();
  //vTaskDelay(M2T(50));
  //for (i=0; i<10000; i++);
  // Activate MPU6050
  mpu6050SetSleepEnabled(FALSE);
  // Enable temp sensor
  mpu6050SetTempSensorEnabled(TRUE);
  // Disable interrupts
  mpu6050SetIntEnabled(FALSE);
  // Connect the HMC5883L to the main I2C bus
  mpu6050SetI2CBypassEnabled(TRUE);
  // Set x-axis gyro as clock source
  mpu6050SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
  // Set gyro full scale range
  mpu6050SetFullScaleGyroRange(IMU_GYRO_FS_CFG);
  // Set accelerometer full scale range
  mpu6050SetFullScaleAccelRange(IMU_ACCEL_FS_CFG);


  // Set output rate (1): 1000 / (1 + 1) = 500Hz
  mpu6050SetRate(0); //mpu6050SetRate(1);
  // Set digital low-pass bandwidth. Set to 100 Hz for now.
  mpu6050SetDLPFMode(MPU6050_DLPF_BW_98);

  imuBiasInit(&gyroBias);
  imuBiasInit(&accelBias);
  varianceSampleTime = -GYRO_MIN_BIAS_TIMEOUT_MS + 1;
  imuAccLpfAttFactor = IMU_ACC_IIR_LPF_ATT_FACTOR;

  isInit = TRUE;
  //GPIO_WriteBit(GPIOB, GPIO_Pin_5,  Bit_RESET );
}


void imu6Read(Axis3f* gyroOut, Axis3f* accOut)
{
  mpu6050GetMotion6(&accelMpu.x, &accelMpu.y, &accelMpu.z, &gyroMpu.x, &gyroMpu.y, &gyroMpu.z);

  imuAddBiasValue(&gyroBias, &gyroMpu);

  if (!accelBias.isBiasValueFound)
  {
    imuAddBiasValue(&accelBias, &accelMpu);
  }

  if (!gyroBias.isBiasValueFound)
  {
    imuFindBiasValue(&gyroBias);

  }
  //GPIO_WriteBit(GPIOB, GPIO_Pin_5,  Bit_SET );



  imuAccIIRLPFilter(&accelMpu, &accelLPF, &accelStoredFilterValues,
                    (int32_t)imuAccLpfAttFactor);


  //GPIO_WriteBit(GPIOB, GPIO_Pin_5,  Bit_RESET );


  // Re-map outputs
  gyroOut->x = (gyroMpu.x - gyroBias.bias.x) * IMU_DEG_PER_LSB_CFG;
  gyroOut->y = (gyroMpu.y - gyroBias.bias.y) * IMU_DEG_PER_LSB_CFG;
  gyroOut->z = (gyroMpu.z - gyroBias.bias.z) * IMU_DEG_PER_LSB_CFG;
  accOut->x = (accelLPF.x - accelBias.bias.x) * IMU_G_PER_LSB_CFG;
  accOut->y = (accelLPF.y - accelBias.bias.y) * IMU_G_PER_LSB_CFG;
  accOut->z = (accelLPF.z - accelBias.bias.z) * IMU_G_PER_LSB_CFG;

}

bool imu6IsCalibrated(void)
{
  bool status;

  status = gyroBias.isBiasValueFound;

  return status;
}

static void imuBiasInit(BiasObj* bias)
{
  bias->isBufferFilled = FALSE;
  bias->bufHead = bias->buffer;
}

/**
 * Calculates the variance and mean for the bias buffer.
 */
static void imuCalculateVarianceAndMean(BiasObj* bias, Axis3i32* varOut, Axis3i32* meanOut)
{
  uint32_t i;
  int32_t sum[GYRO_NBR_OF_AXES] = {0};
  int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < IMU_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
    sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
    sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
    sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
  }

  varOut->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / IMU_NBR_OF_BIAS_SAMPLES);
  varOut->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / IMU_NBR_OF_BIAS_SAMPLES);
  varOut->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / IMU_NBR_OF_BIAS_SAMPLES);

  meanOut->x = sum[0] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->y = sum[1] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->z = sum[2] / IMU_NBR_OF_BIAS_SAMPLES;

  isInit = TRUE;
}

/**
 * Calculates the mean for the bias buffer.
 */
static void __attribute__((used)) imuCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut)
{
  uint32_t i;
  int32_t sum[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < IMU_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
  }

  meanOut->x = sum[0] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->y = sum[1] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->z = sum[2] / IMU_NBR_OF_BIAS_SAMPLES;

}

/**
 * Adds a new value to the variance buffer and if it is full
 * replaces the oldest one. Thus a circular buffer.
 */
static void imuAddBiasValue(BiasObj* bias, Axis3i16* dVal)
{
  bias->bufHead->x = dVal->x;
  bias->bufHead->y = dVal->y;
  bias->bufHead->z = dVal->z;
  bias->bufHead++;

  if (bias->bufHead >= &bias->buffer[IMU_NBR_OF_BIAS_SAMPLES])
  {
    bias->bufHead = bias->buffer;
    bias->isBufferFilled = TRUE;
  }
}

/**
 * Checks if the variances is below the predefined thresholds.
 * The bias value should have been added before calling this.
 * @param bias  The bias object
 */
static bool imuFindBiasValue(BiasObj* bias)
{
  bool foundBias = FALSE;

  if (bias->isBufferFilled)
  {
    Axis3i32 variance;
    Axis3i32 mean;

    imuCalculateVarianceAndMean(bias, &variance, &mean);


    if (variance.x < GYRO_VARIANCE_THRESHOLD_X &&
        variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
        variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
        (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount()))
    {
      varianceSampleTime = xTaskGetTickCount();
      bias->bias.x = mean.x;
      bias->bias.y = mean.y;
      bias->bias.z = mean.z;
      foundBias = TRUE;
      bias->isBiasValueFound = TRUE;
    }
  }

  return foundBias;
}

static void imuAccIIRLPFilter(Axis3i16* in, Axis3i16* out, Axis3i32* storedValues, int32_t attenuation)
{
  out->x = iirLPFilterSingle(in->x, attenuation, &storedValues->x);
  out->y = iirLPFilterSingle(in->y, attenuation, &storedValues->y);
  out->z = iirLPFilterSingle(in->z, attenuation, &storedValues->z);
}



