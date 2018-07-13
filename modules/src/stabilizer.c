/**
 * File that receives IMU data, gains, pilot inputs and update control signals to actuators
 * ELKA control software, UMD
 */

#include <inttypes.h>
#include <stdint.h>
#include <string.h>

#include "stm32f4xx_conf.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "math.h"
#include "semphr.h"

#include "stabilizer.h"
#include "sensfusion6.h"
#include "imu.h"
#include "motors.h"
#include <stdbool.h>
#include "queue.h"
#include "hw_config.h"
#include "mpu6050.h"

//#include "px4flow.h"

#define PRIVATE

Axis3f gyro; // Gyro axis data in deg/s
Axis3f acc;  // Accelerometer axis data in mG
Axis3i16   gyroMpu;
Axis3i16   accelMpu;
int16_t pitchkp = -2;
int16_t pitchkd = 40;
int16_t rollkp = 2;
int16_t rollkd = 40;
int16_t yawkp = 80;
int16_t pitchki, rollki;
float f_actuatorThrust, f_actuatorRoll, f_actuatorPitch, f_actuatorYaw;

PRIVATE float eulerRollActual;
PRIVATE float eulerPitchActual;
PRIVATE float eulerYawActual;
PRIVATE float eulerRollDesired;
PRIVATE float eulerPitchDesired;
PRIVATE float eulerYawDesired;
PRIVATE float rollRateDesired;
PRIVATE float pitchRateDesired;
PRIVATE float yawRateDesired;
PRIVATE float fusionDt;

int16_t actuatorThrust = 1000;
int16_t  actuatorRoll = 0;
int16_t  actuatorPitch = 0;
int16_t  actuatorYaw =0;
int16_t actuatorLeftServo = 1500;
int16_t actuatorRightServo = 1500;
int16_t baseline = 1000;

int16_t trimThrust = 900; //900
int16_t tempThrust = 900;
int16_t trimRoll = 0;
int16_t trimPitch = 0;
int16_t trimYaw = 0;
int16_t trimLeftServo = 1500;
int16_t trimRightServo = 1500;
float eulerPitchDesired = 0.0;
int8_t t_sens = 10;//10
int8_t r_sens = 2;//3
int8_t p_sens = 2;//3
int8_t y_sens = 10;//30;
float thrust_sens = 1.0;
float roll_sens = 1.0;
float yaw_sens = 1.0;
float pitch_sens = 1.0;

// Gain sensitivity (useful if gains sent from LabView)
float _labview_pos_sens = 1e-4;
float _labview_vel_sens = 1e-3;
float _labview_int_sens = 1e-4;
float _labview_ang_sens = 1e-2;

uint32_t leftmotor;
uint32_t rightmotor;
uint32_t tail;
uint32_t ls;
uint32_t rs;

int32_t motorPowerLeftfront;
int32_t motorPowerRightfront;
int32_t motorPowerLeftrear;
int32_t motorPowerRightrear;

float gyro_x_filt;
float gyro_y_filt;
float gyro_z_filt;

float theta;

float _pos_z_fb, _dpos_z_fb;

float _pitch_flow, _roll_flow;

uint8_t _snap_serial_state;

#if defined(ELKA_DEBUG)
float dbg_[20];
#endif

static void distributePower(const uint16_t thrust, const int16_t roll,
		const int16_t pitch, const int16_t yaw, const int16_t lefts, const int16_t rights);
static uint16_t limitThrust(int32_t value);
static void stabilizerTask(void* param);
static void imuinit();
// Parse most recent packet from snapdragon
int16_t snap_cmd_parse(
    pose_s *cmd);
void rotate_pose(pose_s *init,
                 pose_s *rot);
void copy_pose_cmds(pose_s *src,
                    pose_s *dst);
// Form actuator command for current euler angles
// and most recent packet from snapdragon
void form_actuator_ctl_cmd(
    int16_t msg_type,
    pose_s *pose_desired,
    actuator_ctl_s *actuator_ctl);
int16_t check_header(
    rx_xbee *buf_rx);

xQueueHandle  tx1Queue, eulerqueue,spektrumqueue;
xSemaphoreHandle queuewritten;

//float f_snapCh1, f_snapCh2, f_snapCh3, f_snapCh4;
int16_t snapCh1, snapCh2, snapCh3, snapCh4;
//extern cb le_snap;

/* Initialize Motors, IMU, Sensor Fusion and Stabilizer Task */
void stabilizerInit(void)
{
	//if(isInit)
	//return;

	motorsInit();
	imu6Init();
	sensfusion6Init();
	int i;
	for (i=0; i<100000; i++);
	//init_I2C2();
	rollRateDesired = 0;
	pitchRateDesired = 0;
	yawRateDesired = 0;
	//vSemaphoreCreateBinary(queuewritten);


	xTaskCreate(stabilizerTask, (const signed char * const)"STABILIZER",
			configMINIMAL_STACK_SIZE+500, NULL, /*Piority*/1, NULL);

	//isInit = TRUE;
}

static void stabilizerTask(void* param)
{
	//int i;
	imuinit();
	uint32_t lastWakeTime;
	lastWakeTime = xTaskGetTickCount ();
	uint32_t attitudeCounter = 0;
	//int initialize=0;
	//int spektrumchanneltemp[6];


	//-------------------------------------------------------------------------------------

  pose_s pose_desired;
  actuator_ctl_s actuator_ctl;
  elka_packet_s elka_pkt;
  int16_t snap_msg_type;
  memset(&pose_desired, 0, sizeof(pose_desired));
  memset(&actuator_ctl, 0, sizeof(actuator_ctl));
  memset(&elka_pkt,0,sizeof(elka_pkt));

  _roll_flow = 0;
  _pitch_flow = 0;

  // Default pose feedback
  pose_desired.gains.k[0]=.1; // => 1000mm error -> +100 fb
  pose_desired.gains.k[1]=.1; // => 1000mm/s error -> +100 fb
  pose_desired.gains.k[6]=8.5; // => 1 rad error -> +10 fb
  pose_desired.gains.k[7]=9; // => 1 rad/s error -> +15 fb

  pose_desired.x_sens=0.006; // 5000mm error -> 30deg error
  pose_desired.y_sens=0.006; // 5000mm error -> 30deg error
  pose_desired.vx_sens=0.01; // 3000mm/s error -> 30deg error
  pose_desired.vy_sens=0.01; // 3000mm/s error -> 30deg error

  pose_desired.spektrum_roll_trim = RAW_ROLL_BASELINE;
  pose_desired.spektrum_pitch_trim = RAW_PITCH_BASELINE;
  pose_desired.spektrum_thrust_trim = RAW_THRUST_BASELINE;
  pose_desired.spektrum_yaw_trim = RAW_YAW_BASELINE;

  memset(pose_desired.e,0,ELKA_MSG_POS_DATA_LEN);

	while (1)
	{

	  /*
		for(i=0;i<1000;i++)
		{
			if (spektrumchannel[5]>0)
			{
				if(initialize==0)
				{
					for (i=0;i<5;i++)
					{
						spektrumchanneltemp[i] = spektrumchannel[i];
					}
					//actuatorThrustTemp =

				}
				initialize=1;
			}
		}
		*/

		vTaskDelayUntil(&lastWakeTime,F2T(IMU_UPDATE_FREQ));
		//start = TIM4->CNT;
		imu6Read(&gyro, &acc);
		//mpu6050GetMotion6(&accelMpu.x, &accelMpu.y, &accelMpu.z, &gyroMpu.x, &gyroMpu.y, &gyroMpu.z);
		if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
		{
			sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, FUSION_UPDATE_DT);
			sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
			attitudeCounter = 0;
			euler.data[0] = (int)(eulerRollActual*100);
			euler.data[1] = (int)((eulerPitchActual)*100);
			euler.data[2] = (int)(eulerPitchDesired*100);
		}

		if (rx_buf.data[1]==PID_GAINS && rx_buf.data[2]==255 && rx_buf.data[3] == 255)
		{

			pitchkp = (rx_buf.data[4]<<8)+rx_buf.data[5];
			pitchki = (rx_buf.data[6]<<8)+rx_buf.data[7];
			pitchkd = (rx_buf.data[8]<<8)+rx_buf.data[9];
			rollkp = (rx_buf.data[10]<<8)+rx_buf.data[11];
			rollki = (rx_buf.data[12]<<8)+rx_buf.data[13];
			rollkd = (rx_buf.data[14]<<8)+rx_buf.data[15];
			yawkp = (rx_buf.data[16]<<8)+rx_buf.data[17];

		  pose_desired.x_sens = ((float)((rx_buf.data[18]<<8)+rx_buf.data[19]))*
					_labview_pos_sens;
		  pose_desired.y_sens=pose_desired.x_sens;
		  pose_desired.vx_sens = ((float)((rx_buf.data[20]<<8)+rx_buf.data[21]))*
					_labview_vel_sens;
		  pose_desired.vy_sens=pose_desired.vx_sens;
		  pose_desired.ix_sens = ((float)((rx_buf.data[22]<<8)+rx_buf.data[23]))*
		      _labview_int_sens;
		  pose_desired.iy_sens = pose_desired.ix_sens;
		  pose_desired.gains.k[0] = ((float)((rx_buf.data[24]<<8)+rx_buf.data[25]))*
					_labview_pos_sens;
		  pose_desired.gains.k[1] = ((float)((rx_buf.data[26]<<8)+rx_buf.data[27]))*
					_labview_vel_sens;
		  pose_desired.gains.k[2] = ((float)((rx_buf.data[28]<<8)+rx_buf.data[29]))*
          _labview_int_sens;
		  pose_desired.gains.k[7] = ((float)((rx_buf.data[30]<<8)+rx_buf.data[31]))*
		      _labview_ang_sens;
		  pose_desired.gains.k[9] = ((float)((rx_buf.data[32]<<8)+rx_buf.data[33]))*
		      _labview_int_sens;
		}

		else if (rx_buf.data[1] == SENSITIVITY && rx_buf.data[2]==255 && rx_buf.data[3] == 255)
		{
			t_sens = (rx_buf.data[4]<<8)+rx_buf.data[5];
			r_sens = (rx_buf.data[6]<<8)+rx_buf.data[7];
			p_sens = (rx_buf.data[8]<<8)+rx_buf.data[9];
			y_sens = (rx_buf.data[10]<<8)+rx_buf.data[11];
		}
		else if (rx_buf.data[1] == TRIM && rx_buf.data[2]==255 && rx_buf.data[3] == 255)// && k==2)
		{
			trimThrust = (rx_buf.data[4]<<8)+rx_buf.data[5];
			trimRoll = (rx_buf.data[6]<<8)+rx_buf.data[7];
			trimPitch = (rx_buf.data[8]<<8)+rx_buf.data[9];
			trimYaw = (rx_buf.data[10]<<8)+rx_buf.data[11];
			trimLeftServo = (rx_buf.data[12]<<8)+rx_buf.data[13];
			trimRightServo = (rx_buf.data[14]<<8)+rx_buf.data[15];
		}

    gyro_x_filt = 0.8*gyro_x_filt+0.2*gyro.x;
    gyro_y_filt = 0.8*gyro_y_filt+0.2*gyro.y;
    gyro_z_filt = 0.8*gyro_z_filt+0.2*gyro.z;

    pose_desired.spektrum_roll = pose_desired.spektrum_roll_trim;
    pose_desired.spektrum_pitch = pose_desired.spektrum_pitch_trim;
    pose_desired.spektrum_thrust = pose_desired.spektrum_thrust_trim;
    pose_desired.spektrum_yaw = pose_desired.spektrum_yaw_trim;
    pose_desired.f_roll_d = r_sens*(pose_desired.spektrum_roll-
                                  RAW_ROLL_BASELINE)*0.10;
    pose_desired.f_pitch_d = p_sens*(pose_desired.spektrum_pitch-
                                   RAW_PITCH_BASELINE)*0.10;

    snap_msg_type=snap_cmd_parse(
        &pose_desired);

    switch(snap_msg_type) {
      case MSG_TYPE_ERROR:
        break;
      case MSG_TYPE_NONE:
        break;
      case MSG_TYPE_KILL:
        pose_desired.f_roll_d = r_sens*(pose_desired.spektrum_roll-
                                      RAW_ROLL_BASELINE)*0.10;
        pose_desired.f_pitch_d = p_sens*(pose_desired.spektrum_pitch-
                                       RAW_PITCH_BASELINE)*0.10;
      case MSG_TYPE_SPEKTRUM:
        // Use pitch stick to command attitude angle
        // instead of actuator inputs.
        pose_desired.f_roll_d = r_sens*(pose_desired.spektrum_roll-
                                    RAW_ROLL_BASELINE)*0.10;
        pose_desired.f_pitch_d = p_sens*(pose_desired.spektrum_pitch-
                                     RAW_PITCH_BASELINE)*0.10;
        memset(pose_desired.integral_bias,'0',sizeof(pose_desired.integral_bias));
        break;
      case MSG_TYPE_SETPOINT:
      case MSG_TYPE_VISION_POS:
      case MSG_TYPE_LOCAL_POS:

        // Update integral bias
        // x integral bias
        if (pose_desired.e[0] > INTEGRAL_GAIN_LINEAR_THRESHOLD)
          pose_desired.integral_bias[0] = 0;
        else
          pose_desired.integral_bias[0] += pose_desired.e[0]*pose_desired.dt;

        // y integral bias
        if (pose_desired.e[1] > INTEGRAL_GAIN_LINEAR_THRESHOLD)
          pose_desired.integral_bias[1] = 0;
        else
          pose_desired.integral_bias[1] += pose_desired.e[1]*pose_desired.dt;

        // z integral bias
        if (pose_desired.e[2] > INTEGRAL_GAIN_LINEAR_THRESHOLD)
          pose_desired.integral_bias[2] = 0;
        else
          pose_desired.integral_bias[2] += pose_desired.e[2]*pose_desired.dt;

        // yaw integral bias
        if (pose_desired.e[6] > INTEGRAL_GAIN_ANGULAR_THRESHOLD)
          pose_desired.integral_bias[3] = 0;
        else
          pose_desired.integral_bias[3] += pose_desired.e[6]*pose_desired.dt;

        // Scale error by sensitivity to get desired roll angle
        pose_desired.f_roll_d = pose_desired.x_sens*pose_desired.e[0] +
            pose_desired.vx_sens*pose_desired.e[3] +
            pose_desired.ix_sens*pose_desired.integral_bias[0];
        pose_desired.f_pitch_d = pose_desired.y_sens*pose_desired.e[1] +
            pose_desired.vy_sens*pose_desired.e[4] +
            pose_desired.iy_sens*pose_desired.integral_bias[1];

        // Impose limits to desired roll/pitch angles
        limit_pose_desired(&pose_desired);
        break;
      case MSG_TYPE_THRUST:
        //pose_desired.spektrum_thrust = snap_cmd.spektrum_thrust;
        break;
      case MSG_TYPE_GAINS:
        break;
      case MSG_TYPE_TEST:
        break;
      default:
        break;
    }

    form_actuator_ctl_cmd(
      snap_msg_type,
      &pose_desired,
      &actuator_ctl);

#if defined(ELKA_DEBUG) && defined(DEBUG_POSE_ERROR)
    euler.data[3] = (int16_t)(((int32_t)pose_desired.e[0] >> 16) & 0xffff);
    euler.data[4] = (int16_t)((int32_t)pose_desired.e[0] & 0xffff);
    euler.data[5] = (int16_t)(((int32_t)pose_desired.e[1] >> 16) & 0xffff);
    euler.data[6] = (int16_t)((int32_t)pose_desired.e[1] & 0xffff);
    euler.data[7] = (int16_t)(((int32_t)pose_desired.e[4] >> 16) & 0xffff);
    euler.data[8] = (int16_t)((int32_t)pose_desired.e[4] & 0xffff);
    euler.data[9] = (int16_t)(((int32_t)pose_desired.e[5] >> 16) & 0xffff);
    euler.data[10] = (int16_t)((int32_t)pose_desired.e[5] & 0xffff);
    euler.data[11] = (int16_t)(((int32_t)pose_desired.e[6] >> 16) & 0xffff);
    euler.data[12] = (int16_t)((int32_t)pose_desired.e[6] & 0xffff);
    /*
    dbg_[0]=pose_desired.e[0];
    dbg_[1]=pose_desired.e[1];
    dbg_[2]=pose_desired.e[2];
    dbg_[3]=pose_desired.e[3];
    dbg_[4]=pose_desired.e[4];
    dbg_[5]=pose_desired.e[5];
    dbg_[6]=pose_desired.e[6];
    dbg_[7]=pose_desired.e[7];
    dbg_[8]=actuator_ctl.thrust;
    dbg_[9]=actuator_ctl.roll;
    dbg_[10]=actuator_ctl.pitch;
    dbg_[11]=actuator_ctl.yaw;
    dbg_[12]=pose_desired.x_sens;
    dbg_[13]=pose_desired.y_sens;
    dbg_[14]=pose_desired.vx_sens;
    dbg_[15]=pose_desired.vy_sens;
    dbg_[16]=pose_desired.gains.k[0];
    dbg_[17]=pose_desired.gains.k[1];
    dbg_[18]=pose_desired.gains.k[6];
    dbg_[19]=pose_desired.gains.k[7];
    */
#elif defined(ELKA_DEBUG) && defined(DEBUG_CTL_INPUTS)
    euler.data[3] = (int16_t)(((int32_t)actuator_ctl.thrust >> 16) & 0xffff);
    euler.data[4] = (int16_t)((int32_t)actuator_ctl.thrust & 0xffff);
    euler.data[5] = (int16_t)(((int32_t)actuator_ctl.roll >> 16) & 0xffff);
    euler.data[6] = (int16_t)((int32_t)actuator_ctl.roll & 0xffff);
    euler.data[7] = (int16_t)(((int32_t)actuator_ctl.pitch >> 16) & 0xffff);
    euler.data[8] = (int16_t)((int32_t)actuator_ctl.pitch & 0xffff);
    euler.data[9] = (int16_t)(((int32_t)actuator_ctl.yaw >> 16) & 0xffff);
    euler.data[10] = (int16_t)((int32_t)actuator_ctl.yaw & 0xffff);
    /*
    dbg_[0]=pose_desired.e[0];
    dbg_[1]=pose_desired.e[1];
    dbg_[2]=pose_desired.e[2];
    dbg_[3]=pose_desired.e[3];
    dbg_[4]=pose_desired.e[4];
    dbg_[5]=pose_desired.e[5];
    dbg_[6]=pose_desired.e[6];
    dbg_[7]=pose_desired.e[7];
    dbg_[8]=actuator_ctl.thrust;
    dbg_[9]=actuator_ctl.roll;
    dbg_[10]=actuator_ctl.pitch;
    dbg_[11]=actuator_ctl.yaw;
    dbg_[12]=pose_desired.x_sens;
    dbg_[13]=pose_desired.y_sens;
    dbg_[14]=pose_desired.vx_sens;
    dbg_[15]=pose_desired.vy_sens;
    dbg_[16]=pose_desired.gains.k[0];
    dbg_[17]=pose_desired.gains.k[1];
    dbg_[18]=pose_desired.gains.k[6];
    dbg_[19]=pose_desired.gains.k[7];
    */
#elif defined(ELKA_DEBUG) && defined(DEBUG_YAW)
    euler.data[3]=(int16_t)(((int32_t)(pose_desired.e[6]*100)>>16)&0xffff);
    euler.data[4]=(int16_t)((int32_t)(pose_desired.e[6]*100)&0xffff);
    euler.data[5]=(int16_t)(((int32_t)(pose_desired.e[7]*100)>>16)&0xffff);
    euler.data[6]=(int16_t)((int32_t)(pose_desired.e[7]*100)&0xffff);
    euler.data[7]=(int16_t)(((int32_t)(pose_desired.gains.k[6]*100)>>16)&0xffff);
    euler.data[8]=(int16_t)((int32_t)(pose_desired.gains.k[6]*100)&0xffff);
    euler.data[9]=(int16_t)(((int32_t)(pose_desired.gains.k[7]*100)>>16)&0xffff);
    euler.data[10]=(int16_t)((int32_t)(pose_desired.gains.k[7]*100)&0xffff);
    euler.data[11] = (int16_t)(((int32_t)actuator_ctl.yaw >> 16) & 0xffff);
    euler.data[12] = (int16_t)((int32_t)actuator_ctl.yaw & 0xffff);
#elif defined(ELKA_DEBUG) && defined(DEBUG_GAINS)
    euler.data[3] = (int16_t)(((int32_t)(pose_desired.x_sens*1000) >> 16) & 0xffff);
    euler.data[4] = (int16_t)((int32_t)(pose_desired.x_sens*1000) & 0xffff);
    euler.data[5] = (int16_t)(((int32_t)(pose_desired.vx_sens*1000) >> 16) & 0xffff);
    euler.data[6] = (int16_t)((int32_t)(pose_desired.vx_sens*1000) & 0xffff);
    euler.data[7] = (int16_t)(((int32_t)rollkp >> 16) & 0xffff);
    euler.data[8] = (int16_t)((int32_t)rollkp & 0xffff);
    euler.data[9] = (int16_t)(((int32_t)(pose_desired.gains.k[0]*1000) >> 16) & 0xffff);
    euler.data[10] = (int16_t)((int32_t)(pose_desired.gains.k[0]*1000) & 0xffff);
    euler.data[11] = (int16_t)(((int32_t)(pose_desired.gains.k[1]*1000) >> 16) & 0xffff);
    euler.data[12] = (int16_t)((int32_t)(pose_desired.gains.k[1]*1000) & 0xffff);
#endif

    distributePower(actuator_ctl.thrust,
                    actuator_ctl.roll,
                    actuator_ctl.pitch,
                    actuator_ctl.yaw,
                    0, 0);

	}
}

static void distributePower(const uint16_t thrust, const int16_t roll,
		const int16_t pitch, const int16_t yaw, const int16_t lefts, const int16_t rights)
{
	motorPowerLeftfront =  limitThrust(thrust + roll - pitch - yaw );
	motorPowerRightfront = limitThrust(thrust - roll - pitch + yaw );
	motorPowerLeftrear = limitThrust(thrust + roll + pitch + yaw );
	motorPowerRightrear =  limitThrust(thrust - roll + pitch - yaw );

	motorsSetRatio(MOTOR_LEFTFRONT, motorPowerLeftfront);
	motorsSetRatio(MOTOR_RIGHTFRONT, motorPowerRightfront);
	motorsSetRatio(MOTOR_LEFTREAR, motorPowerLeftrear);
	motorsSetRatio(MOTOR_RIGHTREAR, motorPowerRightrear);
}

static uint16_t limitThrust(int32_t value)
{
	if(value > 2000)
	{
		value = 2000;
	}
	else if(value < 1000 && value >200)
	{
		value = 1000;
	}

	else if(value <200)
	{
		value = 0;
	}

	return (uint16_t)value;
}

void limit_pose_desired(pose_s *p) {
  if (p->f_roll_d > ROLL_LIM) p->f_roll_d = ROLL_LIM;
  if (p->f_roll_d < -ROLL_LIM) p->f_roll_d = -ROLL_LIM;
  if (p->f_pitch_d > PITCH_LIM) p->f_pitch_d = PITCH_LIM;
  if (p->f_pitch_d < -PITCH_LIM) p->f_pitch_d = -PITCH_LIM;
}

void limit_feedback(actuator_ctl_s *c) {
  if (c->f_thrust > RAW_THRUST_LIM) c->f_thrust = RAW_THRUST_LIM;
  else if (c->f_thrust < trimThrust) c->f_thrust = trimThrust;
  if (c->f_yaw > YAW_FB_LIM) c->f_yaw = YAW_FB_LIM;
  if (c->f_yaw < -YAW_FB_LIM) c->f_yaw = -YAW_FB_LIM;
}


void imuinit(void)
{
	static int i;
	mpu6050Reset();
	//vTaskDelay(M2T(50));
	for (i=0; i<10000; i++);
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
	mpu6050SetFullScaleGyroRange( MPU6050_GYRO_FS_2000);
	// Set accelerometer full scale range
	mpu6050SetFullScaleAccelRange(MPU6050_ACCEL_FS_8);


	// Set output rate (1): 1000 / (1 + 1) = 500Hz
	mpu6050SetRate(0); //mpu6050SetRate(1);
	// Set digital low-pass bandwidth. Set to 100 Hz for now.
	mpu6050SetDLPFMode(MPU6050_DLPF_BW_98);
}

int16_t snap_cmd_parse(
    pose_s *cmd) {
  static rx_xbee tmp;
  memcpy(&tmp,&snap_buf_rx,sizeof(rx_xbee));
  int16_t msg_type = check_header(&tmp);

  if (msg_type == MSG_TYPE_ERROR) {
    _snap_serial_state = SERIAL_STATE_NONE;
  } else if (msg_type == MSG_TYPE_NONE) {
    _snap_serial_state = SERIAL_STATE_NONE;
  } else if (msg_type == MSG_TYPE_KILL) {
    _snap_serial_state = SERIAL_STATE_KILL;

    cmd->spektrum_roll=RAW_ROLL_BASELINE;
    cmd->spektrum_pitch=RAW_PITCH_BASELINE;
    cmd->spektrum_thrust=RAW_THRUST_BASELINE;
    cmd->spektrum_yaw=RAW_YAW_BASELINE;

    cmd->spektrum_roll_trim=RAW_ROLL_BASELINE;
    cmd->spektrum_pitch_trim=RAW_PITCH_BASELINE;
    cmd->spektrum_thrust_trim=RAW_THRUST_BASELINE;
    cmd->spektrum_yaw_trim=RAW_YAW_BASELINE;

    /*
    deserialize(
        &(cmd->spektrum_roll_trim),
        &(tmp.data[ELKA_LOCAL_MSG_OFFSET+SERIAL_HEADER_DATA_OFFSET]),
        2);
    deserialize(
        &(cmd->spektrum_pitch_trim),
        &(tmp.data[ELKA_LOCAL_MSG_OFFSET+SERIAL_HEADER_DATA_OFFSET+2]),
        2);
    deserialize(
        &(cmd->spektrum_thrust_trim),
        &(tmp.data[ELKA_LOCAL_MSG_OFFSET+SERIAL_HEADER_DATA_OFFSET+4]),
        2);
    deserialize(
        &(cmd->spektrum_yaw_trim),
        &(tmp.data[ELKA_LOCAL_MSG_OFFSET+SERIAL_HEADER_DATA_OFFSET+6]),
        2);
     */

  } else if (msg_type == MSG_TYPE_SPEKTRUM) {
    _snap_serial_state = SERIAL_STATE_SPEKTRUM;

    deserialize(
        &(cmd->spektrum_roll),
        &tmp.data[ELKA_LOCAL_MSG_OFFSET+SERIAL_HEADER_DATA_OFFSET],
        2);
    deserialize(
        &(cmd->spektrum_pitch),
        &tmp.data[ELKA_LOCAL_MSG_OFFSET+SERIAL_HEADER_DATA_OFFSET+2],
        2);
    deserialize(
        &(cmd->spektrum_thrust),
        &tmp.data[ELKA_LOCAL_MSG_OFFSET+SERIAL_HEADER_DATA_OFFSET+4],
        2);
    deserialize(
        &(cmd->spektrum_yaw),
        &tmp.data[ELKA_LOCAL_MSG_OFFSET+SERIAL_HEADER_DATA_OFFSET+6],
        2);

  } else if (msg_type == MSG_TYPE_VISION_POS ||
             msg_type == MSG_TYPE_LOCAL_POS ||
             msg_type == MSG_TYPE_SETPOINT) {

    // Receive dt, hover thrust, and error array
    deserialize(
      &(cmd->dt),
      &(tmp.data[ELKA_LOCAL_MSG_OFFSET+SERIAL_HEADER_DATA_OFFSET]),
      4);
    deserialize(
      &(cmd->spektrum_thrust),
      &(tmp.data[ELKA_LOCAL_MSG_OFFSET+SERIAL_HEADER_DATA_OFFSET+4]),
      2);
    deserialize(
      &(cmd->e),
      &(tmp.data[ELKA_LOCAL_MSG_OFFSET+SERIAL_HEADER_DATA_OFFSET+6]),
      ELKA_MSG_POS_DATA_LEN);

    if (msg_type == MSG_TYPE_VISION_POS) {
      _snap_serial_state = SERIAL_STATE_VISION_POS;
    } else if (msg_type == MSG_TYPE_LOCAL_POS) {
      _snap_serial_state = SERIAL_STATE_LOCAL_POS;
    } else if (msg_type == MSG_TYPE_SETPOINT) {
      _snap_serial_state = SERIAL_STATE_SETPOINT;
    }
  } else if (msg_type == MSG_TYPE_THRUST) {
    // Don't set serial state
    //_snap_serial_state = SERIAL_STATE_THRUST;
    deserialize(
      &(cmd->spektrum_thrust),
      &(tmp.data[ELKA_LOCAL_MSG_OFFSET+SERIAL_HEADER_DATA_OFFSET]),
      2);

  } else if (msg_type == MSG_TYPE_GAINS) {
    // Pass for now
  } else if (msg_type == MSG_TYPE_TEST) {
    // Do nothing here. Return message is formed in main loop
  } else {
    _snap_serial_state = SERIAL_STATE_NONE;
  }

  return msg_type;
}

void copy_pose_cmds(pose_s *src,
                    pose_s *dst) {
  dst->spektrum_roll = src->spektrum_roll;
  dst->spektrum_pitch = src->spektrum_pitch;
  dst->spektrum_thrust = src->spektrum_thrust;
  dst->spektrum_yaw = src->spektrum_yaw;
  dst->f_roll_d = src->f_roll_d;
  dst->f_pitch_d = src->f_pitch_d;
  dst->f_yaw_d = src->f_yaw_d;
}

void form_actuator_ctl_cmd(
    int16_t msg_type,
    pose_s *pose_desired,
    actuator_ctl_s *actuator_ctl) {

  static float f_thrust_prev;
  f_thrust_prev = actuator_ctl->f_thrust;

  // Attitude ctl + spektrum ctl
  actuator_ctl->f_roll =
    rollkp*(eulerRollActual-pose_desired->f_roll_d) +
    rollkd*gyro_x_filt*0.017 + trimRoll;
  actuator_ctl->f_pitch =
    pitchkp*(eulerPitchActual-pose_desired->f_pitch_d) +
    pitchkd*gyro_y_filt*0.017 + trimPitch;
  actuator_ctl->f_thrust =
    (t_sens*(pose_desired->spektrum_thrust-RAW_THRUST_BASELINE)*0.1) + trimThrust;
  actuator_ctl->f_yaw =
    y_sens*(pose_desired->spektrum_yaw-RAW_YAW_BASELINE)*0.1;

  switch(msg_type) {
    case MSG_TYPE_ERROR:
    case MSG_TYPE_NONE:
    case MSG_TYPE_KILL:
      actuator_ctl->f_roll = 0;
      actuator_ctl->f_pitch = 0;
      actuator_ctl->f_yaw = 0;
      actuator_ctl->f_thrust = trimThrust;
      break;
    case MSG_TYPE_SPEKTRUM:
      actuator_ctl->f_yaw += trimYaw - yawkp*gyro_z_filt*0.017;
      break;
    case MSG_TYPE_THRUST:
      break;
    case MSG_TYPE_VISION_POS:
    case MSG_TYPE_LOCAL_POS:
    case MSG_TYPE_SETPOINT:
      // Altitude feedback
      _dpos_z_fb = -pose_desired->gains.k[0]*pose_desired->e[2]
                   -pose_desired->gains.k[1]*pose_desired->e[5]
                   -pose_desired->gains.k[2]*pose_desired->integral_bias[2];

      // Limit thrust change
      if (_dpos_z_fb > DVERT_POS_FB_LIM) _dpos_z_fb = DVERT_POS_FB_LIM;
      if (_dpos_z_fb < -DVERT_POS_FB_LIM) _dpos_z_fb = -DVERT_POS_FB_LIM;
      actuator_ctl->f_thrust += _dpos_z_fb;


      // Yaw feedback -> yaw angle fb from vio
      //              -> yaw rate fb from gyro measurements
      actuator_ctl->f_yaw += pose_desired->gains.k[7]
          *sin(pose_desired->e[6]/2)*SIGN(M_PI-fabs(pose_desired->e[6]))
          + trimYaw - yawkp*gyro_z_filt*0.017 +
          pose_desired->gains.k[9]*pose_desired->integral_bias[3];

      // Limit thrust and yaw feedback
      limit_feedback(actuator_ctl);

      break;
    default:
      break;
  }

  if (actuator_ctl->f_thrust - f_thrust_prev > DTHRUST_MAX)
    actuator_ctl->f_thrust = f_thrust_prev + DTHRUST_MAX;
  else if (actuator_ctl->f_thrust - f_thrust_prev < -DTHRUST_MAX)
    actuator_ctl->f_thrust = f_thrust_prev - DTHRUST_MAX;

  actuator_ctl->thrust =
    (int)(actuator_ctl->f_thrust);
  actuator_ctl->roll =
    (int)(actuator_ctl->f_roll);
  actuator_ctl->pitch =
    (int)(actuator_ctl->f_pitch);
  actuator_ctl->yaw =
    (int)(actuator_ctl->f_yaw);
}

int16_t check_header(
    rx_xbee *buf_rx) {
  if (buf_rx->data[ELKA_LOCAL_MSG_OFFSET+SERIAL_HEADER_MSG_TYPE+1] ==
      SERIAL_HEADER_ASSURANCE_BYTE &&
     buf_rx->data[ELKA_LOCAL_MSG_OFFSET+SERIAL_HEADER_MSG_TYPE+2] ==
      SERIAL_HEADER_ASSURANCE_BYTE) {
    return buf_rx->data[ELKA_LOCAL_MSG_OFFSET+SERIAL_HEADER_MSG_TYPE];
  } else {
    return MSG_TYPE_ERROR;
  }
}
