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
 */
#ifndef STABALIZER_H_
#define STABALIZER_H_

#include <stdbool.h>
//#include <nrf24l01.h>
#include "px4flow.h"
#include "serial_tasks.h"

/* Using frame of reference:
 *  BACKWARD-LEFT-DOWN (x-y-z +)
 *  PITCH FORWARD-ROLL RIGHT-YAW ? (+) //TODO find positive yaw direction
 */

#define ATTITUDE_UPDATE_RATE_DIVIDER  1
#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) //(float)(ATTITUDE_UPDATE_RATE_DIVIDER/configTICK_RATE_HZ) //
#define PID_GAINS 1
#define SENSITIVITY 2
#define TRIM 3
#define SNAPDRAGON 4


/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */

#define NUM_JOYSTICK_CHANNELS 4
// Channel for kill switch
#define SPEKTRUM_KILL_CHANNEL 5
// Channel for switch to resume control with Spektrum RC
#define SPEKTRUM_CONTROL_CHANNEL 6

#define POSITION_OFFSET UINT32_MAX/2
// Defines close enough to setpoint position in meters
#define POSITION_EPSILON 0.1
#define VELOCITY_EPSILON 0.1
#define ANGLE_ERROR_EPSILON 0.1
#define POSITION_ERROR_DEFAULT 0.5
#define VELOCITY_ERROR_DEFAULT 0.5
#define ANGLE_ERROR_DEFAULT 0.5
#define INTEGRAL_GAIN_LINEAR_THRESHOLD 0.8
#define INTEGRAL_GAIN_ANGULAR_THRESHOLD 2.3

#define POSITION_MULTIPLIER_POS_RAW 10000
#define VELOCITY_MULTIPLIER_VEL_RAW 10000
#define ANGLE_MULTIPLIER_ANG_RAW 10000
#define POSITION_MULTIPLIER_MM_RAW 100
#define VELOCITY_MULTIPLIER_MM_RAW 100
#define ANGLE_MULTIPLIER_MRAD_RAW 100

// Float compare epsilon
#define FLOAT_CMP_EPS 1e-7f

// No vertical position limit for change in feedback
#define DVERT_POS_FB_LIM 200.0f // pwm/update
#define RAW_THRUST_LIM 1730.0f // pwm
#define DTHRUST_MAX 20.0f // pwm/update
#define YAW_FB_LIM 100.0f // pwm
#define ROLL_LIM 30.0f // degrees
#define PITCH_LIM 30.0f // degrees

#define SERIAL_STATE_NONE 0
#define SERIAL_STATE_KILL 1
#define SERIAL_STATE_SPEKTRUM 2
#define SERIAL_STATE_VISION_POS 3
#define SERIAL_STATE_LOCAL_POS 4
#define SERIAL_STATE_SETPOINT 5
#define SERIAL_STATE_THRUST 6
// Spektrum roll,pitch,yaw sticks center around ~1500 (+/- trim)
// Spektrum thrust varies from [0 1000] (+/- trim)
#define RAW_ROLL_BASELINE 1500
#define RAW_PITCH_BASELINE 1500
#define RAW_THRUST_BASELINE 0
#define RAW_YAW_BASELINE 1500

// Sign function
#define SIGN(x) (x>0)-(x<0)

typedef struct {
  // raw r,p,t,y from snapdragon/spektrum
  int16_t spektrum_roll,spektrum_pitch,
          spektrum_thrust,spektrum_yaw;
  int16_t spektrum_roll_trim,spektrum_pitch_trim,
          spektrum_thrust_trim,spektrum_yaw_trim;
  // x/y->roll/pitch sensitivity
  float x_sens,y_sens,vx_sens,vy_sens,ix_sens,iy_sens;
  float f_roll_d,f_pitch_d,f_yaw_d;
  // [x,y,z,vx,vy,vz,yaw,vyaw] error
  float e[8];
  // [x,y,z,yaw] integral bias
  float integral_bias[4];
  float dt; // seconds
  // kp,kd gains
  gains_s gains;
} pose_s;

typedef struct {
  float f_thrust, f_roll, f_pitch, f_yaw;
  int32_t thrust, roll, pitch, yaw;
} actuator_ctl_s;

typedef struct
{
    uint8_t data[24];
  } rxpacket;

typedef struct
{
	int16_t data[15];
} eulerstruct;

uint16_t spektrumchannel[7];
i2c_frame px4_test;
int16_t snapCh[4];

eulerstruct euler;

void stabilizerInit(void);

bool stabilizerTest(void);

// Impose roll, pitch desired limits.
// Only necessary for autonomous case.
// Vehicle is fine in manual piloted case.
void limit_pose_desired(pose_s *p);

// Impose thrust, yaw feedback limits.
// Only necessary for autonomuos case.
// Vehicle is fine in manual piloted case.
void limit_feedback(actuator_ctl_s *c);

#endif /* STABALIZER_H_ */
