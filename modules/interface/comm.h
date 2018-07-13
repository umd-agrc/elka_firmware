#ifndef __COMM_H__
#define __COMM_H__

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "math.h"

#define ELKA_DEBUG 1
//#define DEBUG_GAINS 1
//#define DEBUG_POSE_ERROR 1
#define DEBUG_CTL_INPUTS 1
//#define DEBUG_YAW 1

#define SERIAL_HEADER_MSG_LEN 0
#define SERIAL_HEADER_MSG_TYPE 1
#define SERIAL_HEADER_DATA_OFFSET 4
#define ELKA_MSG_TYPE 1
#define ELKA_MSG_DATA_OFFSET 4

// Distance offset of a local message from the global header
#define SERIAL_HEADER_LOCAL_MSG_OFFSET 1
#define ELKA_LOCAL_MSG_OFFSET 1

#define SERIAL_HEADER_LEN 3
#define SERIAL_HEADER_ASSURANCE_BYTE 255
#define ELKA_MSG_HEADER_LEN 3
#define ELKA_MSG_HEADER_ASSURANCE_BYTE 255

// Define lengths of data sections of various messages
#define ELKA_MSG_POS_DATA_LEN 32

#define MSG_TYPE_ERROR -1
#define MSG_TYPE_NONE 0
#define MSG_TYPE_KILL 1
#define MSG_TYPE_SPEKTRUM 2
#define MSG_TYPE_VISION_POS 3
#define MSG_TYPE_LOCAL_POS 4
#define MSG_TYPE_SETPOINT 5
#define MSG_TYPE_THRUST 6
#define MSG_TYPE_GAINS 7
#define MSG_TYPE_TEST 8

// Define message destinations
#define DST_ELKA 0
#define DST_SNAP 1
#define DST_XBEE 2

// Define max message length
#define MAX_MSG_LEN 55
#define MAX_ELKA_MSG_LEN 55

// FIXME move this to a general macros file
#define ELKA_SUCCESS 0
#define SERIAL_ERROR -2
#define MSG_ERROR -3
#define PKT_ERROR -4

#define GAINS_LEN 10

#define NEW_SETPOINT 1

// Types pertinent for messaging
// TODO move to another file (e.g. elka_types.h)
// Gains for errors given in mm,mm/s,rad,rad/s
typedef struct {
  // Gains format: kp_z,kd_z,ki_z,kp_r,kd_rr,kp_p,kd_pr,kp_yaw,kd_yawrate,ki_yaw
  float k[GAINS_LEN];
  // Sensitivity format: x,vx,y,vy,z,vz
  float sens[6];
} gains_s;

typedef struct {
  uint8_t len, num_msgs, data[MAX_MSG_LEN];
} elka_packet_s;

typedef struct {
  uint8_t len, type, data[MAX_MSG_LEN];
} elka_msg_s;

// Send to a destination defined as DST_<loc>
//int8_t send_msg(uint8_t dst,uint8_t msg_type,void *data,uint8_t len);
//int8_t send_pkt(uint8_t dst,elka_packet_s *snd);

/* Serialize data into an array of max size MAX_MSG_LEN
 * @param dst = destination
 * @param src = source
 * @return SUCCESS if successful else SERIAL_ERROR
 */
int8_t serialize(uint8_t *dst, void *src, uint8_t len);
int8_t deserialize(void *dst, uint8_t *src, uint8_t len);

// Get byte of float
// @param f = float to get byte of
// @param msbf = most significant byte first?
uint8_t get_byte_n(float f,uint8_t n,bool msbf);

void pack_msg_header(uint8_t *msg,uint8_t msg_type,uint8_t len);
int pack_test_msg(elka_packet_s *snd);
int pack_gains_msg(elka_packet_s *snd,gains_s *g,uint16_t base_thrust);

void write_msg_header(
    elka_packet_s *snd,
    uint8_t idx,
    uint16_t data_len,
    uint8_t msg_type);
int append_pkt(
    elka_packet_s *snd,
    uint8_t msg_type,
    uint16_t data_len,
    void *data);

#define RX_XBEE_DATA_LENGTH 55

typedef struct
{
  uint8_t data[RX_XBEE_DATA_LENGTH];
} rx_xbee;

rx_xbee rx_buf,
        snap_buf_rx;
uint8_t snap_buf_tx_len;

#endif
