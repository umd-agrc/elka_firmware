#include "comm.h"


inline uint8_t get_byte_n(float f,uint8_t n,bool msbf) {
 if (msbf)
   return ((int)f>>(4-n))&0xff;
 else
   return ((int)f>>n)&0xff;
}

int8_t serialize(uint8_t *dst, void *src, uint8_t len) {
  if (len < MAX_MSG_LEN) {
    memcpy(dst,src,len);
    return ELKA_SUCCESS;
  } else return SERIAL_ERROR;
}
int8_t deserialize(void *dst, uint8_t *src, uint8_t len) {
  if (len < MAX_MSG_LEN) {
    memcpy(dst,src,len);
    return ELKA_SUCCESS;
  } else return SERIAL_ERROR;
}

inline void pack_msg_header(uint8_t *msg,uint8_t msg_type,uint8_t len){
 *msg=len;
 *(msg+1)=msg_type;
 *(msg+2)=255;
 *(msg+3)=255;
}

int pack_test_msg(elka_packet_s *snd) {
  uint8_t data_len=9,data[9]={2,3,4,2,3,4,2,3,4};
  return append_pkt(
      snd,
      MSG_TYPE_TEST,
      data_len,
      data);
}

int pack_gains_msg(elka_packet_s *snd,gains_s *g,uint16_t base_thrust) {
  uint8_t data_len=58,data[58];
  serialize(data,base_thrust,2);
  serialize(&(data[2]),g->sens,24);
  serialize(&(data[26]),g->k,32);
  return append_pkt(
      snd,
      MSG_TYPE_GAINS,
      data_len,
      data);
}

void write_msg_header(
    elka_packet_s *snd,
    uint8_t idx,
    uint16_t data_len,
    uint8_t msg_type) {
  if (snd->num_msgs==0) {
    snd->len++;
    snd->data[0]=0;
    idx++;
  }

  snd->len+=ELKA_MSG_HEADER_LEN+1;
  snd->data[0]+=data_len+ELKA_MSG_HEADER_LEN+1;

  snd->data[idx]=data_len+ELKA_MSG_HEADER_LEN;
  snd->data[idx+1]=msg_type;
  snd->data[idx+2]=ELKA_MSG_HEADER_ASSURANCE_BYTE;
  snd->data[idx+3]=ELKA_MSG_HEADER_ASSURANCE_BYTE;
}

int append_pkt(
    elka_packet_s *snd,
    uint8_t msg_type,
    uint16_t data_len,
    void *data) {
  if (snd->len + data_len+ELKA_MSG_HEADER_LEN+1 > MAX_ELKA_MSG_LEN) {
    return PKT_ERROR;
  }

  write_msg_header(snd,snd->len,data_len,msg_type);
  serialize(
      &(snd->data[snd->len]),
      data,data_len);
  snd->len+=data_len;
  snd->num_msgs++;
  return snd->len;
}

/*
int8_t send_msg(uint8_t dst,uint8_t msg_type,void *data,uint8_t len) {
  uint8_t global_msg_len=len+SERIAL_HEADER_LEN+1;
  static uint8_t msg[MAX_MSG_LEN];
  int8_t serialize_ret;

  if (global_msg_len > MAX_MSG_LEN) return MSG_ERROR;
  else {
    msg[0]=global_msg_len;
    pack_msg_header(&(msg[1]),msg_type,len);

    if ((serialize_ret=serialize(&(msg[SERIAL_HEADER_DATA_OFFSET]),data,len))
        != ELKA_SUCCESS)
      return serialize_ret;
    else if (dst == DST_SNAP) {
      memcpy(snap_buf_tx.data,msg,global_msg_len);
      snap_buf_tx_len=global_msg_len;
      snap_buf_tx_data_rdy=true;
    }
  }
  return ELKA_SUCCESS;
}

int8_t send_pkt(uint8_t dst,elka_packet_s *snd) {
  if (dst == DST_SNAP) {
    memcpy(snap_buf_tx.data,snd->data,snd->len);
    snap_buf_tx_len=snd->len;
    snap_buf_tx_data_rdy=true;
  }
  return ELKA_SUCCESS;
}
*/
