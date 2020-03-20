#ifndef __LPS_TWR_TAG_H__
#define __LPS_TWR_TAG_H__

#include "locodeck.h"
#include "libdw1000.h"
#include "mac.h"

#define LPS_TWR_POLL    0x01  
#define LPS_TWR_ANSWER  0x02
#define LPS_TWR_FINAL   0x03
#define LPS_TWR_REPORT  0x04

#define LPS_TWR_TYPE  0
#define LPS_TWR_SEQ   1
#define LPS_TWR_ENABLE
#define NumUWB 3

extern uwbAlgorithm_t uwbTwrTagAlgorithm;

typedef struct {
  uint8_t pollRx[5];
  uint8_t answerTx[5];
  uint8_t finalRx[5];
  uint16_t reciprocalDistance;
  float_t selfVx;
  float_t selfVy;
  float_t selfGz;
  float_t selfh;
} __attribute__((packed)) lpsTwrTagReportPayload_t;

bool twrGetSwarmInfo(int robNum, uint16_t* range, float* vx, float* vy, float* gyroZ, float* height);
void relativeVel0Read(float* vx0, float* vy0, int nRob);
#endif // __LPS_TWR_TAG_H__