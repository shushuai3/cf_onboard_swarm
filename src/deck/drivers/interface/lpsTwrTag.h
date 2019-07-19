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

#ifdef LOCODECK_NR_OF_ANCHORS
#define LOCODECK_NR_OF_TWR_ANCHORS LOCODECK_NR_OF_ANCHORS
#else
#define LOCODECK_NR_OF_TWR_ANCHORS 1
#endif

extern bool taskDelayForTransMode;
extern uwbAlgorithm_t uwbTwrTagAlgorithm;

typedef struct {
  uint8_t pollRx[5];
  uint8_t answerTx[5];
  uint8_t finalRx[5];

  float ownAx;
  float ownAy;
  float ownVx;
  float ownVy;
  float ownGz;
} __attribute__((packed)) lpsTwrTagReportPayload_t;

// float lpsTwrTagGetDistance(const uint8_t anchorId);

#define TWR_RECEIVE_TIMEOUT 1000

#endif // __LPS_TWR_TAG_H__