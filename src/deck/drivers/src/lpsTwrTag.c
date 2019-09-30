#include <string.h>
#include "lpsTwrTag.h"
#include "log.h"
#include "physicalConstants.h"
#include "FreeRTOS.h"
#include "task.h"

#define ANTENNA_OFFSET 154.6   // In meter
#define basicAddr 0xbccf000000000000
#define NumUWB 4
#define selfID 3

static locoAddress_t selfAddress = basicAddr + selfID;
static const uint64_t antennaDelay = (ANTENNA_OFFSET*499.2e6*128)/299792458.0; // In radio tick

typedef struct {
  float distance[NumUWB-1];
  int16_t rangeNumPerSec[NumUWB-1];
} twrState_t;
static twrState_t state;

// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;

bool taskDelayForTransMode;
static packet_t txPacket;
static bool rangingOk;
static uint8_t succededRanging, tdmaSynchronized; //dont know why

// Communication logic between each UWB
static bool current_mode_trans;
static uint8_t current_receiveID;

static bool checkTurn;
static uint32_t checkTurnTick = 0;

static void txcallback(dwDevice_t *dev)
{
  dwTime_t departure;
  dwGetTransmitTimestamp(dev, &departure);
  departure.full += (antennaDelay / 2);

  if(current_mode_trans){
    switch (txPacket.payload[0]) {
      case LPS_TWR_POLL:
        poll_tx = departure;
        break;
      case LPS_TWR_FINAL:
        final_tx = departure;
        break;
      case LPS_TWR_REPORT+1:
        if( (current_receiveID == 0) || (current_receiveID-1 == selfID) ){
          // current_receiveID = current_receiveID;
          current_mode_trans = false;
          taskDelayForTransMode = false;
          dwIdle(dev);
          dwSetReceiveWaitTimeout(dev, 10000);
          dwNewReceive(dev);
          dwSetDefaults(dev);
          dwStartReceive(dev);
          checkTurn = true;
          checkTurnTick = xTaskGetTickCount();
        }else{
          current_receiveID = current_receiveID - 1;
        }
        break;
    }
  }else{
    switch (txPacket.payload[0]) {
      case LPS_TWR_ANSWER:
        answer_tx = departure;
        break;
      case LPS_TWR_REPORT:
        break;
    }
  }
}

static uint32_t range_tick = 0;
static int16_t  range_count= 0;
static void rxcallback(dwDevice_t *dev) {
  dwTime_t arival = { .full=0 };
  int dataLength = dwGetDataLength(dev);
  if (dataLength == 0) return;

  packet_t rxPacket;
  memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);
  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);
  if (rxPacket.destAddress != selfAddress) {
    if(current_mode_trans){
      current_mode_trans = false;
      taskDelayForTransMode = false;
      dwIdle(dev);
      dwSetReceiveWaitTimeout(dev, 10000);
    }
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
    return;
  }

  txPacket.destAddress = rxPacket.sourceAddress;
  txPacket.sourceAddress = rxPacket.destAddress;

  if(current_mode_trans){
    switch(rxPacket.payload[LPS_TWR_TYPE]) {
      case LPS_TWR_ANSWER:
      {
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_FINAL;
        txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
        dwGetReceiveTimestamp(dev, &arival);
        arival.full -= (antennaDelay / 2);
        answer_rx = arival;
        dwNewTransmit(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
        break;
      }
      case LPS_TWR_REPORT:
      {
        lpsTwrTagReportPayload_t *report = (lpsTwrTagReportPayload_t *)(rxPacket.payload+2);
        double tround1, treply1, treply2, tround2, tprop_ctn, tprop;
        memcpy(&poll_rx, &report->pollRx, 5);
        memcpy(&answer_tx, &report->answerTx, 5);
        memcpy(&final_rx, &report->finalRx, 5);
        tround1 = answer_rx.low32 - poll_tx.low32;
        treply1 = answer_tx.low32 - poll_rx.low32;
        tround2 = final_rx.low32 - answer_tx.low32;
        treply2 = final_tx.low32 - answer_rx.low32;
        tprop_ctn = ((tround1*tround2) - (treply1*treply2)) / (tround1 + tround2 + treply1 + treply2);
        tprop = tprop_ctn / LOCODECK_TS_FREQ;
        state.distance[0] = SPEED_OF_LIGHT * tprop;
        rangingOk = true;

        range_count++;
        if(xTaskGetTickCount()>range_tick+1000)
        {
          range_tick = xTaskGetTickCount();
          state.rangeNumPerSec[0] = range_count;
          range_count = 0;  
        }

        lpsTwrTagReportPayload_t *report2 = (lpsTwrTagReportPayload_t *)(txPacket.payload+2);
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_REPORT+1;
        txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
        memcpy(&report2->pollRx, &poll_tx, 5);
        memcpy(&report2->answerTx, &answer_rx, 5);
        memcpy(&report2->finalRx, &final_tx, 5);
        dwNewTransmit(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2+sizeof(lpsTwrTagReportPayload_t));
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);  
        break;
      }
    }
  }else{
    switch(rxPacket.payload[LPS_TWR_TYPE]) {
      case LPS_TWR_POLL:
      {
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_ANSWER;
        txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
        dwGetReceiveTimestamp(dev, &arival);
        arival.full -= (antennaDelay / 2);
        poll_rx = arival;
        dwNewTransmit(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
        break;
      }
      case LPS_TWR_FINAL:
      {
        lpsTwrTagReportPayload_t *report = (lpsTwrTagReportPayload_t *)(txPacket.payload+2);
        dwGetReceiveTimestamp(dev, &arival);
        arival.full -= (antennaDelay / 2);
        final_rx = arival;
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_REPORT;
        txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
        memcpy(&report->pollRx, &poll_rx, 5);
        memcpy(&report->answerTx, &answer_tx, 5);
        memcpy(&report->finalRx, &final_rx, 5);
        dwNewTransmit(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2+sizeof(lpsTwrTagReportPayload_t));
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
        break;
      }
      case (LPS_TWR_REPORT+1):
      {
        lpsTwrTagReportPayload_t *report2 = (lpsTwrTagReportPayload_t *)(rxPacket.payload+2);
        double tround1, treply1, treply2, tround2, tprop_ctn, tprop;
        memcpy(&poll_tx, &report2->pollRx, 5);
        memcpy(&answer_rx, &report2->answerTx, 5);
        memcpy(&final_tx, &report2->finalRx, 5);
        tround1 = answer_rx.low32 - poll_tx.low32;
        treply1 = answer_tx.low32 - poll_rx.low32;
        tround2 = final_rx.low32 - answer_tx.low32;
        treply2 = final_tx.low32 - answer_rx.low32;
        tprop_ctn = ((tround1*tround2) - (treply1*treply2)) / (tround1 + tround2 + treply1 + treply2);
        tprop = tprop_ctn / LOCODECK_TS_FREQ;
        state.distance[0] = SPEED_OF_LIGHT * tprop;
        rangingOk = true;
        range_count++;
        if(xTaskGetTickCount()>range_tick+1000)
        {
          range_tick = xTaskGetTickCount();
          state.rangeNumPerSec[0] = range_count;
          range_count = 0;  
        }
        uint8_t fromID = (uint8_t)(rxPacket.sourceAddress & 0xFF);
        if( selfID == fromID + 1 || selfID == 0 ){
          current_mode_trans = true;
          taskDelayForTransMode = true;
          dwIdle(dev);
          dwSetReceiveWaitTimeout(dev, 1000);
          if(selfID == NumUWB-1)
            current_receiveID = 0;
          else
            current_receiveID = NumUWB - 1;
          if(selfID == 0)
            current_receiveID = NumUWB - 2; // immediate problem
          txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
          txPacket.payload[LPS_TWR_SEQ] = 0;
          txPacket.sourceAddress = selfAddress;
          txPacket.destAddress = basicAddr + current_receiveID;
          dwNewTransmit(dev);
          dwSetDefaults(dev);
          dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
          dwWaitForResponse(dev, true);
          dwStartTransmit(dev);
        }else{
          dwNewReceive(dev);
          dwSetDefaults(dev);
          dwStartReceive(dev);
        }
        break;
      }
    }
  }
}

static uint32_t twrTagOnEvent(dwDevice_t *dev, uwbEvent_t event)
{
  switch(event) {
    case eventPacketReceived:
      rxcallback(dev);
      checkTurn = false;
      break;
    case eventPacketSent:
      txcallback(dev);
      break;
    case eventTimeout:  // Comes back to timeout after each ranging attempt
    case eventReceiveTimeout:
    case eventReceiveFailed:
      if (current_mode_trans==true)
      {
        if (tdmaSynchronized==false) succededRanging++; 
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
        txPacket.payload[LPS_TWR_SEQ] = 0;
        txPacket.sourceAddress = selfAddress;
        txPacket.destAddress = basicAddr + current_receiveID;
        dwNewTransmit(dev);
        dwSetDefaults(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
      }else
      {
        if(xTaskGetTickCount() > checkTurnTick + 20) // > 20ms
        {
          if(checkTurn == true){
            current_mode_trans = true;
            taskDelayForTransMode = true;
            dwIdle(dev);
            dwSetReceiveWaitTimeout(dev, 1000);
            txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
            txPacket.payload[LPS_TWR_SEQ] = 0;
            txPacket.sourceAddress = selfAddress;
            txPacket.destAddress = basicAddr + current_receiveID;
            dwNewTransmit(dev);
            dwSetDefaults(dev);
            dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
            dwWaitForResponse(dev, true);
            dwStartTransmit(dev);
            checkTurn = false;
            break;
          }
        }
        dwNewReceive(dev);
	      dwSetDefaults(dev);
        dwStartReceive(dev);
      }     
      break;
    default:
      configASSERT(false);
  }
  return MAX_TIMEOUT;
}

static void twrTagInit(dwDevice_t *dev)
{
  // Initialize the packet in the TX buffer
  memset(&txPacket, 0, sizeof(txPacket));
  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  txPacket.pan = 0xbccf;

  memset(&poll_tx, 0, sizeof(poll_tx));
  memset(&poll_rx, 0, sizeof(poll_rx));
  memset(&answer_tx, 0, sizeof(answer_tx));
  memset(&answer_rx, 0, sizeof(answer_rx));
  memset(&final_tx, 0, sizeof(final_tx));
  memset(&final_rx, 0, sizeof(final_rx));

  // Communication logic between each UWB
  if(selfID==0)
  {
    current_receiveID = NumUWB-1;
    current_mode_trans = true;
    taskDelayForTransMode = true;    
    dwSetReceiveWaitTimeout(dev, 1000);
  }
  else
  {
    // current_receiveID = 0;
    current_mode_trans = false;
    taskDelayForTransMode = false;
    dwSetReceiveWaitTimeout(dev, 10000);
  }

  checkTurn = false;
  tdmaSynchronized = false;
  rangingOk = false;
}

static bool isRangingOk()
{
  return rangingOk;
}

static bool getAnchorPosition(const uint8_t anchorId, point_t* position) {
  if (anchorId < NumUWB-1)
    return true;
  else
    return false;
}

static uint8_t getAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  for (int i = 0; i < NumUWB-1; i++) {
    unorderedAnchorList[i] = i;
  }
  return NumUWB-1;
}

static uint8_t getActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  uint8_t count = 0;
  for (int i = 0; i < NumUWB-1; i++) {
      unorderedAnchorList[count] = i;
      count++;
  }
  return count;
}

uwbAlgorithm_t uwbTwrTagAlgorithm = {
  .init = twrTagInit,
  .onEvent = twrTagOnEvent,
  .isRangingOk = isRangingOk,
  .getAnchorPosition = getAnchorPosition,
  .getAnchorIdList = getAnchorIdList,
  .getActiveAnchorIdList = getActiveAnchorIdList,
};

LOG_GROUP_START(ranging)
LOG_ADD(LOG_FLOAT, distance0, &state.distance[0])
LOG_ADD(LOG_INT16, rangeNum0, &state.rangeNumPerSec[0])
LOG_GROUP_STOP(ranging)