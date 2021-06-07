#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "commander.h"
#include "relative_localization.h"
#include "num.h"
#include "param.h"
#include "debug.h"
#include <stdlib.h> // random
#include "lpsTwrTag.h" // UWBNum
#include "configblock.h"
#include "uart2.h"
#include "log.h"
#include <math.h>
#include "estimator_kalman.h"
#define USE_MONOCAM 0

static bool isInit;
static bool onGround = true;
static bool keepFlying = false;
static setpoint_t setpoint;
static float relaVarInCtrl[NumUWB][STATE_DIM_rl];
static float inputVarInCtrl[NumUWB][STATE_DIM_rl];
static uint8_t selfID;
static float height;

static float relaCtrl_p = 2.0f;
static float relaCtrl_i = 0.0001f;
static float relaCtrl_d = 0.01f;
// static float NDI_k = 2.0f;
static char c = 0; // monoCam

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;
  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;
  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;
  setpoint->velocity_body = true;
  commanderSetSetpoint(setpoint, 3);
}

static void flyRandomIn1meter(float vel){
  float randomYaw = (rand() / (float)RAND_MAX) * 6.28f; // 0-2pi rad
  float randomVel = vel*(rand() / (float)RAND_MAX); // 0-1 m/s
  float vxBody = randomVel * cosf(randomYaw);
  float vyBody = randomVel * sinf(randomYaw);
  for (int i=1; i<100; i++) {
    setHoverSetpoint(&setpoint, vxBody, vyBody, height, 0);
    vTaskDelay(M2T(10));
  }
  for (int i=1; i<100; i++) {
    setHoverSetpoint(&setpoint, -vxBody, -vyBody, height, 0);
    vTaskDelay(M2T(10));
  }
}

#if USE_MONOCAM
static float PreErr_yaw = 0;
static float IntErr_yaw = 0;
static uint32_t PreTimeYaw;
static void flyViaDoor(char camYaw){
  if(camYaw){
    float dt = (float)(xTaskGetTickCount()-PreTimeYaw)/configTICK_RATE_HZ;
    PreTimeYaw = xTaskGetTickCount();
    if(dt > 1) // skip the first run of the EKF
      return;
    // pid control for door flight
    float err_yaw;
    if(camYaw>128)
      err_yaw = -(camYaw - 128 - 64); // 0-128, nominal value = 64
    else
      err_yaw = -(camYaw - 64); // 0-128, nominal value = 64
    float pid_vyaw = 1.0f * err_yaw;
    float dyaw = (err_yaw - PreErr_yaw) / dt;
    PreErr_yaw = err_yaw;
    pid_vyaw += 0.01f * dyaw;
    IntErr_yaw += err_yaw * dt;
    pid_vyaw += 0.0001f * constrain(IntErr_yaw, -10.0f, 10.0f);
    pid_vyaw = constrain(pid_vyaw, -100, 100);  
    if(camYaw<128)
      setHoverSetpoint(&setpoint, 1, 0, 0.5f, pid_vyaw); // deg/s
    else
      setHoverSetpoint(&setpoint, 0, -0.3f, 0.5f, pid_vyaw); // deg/s
  }else{
    setHoverSetpoint(&setpoint, 1.5f, 0, 0.5f, 45);
  }
}
#endif

#define SIGN(a) ((a>=0)?1:-1)
static float targetX;
static float targetY;
static float PreErr_x = 0;
static float PreErr_y = 0;
static float IntErr_x = 0;
static float IntErr_y = 0;
static uint32_t PreTime;
static void formation0asCenter(float tarX, float tarY){
  float dt = (float)(xTaskGetTickCount()-PreTime)/configTICK_RATE_HZ;
  PreTime = xTaskGetTickCount();
  if(dt > 1) // skip the first run of the EKF
    return;
  // pid control for formation flight
  float err_x = -(tarX - relaVarInCtrl[0][STATE_rlX]);
  float err_y = -(tarY - relaVarInCtrl[0][STATE_rlY]);
  float pid_vx = relaCtrl_p * err_x;
  float pid_vy = relaCtrl_p * err_y;
  float dx = (err_x - PreErr_x) / dt;
  float dy = (err_y - PreErr_y) / dt;
  PreErr_x = err_x;
  PreErr_y = err_y;
  pid_vx += relaCtrl_d * dx;
  pid_vy += relaCtrl_d * dy;
  IntErr_x += err_x * dt;
  IntErr_y += err_y * dt;
  pid_vx += relaCtrl_i * constrain(IntErr_x, -0.5, 0.5);
  pid_vy += relaCtrl_i * constrain(IntErr_y, -0.5, 0.5);
  pid_vx = constrain(pid_vx, -1.5f, 1.5f);
  pid_vy = constrain(pid_vy, -1.5f, 1.5f);

  // float rep_x = 0.0f;
  // float rep_y = 0.0f;
  // for(uint8_t i=0; i<NumUWB; i++){
  //   if(i!=selfID){
  //     float dist = relaVarInCtrl[i][STATE_rlX]*relaVarInCtrl[i][STATE_rlX] + relaVarInCtrl[i][STATE_rlY]*relaVarInCtrl[i][STATE_rlY];
  //     dist = sqrtf(dist);
  //     rep_x += -0.5f * (SIGN(0.5f - dist) + 1) / (abs(relaVarInCtrl[i][STATE_rlX]) + 0.001f) * SIGN(relaVarInCtrl[i][STATE_rlX]);
  //     rep_y += -0.5f * (SIGN(0.5f - dist) + 1) / (abs(relaVarInCtrl[i][STATE_rlY]) + 0.001f) * SIGN(relaVarInCtrl[i][STATE_rlY]);
  //   }
  // }
  // rep_x = constrain(rep_x, -1.5f, 1.5f);
  // rep_y = constrain(rep_y, -1.5f, 1.5f);

  // pid_vx = constrain(pid_vx + rep_x, -1.5f, 1.5f);
  // pid_vy = constrain(pid_vy + rep_y, -1.5f, 1.5f);

  setHoverSetpoint(&setpoint, pid_vx, pid_vy, height, 0);
}

// static void NDI_formation0asCenter(float tarX, float tarY){
//   float err_x = -(tarX - relaVarInCtrl[0][STATE_rlX]);
//   float err_y = -(tarY - relaVarInCtrl[0][STATE_rlY]);
//   float rela_yaw = relaVarInCtrl[0][STATE_rlYaw];
//   float Ru_x = cosf(rela_yaw)*inputVarInCtrl[0][STATE_rlX] - sinf(rela_yaw)*inputVarInCtrl[0][STATE_rlY];
//   float Ru_y = sinf(rela_yaw)*inputVarInCtrl[0][STATE_rlX] + cosf(rela_yaw)*inputVarInCtrl[0][STATE_rlY];
//   float ndi_vx = NDI_k*err_x + 0*Ru_x;
//   float ndi_vy = NDI_k*err_y + 0*Ru_y;
//   ndi_vx = constrain(ndi_vx, -1.5f, 1.5f);
//   ndi_vy = constrain(ndi_vy, -1.5f, 1.5f);  
//   setHoverSetpoint(&setpoint, ndi_vx, ndi_vy, height, 0);
// }

void relativeControlTask(void* arg)
{
  static uint32_t ctrlTick, lastTick;
  systemWaitStart();
  static logVarId_t logIdStateIsFlying;
  logIdStateIsFlying = logGetVarId("kalman", "inFlight");
  // height = (float)selfID*0.1f+0.2f;
  while(1) {
    vTaskDelay(10);
    if(selfID==0){
      keepFlying = logGetUint(logIdStateIsFlying);
      keepFlying = command_share(selfID, keepFlying);
      continue;
    }
#if USE_MONOCAM
    if(selfID==0)
      uart2Getchar(&c);
#endif
    keepFlying = command_share(selfID, keepFlying);
    if(relativeInfoRead((float *)relaVarInCtrl, (float *)inputVarInCtrl) && keepFlying){
      // take off
      if(onGround){
        estimatorKalmanInit(); // reseting kalman filter
        vTaskDelay(M2T(2000));
        for (int i=0; i<50; i++) {
          setHoverSetpoint(&setpoint, 0, 0, 0.3f, 0);
          vTaskDelay(M2T(100));
        }
        onGround = false;
        ctrlTick = xTaskGetTickCount();
      }

      // control loop
      // setHoverSetpoint(&setpoint, 0, 0, height, 0); // hover
      uint32_t tickInterval = xTaskGetTickCount() - ctrlTick;
      if( tickInterval < 20000){
          flyRandomIn1meter(1.0f); // random flight within first 10 seconds
        targetX = relaVarInCtrl[0][STATE_rlX];
        targetY = relaVarInCtrl[0][STATE_rlY];
      }
      else
      {

#if USE_MONOCAM
        if(selfID==0)
          flyViaDoor(c);
        else
          formation0asCenter(targetX, targetY);
#else
        if ( (tickInterval > 20000) && (tickInterval < 30000) ){ // formation
          srand((unsigned int) relaVarInCtrl[0][STATE_rlX]*100);
            formation0asCenter(targetX, targetY);
            // NDI_formation0asCenter(targetX, targetY);
            lastTick = tickInterval;
        }

        static float relaXof2in1=1.0f, relaYof2in1=0.0f;
        if ( (tickInterval > 30000) ){
          if(tickInterval - lastTick > 5000)
          {
            lastTick = tickInterval;
            relaXof2in1 = (rand() / (float)RAND_MAX) * 2.5f; // in front
            relaYof2in1 = (rand() / (float)RAND_MAX) * 3.0f - 1.5f;
            height = (rand() / (float)RAND_MAX) * 0.8f + 0.2f;
          }
          targetX = -cosf(relaVarInCtrl[0][STATE_rlYaw])*relaXof2in1 + sinf(relaVarInCtrl[0][STATE_rlYaw])*relaYof2in1;
          targetY = -sinf(relaVarInCtrl[0][STATE_rlYaw])*relaXof2in1 - cosf(relaVarInCtrl[0][STATE_rlYaw])*relaYof2in1;
          formation0asCenter(targetX, targetY); 
        }
#endif
      }

    }else{
      // landing procedure
      if(!onGround){
        for (int i=1; i<5; i++) {
          if(selfID!=0){
          setHoverSetpoint(&setpoint, 0, 0, 0.3f-(float)i*0.05f, 0);
          vTaskDelay(M2T(10));
          }
        }
        onGround = true;
      } 
    }
  }
}

void relativeControlInit(void)
{
  if (isInit)
    return;
  selfID = (uint8_t)(((configblockGetRadioAddress()) & 0x000000000f) - 5);
#if USE_MONOCAM
  if(selfID==0)
    uart2Init(115200); // only CF0 has monoCam and usart comm
#endif
  xTaskCreate(relativeControlTask,"relative_Control",configMINIMAL_STACK_SIZE, NULL,3,NULL );
  height = 0.5f;
  isInit = true;
}

PARAM_GROUP_START(relative_ctrl)
PARAM_ADD(PARAM_UINT8, keepFlying, &keepFlying)
PARAM_ADD(PARAM_FLOAT, relaCtrl_p, &relaCtrl_p)
PARAM_ADD(PARAM_FLOAT, relaCtrl_i, &relaCtrl_i)
PARAM_ADD(PARAM_FLOAT, relaCtrl_d, &relaCtrl_d)
PARAM_GROUP_STOP(relative_ctrl)

LOG_GROUP_START(mono_cam)
LOG_ADD(LOG_UINT8, charCam, &c)
LOG_GROUP_STOP(mono_cam)