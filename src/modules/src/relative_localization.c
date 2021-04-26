#include "relative_localization.h"
#include "debug.h"

#include <string.h>
#include <stdint.h>
#include <math.h>
#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"

#include "param.h"

#include "log.h"
#include "system.h"

#include <radiolink.h>
#include "estimator_kalman.h"
#include "lpsTwrTag.h"

static bool isInit;

static float Qv = 1.0f; // velocity deviation
static float Qr = 0.7f; // yaw rate deviation
static float Ruwb = 3.5f; // ranging deviation
static float InitCovPos = 1000.0f;
static float InitCovYaw = 1.0f;

static relaVariable_t relaVar[NumUWB];
static float inputVar[NumUWB][STATE_DIM_rl];

static float A[STATE_DIM_rl][STATE_DIM_rl];
static float h[STATE_DIM_rl] = {0};
static arm_matrix_instance_f32 H = {1, STATE_DIM_rl, h};
static arm_matrix_instance_f32 Am = { STATE_DIM_rl, STATE_DIM_rl, (float *)A};

// Temporary matrices for the covariance updates
static float tmpNN1d[STATE_DIM_rl * STATE_DIM_rl];
static arm_matrix_instance_f32 tmpNN1m = { STATE_DIM_rl, STATE_DIM_rl, tmpNN1d};
static float tmpNN2d[STATE_DIM_rl * STATE_DIM_rl];
static arm_matrix_instance_f32 tmpNN2m = { STATE_DIM_rl, STATE_DIM_rl, tmpNN2d};
static float K[STATE_DIM_rl];
static arm_matrix_instance_f32 Km = {STATE_DIM_rl, 1, (float *)K};
static float tmpNN3d[STATE_DIM_rl * STATE_DIM_rl];
static arm_matrix_instance_f32 tmpNN3m = {STATE_DIM_rl, STATE_DIM_rl, tmpNN3d};
static float HTd[STATE_DIM_rl * 1];
static arm_matrix_instance_f32 HTm = {STATE_DIM_rl, 1, HTd};
static float PHTd[STATE_DIM_rl * 1];
static arm_matrix_instance_f32 PHTm = {STATE_DIM_rl, 1, PHTd};

static bool fullConnect = false; // a flag for control (fly or not)
static int8_t connectCount = 0; // watchdog for detecting the connection

static float vxj, vyj, rj; // receive vx, vy, gz and distance
static float vxi, vyi, ri; // self vx, vy, gz
static uint16_t dij; // distance between self i and other j
static float hi, hj; // height of robot i and j
static float hij;

static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst)); }
static inline void mat_inv(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst)); }
static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }
static inline float arm_sqrt(float32_t in)
{ float pOut = 0; arm_status result = arm_sqrt_f32(in, &pOut); configASSERT(ARM_MATH_SUCCESS == result); return pOut; }

void relativeLocoInit(void)
{
  if (isInit)
    return;
  xTaskCreate(relativeLocoTask,"relative_Localization",ZRANGER_TASK_STACKSIZE, NULL,ZRANGER_TASK_PRI,NULL );
  isInit = true;
}

void relativeLocoTask(void* arg)
{
  systemWaitStart();
  // Initialize EKF for relative localization
  for (int n=0; n<NumUWB; n++) {
    for (int i=0; i<STATE_DIM_rl; i++) {
      for (int j=0; j<STATE_DIM_rl; j++) {
        relaVar[n].P[i][j] = 0;
      }
    }
    relaVar[n].P[STATE_rlX][STATE_rlX] = InitCovPos;
    relaVar[n].P[STATE_rlY][STATE_rlY] = InitCovPos;
    relaVar[n].P[STATE_rlYaw][STATE_rlYaw] = InitCovYaw;  
    relaVar[n].S[STATE_rlX] = 0;
    relaVar[n].S[STATE_rlY] = 0;
    relaVar[n].S[STATE_rlYaw] = 0;
    relaVar[n].receiveFlag = false;
  }

  while(1) {
    vTaskDelay(10);
    for (int n=0; n<NumUWB; n++) {
      if (twrGetSwarmInfo(n, &dij, &vxj, &vyj, &rj, &hj)){
        connectCount = 0;
        estimatorKalmanGetSwarmInfo(&vxi, &vyi, &ri, &hi);
        if(relaVar[n].receiveFlag){
          uint32_t osTick = xTaskGetTickCount();
          float dtEKF = (float)(osTick - relaVar[n].oldTimetick)/configTICK_RATE_HZ;
          relaVar[n].oldTimetick = osTick;
          relativeEKF(n, vxi, vyi, ri, hi, vxj, vyj, rj, hj, dij, dtEKF);
          hij = hj-hi;
          inputVar[n][STATE_rlX] = vxj;
          inputVar[n][STATE_rlY] = vyj;
          inputVar[n][STATE_rlYaw] = rj;
        }else{
          relaVar[n].oldTimetick = xTaskGetTickCount();
          relaVar[n].receiveFlag = true;
          fullConnect = true;
        }
      }
    }
    connectCount++;
    if(connectCount>100){
      fullConnect = true; // disable control if there is no ranging after 1 second
    }
  }
}

void relativeEKF(int n, float vxi, float vyi, float ri, float hi, float vxj, float vyj, float rj, float hj, uint16_t dij, float dt)
{
  // some preprocessing
  arm_matrix_instance_f32 Pm = {STATE_DIM_rl, STATE_DIM_rl, (float *)relaVar[n].P};
  vxi = vxi*0.88f;
  vyi = vyi*0.88f;
  vxj = vxj*0.88f;
  vyj = vyj*0.88f;
  float cyaw = arm_cos_f32(relaVar[n].S[STATE_rlYaw]);
  float syaw = arm_sin_f32(relaVar[n].S[STATE_rlYaw]);
  float xij = relaVar[n].S[STATE_rlX];
  float yij = relaVar[n].S[STATE_rlY];

  // prediction
  relaVar[n].S[STATE_rlX] = xij + (cyaw*vxj-syaw*vyj-vxi+ri*yij)*dt;
  relaVar[n].S[STATE_rlY] = yij + (syaw*vxj+cyaw*vyj-vyi-ri*xij)*dt;
  relaVar[n].S[STATE_rlYaw] = relaVar[n].S[STATE_rlYaw] + (rj-ri)*dt;

  A[0][0] = 1;
  A[0][1] = ri*dt;
  A[0][2] = (-syaw*vxj-cyaw*vyj)*dt;
  A[1][0] = -ri*dt;
  A[1][1] = 1;
  A[1][2] = (cyaw*vxj-syaw*vyj)*dt;
  A[2][0] = 0;
  A[2][1] = 0;
  A[2][2] = 1;

  mat_mult(&Am, &Pm, &tmpNN1m); // A P
  mat_trans(&Am, &tmpNN2m); // A'
  mat_mult(&tmpNN1m, &tmpNN2m, &Pm); // A P A'

  // BQB' = [ Qv*c^2 + Qv*s^2 + Qr*y^2 + Qv,                       -Qr*x*y, -Qr*y]
  //        [                       -Qr*x*y, Qv*c^2 + Qv*s^2 + Qr*x^2 + Qv,  Qr*x]
  //        [                         -Qr*y,                          Qr*x,  2*Qr]*dt^2
  float dt2 = dt*dt;
  relaVar[n].P[0][0] += dt2*(Qv+Qv+Qr*yij*yij);
  relaVar[n].P[0][1] += dt2*(-Qr*xij*yij);
  relaVar[n].P[0][2] += dt2*(-Qr*yij);
  relaVar[n].P[1][0] += dt2*(-Qr*xij*yij);
  relaVar[n].P[1][1] += dt2*(Qv+Qv+Qr*xij*xij);
  relaVar[n].P[1][2] += dt2*(Qr*xij);
  relaVar[n].P[2][0] += dt2*(-Qr*yij);
  relaVar[n].P[2][1] += dt2*(Qr*xij);
  relaVar[n].P[2][2] += dt2*(2*Qr);

  xij = relaVar[n].S[STATE_rlX];
  yij = relaVar[n].S[STATE_rlY];
  float distPred = arm_sqrt(xij*xij+yij*yij+(hi-hj)*(hi-hj))+0.0001f;
  float distMeas = (float)(dij/1000);
  distMeas = distMeas - (0.048f*distMeas + 0.65f); // UWB biad model
  h[0] = xij/distPred;
  h[1] = yij/distPred;
  h[2] = 0;

  mat_trans(&H, &HTm); // H'
  mat_mult(&Pm, &HTm, &PHTm); // PH'
  float HPHR = powf(Ruwb, 2);// HPH' + R
  for (int i=0; i<STATE_DIM_rl; i++) { // Add the element of HPH' to the above
    HPHR += H.pData[i]*PHTd[i]; // this obviously only works if the update is scalar (as in this function)
  }
  for (int i=0; i<STATE_DIM_rl; i++) {
    K[i] = PHTd[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
    relaVar[n].S[i] = relaVar[n].S[i] + K[i] * (distMeas - distPred); // state update
  }     
  mat_mult(&Km, &H, &tmpNN1m); // KH
  for (int i=0; i<STATE_DIM_rl; i++) { tmpNN1d[STATE_DIM_rl*i+i] -= 1; } // KH - I
  mat_trans(&tmpNN1m, &tmpNN2m); // (KH - I)'
  mat_mult(&tmpNN1m, &Pm, &tmpNN3m); // (KH - I)*P
  mat_mult(&tmpNN3m, &tmpNN2m, &Pm); // (KH - I)*P*(KH - I)'
}

bool relativeInfoRead(float* relaVarParam, float* inputVarParam){
  if(fullConnect){
    for(int i=0; i<NumUWB; i++){
      *(relaVarParam + i*STATE_DIM_rl + 0) = relaVar[i].S[STATE_rlX];
      *(relaVarParam + i*STATE_DIM_rl + 1) = relaVar[i].S[STATE_rlY];
      *(relaVarParam + i*STATE_DIM_rl + 2) = relaVar[i].S[STATE_rlYaw];
      *(inputVarParam + i*STATE_DIM_rl + 0) = inputVar[i][STATE_rlX];
      *(inputVarParam + i*STATE_DIM_rl + 1) = inputVar[i][STATE_rlY];
      *(inputVarParam + i*STATE_DIM_rl + 2) = inputVar[i][STATE_rlYaw];
    }   
    return true;
  }
  else
    return false;    
}

LOG_GROUP_START(relative_pos)
LOG_ADD(LOG_FLOAT, rlX0, &relaVar[0].S[STATE_rlX])
LOG_ADD(LOG_FLOAT, rlY0, &relaVar[0].S[STATE_rlY])
LOG_ADD(LOG_FLOAT, rlYaw0, &relaVar[0].S[STATE_rlYaw])
LOG_ADD(LOG_FLOAT, rlX1, &relaVar[1].S[STATE_rlX])
LOG_ADD(LOG_FLOAT, rlY1, &relaVar[1].S[STATE_rlY])
LOG_ADD(LOG_FLOAT, rlYaw1, &relaVar[1].S[STATE_rlYaw])
LOG_ADD(LOG_FLOAT, rlZ1, &hij)
LOG_ADD(LOG_FLOAT, rlX2, &relaVar[2].S[STATE_rlX])
LOG_ADD(LOG_FLOAT, rlY2, &relaVar[2].S[STATE_rlY])
LOG_ADD(LOG_FLOAT, rlYaw2, &relaVar[2].S[STATE_rlYaw])
LOG_GROUP_STOP(relative_pos)

PARAM_GROUP_START(arelative_pos)
PARAM_ADD(PARAM_FLOAT, noiFlow, &Qv) // make sure the name is not too long
PARAM_ADD(PARAM_FLOAT, noiGyroZ, &Qr)
PARAM_ADD(PARAM_FLOAT, noiUWB, &Ruwb)
PARAM_ADD(PARAM_FLOAT, Ppos, &InitCovPos)
PARAM_ADD(PARAM_FLOAT, Pyaw, &InitCovYaw)
PARAM_GROUP_STOP(arelative_pos)