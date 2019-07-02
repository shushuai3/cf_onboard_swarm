
#include "relative_loco.h"
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

static float procNoiseFlow_xy = 0.1f;
static float InitCovarianceX = 0.1f;
static float InitCovarianceY = 0.3f;
static float measNoiseDist = 0.2f;

static bool read_loco_ok = false;

typedef enum
{
  STATE_rlX, STATE_rlY, STATE_DIM_rl
} relative_stateIdx_t;
static float S[STATE_DIM_rl];
static float P[STATE_DIM_rl][STATE_DIM_rl];
static float A[STATE_DIM_rl][STATE_DIM_rl];
static float h[STATE_DIM_rl] = {0};
static arm_matrix_instance_f32 H = {1, STATE_DIM_rl, h};
static arm_matrix_instance_f32 Pm = {STATE_DIM_rl, STATE_DIM_rl, (float *)P};
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


static float ano_dis, ano_ax, ano_ay, ano_vx, ano_vy, ano_gz;
static float own_ax, own_ay, own_vx, own_vy, own_gz;
static uint16_t dist_mm;
static int32_t lastPrediction;

/**
 * Supporting and utility functions
 */

static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst)); }
static inline void mat_inv(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst)); }
static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }
static inline float arm_sqrt(float32_t in)
{ float pOut = 0; arm_status result = arm_sqrt_f32(in, &pOut); configASSERT(ARM_MATH_SUCCESS == result); return pOut; }

#define PREDICT_RATE_RL RATE_25_HZ
//static discrete_ekf ekf_rl;
void relativeLocoInit(void)
{
  if (isInit)
    return;
  // ekf_rl.dt = 0.005;
  // discrete_ekf_new(&ekf_rl);
  xTaskCreate(relativeLocoTask,"relative_Loco",ZRANGER_TASK_STACKSIZE, NULL,ZRANGER_TASK_PRI,NULL );
  isInit = true;
}
void relativeLocoTask(void* arg)
{
  systemWaitStart();

  // Initialize EKF for relative localization
  S[STATE_rlX] = 0;
  S[STATE_rlY] = 1.2f;
  for (int i=0; i< STATE_DIM_rl; i++) {
    for (int j=0; j < STATE_DIM_rl; j++) {
      P[i][j] = 0; // set covariances to zero (diagonals will be changed from zero in the next section)
    }
  }
  // initialize state variances
  P[0][0]  = powf(InitCovarianceX, 2);
  P[1][1]  = powf(InitCovarianceY, 2);

  lastPrediction = xTaskGetTickCount();

  //int h = 0;
  while(1) {
    vTaskDelay(10);
    uint32_t osTick = xTaskGetTickCount();
    if( (osTick - lastPrediction) >= configTICK_RATE_HZ/PREDICT_RATE_RL )
    {
      if( get_relative_info(&ano_dis, &ano_ax, &ano_ay, &ano_vx, &ano_vy, &ano_gz) )
      {
        dist_mm = (uint16_t)(ano_dis*1000);
        float dt = (float)(osTick-lastPrediction)/configTICK_RATE_HZ;
        estimatorKalmanGetSwarmInfo(&own_ax, &own_ay, &own_vx, &own_vy, &own_gz);
        // predict
        A[0][0] = 1;
        A[0][1] = 0;
        A[1][0] = 0;
        A[1][1] = 1;
        mat_mult(&Am, &Pm, &tmpNN1m); // A P
        mat_trans(&Am, &tmpNN2m); // A'
        mat_mult(&tmpNN1m, &tmpNN2m, &Pm); // A P A'
        float dPsi = ano_gz - own_gz;
        S[STATE_rlX] += dt*(cosf(dPsi)*ano_vx - sinf(dPsi)*ano_vy - own_vx);
        S[STATE_rlY] += dt*(sinf(dPsi)*ano_vx + cosf(dPsi)*ano_vy - own_vy);
        P[0][0] += powf(procNoiseFlow_xy, 2);  // add process noise on position
        P[1][1] += powf(procNoiseFlow_xy, 2);  // add process noise on position
        float measDist = ano_dis;
        float predDist = arm_sqrt(powf(S[STATE_rlX], 2) + powf(S[STATE_rlY], 2));
        if(predDist!=0.0f)
        {
          h[0] = S[STATE_rlX]/predDist;
          h[1] = S[STATE_rlY]/predDist;
        }else
        {
          h[0] = 0.5f;
          h[1] = 0.5f;
        }
        mat_trans(&H, &HTm);
        mat_mult(&Pm, &HTm, &PHTm); // PH'
        float HPHR = powf(measNoiseDist, 2);// HPH' + R
        for (int i=0; i<STATE_DIM_rl; i++) { // Add the element of HPH' to the above
          HPHR += H.pData[i]*PHTd[i]; // this obviously only works if the update is scalar (as in this function)
        }
        for (int i=0; i<STATE_DIM_rl; i++) {
          K[i] = PHTd[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
          S[i] = S[i] + K[i] * (measDist - predDist); // state update
        }     
        mat_mult(&Km, &H, &tmpNN1m); // KH
        for (int i=0; i<STATE_DIM_rl; i++) { tmpNN1d[STATE_DIM_rl*i+i] -= 1; } // KH - I
        mat_trans(&tmpNN1m, &tmpNN2m); // (KH - I)'
        mat_mult(&tmpNN1m, &Pm, &tmpNN3m); // (KH - I)*P
        mat_mult(&tmpNN3m, &tmpNN2m, &Pm); // (KH - I)*P*(KH - I)'
        lastPrediction = osTick;
        read_loco_ok = true;
      }
    }
  }
}

bool read_rl_loco(float* rlx, float* rly)
{
  if(read_loco_ok==true)
  {
    *rlx = S[STATE_rlX];
    *rly = S[STATE_rlY];
    read_loco_ok = false;
    return(true);
  }else
  {
    return(false);
  }
}

LOG_GROUP_START(relative_pos)
LOG_ADD(LOG_FLOAT, rlX, &S[STATE_rlX])
LOG_ADD(LOG_FLOAT, rlY, &S[STATE_rlY])

LOG_ADD(LOG_UINT16, dis_mm, &dist_mm)
LOG_ADD(LOG_FLOAT, another_vx, &ano_vx)
LOG_ADD(LOG_FLOAT, another_vy, &ano_vy)
LOG_ADD(LOG_FLOAT, another_gz, &ano_gz)

LOG_ADD(LOG_FLOAT, own_vx, &own_vx)
LOG_ADD(LOG_FLOAT, own_vy, &own_vy)
LOG_ADD(LOG_FLOAT, own_gz, &own_gz)
LOG_GROUP_STOP(relative_pos)

PARAM_GROUP_START(relative_pos)
PARAM_ADD(PARAM_FLOAT, NoiseFlow, &procNoiseFlow_xy) // make sure the name if not too long
PARAM_ADD(PARAM_FLOAT, initCovX, &InitCovarianceX)
PARAM_ADD(PARAM_FLOAT, initCovY, &InitCovarianceY)
PARAM_ADD(PARAM_FLOAT, NoiseDist, &measNoiseDist)
PARAM_GROUP_STOP(relative_pos)