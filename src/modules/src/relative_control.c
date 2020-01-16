#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "commander.h"
#include "relative_localization.h"
#include "num.h"
#include "param.h"
#include "debug.h"
#include <stdlib.h>

static bool isInit;
static bool onGround = true;
static uint8_t keepFlying = 0;
static setpoint_t setpoint;

// static float rl_x_p = 0.5f;
// static float rl_x_i = 0.0000f;
// static float rl_x_d = 0.01f;

// static float rl_y_p = 0.5f;
// static float rl_y_i = 0.0000f;
// static float rl_y_d = 0.01f;

// static float ref_rl_x = 0.0f;
// static float ref_rl_y = 1.2f;

// static float rl_x = 0.0f;
// static float rl_y = 1.2f;

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

static void flyRandomIn1meter(void){
  float_t randomYaw = (rand() / (float)RAND_MAX) * 6.28f; // 0-2pi rad
  float_t randomVel = (rand() / (float)RAND_MAX); // 0-1 m/s
  float_t vxBody = randomVel * cosf(randomYaw);
  float_t vyBody = randomVel * sinf(randomYaw);
  for (int i=1; i<100; i++) {
    setHoverSetpoint(&setpoint, vxBody, vyBody, 0.5, 0);
    vTaskDelay(M2T(10));
  }
  for (int i=1; i<100; i++) {
    setHoverSetpoint(&setpoint, -vxBody, -vyBody, 0.5, 0);
    vTaskDelay(M2T(10));
  }
}

void relativeControlTask(void* arg)
{
  systemWaitStart();
  // static float PreErr_rl_x = 0;
  // static float PreErr_rl_y = 0;
  // static float IntErr_rl_x = 0;
  // static float IntErr_rl_y = 0;
  // static float PreTime;
  // PreTime = xTaskGetTickCount();
  // vTaskDelay(10000);
  // PreTime = xTaskGetTickCount();
  while(1) {
    vTaskDelay(10);
    // uint32_t osTick = xTaskGetTickCount();
    // if( (osTick - PreTime) >= configTICK_RATE_HZ/RATE_50_HZ )
    // {
    //   if(read_rl_loco(&rl_x, &rl_y))
    //   {
    //     float dt = (float)(osTick-PreTime)/configTICK_RATE_HZ;
    //     float err_rl_x = -(ref_rl_x - rl_x);
    //     float err_rl_y = -(ref_rl_y - rl_y);

    //     float pid_vx = rl_x_p * err_rl_x;
    //     float pid_vy = rl_y_p * err_rl_y;

    //     float derivX = (err_rl_x - PreErr_rl_x) / dt;
    //     float derivY = (err_rl_y - PreErr_rl_y) / dt;
    //     pid_vx += rl_x_d * derivX;
    //     pid_vy += rl_y_d * derivY;

    //     IntErr_rl_x += err_rl_x * dt;
    //     IntErr_rl_y += err_rl_y * dt;

    //     pid_vx += rl_x_i * constrain(IntErr_rl_x, -0.5, 0.5);
    //     pid_vy += rl_y_i * constrain(IntErr_rl_y, -0.5, 0.5);

    //     pid_vx = constrain(pid_vx, -1, 1);
    //     pid_vy = constrain(pid_vy, -1, 1);
        
    //     PreErr_rl_x = err_rl_x;
    //     PreErr_rl_y = err_rl_y;

        // setHoverSetpoint(&setpoint, pid_vx, pid_vy, 0.4, 0);
        if(relativeInfoRead() && keepFlying){
          // take off
          if(onGround){
            for (int i=0; i<5; i++) {
              setHoverSetpoint(&setpoint, 0, 0, 0.3f, 0);
              vTaskDelay(M2T(100));
            }
            onGround = false;
          }

          // control loop
          // setHoverSetpoint(&setpoint, 0, 0, 0.5, 0); // hover
          flyRandomIn1meter(); // random flight

        }else{
          // landing procedure
          if(!onGround){
            for (int i=1; i<5; i++) {
              setHoverSetpoint(&setpoint, 0, 0, 0.3f-(float)i*0.05f, 0);
              vTaskDelay(M2T(10));
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
  xTaskCreate(relativeControlTask,"relative_Control",2*configMINIMAL_STACK_SIZE, NULL,3,NULL );
  isInit = true;
}

PARAM_GROUP_START(relative_ctrl)
PARAM_ADD(PARAM_UINT8, keepFlying, &keepFlying)
PARAM_GROUP_STOP(relative_ctrl)