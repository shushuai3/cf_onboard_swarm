#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "commander.h"

static bool isInit;

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
}

void relativeControlTask(void* arg)
{
  static setpoint_t setpoint;
  int i = 0;

  systemWaitStart();
  vTaskDelay(M2T(15000));

  for (i=0; i<20; i++) {
    setHoverSetpoint(&setpoint, 0, 0, 0.4, 0);
    commanderSetSetpoint(&setpoint, 3);
    vTaskDelay(M2T(100));
  }

  for (i=0; i<20; i++) {
    setHoverSetpoint(&setpoint, 0, 0, 0.6, 0);
    commanderSetSetpoint(&setpoint, 3);
    vTaskDelay(M2T(100));
  }

  for (i=0; i<50; i++) {
    setHoverSetpoint(&setpoint, 0.5, 0.2, 0.6, 0);
    commanderSetSetpoint(&setpoint, 3);
    vTaskDelay(M2T(100));
  }

  for (i=0; i<50; i++) {
    setHoverSetpoint(&setpoint, -0.3, -0.3, 0.6, 0);
    commanderSetSetpoint(&setpoint, 3);
    vTaskDelay(M2T(100));
  }

  for (i=0; i<30; i++) {
    setHoverSetpoint(&setpoint, -0.5, 0.1, 0.6, 0);
    commanderSetSetpoint(&setpoint, 3);
    vTaskDelay(M2T(100));
  }

  for (i=0; i<30; i++) {
    setHoverSetpoint(&setpoint, 0, 0, 0.2, 0);
    commanderSetSetpoint(&setpoint, 3);
    vTaskDelay(M2T(100));
  }
  
  while(1) {
    vTaskDelay(1000);
  }
}

void relativeControlInit(void)
{
  if (isInit)
    return;
  xTaskCreate(relativeControlTask,"relative_Control",2*configMINIMAL_STACK_SIZE, NULL,3,NULL );
  isInit = true;
}