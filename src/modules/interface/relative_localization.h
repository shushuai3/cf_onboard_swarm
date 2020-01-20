#ifndef RELATIVELOCA_H_
#define RELATIVELOCA_H_
#include "system.h"

typedef enum {
  STATE_rlX, STATE_rlY, STATE_rlYaw, STATE_DIM_rl
} relative_stateIdx_t;

typedef enum
{
  INPUT_vxi, INPUT_vyi, INPUT_ri, INPUT_vxj, INPUT_vyj, INPUT_rj, INPUT_DIM
} relative_inputIdx_t;

typedef struct
{
  float S[STATE_DIM_rl];
  float P[STATE_DIM_rl][STATE_DIM_rl];
  uint32_t oldTimetick;
  bool receiveFlag;
} relaVariable_t;

void relativeLocoInit(void);
void relativeLocoTask(void* arg);
void relativeEKF(int n, float vxi, float vyi, float ri, float vxj, float vyj, float rj, uint16_t dij, float dt);
bool relativeInfoRead(float* relaVarParam);
#endif