/**
Shushuai li
 *
 */

#ifndef RELATIVELOCA_H_
#define RELATIVELOCA_H_
#include "system.h"
void relativeLocoInit(void);
void relativeLocoTask(void* arg);
void relativeEKF(int n, float vxi, float vyi, float ri, float vxj, float vyj, float rj, uint16_t dij, float dt);
#endif
