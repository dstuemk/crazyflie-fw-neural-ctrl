#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__

#include "stabilizer_types.h"
#include <stdint.h>
#include <stdlib.h>

#define REMOTE_CONTROL_RATE RATE_100_HZ

typedef _Float16 float16_t;

void remoteControlTaskInit();
bool remoteControlTaskTest();

void remoteControlTaskEnqueueState(state_t state);
void remoteControlTaskPeekPwmBypass(bool *pwm_bypass);

#endif 