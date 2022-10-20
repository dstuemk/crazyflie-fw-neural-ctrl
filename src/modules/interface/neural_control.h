#ifndef __NEURAL_CONTROL_H__
#define __NEURAL_CONTROL_H__

#include "stabilizer_types.h"
#include <stdint.h>
#include <stdlib.h>

#define NEURAL_CONTROL_RATE RATE_100_HZ

typedef _Float16 float16_t;

void neuralControlTaskInit();
bool neuralControlTaskTest();

void neuralControlTaskEnqueueMotorPower(
    uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);
void neuralControlTaskEnqueueState(state_t state);
void neuralControlTaskPeekPwmBypass(bool *pwm_bypass);

#endif 