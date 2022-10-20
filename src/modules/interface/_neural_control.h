// #ifndef _NEURALCONTROL_TASK_H_
// #define _NEURALCONTROL_TASK_H_

// #include <stdbool.h>
// #include "neural_net.h"
// #include "stabilizer_types.h"

// #define NERUAL_CONTROL_RATE RATE_50_HZ

// void neuralControlTaskInit();
// bool neuralControlTaskTest();

// void convertSetpoint(neuralSetpoint_t *neuralSetpoint, setpoint_t *commandSetpoint);
// void safetyController(droneState_t *droneState, controlState_t *controlState);
// void loadNetworkInput(droneState_t *droneState, float16_t *lastOutput, float16_t *networkInput, float16_t staringTime, Network network);
// void logger(droneState_t *droneState, neuralSetpoint_t *neuralSetpoint, controlState_t *controlState, float16_t time_stamp, uint32_t loggingFreqCounter);
// void createDroneState(state_t *kalmanState, droneState_t *droneState, droneState_t *lastDroneState);
// void ToQuaternion(float yaw, float pitch, float roll, float *w, float *x, float *y, float *z);

// void neuralControlTaskEnqueueNetwork(Network *network);
// void neuralControlTaskEnqueueControlState(controlState_t controlState);
// void neuralControlTaskPeekControlState(controlState_t *controlState);
// void neuralControlTaskEnqueueState(state_t state);
// void neuralControlTaskPeekPwmBypass(bool *pwmBypass);
// void neuralControlTaskPeekRatePidBypass(bool *ratePidBypass);
// #endif // __NEURALCONTROL_TASK_H__
