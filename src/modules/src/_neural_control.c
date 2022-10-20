// #include "config.h"
// #include "debug.h"
// #include "FreeRTOS.h"
// #include "queue.h"
// #include "static_mem.h"
// #include "task.h"
// #include "neural_control.h"
// #include "neural_com.h"
// #include "param.h"
// #include "pid.h"
// #include "motors.h"
// #include "commander.h"
// #include "log.h"
// #include "rateSupervisor.h"
// #include "led.h"
// #include "arm_math.h"
// #include "stm32fxxx.h"
// #include "pm.h"
// #include "filter.h"


// /*
// *   Definitions for PWM Control
// */
// #define PWM_100_HZ_CONTROL      100
// #define PWM_200_HZ_CONTROL      200
// #define PWM_500_HZ_CONTROL      500

// #define FREQUENCY_DIVIDER       2   // 100 Hz / FREQUENCY_DIVIDER 


// #define PWM_CONTROL_FREQ        PWM_100_HZ_CONTROL


// static setpoint_t landSetpoint = {.mode.z = modeAbs};
// const setpoint_t hoverSetpoint = {
//     .position = {.x = 0.0, .y = 0.0, .z = 1.0},
//     .mode = {.x = modeAbs, .y = modeAbs, .z = modeAbs}};

// setpoint_t resetPIDsetpoint = {.thrust = 0.0, .mode = {.pitch = modeAbs, .roll = modeAbs, .yaw = modeAbs}};
// setpoint_t attSetpoint = {.mode = {.pitch = modeAbs, .roll = modeAbs, .yaw = modeAbs}};
// setpoint_t attRateSetpoint = {.mode = {.pitch = modeVelocity, .roll = modeVelocity, .yaw = modeVelocity}};

// static bool isInit = false;
// const float deg2rad = 2 * PI / 360;
// const float rad2deg = 180 / PI;


// /*
// *   Queues for shared information between communicator and controller 
// */
// static xQueueHandle networkQueue;
// static xQueueHandle controlStateQueue;
// static xQueueHandle kalmanStateQueue;
// STATIC_MEM_QUEUE_ALLOC(networkQueue, 1, sizeof(Network *));
// STATIC_MEM_QUEUE_ALLOC(controlStateQueue, 1, sizeof(controlState_t));
// STATIC_MEM_QUEUE_ALLOC(kalmanStateQueue, 1, sizeof(state_t));

// static xQueueHandle pwmBypassQueue;
// static xQueueHandle ratePidBypassQueue;
// static xQueueHandle neuralOutputQueue;
// STATIC_MEM_QUEUE_ALLOC(pwmBypassQueue, 1, sizeof(int));
// STATIC_MEM_QUEUE_ALLOC(ratePidBypassQueue, 1, sizeof(int));
// STATIC_MEM_QUEUE_ALLOC(neuralOutputQueue, 1, sizeof(float16_t*));

// static void neuralControlTask(void *);
// STATIC_MEM_TASK_ALLOC(neuralControlTask, NEURALCONTROL_TASK_STACKSIZE);

// static bool rateWarningDisplayed = false;
// static rateSupervisor_t rateSupervisorContext;

// static uint32_t counter = 0;
// static uint32_t ffcounter = 0;
// static uint32_t first_NN_call = 1;


// void neuralControlTaskInit()
// {
//   networkQueue = STATIC_MEM_QUEUE_CREATE(networkQueue);
//   controlStateQueue = STATIC_MEM_QUEUE_CREATE(controlStateQueue);
//   kalmanStateQueue = STATIC_MEM_QUEUE_CREATE(kalmanStateQueue);
//   pwmBypassQueue = STATIC_MEM_QUEUE_CREATE(pwmBypassQueue);
//   ratePidBypassQueue = STATIC_MEM_QUEUE_CREATE(ratePidBypassQueue);
//   neuralOutputQueue = STATIC_MEM_QUEUE_CREATE(neuralOutputQueue);

//   STATIC_MEM_TASK_CREATE(neuralControlTask, neuralControlTask, NEURALCONTROL_TASK_NAME, NULL, NEURALCONTROL_TASK_PRI);

//   isInit = true;
// }

// bool neuralControlTaskTest()
// {
//   return isInit;
// }

// static void neuralControlTask(void *parameters)
// {
//   DEBUG_PRINT("Neural-Control task main function is running!\n");

//   Network *activeNetwork;
//   controlState_t controlState;
//   state_t kalmanState;
//   droneState_t droneState;
//   droneState_t lastDroneState;

//   neuralSetpoint_t neuralSetpoint;
//   setpoint_t commandSetpoint;

//   float16_t networkInput[NETWORK_INPUT_DIMENSION];
//   for(int i=0; i<NETWORK_INPUT_DIMENSION; i++){
//     networkInput[i]=0;
//   }
//   float16_t normalizedNetworkInput[NETWORK_INPUT_DIMENSION];
//   float16_t lastOutput[NETWORK_OUTPUT_DIMENSION];


//   bool pwmBypass = false;
//   bool ratePidBypass = false;
//   int delaytime = 1000 / PWM_CONTROL_FREQ;
//   uint32_t loggingFreqCounter = PWM_CONTROL_FREQ / 100;

//   xQueueReceive(networkQueue, &activeNetwork, portMAX_DELAY);

//   // vTaskDelay(3000);

//   rateSupervisorInit(&rateSupervisorContext, xTaskGetTickCount(), M2T(1000), PWM_CONTROL_FREQ-10, PWM_CONTROL_FREQ+10, 1);
//   uint32_t nn_start_time = xTaskGetTickCount();
//   TickType_t lastTime = xTaskGetTickCount();
  

//   while (1)
//   {
//     vTaskDelayUntil(&lastTime, delaytime); 

//     xQueuePeek(kalmanStateQueue, &kalmanState, 0);
//     xQueuePeek(controlStateQueue, &controlState, 0);

//     createDroneState(&kalmanState, &droneState, &lastDroneState);

//     switch (controlState)
//     {
//     case CONTROL_STATE_IDLE:
//       nn_start_time = xTaskGetTickCount();
//       break;
//     case CONTROL_STATE_NEURAL:
//       ledSet(ERR_LED1, 1);
//       ledSet(ERR_LED2, 1);

//       if(first_NN_call == 1)  // FIXME Mathias: loadNetworkInput must depend on history size
//       {
//         loadNetworkInput(&droneState, lastOutput, networkInput, (float16_t)(((double)(xTaskGetTickCount() - nn_start_time))/1000.0),*activeNetwork);
//         loadNetworkInput(&droneState, lastOutput, networkInput, (float16_t)(((double)(xTaskGetTickCount() - nn_start_time))/1000.0),*activeNetwork);
//         first_NN_call = 0;
//       }

//       if(ffcounter==0)  // use 2 in order to fill history with 2 states
//       {            
//         loadNetworkInput(&droneState, lastOutput, networkInput, (float16_t)(((double)(xTaskGetTickCount() - nn_start_time))/1000.0),*activeNetwork);
//         networkNormalizeInput(*activeNetwork, networkInput,normalizedNetworkInput);

//         networkFeedForward(*activeNetwork, normalizedNetworkInput, neuralSetpoint.value);
//       }
//       ffcounter++;
//       if(ffcounter==FREQUENCY_DIVIDER)  // this is the divider, e.g. 100 / ffcounter is control loop frequency
//       {
//         ffcounter=0;
//       }
//       // DEBUG_PRINT("NN 0:(%f %f %f %f) \n", (double)  neuralSetpoint.value[0], (double) neuralSetpoint.value[1], (double) neuralSetpoint.value[2], (double) neuralSetpoint.value[3]);
//       // vTaskDelay(50);
//       memcpy(lastOutput, &(neuralSetpoint.value), NETWORK_OUTPUT_DIMENSION*sizeof(float16_t));
//       networkClipOutput(*activeNetwork, neuralSetpoint.value, neuralSetpoint.clippedValue);
//       networkScaleOutput(*activeNetwork, neuralSetpoint.clippedValue);

//       neuralSetpoint.mode = (*activeNetwork)->controlMode;
//       convertSetpoint(&neuralSetpoint, &commandSetpoint);
//       commanderSetSetpoint(&commandSetpoint, 5);

//       logger(&droneState, &neuralSetpoint, &controlState, (float16_t)(((double)(xTaskGetTickCount() - nn_start_time))/1000.0), loggingFreqCounter);
//       safetyController(&droneState, &controlState);

//       break;
//     case CONTROL_STATE_HOVER:
//       commandSetpoint = hoverSetpoint;
//       commanderSetSetpoint(&commandSetpoint, 2);
//       first_NN_call = 1;
      
//       // lines that just need to run once before/after CONTROL_STATE_NEURAL (ToDo: more elegant solution?)
//       nn_start_time = xTaskGetTickCount();

//       //lastOutput[0]=(float16_t)(((double)motorsGetRatio(0) - 30000.0) / 30000.0);
//       //lastOutput[1]=(float16_t)(((double)motorsGetRatio(1) - 30000.0) / 30000.0);
//       //lastOutput[2]=(float16_t)(((double)motorsGetRatio(2) - 30000.0) / 30000.0);
//       // lastOutput[3]=(float16_t)(((double)motorsGetRatio(3) - 30000.0) / 30000.0);


//       // TODO: use next lines for Attitude Control:
//       lastOutput[0]=(float16_t)(0.0);
//       lastOutput[1]=(float16_t)(0.0);
//       lastOutput[2]=(float16_t)(0.0);
//       lastOutput[3]=(float16_t)(0.0);
//       ledSet(ERR_LED1, 0);
//       ledSet(ERR_LED2, 0);

//       break;
//     case CONTROL_STATE_LAND:
//       commandSetpoint = landSetpoint;
//       commanderSetSetpoint(&commandSetpoint, 2);
//       if (droneState.z < 0.2)
//       {
//         DEBUG_PRINT("Landed!\n");
//         controlState = CONTROL_STATE_IDLE;
//         xQueueOverwrite(controlStateQueue, &controlState);
//       }
//       break;
//     default:
//       DEBUG_PRINT("Invalid State!\n");
//     }

//     pwmBypass = (controlState == CONTROL_STATE_NEURAL) && (neuralSetpoint.mode == CONTROL_MODE_MOTORS);
//     xQueueOverwrite(pwmBypassQueue, &pwmBypass);
//     ratePidBypass = (controlState == CONTROL_STATE_NEURAL) && (neuralSetpoint.mode == CONTROL_MODE_ATTITUDE_RATE);
//     xQueueOverwrite(ratePidBypassQueue, &ratePidBypass);

//     if (!rateSupervisorValidate(&rateSupervisorContext, xTaskGetTickCount()))
//     {
//       if (!rateWarningDisplayed)
//       {
//         DEBUG_PRINT("WARNING: neural loop rate is offf (%lu Hz)\n", rateSupervisorLatestCount(&rateSupervisorContext));
//         rateWarningDisplayed = true;
//       }
//     }
//   }  // end of while
// }

// void safetyController(droneState_t *droneState, controlState_t *controlState)
// {
//   if (droneState->x < -2.5 || droneState->x > 2.5 ||
//       droneState->y < -2.5 || droneState->y > 2.5 ||
//       // droneState->z > 2.0 ||
//       // droneState->x_dot < -1.0 || droneState->x_dot > 1.0 ||
//       // droneState->y_dot < -1.0 || droneState->y_dot > 1.0 ||
//       // droneState->z_dot < -1.5 || droneState->z_dot > 2.5 ||
//       droneState->roll < (-30 * deg2rad) || droneState->roll > (30 * deg2rad) ||
//       droneState->pitch < (-30 * deg2rad) || droneState->pitch > (30 * deg2rad) ||
//       droneState->roll_dot < (-800 * deg2rad) || droneState->roll_dot > (800 * deg2rad) ||
//       droneState->pitch_dot < (-800 * deg2rad) || droneState->pitch_dot > (800 * deg2rad))
//   {
    
//     DEBUG_PRINT("Safety Stop!\nX: %f\nY: %f\nZ: %f\nVX: %f\nVY: %f\nVZ: %f\nRoll: %f\nPitch: %f\n rd: %f \n pd: %f \n",
//                 (double)droneState->x, (double)droneState->y,
//                 (double)droneState->z, (double)droneState->x_dot,
//                 (double)(droneState->y_dot), (double)(droneState->z_dot),
//                 (double)(droneState->roll / deg2rad), (double)(droneState->pitch / deg2rad),
//                 (double)(droneState->roll_dot / deg2rad), (double)(droneState->pitch_dot / deg2rad));

//     *controlState = CONTROL_STATE_HOVER;
//     xQueueOverwrite(controlStateQueue, controlState);
    
//   }
// }

// void loadNetworkInput(droneState_t *droneState, float16_t *lastOutput, float16_t *networkInput, float16_t startingTime, Network network)
// {
//   //length of Input to keep and shift
//   size_t keepInputLength = network->layers[0].layerSize - CIRCLE_NETWORK_INPUT_DIMENSION;
//   //shift Input-array 20 steps to the left
//   memmove(networkInput,networkInput+CIRCLE_NETWORK_INPUT_DIMENSION,keepInputLength*sizeof(float16_t));

//   // copies x, y ,z
//   memcpy(networkInput+keepInputLength, &(droneState->x), 3*sizeof(float16_t));
//   // copies roll, pitch, yaw
//   //memcpy(networkInput + 3, &(droneState->roll), 3*sizeof(float16_t));
//   // copies quaternion (a,b,c,d)
//   memcpy(networkInput+keepInputLength + 3, &(droneState->quaternion_a), 4*sizeof(float16_t));
//   // copies 3x3 entries of rotation matrix
//   // memcpy(networkInput + 3, &(droneState->R1), 9*sizeof(float16_t)); 
//   // copies x_dot, y_dot ,z_dot
//   memcpy(networkInput+keepInputLength + 7, &(droneState->x_dot), 3*sizeof(float16_t));
//   // copies roll_dot, pitch_dot ,yaw_dot
//   memcpy(networkInput+keepInputLength + 10, &(droneState->roll_dot), 3*sizeof(float16_t));
//   // copies last four outputs
//   memcpy(networkInput+keepInputLength + 16, lastOutput, NETWORK_OUTPUT_DIMENSION*sizeof(float16_t));
//   // copies error xyz
//   float16_t error_xyz[3];
//   float16_t alpha = 2.0f * PI / 3.0f;
//   error_xyz[0] = (float16_t)(0.25 * (1.0 - cos(startingTime * alpha))) - droneState->x;
//   error_xyz[1] = (float16_t)(0.25 * sin(startingTime * alpha)) - droneState->y;
//   error_xyz[2] = 1.0f - droneState->z;
//   memcpy(networkInput+keepInputLength + 13, error_xyz, 3*sizeof(float16_t));

//   // DEBUG_PRINT("networkInput: \n[");
//   // for(int i=0; i<network->layers[0].layerSize; i++){
//   //   DEBUG_PRINT("%f,",(double)networkInput[i]);
//   // }
//   // DEBUG_PRINT("]\n------------\n");
//   // vTaskDelay(1000);
  
// }

// void logger(droneState_t *droneState, neuralSetpoint_t *neuralSetpoint, controlState_t *controlState, float16_t time_stamp, uint32_t loggingFreqCounter)
// {
//   if(counter == loggingFreqCounter)
//   {   
//     counter = 0;
//     for (uint8_t i = 0; i < 12; i++)
//     {
//       neuralControlTaskEnqueueLogs(*(((float16_t *)droneState) + i));
//     }
//     for (uint8_t i = 0; i < 4; i++)
//     {
//       neuralControlTaskEnqueueLogs((float16_t) neuralSetpoint->value[i]);
//     }
//     for (uint8_t i = 0; i < 4; i++)
//     {
//       neuralControlTaskEnqueueLogs((float16_t) motorsGetRatio(i));
//     }

//     neuralControlTaskEnqueueLogs((float16_t) pmGetBatteryVoltage());
//     neuralControlTaskEnqueueLogs(time_stamp);
    
//   }
//   counter++;
// }

// void convertSetpoint(neuralSetpoint_t *neuralSetpoint, setpoint_t *commandSetpoint)
// // TODO: Need sensor data
// {

//   switch (neuralSetpoint->mode)
//   {
//   case CONTROL_MODE_MOTORS:
//     motorsSetRatio(0, neuralSetpoint->clippedValue[0]);
//     motorsSetRatio(1, neuralSetpoint->clippedValue[1]);
//     motorsSetRatio(2, neuralSetpoint->clippedValue[2]);
//     motorsSetRatio(3, neuralSetpoint->clippedValue[3]);
//     *commandSetpoint = resetPIDsetpoint;
//     break;
//   case CONTROL_MODE_ATTITUDE_RATE:
//     //TODO: Call resetNeuralAttitudeRateControllers when switching neuralSetpoint->mode
//     attRateSetpoint.thrust = neuralSetpoint->clippedValue[0];
//     attRateSetpoint.attitudeRate.roll = neuralSetpoint->clippedValue[1];
//     attRateSetpoint.attitudeRate.pitch = neuralSetpoint->clippedValue[2];
//     attRateSetpoint.attitudeRate.yaw = neuralSetpoint->clippedValue[3];
//     *commandSetpoint = attRateSetpoint;
//     break;
//   case CONTROL_MODE_ATTITUDE:
//     // DEBUG_PRINT("a");

//     attSetpoint.thrust = neuralSetpoint->value[0]*10000+45000; //41000;//
//     DEBUG_PRINT("thrust: %f\n", (double) neuralSetpoint->value[0]*10000+45000);
//     attSetpoint.attitude.roll = neuralSetpoint->value[1]*10;
//     attSetpoint.attitude.pitch = -neuralSetpoint->value[2]*10; //pitch flipped
//     attSetpoint.attitude.yaw = neuralSetpoint->value[3]*10;
//     *commandSetpoint = attSetpoint;

//     // DEBUG_PRINT("NN:(%f %f %f %f) \n", (double) attSetpoint.thrust, (double) attSetpoint.attitude.roll, (double) attSetpoint.attitude.pitch, (double) attSetpoint.attitude.yaw);
//     // vTaskDelay(50);

//     break;
//   default:
//     break;
//   }
// }

// void createDroneState(state_t *kalmanState, droneState_t *droneState, droneState_t *lastDroneState)
// {
//   // current time:
//   droneState->time = (float)(xTaskGetTickCount() / 1000.0);

//   // xyz and xyz_dot:
//   droneState->x = (float16_t)kalmanState->position.x;
//   droneState->y = (float16_t)kalmanState->position.y;
//   droneState->z = (float16_t)kalmanState->position.z;
//   droneState->x_dot = (float16_t)kalmanState->velocity.x;
//   droneState->y_dot = (float16_t)kalmanState->velocity.y;
//   droneState->z_dot = (float16_t)kalmanState->velocity.z;

//   // rpy in radian not degree; p inverted:
//   droneState->roll = (float16_t)(kalmanState->attitude.roll * deg2rad);
//   droneState->pitch = (float16_t)(-kalmanState->attitude.pitch * deg2rad);
//   droneState->yaw = (float16_t)(kalmanState->attitude.yaw * deg2rad);

//   // attitude as rotation matrix (3x3)
//   droneState->R1 = (float16_t)(kalmanState->flat_rotation_matrix.a);
//   droneState->R2 = (float16_t)(kalmanState->flat_rotation_matrix.b);
//   droneState->R3 = (float16_t)(kalmanState->flat_rotation_matrix.c);
//   droneState->R4 = (float16_t)(kalmanState->flat_rotation_matrix.d);
//   droneState->R5 = (float16_t)(kalmanState->flat_rotation_matrix.e);
//   droneState->R6 = (float16_t)(kalmanState->flat_rotation_matrix.f);
//   droneState->R7 = (float16_t)(kalmanState->flat_rotation_matrix.g);
//   droneState->R8 = (float16_t)(kalmanState->flat_rotation_matrix.h);
//   droneState->R9 = (float16_t)(kalmanState->flat_rotation_matrix.i);

//   // rpy_dot:
//   // droneState->roll_dot = (droneState->roll - lastDroneState->roll) / ((float16_t)(droneState->time - lastDroneState->time));
//   // droneState->pitch_dot = (droneState->pitch - lastDroneState->pitch) / ((float16_t)(droneState->time - lastDroneState->time));
//   // droneState->yaw_dot = (droneState->yaw - lastDroneState->yaw) / ((float16_t)(droneState->time - lastDroneState->time));

//   // attitude_rate from gyro which delivers [deg/s]
//   // droneState->roll_dot = (float16_t)(kalmanState->attitude_rate.x * deg2rad);
//   // droneState->pitch_dot = (float16_t)(kalmanState->attitude_rate.y * deg2rad);
//   // droneState->yaw_dot = (float16_t)(kalmanState->attitude_rate.z * deg2rad);

//   droneState->roll_dot = (float16_t)(kalmanState->attitude_rate_unfiltered.x * deg2rad);
//   droneState->pitch_dot = (float16_t)(kalmanState->attitude_rate_unfiltered.y * deg2rad);
//   droneState->yaw_dot = (float16_t)(kalmanState->attitude_rate_unfiltered.z * deg2rad);

//   // TODO: use filter
//   // applyAxis3fLpf((lpf2pData*)(&gyroLpf), &sensorData.gyro);

//   // droneState->gyro_x = (float16_t)(kalmanState->attitude_rate_unfiltered.x * deg2rad);
//   // droneState->gyro_y = (float16_t)(kalmanState->attitude_rate_unfiltered.y * deg2rad);
//   // droneState->gyro_z = (float16_t)(kalmanState->attitude_rate_unfiltered.z * deg2rad);

//   droneState->gyro_x = (float16_t)(kalmanState->attitude_rate.x * deg2rad);
//   droneState->gyro_y = (float16_t)(-kalmanState->attitude_rate.y * deg2rad);
//   droneState->gyro_z = (float16_t)(kalmanState->attitude_rate.z * deg2rad);

//   // DEBUG_PRINT("==== Sven in the house ====\n");

//   // DEBUG_PRINT("Old Gyro data: X: %f Y: %f Z: %f \n",
//   //           (double)(droneState->roll_dot * rad2deg), (double)(droneState->pitch_dot * rad2deg), (double)(droneState->yaw_dot * rad2deg));

//   // DEBUG_PRINT("New Gyro data: X: %f Y: %f Z: %f \n",
//   //           (double)kalmanState->attitude_rate.x, (double)kalmanState->attitude_rate.y, (double)kalmanState->attitude_rate.z);

//   // quaternion abcd:
//   float w, x, y, z;
//   ToQuaternion((float)droneState->yaw, (float)droneState->pitch, (float)droneState->roll, &w, &x, &y, &z);
//   droneState->quaternion_a = (float16_t)x;
//   droneState->quaternion_b = (float16_t)y;
//   droneState->quaternion_c = (float16_t)z;
//   droneState->quaternion_d = (float16_t)w;

//   // save last droneState:
//   *lastDroneState = *droneState;

// }

// void ToQuaternion(float yaw, float pitch, float roll, float *w, float *x, float *y, float *z) // yaw (Z), pitch (Y), roll (X)
// {
//   // Abbreviations for the various angular functions
//   float cy = cosf(yaw * 0.5f);
//   float sy = sinf(yaw * 0.5f);
//   float cp = cosf(pitch * 0.5f);
//   float sp = sinf(pitch * 0.5f);
//   float cr = cosf(roll * 0.5f);
//   float sr = sinf(roll * 0.5f);

//   *w = cr * cp * cy + sr * sp * sy;
//   *x = sr * cp * cy - cr * sp * sy;
//   *y = cr * sp * cy + sr * cp * sy;
//   *z = cr * cp * sy - sr * sp * cy;
// }

// void neuralControlTaskEnqueueNetwork(Network *network)
// {
//   xQueueSend(networkQueue, &network, 0);
// }

// void neuralControlTaskEnqueueControlState(controlState_t controlState)
// {
//   xQueueOverwrite(controlStateQueue, &controlState);
// }

// void neuralControlTaskPeekControlState(controlState_t *controlState)
// {
//   xQueuePeek(controlStateQueue, controlState, 0);
// }

// void neuralControlTaskEnqueueState(state_t state)
// {
//   xQueueOverwrite(kalmanStateQueue, &state);
// }

// void neuralControlTaskPeekPwmBypass(bool *pwmBypass)
// {
//   xQueuePeek(pwmBypassQueue, pwmBypass, 0);
// }

// void neuralControlTaskPeekRatePidBypass(bool *ratePidBypass)
// {
//   xQueuePeek(ratePidBypassQueue, ratePidBypass, 0);
// }


