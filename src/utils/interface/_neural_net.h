// #ifndef _NEURAL_NET_H_
// #define _NEURAL_NET_H_

// #include <stdlib.h>
// #include <stdint.h>

// #define CIRCLE_NETWORK_INPUT_DIMENSION 20
// #define NETWORK_INPUT_DIMENSION 100 //21// 3 (xyz) + 4 (quaternion) + 3 (vel) + 3 (rpy_dot) + 4 (last_action) = 17 + 1 (add one to avoid errors)
// #define NETWORK_OUTPUT_DIMENSION 4

// /*
// *   Data structures and types 
// */

// typedef _Float16 float16_t;

// typedef enum
// {
//     NET_SUCCESS,
//     MALLOC_FAILED,
//     UNKNOWN_ACTIVATION_FUNCTION
// } netErrCode_t;

// typedef enum
// {
//     IDENTITY,
//     RELU,
//     TANH,
//     SVIDENTITY
// } activationFunction_t;

// typedef enum
// {
//     CONTROL_STATE_START,
//     CONTROL_STATE_IDLE,
//     CONTROL_STATE_NEURAL,
//     CONTROL_STATE_HOVER,
//     CONTROL_STATE_LAND
// } controlState_t;

// typedef enum
// {
//     CONTROL_MODE_MOTORS,
//     CONTROL_MODE_ATTITUDE_RATE,
//     CONTROL_MODE_ATTITUDE
// } controlMode_t;

// typedef enum
// {
//     COMMAND_TRANSFER_NETWORK,
//     COMMAND_START_NEURAL_CONTROL,
//     COMMAND_STOP_NEURAL_CONTROL,
//     COMMAND_HOVER,
//     COMMAND_LAND
// } command_t;

// typedef struct neuron_s
// {
//     float16_t activation;
//     float16_t * weights;
//     float16_t bias;
// } neuron_t;

// typedef struct layer_s
// {
//     size_t layerSize;
//     neuron_t * neurons;
//     float16_t (*activationFunction)(float16_t);
// } layer_t;

// // saves a & b parameters for scaling a vector x: a*x + b
// typedef struct scaler_s
// {
//     float16_t * a;
//     float16_t * b;
// } scaler_t;

// typedef struct network_s
// {
//     size_t networkSize;
//     layer_t * layers;
//     scaler_t inputNormalizer;
//     scaler_t outputScaler;
//     controlMode_t controlMode;
//     float16_t checksum;
// } network_t;

// typedef network_t *Network;

// typedef struct droneState_s
// {
//     float16_t x;
//     float16_t y;
//     float16_t z;
//     float16_t x_dot;
//     float16_t y_dot;
//     float16_t z_dot;
//     float16_t roll;
//     float16_t pitch;
//     float16_t yaw;
//     float16_t roll_dot;
//     float16_t pitch_dot;
//     float16_t yaw_dot;
//     float16_t quaternion_a;
//     float16_t quaternion_b;
//     float16_t quaternion_c;
//     float16_t quaternion_d;
//     float16_t gyro_x;   // Added by Sven (Raw Gyro sensor values)
//     float16_t gyro_y;   // Added by Sven (Raw Gyro sensor values)
//     float16_t gyro_z;   // Added by Sven (Raw Gyro sensor values)
//     // R1 - R9: Added (04.08.2021) by Sven to make state identical to USC paper (Molchanov et al.)
//     float16_t R1;
//     float16_t R2;
//     float16_t R3;
//     float16_t R4;
//     float16_t R5;
//     float16_t R6;
//     float16_t R7;
//     float16_t R8;
//     float16_t R9;
//     float time;
// } droneState_t;

// typedef struct neuralSetpoint_s
// {
//     float16_t value[NETWORK_OUTPUT_DIMENSION];
//     float16_t clippedValue[NETWORK_OUTPUT_DIMENSION];
//     controlMode_t mode;
// } neuralSetpoint_t;

// /*
// *   Function declarations
// */

// netErrCode_t networkInit(Network network, size_t networkSize, size_t *layerSize);
// netErrCode_t networkDestroy(Network network);

// netErrCode_t networkSetActivationFunction(Network network, size_t layer, activationFunction_t activationFunction);
// netErrCode_t networkSetWeight(Network network, size_t layer, size_t neuron, size_t weight, float16_t value);
// netErrCode_t networkSetBias(Network network, size_t layer, size_t neuron, float16_t value);
// netErrCode_t networkSetInputNormalizer(Network network, float16_t *a, float16_t *b);
// netErrCode_t networkSetOutputScaler(Network network, float16_t *a, float16_t *b);
// netErrCode_t networkSetControlMode(Network network, controlMode_t controlMode);

// netErrCode_t networkNormalizeInput(Network network, float16_t *input, float16_t *normalizedInput);

// netErrCode_t networkFeedForward(Network network, float16_t *input, float16_t *output);

// netErrCode_t networkScaleOutput(Network network, float16_t *output);

// netErrCode_t networkClipOutput(Network network, float16_t *input, float16_t *output);

// float16_t activationFunctionIdentity(float16_t x);
// float16_t activationFunctionRelu(float16_t x);
// float16_t activationFunction_tanh(float16_t x);
// float16_t activationFunctionSvidentity(float16_t x);
// float16_t networkCalculateChecksum(Network network);

// #endif