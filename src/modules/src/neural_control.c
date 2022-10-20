#include "FreeRTOS.h"
#include "debug.h"
#include "queue.h"
#include "neural_control.h"
#include "config.h"
#include "static_mem.h"
#include "task.h"
#include "crtp.h"
#include "commander.h"
#include "rateSupervisor.h"
#include "motors.h"
#include <math.h>
#include <string.h>

#include "neural_forward.h"
#include "neural_recurrent.h"
#include "neural_cascaded.h"

#include "neural_forward.c"
#include "neural_recurrent.c"
#include "neural_cascaded.c"

#define max(a,b)            (((a) > (b)) ? (a) : (b))
#define min(a,b)            (((a) < (b)) ? (a) : (b))

static const float deg2rad = 2 * M_PI / 360;
static const float G = 9.81;

typedef struct {
    float16_t x;
    float16_t y;
    float16_t z;
    float16_t x_dot;
    float16_t y_dot;
    float16_t z_dot;
    float16_t roll;
    float16_t pitch;
    float16_t yaw;
    float16_t roll_dot;
    float16_t pitch_dot;
    float16_t yaw_dot;
    float16_t quaternion_a;
    float16_t quaternion_b;
    float16_t quaternion_c;
    float16_t quaternion_d;
    float16_t gyro_x;   // Added by Sven (Raw Gyro sensor values)
    float16_t gyro_y;   // Added by Sven (Raw Gyro sensor values)
    float16_t gyro_z;   // Added by Sven (Raw Gyro sensor values)
    float16_t acc_x;    // Added by Daniel (Acc values)
    float16_t acc_y;    // Added by Daniel (Acc values)
    float16_t acc_z;    // Added by Daniel (Acc values)
    // R1 - R9: Added (04.08.2021) by Sven to make state identical to USC paper (Molchanov et al.)
    //float16_t R1;
    //float16_t R2;
    //float16_t R3;
    //float16_t R4;
    //float16_t R5;
    //float16_t R6;
    //float16_t R7;
    //float16_t R8;
    //float16_t R9;
    // M1 - M4: Last Motor commands, added by Daniel
    float16_t M1;
    float16_t M2;
    float16_t M3;
    float16_t M4;
    float16_t time;
} neural_drone_state_t;

typedef enum {
    NEURALCONTROL_CMD_HOVER = 0x00,
    NEURALCONTROL_CMD_LAND  = 0x01,
    NEURALCONTROL_CMD_EXEC  = 0x02,
    NEURALCONTROL_CMD_START = 0x03,
    NEURALCONTROL_CMD_TRANSF= 0x04,
    NEURALCONTROL_CMD_ADJUST= 0x05,
    NEURALCONTROL_CMD_NONE  = 0x06,
    NEURALCONTROL_CMD_CIRCLE= 0x07,
} NEURALCONTROL_CMD_ID;

typedef struct {
    NEURALCONTROL_CMD_ID cmd_id;
    uint8_t              cmd_data[128];
} neural_control_cmd_t;

static const setpoint_t landSetpoint = {.mode.z = modeAbs};
static const setpoint_t hoverSetpoint = {
     .position = {.x = 0.0, .y = 0.0, .z = 1.0},
     .mode = {.x = modeAbs, .y = modeAbs, .z = modeAbs}};
static const setpoint_t resetPIDsetpoint = {
        .thrust = 0.0, 
        .mode = {.pitch = modeAbs, .roll = modeAbs, .yaw = modeAbs}};

typedef enum {
    NEURALCONTROL_ON_GROUND   = 0x00,
    NEURALCONTROL_TAKE_OFF    = 0x01,
    NEURALCONTROL_HOVERING    = 0x02,
    NEURALCONTROL_EXECUTING   = 0x03,
    NEURALCONTROL_TRANSFERING = 0x04,
    NEURALCONTROL_LANDING     = 0x05,
    NEURALCONTROL_INIT        = 0x06,
    NEURALCONTROL_ADJUSTING   = 0x07,
    NEURALCONTROL_CIRCLING    = 0x08,
} NEURALCONTROL_STATE;

static const char *neuralcontrol_state_names[] = {
    "ON_GROUND",
    "TAKE_OFF",
    "HOVERING",
    "EXECUTING",
    "TRANSFERING",
    "LANDING",
    "INIT",
    "ADJUSTING",
    "CIRCLING"
};

typedef enum {
    NEURALCONTROL_NET_FF    = 0x00,
    NEURALCONTROL_NET_RNN   = 0x01,
    NEURALCONTROL_NET_CASC  = 0x02,
    NEURALCONTROL_NET_NONE  = 0x03
} NEURALCONTROL_NET;

static const char *neuralcontrol_net_names[] = {
    "NET_FF",
    "NET_RNN",
    "NET_CASC",
    "NET_NONE"
};

typedef enum {
    NEURALCONTROL_FF_NORMAL_BETA    = 0x00,
    NEURALCONTROL_FF_NORMAL_GAMMA   = 0x01,
    NEURALCONTROL_FF_NORMAL_MEAN    = 0x02,
    NEURALCONTROL_FF_NORMAL_OUTPUT  = 0x03,
    NEURALCONTROL_FF_NORMAL_STDEV   = 0x04,
    NEURALCONTROL_FF_HIDDEN1_BIAS   = 0x05,
    NEURALCONTROL_FF_HIDDEN1_KERNEL = 0x06,
    NEURALCONTROL_FF_HIDDEN1_OUTPUT = 0x07,
    NEURALCONTROL_FF_HIDDEN2_BIAS   = 0x08,
    NEURALCONTROL_FF_HIDDEN2_KERNEL = 0x09,
    NEURALCONTROL_FF_HIDDEN2_OUTPUT = 0x0A,
    NEURALCONTROL_FF_OUTPUT_BIAS    = 0x0B,
    NEURALCONTROL_FF_OUTPUT_KERNEL  = 0x0C,
    NEURALCONTROL_FF_OUTPUT_OUTPUT  = 0x0D
} NEURALCONTROL_FF_LAYER;

typedef enum {
    NEURALCONTROL_RNN_NORMAL_BETA     = 0x00,
    NEURALCONTROL_RNN_NORMAL_GAMMA    = 0x01,
    NEURALCONTROL_RNN_NORMAL_MEAN     = 0x02,
    NEURALCONTROL_RNN_NORMAL_OUTPUT   = 0x03,
    NEURALCONTROL_RNN_NORMAL_STDEV    = 0x04,
    NEURALCONTROL_RNN_HIDDEN1_BIAS    = 0x05,
    NEURALCONTROL_RNN_HIDDEN1_KERNEL  = 0x06,
    NEURALCONTROL_RNN_HIDDEN1_OUTPUT  = 0x07,
    NEURALCONTROL_RNN_HIDDEN1_RKERNEL = 0x08,
    NEURALCONTROL_RNN_HIDDEN2_BIAS    = 0x09,
    NEURALCONTROL_RNN_HIDDEN2_KERNEL  = 0x0A,
    NEURALCONTROL_RNN_HIDDEN2_OUTPUT  = 0x0B,
    NEURALCONTROL_RNN_HIDDEN2_RKERNEL = 0x0C,
    NEURALCONTROL_RNN_OUTPUT_BIAS     = 0x0D,
    NEURALCONTROL_RNN_OUTPUT_KERNEL   = 0x0E,
    NEURALCONTROL_RNN_OUTPUT_OUTPUT   = 0x0F
} NEURALCONTROL_RNN_LAYER;

typedef enum {
    NEURALCONTROL_CASC_NORMAL_BETA     = 0x00,
    NEURALCONTROL_CASC_NORMAL_GAMMA    = 0x01,
    NEURALCONTROL_CASC_NORMAL_MEAN     = 0x02,
    NEURALCONTROL_CASC_NORMAL_OUTPUT   = 0x03,
    NEURALCONTROL_CASC_NORMAL_STDEV    = 0x04,
    NEURALCONTROL_CASC_CONCAT12_OUTPUT = 0x05,
    NEURALCONTROL_CASC_HIDDEN1_BIAS    = 0x06,
    NEURALCONTROL_CASC_HIDDEN1_KERNEL  = 0x07,
    NEURALCONTROL_CASC_HIDDEN1_OUTPUT  = 0x08,
    NEURALCONTROL_CASC_HIDDEN1_RKERNEL = 0x09,
    NEURALCONTROL_CASC_HIDDEN2_BIAS    = 0x0A,
    NEURALCONTROL_CASC_HIDDEN2_KERNEL  = 0x0B,
    NEURALCONTROL_CASC_HIDDEN2_OUTPUT  = 0x0C,
    NEURALCONTROL_CASC_OUTPUT_BIAS     = 0x0D,
    NEURALCONTROL_CASC_OUTPUT_KERNEL   = 0x0E,
    NEURALCONTROL_CASC_OUTPUT_OUTPUT   = 0x0F
} NEURALCONTROL_CASC_LAYER;

static const char *neuralcontrol_layer_names[3][16] = {
    {
        "FF_NORMAL_BETA",
        "FF_NORMAL_GAMMA",
        "FF_NORMAL_MEAN",
        "FF_NORMAL_OUTPUT",
        "FF_NORMAL_STDEV",
        "FF_HIDDEN1_BIAS",
        "FF_HIDDEN1_KERNEL",
        "FF_HIDDEN1_OUTPUT",
        "FF_HIDDEN2_BIAS",
        "FF_HIDDEN2_KERNEL",
        "FF_HIDDEN2_OUTPUT",
        "FF_OUTPUT_BIAS",
        "FF_OUTPUT_KERNEL",
        "FF_OUTPUT_OUTPUT",
        "---",
        "---"
    },
    {
        "RNN_NORMAL_BETA",
        "RNN_NORMAL_GAMMA",
        "RNN_NORMAL_MEAN",
        "RNN_NORMAL_OUTPUT",
        "RNN_NORMAL_STDEV",
        "RNN_HIDDEN1_BIAS",
        "RNN_HIDDEN1_KERNEL",
        "RNN_HIDDEN1_OUTPUT",
        "RNN_HIDDEN1_RKERNEL",
        "RNN_HIDDEN2_BIAS",
        "RNN_HIDDEN2_KERNEL",
        "RNN_HIDDEN2_OUTPUT",
        "RNN_HIDDEN2_RKERNEL",
        "RNN_OUTPUT_BIAS",
        "RNN_OUTPUT_KERNEL",
        "RNN_OUTPUT_OUTPUT"
    },
    {
        "CASC_NORMAL_BETA",
        "CASC_NORMAL_GAMMA",
        "CASC_NORMAL_MEAN",
        "CASC_NORMAL_OUTPUT",
        "CASC_NORMAL_STDEV",
        "CASC_CONCAT12_OUTPUT",
        "CASC_HIDDEN1_BIAS",
        "CASC_HIDDEN1_KERNEL",
        "CASC_HIDDEN1_OUTPUT",
        "CASC_HIDDEN1_RKERNEL",
        "CASC_HIDDEN2_BIAS",
        "CASC_HIDDEN2_KERNEL",
        "CASC_HIDDEN2_OUTPUT",
        "CASC_OUTPUT_BIAS",
        "CASC_OUTPUT_KERNEL",
        "CASC_OUTPUT_OUTPUT"
    }
};


#define NEURALCONTROL_NET_MAX_H 5

static float16_t neuralcontrol_net_input[14*NEURALCONTROL_NET_MAX_H] = {0};
static int neuralcontrol_net_H = 0;
static k2c_tensor neuralcontrol_ff_input = { 0 };
static k2c_tensor neuralcontrol_rnn_input = { 0 };

static void neuralControlInitNet() {
    neuralcontrol_net_H = neural_forward_input_shapes[0][2] / 14;
    k2c_tensor _neuralcontrol_ff_input = {
        &neuralcontrol_net_input[0],
        2,
        14*neuralcontrol_net_H,
        { 1,14*neuralcontrol_net_H, 1, 1, 1}
    };
    neuralcontrol_ff_input = _neuralcontrol_ff_input;
    k2c_tensor _neuralcontrol_rnn_input = {
        &neuralcontrol_net_input[14*(neuralcontrol_net_H - 1)],
        2,
        14,
        { 1,14, 1, 1, 1}
    };
    neuralcontrol_rnn_input = _neuralcontrol_rnn_input;

    neural_recurrent_reset_states();
}

static void neuralControlUpdateNetInput(neural_drone_state_t droneState, uint32_t netStartTicks) {
    uint16_t offset = (neuralcontrol_net_H - 1) * 14;
    memmove(&neuralcontrol_net_input[0], 
      &neuralcontrol_net_input[14], 
      sizeof(neuralcontrol_net_input) - 14*sizeof(float16_t));
    neuralcontrol_net_input[0 + offset] = droneState.z;
    neuralcontrol_net_input[1 + offset] = droneState.acc_x;
    neuralcontrol_net_input[2 + offset] = droneState.acc_y;
    neuralcontrol_net_input[3 + offset] = droneState.acc_z;
    neuralcontrol_net_input[4 + offset] = droneState.roll_dot;
    neuralcontrol_net_input[5 + offset] = droneState.pitch_dot;
    neuralcontrol_net_input[6 + offset] = droneState.yaw_dot;

    float netTime = (float)(xTaskGetTickCount() - netStartTicks) / (float)M2T(1000);
    float alpha = 2.0f * (float)M_PI / 3.0f;
    neuralcontrol_net_input[7 + offset] = 0.25f * (float)(1.0 - cos(netTime * alpha)) - droneState.x;
    neuralcontrol_net_input[8 + offset] = 0.25f * (float)sin(netTime * alpha) - droneState.y;
    neuralcontrol_net_input[9 + offset] = 1.0f - droneState.z;
    
    neuralcontrol_net_input[10 + offset] = droneState.M1; 
    neuralcontrol_net_input[11 + offset] = droneState.M2; 
    neuralcontrol_net_input[12 + offset] = droneState.M3; 
    neuralcontrol_net_input[13 + offset] = droneState.M4; 
}

static NEURALCONTROL_NET neuralcontrol_current_net = NEURALCONTROL_NET_NONE;
static float16_t *neuralcontrol_weights[16] = { NULL }; 

static void neuralControlBuildNet(NEURALCONTROL_NET net_type, int8_t layer_idx, 
  uint16_t layer_sz, uint16_t param_idx, float16_t param_val) {
    // Save type of neural network (recurrent, feed-forward or cascaded)
    neuralcontrol_current_net = net_type;

    // Initialize building phase
    if (layer_idx == 0 && param_idx == 0) {
        DEBUG_PRINT("Start receiving network: %s\n", neuralcontrol_net_names[net_type]);
        // Wipe out memory (if allocated)
        for (uint8_t idx=0; 
             idx < sizeof(neuralcontrol_weights) / sizeof(neuralcontrol_weights[0]); 
             idx++) {
            if (neuralcontrol_weights[idx] != NULL) {
                free(neuralcontrol_weights[idx]);
                neuralcontrol_weights[idx] = NULL;
            }
        }
    }

    // Copy parameters
    if (param_idx % (layer_sz / 10) == 0)
        DEBUG_PRINT("Receiving Paramter %d / %d (%f) of %s\n", 
          param_idx, layer_sz, (double)param_val, neuralcontrol_layer_names[net_type][layer_idx]);
    if (param_idx == 0) {
        neuralcontrol_weights[layer_idx] = malloc(sizeof(float16_t) * layer_sz);
    }
    neuralcontrol_weights[layer_idx][param_idx] = param_val;
}

static int32_t neuralcontrol_compute_nops;
static int32_t neuralcontrol_compute_low;
static int32_t neuralcontrol_compute_high;

static float neuralcontrol_inference_time = 0.0008f; // Inference time in seconds

static void neuralControlComputeSpeedInit() {
    neuralcontrol_compute_nops = 1000;
    neuralcontrol_compute_low  = 0;
    neuralcontrol_compute_high = 40000;
}

static void neuralControlComputeSlower() {
    neuralcontrol_compute_low = neuralcontrol_compute_nops;
    //neuralcontrol_compute_nops = ((float)neuralcontrol_compute_high - neuralcontrol_compute_low) / 2.f + neuralcontrol_compute_low;
    neuralcontrol_compute_nops = /* Increase execution time less agressively */
        ceilf((float)(neuralcontrol_compute_high - neuralcontrol_compute_low) / 8.f) + neuralcontrol_compute_low;
}

static void neuralControlComputeFaster() {
    neuralcontrol_compute_high = neuralcontrol_compute_nops;
    neuralcontrol_compute_nops = 
        floorf((float)(neuralcontrol_compute_high - neuralcontrol_compute_low) / 2.f) + neuralcontrol_compute_low;
}

static void neuralControlComputeNetOutput(float16_t* buffer) {
    k2c_tensor c_dense = {buffer,2,4,{1,4,1,1,1}}; 
    switch(neuralcontrol_current_net) {
        case NEURALCONTROL_NET_FF:
            neural_forward(
                &neuralcontrol_ff_input,
                &c_dense,
                neuralcontrol_weights[NEURALCONTROL_FF_NORMAL_OUTPUT],
                neuralcontrol_weights[NEURALCONTROL_FF_NORMAL_MEAN],
                neuralcontrol_weights[NEURALCONTROL_FF_NORMAL_STDEV],
                neuralcontrol_weights[NEURALCONTROL_FF_NORMAL_GAMMA],
                neuralcontrol_weights[NEURALCONTROL_FF_NORMAL_BETA],
                neuralcontrol_weights[NEURALCONTROL_FF_HIDDEN1_OUTPUT],
                neuralcontrol_weights[NEURALCONTROL_FF_HIDDEN1_KERNEL],
                neuralcontrol_weights[NEURALCONTROL_FF_HIDDEN1_BIAS],
                neuralcontrol_weights[NEURALCONTROL_FF_HIDDEN2_OUTPUT],
                neuralcontrol_weights[NEURALCONTROL_FF_HIDDEN2_KERNEL],
                neuralcontrol_weights[NEURALCONTROL_FF_HIDDEN2_BIAS],
                neuralcontrol_weights[NEURALCONTROL_FF_OUTPUT_OUTPUT],
                neuralcontrol_weights[NEURALCONTROL_FF_OUTPUT_KERNEL],
                neuralcontrol_weights[NEURALCONTROL_FF_OUTPUT_BIAS]);
            break;
        case NEURALCONTROL_NET_RNN:
            neural_recurrent(
                &neuralcontrol_rnn_input,
                &c_dense,
                neuralcontrol_weights[NEURALCONTROL_RNN_NORMAL_OUTPUT],
                neuralcontrol_weights[NEURALCONTROL_RNN_NORMAL_MEAN],
                neuralcontrol_weights[NEURALCONTROL_RNN_NORMAL_STDEV],
                neuralcontrol_weights[NEURALCONTROL_RNN_NORMAL_GAMMA],
                neuralcontrol_weights[NEURALCONTROL_RNN_NORMAL_BETA],
                neuralcontrol_weights[NEURALCONTROL_RNN_HIDDEN1_OUTPUT],
                neuralcontrol_weights[NEURALCONTROL_RNN_HIDDEN1_KERNEL],
                neuralcontrol_weights[NEURALCONTROL_RNN_HIDDEN1_RKERNEL],
                neuralcontrol_weights[NEURALCONTROL_RNN_HIDDEN1_BIAS],
                neuralcontrol_weights[NEURALCONTROL_RNN_HIDDEN2_OUTPUT],
                neuralcontrol_weights[NEURALCONTROL_RNN_HIDDEN2_KERNEL],
                neuralcontrol_weights[NEURALCONTROL_RNN_HIDDEN2_RKERNEL],
                neuralcontrol_weights[NEURALCONTROL_RNN_HIDDEN2_BIAS],
                neuralcontrol_weights[NEURALCONTROL_RNN_OUTPUT_OUTPUT],
                neuralcontrol_weights[NEURALCONTROL_RNN_OUTPUT_KERNEL],
                neuralcontrol_weights[NEURALCONTROL_RNN_OUTPUT_BIAS]);
            break;
        case NEURALCONTROL_NET_CASC:
            neural_cascaded(
                &neuralcontrol_rnn_input,
                &c_dense,
                neuralcontrol_weights[NEURALCONTROL_CASC_NORMAL_OUTPUT],
                neuralcontrol_weights[NEURALCONTROL_CASC_NORMAL_MEAN],
                neuralcontrol_weights[NEURALCONTROL_CASC_NORMAL_STDEV],
                neuralcontrol_weights[NEURALCONTROL_CASC_NORMAL_GAMMA],
                neuralcontrol_weights[NEURALCONTROL_CASC_NORMAL_BETA],
                neuralcontrol_weights[NEURALCONTROL_CASC_HIDDEN1_OUTPUT],
                neuralcontrol_weights[NEURALCONTROL_CASC_HIDDEN1_KERNEL],
                neuralcontrol_weights[NEURALCONTROL_CASC_HIDDEN1_RKERNEL],
                neuralcontrol_weights[NEURALCONTROL_CASC_HIDDEN1_BIAS],
                neuralcontrol_weights[NEURALCONTROL_CASC_CONCAT12_OUTPUT],
                neuralcontrol_weights[NEURALCONTROL_CASC_HIDDEN2_OUTPUT],
                neuralcontrol_weights[NEURALCONTROL_CASC_HIDDEN2_KERNEL],
                neuralcontrol_weights[NEURALCONTROL_CASC_HIDDEN2_BIAS],
                neuralcontrol_weights[NEURALCONTROL_CASC_OUTPUT_OUTPUT],
                neuralcontrol_weights[NEURALCONTROL_CASC_OUTPUT_KERNEL],
                neuralcontrol_weights[NEURALCONTROL_CASC_OUTPUT_BIAS]);
            break;
        case NEURALCONTROL_NET_NONE:
            break;
    }

    for (int32_t nop_ctr = 0; nop_ctr < neuralcontrol_compute_nops; nop_ctr++) {
        asm volatile ("nop");
    }
}

typedef struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPower_t;

static void neuralControlTask(void *);
STATIC_MEM_TASK_ALLOC(neuralControlTask, NEURALCONTROL_TASK_STACKSIZE);

static xQueueHandle motorPowerQueue;
STATIC_MEM_QUEUE_ALLOC(motorPowerQueue, 1, sizeof(motorPower_t));

static xQueueHandle kalmanStateQueue;
STATIC_MEM_QUEUE_ALLOC(kalmanStateQueue, 1, sizeof(state_t));

static xQueueHandle pwmBypassQueue;
STATIC_MEM_QUEUE_ALLOC(pwmBypassQueue, 1, sizeof(int));

static rateSupervisor_t neuralRateSupervisor;

static void neuralCreateDroneState(state_t *, float16_t*, neural_drone_state_t *);

static bool neuralSafetyCheck(neural_drone_state_t);

void neuralControlTaskInit() {
    kalmanStateQueue = STATIC_MEM_QUEUE_CREATE(kalmanStateQueue);
    pwmBypassQueue = STATIC_MEM_QUEUE_CREATE(pwmBypassQueue);
    motorPowerQueue = STATIC_MEM_QUEUE_CREATE(motorPowerQueue);
    STATIC_MEM_TASK_CREATE(neuralControlTask, neuralControlTask, NEURALCONTROL_TASK_NAME, NULL, NEURALCONTROL_TASK_PRI);
}

bool neuralControlTaskTest() {
    return true;
}

static void neuralSendControlState(NEURALCONTROL_STATE neural_control_state) {
    CRTPPacket packet;
    packet.header = CRTP_HEADER(CRTP_PORT_NEURALCONTROL, 0);
    packet.size = 2;
    packet.data[0] = 0xFF;
    packet.data[1] = neural_control_state;
    crtpSendPacket(&packet);
}

static void neuralSendDroneState(neural_drone_state_t drone_state) {
    CRTPPacket packet;

    const int16_t bytes_per_packet = 29;

    for(int16_t packet_idx=0; 
      packet_idx < ceilf(((float)sizeof(drone_state)) / bytes_per_packet); 
      packet_idx++) {
        packet.header = CRTP_HEADER(CRTP_PORT_NEURALCONTROL, 0);
        packet.size = 30;

        int16_t data_size = bytes_per_packet;
        int16_t diff = sizeof(drone_state) - (packet_idx + 1)*bytes_per_packet;
        if (diff < 0)
            data_size += diff;

        memcpy(packet.data + 1, ((uint8_t *)&drone_state) + packet_idx*bytes_per_packet, data_size);
        packet.data[0] = packet_idx;
        crtpSendPacket(&packet);
    }
}

static void neuralReceiveCommand(neural_control_cmd_t *cmd_buffer) {
    cmd_buffer->cmd_id = NEURALCONTROL_CMD_NONE;

    CRTPPacket packet;
    
    if(crtpReceivePacketWait(CRTP_PORT_NEURALCONTROL, &packet, 1) == pdTRUE) {
        switch(packet.data[0]) {
            case NEURALCONTROL_CMD_START:
                cmd_buffer->cmd_id = NEURALCONTROL_CMD_START;
                break;

            case NEURALCONTROL_CMD_HOVER:
                cmd_buffer->cmd_id = NEURALCONTROL_CMD_HOVER;
                break;
            
            case NEURALCONTROL_CMD_LAND:
                cmd_buffer->cmd_id = NEURALCONTROL_CMD_LAND;
                break;
            
            case NEURALCONTROL_CMD_EXEC:
                cmd_buffer->cmd_id = NEURALCONTROL_CMD_EXEC;
                break;

            case NEURALCONTROL_CMD_CIRCLE:
                cmd_buffer->cmd_id = NEURALCONTROL_CMD_CIRCLE;
                break;

            case NEURALCONTROL_CMD_ADJUST:
            {
                cmd_buffer->cmd_id = NEURALCONTROL_CMD_ADJUST;
                neuralcontrol_inference_time = *((float16_t*)&packet.data[1]);
                neuralcontrol_inference_time /= 1000.f; // ms to sec
                break;
            }
            
            case NEURALCONTROL_CMD_TRANSF:
            {
                cmd_buffer->cmd_id = NEURALCONTROL_CMD_TRANSF;
                const int16_t values_per_packet = 10;
                int8_t net_type  = packet.data[1] >> 4;
                int8_t layer_idx = packet.data[1] & 0x0F;
                uint16_t layer_sz = *((uint16_t*)(&packet.data[2]));
                uint16_t param_idx = *((uint16_t*)(&packet.data[4]));
                for (uint16_t idx_val=0; 
                  idx_val < values_per_packet && idx_val + param_idx < layer_sz; 
                  idx_val++) {
                    float16_t param_val = ((float16_t*)(&packet.data[6]))[idx_val];
                    neuralControlBuildNet(net_type, layer_idx, layer_sz, param_idx + idx_val, param_val);   
                }
                break;
            }

            default:
                DEBUG_PRINT("Unkown command received: %d\n", packet.data[0]);
                break;
        }
    }
}

static void neuralSetMotors(float m0, float m1, float m2, float m3) {
    float pwm_values[] = {m0, m1, m2, m3};
    for (uint32_t pwm_idx=0; pwm_idx < 4; pwm_idx++) {
        float set_value = pwm_values[pwm_idx];
        if (set_value > 1.0f) set_value = 1.0f;
        if (set_value < -1.0f) set_value = -1.0f;
        set_value = (set_value + 1.0f)*30000.0f;
        //DEBUG_PRINT("Set Motor %ld to value %d\n", pwm_idx, (uint16_t)set_value);
        motorsSetRatio(pwm_idx, (uint16_t)set_value);
    }
    static setpoint_t setpoint;
    setpoint = resetPIDsetpoint;
    commanderSetSetpoint(&setpoint, 5);
}

static void neuralMotorPowerToAction(motorPower_t motorPower, float16_t *action) {
    uint16_t *motors_u16 = &motorPower;
    for (uint16_t i=0; i < 4; i++) {
        action[i] = (float)motors_u16[i] / 30000.0f - 1.0f; 
    }
}

static bool neuralSafetyCheck(neural_drone_state_t droneState) {
    if (droneState.x < -2.5 || droneState.x > 2.5 ||
       droneState.y < -2.5 || droneState.y > 2.5 ||
       droneState.roll < (-30 * deg2rad) || droneState.roll > (30 * deg2rad) ||
       droneState.pitch < (-30 * deg2rad) || droneState.pitch > (30 * deg2rad) ||
       droneState.roll_dot < (-800 * deg2rad) || droneState.roll_dot > (800 * deg2rad) ||
       droneState.pitch_dot < (-800 * deg2rad) || droneState.pitch_dot > (800 * deg2rad))
       return false;
    return true;
}

static void toQuaternion(float yaw, float pitch, float roll, float *w, float *x, float *y, float *z) {
  // yaw (Z), pitch (Y), roll (X)

  // Abbreviations for the various angular functions
  float cy = cosf(yaw * 0.5f);
  float sy = sinf(yaw * 0.5f);
  float cp = cosf(pitch * 0.5f);
  float sp = sinf(pitch * 0.5f);
  float cr = cosf(roll * 0.5f);
  float sr = sinf(roll * 0.5f);

  *w = cr * cp * cy + sr * sp * sy;
  *x = sr * cp * cy - cr * sp * sy;
  *y = cr * sp * cy + sr * cp * sy;
  *z = cr * cp * sy - sr * sp * cy;
}

static void neuralCreateDroneState(state_t *kalmanState, float16_t* lastAction, neural_drone_state_t *droneState) {
  // current time:
  droneState->time = (float16_t)( (float)xTaskGetTickCount() / (float)M2T(1000) );

  // xyz and xyz_dot:
  droneState->x = (float16_t)kalmanState->position.x;
  droneState->y = (float16_t)kalmanState->position.y;
  droneState->z = (float16_t)kalmanState->position.z;
  droneState->x_dot = (float16_t)kalmanState->velocity.x;
  droneState->y_dot = (float16_t)kalmanState->velocity.y;
  droneState->z_dot = (float16_t)kalmanState->velocity.z;

  // accelerations
  droneState->acc_x = (float16_t)(kalmanState->acc.x * G);
  droneState->acc_y = (float16_t)(kalmanState->acc.y * G);
  droneState->acc_z = (float16_t)(kalmanState->acc.z * G);

  // rpy in radian not degree; p inverted:
  droneState->roll  = (float16_t)(kalmanState->attitude.roll * deg2rad);
  droneState->pitch = (float16_t)(-kalmanState->attitude.pitch * deg2rad);
  droneState->yaw   = (float16_t)(kalmanState->attitude.yaw * deg2rad);

  // attitude as rotation matrix (3x3)
  //droneState->R1 = (float16_t)(kalmanState->flat_rotation_matrix.a);
  //droneState->R2 = (float16_t)(kalmanState->flat_rotation_matrix.b);
  //droneState->R3 = (float16_t)(kalmanState->flat_rotation_matrix.c);
  //droneState->R4 = (float16_t)(kalmanState->flat_rotation_matrix.d);
  //droneState->R5 = (float16_t)(kalmanState->flat_rotation_matrix.e);
  //droneState->R6 = (float16_t)(kalmanState->flat_rotation_matrix.f);
  //droneState->R7 = (float16_t)(kalmanState->flat_rotation_matrix.g);
  //droneState->R8 = (float16_t)(kalmanState->flat_rotation_matrix.h);
  //droneState->R9 = (float16_t)(kalmanState->flat_rotation_matrix.i);

  droneState->roll_dot  = (float16_t)(kalmanState->attitude_rate_unfiltered.x * deg2rad);
  droneState->pitch_dot = (float16_t)(kalmanState->attitude_rate_unfiltered.y * deg2rad);
  droneState->yaw_dot   = (float16_t)(kalmanState->attitude_rate_unfiltered.z * deg2rad);

  droneState->gyro_x = (float16_t)(kalmanState->attitude_rate.x * deg2rad);
  droneState->gyro_y = (float16_t)(-kalmanState->attitude_rate.y * deg2rad);
  droneState->gyro_z = (float16_t)(kalmanState->attitude_rate.z * deg2rad);

  // quaternion abcd:
  float w, x, y, z;
  toQuaternion((float)droneState->yaw, (float)droneState->pitch, (float)droneState->roll, &w, &x, &y, &z);
  droneState->quaternion_a = (float16_t)x;
  droneState->quaternion_b = (float16_t)y;
  droneState->quaternion_c = (float16_t)z;
  droneState->quaternion_d = (float16_t)w;

  // Last action
  droneState->M1 = lastAction[0];
  droneState->M2 = lastAction[1];
  droneState->M3 = lastAction[2];
  droneState->M4 = lastAction[3];

}

void neuralControlTaskEnqueueMotorPower(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
    motorPower_t motor_power = {
        .m1 = m1,
        .m2 = m2,
        .m3 = m3,
        .m4 = m4
    };
    xQueueOverwrite(motorPowerQueue, &motor_power);
}

void neuralControlTaskEnqueueState(state_t state) {
    xQueueOverwrite(kalmanStateQueue, &state);
}

void neuralControlTaskPeekPwmBypass(bool *pwm_bypass) {
    int pwmBypass_int;
    if (pdPASS == xQueuePeek(pwmBypassQueue, &pwmBypass_int, 0))
        *pwm_bypass = pwmBypass_int != 0;
}

static setpoint_t neuralCircleSetpoint(uint32_t iteration) {
    setpoint_t out = {
        .position = {.x = 0.0, .y = 0.0, .z = 1.0},
        .mode = {.x = modeAbs, .y = modeAbs, .z = modeAbs}};
    float t = (float)iteration / 100.0f;
    float alpha = 2.0f * (float)M_PI / 3.0f;
    out.position.x = 0.25f * (float)(1.0 - cos(t * alpha));
    out.position.y = 0.25f * (float)sin(t * alpha);
    out.position.z = 1.0f;
    return out;
}

static void neuralControlTask(void *params) {
    DEBUG_PRINT("Neural-Control task main function is running\n");

    neuralControlInitNet();

    neuralControlComputeSpeedInit();

    crtpInitTaskQueue(CRTP_PORT_NEURALCONTROL);

    rateSupervisorInit(&neuralRateSupervisor, 
        xTaskGetTickCount(), 
        M2T(1000), 
        NEURAL_CONTROL_RATE-NEURAL_CONTROL_RATE/10, 
        NEURAL_CONTROL_RATE+NEURAL_CONTROL_RATE/10, 1);
    TickType_t lastTime = xTaskGetTickCount();

    NEURALCONTROL_STATE neuralcontrol_state = NEURALCONTROL_INIT;

    neuralSendControlState(neuralcontrol_state);

    TickType_t netStartTicks = 0;
    float16_t lastAction[4] = { 0.f, 0.f, 0.f, 0.f }; 

    uint32_t circleStartIt = 0;
    uint32_t circleIt = 0;

    float inferenceTime = 0.f;

    for(uint32_t it_count = 0; true; it_count++) {

        int pwmBypass = 0; 

        state_t kalmanState;
        xQueuePeek(kalmanStateQueue, &kalmanState, 0);

        motorPower_t motorPower;
        xQueuePeek(motorPowerQueue, &motorPower, 0);
        
        neural_drone_state_t droneState;
        neuralCreateDroneState(&kalmanState, lastAction, &droneState);

        neuralControlUpdateNetInput(droneState, netStartTicks);
                
        neural_control_cmd_t control_cmd = { .cmd_id = NEURALCONTROL_CMD_NONE };
        neuralReceiveCommand(&control_cmd);
        
        NEURALCONTROL_STATE neuralcontrol_state_next = neuralcontrol_state;

        setpoint_t new_setpoint;

        // State actions
        switch(neuralcontrol_state) {
            case NEURALCONTROL_INIT:
                if (NEURALCONTROL_CMD_START == control_cmd.cmd_id)
                    neuralcontrol_state_next = NEURALCONTROL_ON_GROUND;
                break;
            
            case NEURALCONTROL_TRANSFERING:
                // TODO
                if (NEURALCONTROL_CMD_START == control_cmd.cmd_id)
                    neuralcontrol_state_next = NEURALCONTROL_ADJUSTING;
                break;
            
            case NEURALCONTROL_ADJUSTING:
                if ((inferenceTime >= neuralcontrol_inference_time - 0.0001f 
                      && inferenceTime <= neuralcontrol_inference_time + 0.0001f) ||
                      neuralcontrol_compute_high - neuralcontrol_compute_low <= 10) {
                    neuralcontrol_state_next = NEURALCONTROL_ON_GROUND;
                    DEBUG_PRINT("Target / Used inference time: %.3f / %.3f ms\n",
                        (double)neuralcontrol_inference_time * 1000.0,
                        (double)inferenceTime * 1000.0);
                }
                break;

            case NEURALCONTROL_ON_GROUND:
                //neuralSendDroneState(droneState);
                if (NEURALCONTROL_CMD_ADJUST == control_cmd.cmd_id)
                    neuralcontrol_state_next = NEURALCONTROL_ADJUSTING;
                if (NEURALCONTROL_CMD_HOVER == control_cmd.cmd_id)
                    neuralcontrol_state_next = NEURALCONTROL_TAKE_OFF;
                if (NEURALCONTROL_CMD_TRANSF == control_cmd.cmd_id)
                    neuralcontrol_state_next = NEURALCONTROL_TRANSFERING;
                break;

            case NEURALCONTROL_TAKE_OFF:
                netStartTicks = xTaskGetTickCount();
                //neuralSendDroneState(droneState);
                new_setpoint = hoverSetpoint;
                commanderSetSetpoint(&new_setpoint, 5);
                if (droneState.z > 0.8) 
                    neuralcontrol_state_next = NEURALCONTROL_HOVERING;
                break;

            case NEURALCONTROL_HOVERING:
                netStartTicks = xTaskGetTickCount();
                //neuralSendDroneState(droneState);
                new_setpoint = hoverSetpoint;
                commanderSetSetpoint(&new_setpoint, 5);
                if (NEURALCONTROL_CMD_LAND == control_cmd.cmd_id)
                    neuralcontrol_state_next = NEURALCONTROL_LANDING;
                if (NEURALCONTROL_CMD_EXEC == control_cmd.cmd_id)
                    neuralcontrol_state_next = NEURALCONTROL_EXECUTING;
                if (NEURALCONTROL_CMD_CIRCLE == control_cmd.cmd_id)
                    neuralcontrol_state_next = NEURALCONTROL_CIRCLING;
                break;
            
            case NEURALCONTROL_CIRCLING:
                neuralSendDroneState(droneState);
                circleIt = it_count - circleStartIt - 1;
                // Update goal position every 250ms
                if (circleIt % 25 == 0) {
                    // Use future setpoint to compensate the slow PID position-control
                    new_setpoint = neuralCircleSetpoint((circleIt / 25 + 2)*25);
                    commanderSetSetpoint(&new_setpoint, 5);
                }
                if (!neuralSafetyCheck(droneState)) {
                    DEBUG_PRINT("PID Circling lead to instable behaviour\n");
                    neuralcontrol_state_next = NEURALCONTROL_LANDING;
                }
                if (NEURALCONTROL_CMD_LAND == control_cmd.cmd_id)
                    neuralcontrol_state_next = NEURALCONTROL_LANDING;
                break;

            case NEURALCONTROL_LANDING:
                netStartTicks = xTaskGetTickCount();
                //neuralSendDroneState(droneState);
                new_setpoint = landSetpoint;
                commanderSetSetpoint(&new_setpoint, 5);
                if (droneState.z < 0.2) 
                    neuralcontrol_state_next = NEURALCONTROL_ON_GROUND;
            break;

            case NEURALCONTROL_EXECUTING:
                neuralSendDroneState(droneState);
                if (!neuralSafetyCheck(droneState)) {
                    DEBUG_PRINT("Neural control lead to instable behaviour\n");
                    neuralcontrol_state_next = NEURALCONTROL_LANDING;
                }
                if (NEURALCONTROL_CMD_HOVER == control_cmd.cmd_id)
                    neuralcontrol_state_next = NEURALCONTROL_HOVERING;
                if (NEURALCONTROL_CMD_LAND == control_cmd.cmd_id)
                    neuralcontrol_state_next = NEURALCONTROL_LANDING;
                break;
        }

        // Transition actions
        if (neuralcontrol_state != neuralcontrol_state_next) {
            DEBUG_PRINT("Neural-State transition %s -> %s\n", 
                neuralcontrol_state_names[neuralcontrol_state],
                neuralcontrol_state_names[neuralcontrol_state_next]);
            neuralSendControlState(neuralcontrol_state_next);
        }
        if (NEURALCONTROL_ON_GROUND == neuralcontrol_state &&
            NEURALCONTROL_ON_GROUND == neuralcontrol_state_next) {
            // Can be used for Debug stuff ...
        }
        if (NEURALCONTROL_ON_GROUND == neuralcontrol_state &&
            NEURALCONTROL_EXECUTING == neuralcontrol_state_next) {
            neuralControlInitNet();
            neuralControlComputeNetOutput(lastAction);
            neuralSetMotors(
                lastAction[0],
                lastAction[1],
                lastAction[2],
                lastAction[3]);
            // Enable PWM bypass
            pwmBypass = 1;
            xQueueOverwrite(pwmBypassQueue, &pwmBypass);
        }
        if (NEURALCONTROL_HOVERING == neuralcontrol_state &&
            NEURALCONTROL_EXECUTING == neuralcontrol_state_next) {
            neuralControlInitNet();
            // Prefill input
            for (uint16_t i=0; i < NEURALCONTROL_NET_MAX_H; i++) {
                lastAction[0] = 0.0; lastAction[1] = 0.0; lastAction[2] = 0.0; lastAction[3] = 0.0;
                neuralCreateDroneState(&kalmanState, lastAction, &droneState);
                neuralControlUpdateNetInput(droneState, netStartTicks);
            }
            neuralControlComputeNetOutput(lastAction);
            neuralSetMotors(
                lastAction[0],
                lastAction[1],
                lastAction[2],
                lastAction[3]);
            // Enable PWM bypass
            pwmBypass = 1;
            xQueueOverwrite(pwmBypassQueue, &pwmBypass);
        }
        if (NEURALCONTROL_HOVERING == neuralcontrol_state &&
            NEURALCONTROL_HOVERING == neuralcontrol_state_next) {
            neuralMotorPowerToAction(motorPower, lastAction);
            if (it_count % 200 == 0) {
                DEBUG_PRINT("pid_action = [");
                for (int action_idx=0; action_idx < 4; action_idx++) {
                    DEBUG_PRINT(" %.3f,", (double)lastAction[action_idx]);
                    vTaskDelay(1);
                }
                DEBUG_PRINT(" ]\n");
            }
        }
        if (NEURALCONTROL_HOVERING == neuralcontrol_state &&
            NEURALCONTROL_CIRCLING == neuralcontrol_state_next) {
            circleStartIt = it_count;
            neuralMotorPowerToAction(motorPower, lastAction);
        }
        if (NEURALCONTROL_CIRCLING == neuralcontrol_state &&
            NEURALCONTROL_CIRCLING == neuralcontrol_state_next) {
            neuralMotorPowerToAction(motorPower, lastAction);
        }
        if (NEURALCONTROL_EXECUTING == neuralcontrol_state &&
            NEURALCONTROL_EXECUTING == neuralcontrol_state_next) {
            neuralControlComputeNetOutput(lastAction);
            neuralSetMotors(
                lastAction[0],
                lastAction[1],
                lastAction[2],
                lastAction[3]);
            // Enable PWM bypass
            pwmBypass = 1;
            xQueueOverwrite(pwmBypassQueue, &pwmBypass);
        }
        if (NEURALCONTROL_EXECUTING == neuralcontrol_state &&
            NEURALCONTROL_LANDING == neuralcontrol_state_next) {
            // Disable PWM bypass
            pwmBypass = 0;
            xQueueOverwrite(pwmBypassQueue, &pwmBypass);
        }
        if (NEURALCONTROL_EXECUTING == neuralcontrol_state &&
            NEURALCONTROL_HOVERING == neuralcontrol_state_next) {
            // Disable PWM bypass
            pwmBypass = 0;
            xQueueOverwrite(pwmBypassQueue, &pwmBypass);
        }
        if (NEURALCONTROL_ON_GROUND == neuralcontrol_state && 
            NEURALCONTROL_ADJUSTING == neuralcontrol_state_next) {
            // Reset upper and lower bound of nop's
            neuralControlComputeSpeedInit();
            // Disable PWM bypass
            pwmBypass = 0;
            xQueueOverwrite(pwmBypassQueue, &pwmBypass);
        }
        if (NEURALCONTROL_ADJUSTING == neuralcontrol_state &&
            NEURALCONTROL_ADJUSTING == neuralcontrol_state_next) {
            // Check Neural net execution time
            neuralControlInitNet();
            TickType_t beforeInference = xTaskGetTickCount();
            uint16_t infer_ctr;
            float16_t dummy[4];
            for (infer_ctr=0; infer_ctr < 10; infer_ctr++) {
                neuralControlComputeNetOutput(dummy);
                if (((float)xTaskGetTickCount() - beforeInference) / (float)M2T(1) > 5.f)
                    break; // Avoid getting killed by watchdog
            }
            inferenceTime = ((float)xTaskGetTickCount() - beforeInference) / (float)M2T(1000) / (float)(infer_ctr + 1);
            DEBUG_PRINT("Avg. Inference Time for %s: %.3f msec.\n", 
                neuralcontrol_net_names[neuralcontrol_current_net],
                (double)inferenceTime * 1000.0);
            if (inferenceTime > neuralcontrol_inference_time + 0.00005f) neuralControlComputeFaster();
            if (inferenceTime < neuralcontrol_inference_time - 0.00005f) neuralControlComputeSlower();
            // Disable PWM bypass
            pwmBypass = 0;
            xQueueOverwrite(pwmBypassQueue, &pwmBypass);
        }
        neuralcontrol_state = neuralcontrol_state_next;

        // Wait remaining time
        vTaskDelayUntil(&lastTime, M2T(1000) / NEURAL_CONTROL_RATE); 

        // Rate supervision
        if (!rateSupervisorValidate(&neuralRateSupervisor, xTaskGetTickCount())) {
            DEBUG_PRINT("WARNING: neural loop rate is off (%lu Hz)\n", 
                rateSupervisorLatestCount(&neuralRateSupervisor));
        }
    }
}