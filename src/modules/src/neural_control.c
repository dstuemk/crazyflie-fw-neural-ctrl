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

#define k2c_float float16_t
#include "k2c_definitions.h"

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
    NEURALCONTROL_CMD_CONFIG= 0x08,
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

/******************************************************/
/** Neural Network Code *******************************/

typedef enum {
    LINEAR  	= 0,
    RELU        = 1
} netlib_activation_t;

static int netlib_activation(netlib_activation_t activation, 
  k2c_tensor input, k2c_tensor output) {
    switch(activation) {
        case LINEAR:
        memcpy(output.array, input.array, input.numel*sizeof(input.array[0]));
        k2c_linear(output.array, output.numel);
        return 0;

        case RELU:
        memcpy(output.array, input.array, input.numel*sizeof(input.array[0]));
        k2c_relu(output.array, output.numel);
        return 0;
    }
    return -1;
}

typedef enum {
    NORMALIZATION   = 0,
    DENSE           = 1,
    LSTM            = 2,
    ACTIVATION      = 3
} netlib_layer_type_t;

typedef struct {
    int n_parameters;
    netlib_layer_type_t netlib_layer_type;
    k2c_tensor input;
    k2c_tensor output;
    int _write_pos;
    unsigned char parameters[0];
} netlib_layer_t;

static k2c_float netlib_work_mem[5300];

static int netlib_layer_inference(netlib_layer_t* layer) {
    switch(layer->netlib_layer_type) {
        case NORMALIZATION:
        {   // parameters [mean,std,gamma,beta]
            int n = layer->input.shape[1];       
            k2c_tensor batch_normalization_mean = 
              { &((k2c_float*)layer->parameters)[0*n],1,n,{n, 1, 1, 1, 1} }; 
            k2c_tensor batch_normalization_stdev = 
              { &((k2c_float*)layer->parameters)[1*n],1,n,{n, 1, 1, 1, 1} }; 
            k2c_tensor batch_normalization_gamma = 
              { &((k2c_float*)layer->parameters)[2*n],1,n,{n, 1, 1, 1, 1} }; 
            k2c_tensor batch_normalization_beta = 
              { &((k2c_float*)layer->parameters)[3*n],1,n,{n, 1, 1, 1, 1} }; 
            k2c_batch_norm(
                &(layer->output),
                &(layer->input),
                &batch_normalization_mean,
                &batch_normalization_stdev,
                &batch_normalization_gamma,
                &batch_normalization_beta,
                1); 
        }
        break;

        case DENSE:
        {   // parameters [kernel,bias] 
            int n_in = layer->input.shape[1];
            int n_out = layer->output.shape[1];
            k2c_tensor dense_kernel = 
              { &((k2c_float*)layer->parameters)[0], 2, n_in*n_out,{n_in,n_out, 1, 1, 1} }; 
            k2c_tensor dense_bias   = 
              { &((k2c_float*)layer->parameters)[n_in*n_out],1,n_out,{n_out, 1, 1, 1, 1} }; 
            k2c_dense(
                &(layer->output),
                &(layer->input),
                &dense_kernel, 
	            &dense_bias,
                k2c_linear,
                netlib_work_mem); 
        }
        break;

        case LSTM:
        {   // parameters [kernel,r_kernel,bias,c_state,h_state]
            int n_in = layer->input.shape[1];
            int n_out = layer->output.shape[1];
            int offset = 0;
            int sz = 4*n_in*n_out;
            k2c_tensor lstm_kernel = 
              {&((k2c_float*)layer->parameters)[offset],2,sz,{4*n_in,n_out, 1, 1, 1}}; 
            offset += sz;
            sz = 4*n_out*n_out;
            k2c_tensor lstm_recurrent_kernel = 
              {&((k2c_float*)layer->parameters)[offset],2,sz,{4*n_out,n_out, 1, 1, 1}}; 
            offset += sz;
            sz = 4*n_out;
            k2c_tensor lstm_bias = 
              {&((k2c_float*)layer->parameters)[offset],1,sz,{sz, 1, 1, 1, 1}}; 
            offset += sz;
            k2c_float* lstm_state = &((k2c_float*)layer->parameters)[offset]; 
            k2c_lstm(
                &(layer->output),
                &(layer->input),
                lstm_state,
                &lstm_kernel, 
                &lstm_recurrent_kernel,
                &lstm_bias,
                netlib_work_mem, 
                0,
                1, 
                k2c_sigmoid,
                k2c_tanh); 

        }
        break;

        case ACTIVATION:
        // parameter (byte): 0=linear, 1=relu
        return netlib_activation(((k2c_float*)layer->parameters)[0],layer->input,layer->output);
    }
    return 0;
}

#define NETLIB_MAX_LAYERS 32

static struct {
    netlib_layer_t* layers[NETLIB_MAX_LAYERS];
    int num_layers;
} netlib_model;

static void netlib_reset_states() {
    for (int i=0; i<netlib_model.num_layers; i++) {
        netlib_layer_t* layer = netlib_model.layers[i];
        switch (layer->netlib_layer_type)
        {
            case LSTM:
            {
                int state_dim = 2*layer->output.numel;
                k2c_float* states = &((k2c_float*) layer->parameters)
                    [layer->n_parameters - state_dim];
                for (int state_idx=0; state_idx<state_dim; state_idx++) {
                    states[state_idx] = 0;
                }
            }
            break;
            
            default:
            break;
        }
    }
}

static void netlib_inferene(k2c_float* input, k2c_float* output) {
    if (0 == netlib_model.num_layers) return;
    netlib_layer_t* layer = NULL;
    for (int i=0; i<netlib_model.num_layers; i++) {
        //DEBUG_PRINT("Prepare Layer %d ... ", i);
        //vTaskDelay(1);
        layer = netlib_model.layers[i];
        if (0 == i)
            memcpy(layer->input.array, input, sizeof(input[0])*layer->input.shape[1]);
        else {
            netlib_layer_t* prev_layer = netlib_model.layers[i-1];
            memcpy(layer->input.array, prev_layer->output.array, 
              sizeof(input[0])*layer->input.shape[1]);
        }
        //DEBUG_PRINT("Compute Layer %d\n", i);
        //vTaskDelay(1);
        netlib_layer_inference(layer);
    }
    memcpy(output, layer->output.array, sizeof(output[0])*layer->output.numel);
}

static void netlib_init_model() {
    netlib_model.num_layers = 0;
    for (int i=0; i<sizeof(netlib_work_mem)/sizeof(netlib_work_mem[0]); i++)
        netlib_work_mem[i] = 0.f;
}

static void netlib_reset() {
    for (int i=0; i<netlib_model.num_layers; i++) {
        netlib_layer_t* layer = netlib_model.layers[i];
        free(layer->input.array);
        free(layer->output.array);
        free(layer);
    }
    netlib_model.num_layers = 0;
    for (int i=0; i<sizeof(netlib_work_mem)/sizeof(netlib_work_mem[0]); i++)
        netlib_work_mem[i] = 0.f;
}

static int netlib_write_layer(unsigned char* bytes_in, int n_bytes) {
    // New Layer
    int layer_idx  = netlib_model.num_layers-1;
    if (0 > layer_idx || netlib_model.layers[layer_idx]->_write_pos < 0) {
        // New layer
        if (NETLIB_MAX_LAYERS - 1 == netlib_model.num_layers) return -1;
        if (n_bytes < 4*sizeof(int)) return -1;
        int n_parameters = ((int*)(bytes_in))[0];
        int netlib_layer_type   = ((int*)(bytes_in))[1];
        int input_dim    = ((int*)(bytes_in))[2];
        int output_dim   = ((int*)(bytes_in))[3];
        int _layer_idx   = netlib_model.num_layers;
        DEBUG_PRINT("New layer\n\t- index %d\n\t- type %d", 
            _layer_idx, netlib_layer_type);
        DEBUG_PRINT("\n\t- input dim %d", input_dim);
        DEBUG_PRINT("\n\t- output dim %d\n\t- # parameters %d\n", 
            output_dim, n_parameters);
        netlib_layer_t** _layer = &netlib_model.layers[_layer_idx];
        *_layer = malloc(sizeof(netlib_layer_t) + sizeof(k2c_float)*n_parameters);
        (*_layer)->n_parameters = n_parameters;
        (*_layer)->netlib_layer_type = netlib_layer_type;
        (*_layer)->input = (k2c_tensor) { 
          malloc(sizeof(k2c_float) * input_dim),
          2, input_dim, {1,input_dim,1,1,1} };
        (*_layer)->output = (k2c_tensor) { 
          malloc(sizeof(k2c_float) * output_dim),
          2, output_dim, {1,output_dim,1,1,1} };
        (*_layer)->_write_pos = 0;
        n_bytes -= 4*sizeof(int);
        bytes_in += 4*sizeof(int);
        netlib_model.num_layers++;
    }

    // Additional Payload
    layer_idx  = netlib_model.num_layers-1;
    netlib_layer_t* layer = netlib_model.layers[layer_idx]; 
    for (int byte_idx=0; byte_idx < n_bytes; byte_idx++) {
        int write_pos = layer->_write_pos;
        layer->parameters[write_pos] = bytes_in[byte_idx];
        layer->_write_pos++;
        if (layer->_write_pos % (100*sizeof(k2c_float)) == sizeof(k2c_float)) {
            k2c_float param_val = *((k2c_float*)&layer->parameters[layer->_write_pos-sizeof(k2c_float)]);
            DEBUG_PRINT("\t- Receive parameter %d: %f\n",
                write_pos / sizeof(k2c_float), (double)param_val);
        }
        if (layer->_write_pos == layer->n_parameters*sizeof(k2c_float)) {
            // Mark layer as finished
            layer->_write_pos = -1;
            // New layer must begin in new data frame
            int ret_val = byte_idx == n_bytes - 1 ? 0 : -1;
            DEBUG_PRINT("\t- Layer received success %d\n", ret_val);
            return ret_val;
        }
    }
    return 0;
}

/** End of: Neural Network Code ***********************/
/******************************************************/

typedef enum {
    SENSOR = 0,
    STATE  = 1
} NEURALCONTROL_OBS_LEVEL;

static const char* neuralcontrol_obs_level_names[] = {
    "SENSOR",
    "STATE"
};

typedef enum {
    X_POS =  0, Y_POS =  1, Z_POS =  2,             // lin. position
    Q_A   =  3, Q_B   =  4, Q_C   =  5, Q_D   =  6, // quat. orientation 
    X_VEL =  7, Y_VEL =  8, Z_VEL =  9,             // lin. velocity
    X_ACC = 10, Y_ACC = 11, Z_ACC = 12,             // lin. acceleration
    R_DOT = 13, P_DOT = 14, Y_DOT = 15,             // angular velocity
    E_X   = 16, E_Y   = 17, E_Z   = 18,             // error to reference
    M1    = 19, M2    = 20, M3    = 21, M4    = 22, // last action
} NEURALCONTOL_OBS_VALUE;

static const char* neuralcontrol_obs_value_names[] = {
    "X_POS", "Y_POS", "Z_POS",
    "Q_A"  , "Q_B"  , "Q_C"  , "Q_D",
    "X_VEL", "Y_VEL", "Z_VEL",
    "X_ACC", "Y_ACC", "Z_ACC",
    "R_DOT", "P_DOT", "Y_DOT",
    "E_X"  , "E_Y"  , "E_Z"  ,
    "M1"  ,  "M2"   , "M3"   , "M4"
};

static int neuralcontrol_obs_mapping[23] = { 0 };

/* Use this Macro to select observations.
 * e.g.  NEURALCONTROL_SET_MAPPING(X_POS,Y_POS,Z_POS) 
 */
#define NEURALCONTROL_SET_MAPPING(...) { \
    memset(neuralcontrol_obs_mapping, 0, sizeof(neuralcontrol_obs_mapping)); \
    for(int i=0; i<sizeof((int[]){__VA_ARGS__})/sizeof(int); i++) \
      neuralcontrol_obs_mapping[((int[]){__VA_ARGS__})[i]] = 1; }

static NEURALCONTROL_OBS_LEVEL neuralcontrol_obs_level = SENSOR;
static int neuralcontrol_net_H = 0;
static k2c_float neuralcontrol_net_input[200] = { 0 };

static void neuralControlConfigure(int history_size, NEURALCONTROL_OBS_LEVEL obs_level) {
    DEBUG_PRINT("Configure H=%d, level=%s\n", 
      history_size, neuralcontrol_obs_level_names[obs_level]);
    neuralcontrol_net_H = history_size;
    neuralcontrol_obs_level = obs_level;
    switch (obs_level) {
        case SENSOR:
        NEURALCONTROL_SET_MAPPING( 
            Z_POS,                  
            X_ACC, Y_ACC, Z_ACC,    
            R_DOT, P_DOT, Y_DOT,    
            E_X  , E_Y  , E_Z  ,    
            M1   , M2   , M3   , M4 
        );
        break;

        case STATE:
        NEURALCONTROL_SET_MAPPING( 
            X_POS, Y_POS, Z_POS, 
            Q_A  , Q_B  , Q_C  , Q_D  ,                
            X_VEL, Y_VEL, Z_VEL,    
            R_DOT, P_DOT, Y_DOT,    
            E_X  , E_Y  , E_Z  ,    
            M1   , M2   , M3   , M4 
        );
        break;
    }

    int count_dim = 0;
    DEBUG_PRINT("Observation entries:");
    for (int i=0; 
      i<sizeof(neuralcontrol_obs_value_names)/sizeof(neuralcontrol_obs_value_names[0]); 
      i++){
        if (neuralcontrol_obs_mapping[i]) {
            DEBUG_PRINT("\n\t- %s", neuralcontrol_obs_value_names[i]);
            count_dim++;
        }
        if (i%5 == 0) vTaskDelay(1);
    }
    int total_dim = count_dim*history_size;
    DEBUG_PRINT("\nRequired input size: %d\n", total_dim);
}

static void neuralControlUpdateNetInput(neural_drone_state_t droneState, uint32_t netStartTicks) {
    // No model -> nothing to do TODO
    if (netlib_model.num_layers == 0)
        return; 

    // Calculate input dimension
    int inp_dim = netlib_model.layers[0]->input.numel / neuralcontrol_net_H;
    uint16_t offset = (neuralcontrol_net_H - 1) * inp_dim;

    // Shift by one time step
    memmove(&neuralcontrol_net_input[0], 
      &neuralcontrol_net_input[inp_dim], 
      sizeof(neuralcontrol_net_input) - inp_dim*sizeof(float16_t));

    // Error to reference
    float netTime = (float)(xTaskGetTickCount() - netStartTicks) / (float)M2T(1000);
    float alpha = 2.0f * (float)M_PI / 3.0f;
    k2c_float err_x = 0.25f * (float)(1.0 - cos(netTime * alpha)) - droneState.x;
    k2c_float err_y = 0.25f * (float)sin(netTime * alpha) - droneState.y;
    k2c_float err_z = 1.0f - droneState.z;

    // Add values to input
    k2c_float obs_values[] = {
        droneState.x           , droneState.y           , droneState.z                                    ,
        droneState.quaternion_a, droneState.quaternion_b, droneState.quaternion_c, droneState.quaternion_d,
        droneState.x_dot       , droneState.y_dot       , droneState.z_dot                                ,
        droneState.acc_x       , droneState.acc_y       , droneState.acc_z                                ,
        droneState.roll_dot    , droneState.pitch_dot   , droneState.yaw_dot                              ,
        err_x                  , err_y                  , err_z                                           ,
        droneState.M1          , droneState.M2          , droneState.M3          , droneState.M4
    };
    int counter = 0;
    for (int i=0; i<sizeof(obs_values)/sizeof(obs_values[0]); i++) {
        if (neuralcontrol_obs_mapping[i]) {
            neuralcontrol_net_input[counter + offset] = obs_values[i];
            counter++;
        }
    }
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
    neuralcontrol_compute_nops = /* Increase execution time less agressively */
        ceilf((float)(neuralcontrol_compute_high - neuralcontrol_compute_low) / 8.f) + neuralcontrol_compute_low;
}

static void neuralControlComputeFaster() {
    neuralcontrol_compute_high = neuralcontrol_compute_nops;
    neuralcontrol_compute_nops = 
        floorf((float)(neuralcontrol_compute_high - neuralcontrol_compute_low) / 2.f) + neuralcontrol_compute_low;
}

static void neuralControlComputeNetOutput(float16_t* buffer) {
    netlib_inferene(neuralcontrol_net_input, buffer);
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
                if (netlib_write_layer(&packet.data[1], packet.size-1)) {
                    DEBUG_PRINT("Error in transfer\n");
                }
                break;
            }

            case NEURALCONTROL_CMD_CONFIG:
            {
                cmd_buffer->cmd_id = NEURALCONTROL_CMD_CONFIG;
                int new_history_size = packet.data[1];
                int new_obs_level    = packet.data[2];
                neuralControlConfigure(new_history_size, new_obs_level);
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

    netlib_init_model();

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
                if (NEURALCONTROL_CMD_ADJUST == control_cmd.cmd_id)
                    neuralcontrol_state_next = NEURALCONTROL_ADJUSTING;
                if (NEURALCONTROL_CMD_HOVER == control_cmd.cmd_id)
                    neuralcontrol_state_next = NEURALCONTROL_TAKE_OFF;
                if (NEURALCONTROL_CMD_TRANSF == control_cmd.cmd_id)
                    neuralcontrol_state_next = NEURALCONTROL_TRANSFERING;
                break;

            case NEURALCONTROL_TAKE_OFF:
                netStartTicks = xTaskGetTickCount();
                new_setpoint = hoverSetpoint;
                commanderSetSetpoint(&new_setpoint, 5);
                if (droneState.z > 0.95 && droneState.z < 1.05
                    && droneState.z_dot > -0.05 && droneState.z_dot < 0.05) 
                    neuralcontrol_state_next = NEURALCONTROL_HOVERING;
                break;

            case NEURALCONTROL_HOVERING:
                netStartTicks = xTaskGetTickCount();
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
            if (netlib_model.num_layers > 0 && it_count % 100 == 0) {
                if (it_count % 500 == 0){
                    DEBUG_PRINT("Reset recurrent memory\n");
                    netlib_reset_states();
                } 
                memset(neuralcontrol_net_input, 0, sizeof(neuralcontrol_net_input));
                DEBUG_PRINT("input = [");
                for (int i=0; i<netlib_model.layers[0]->input.numel; i++) {
                    DEBUG_PRINT("%f,", (double)neuralcontrol_net_input[i]);
                    if (i%5 == 0) {
                        vTaskDelay(1);
                        DEBUG_PRINT("\n");
                    }
                }
                DEBUG_PRINT("]\n");
                neuralControlComputeNetOutput(lastAction);
                DEBUG_PRINT("output = [");
                for (int i=0; i<sizeof(lastAction)/sizeof(lastAction[0]); i++) {
                    DEBUG_PRINT("%f,",(double)lastAction[i]);
                }
                DEBUG_PRINT("]\n");
            } 
        }
        if (NEURALCONTROL_ON_GROUND == neuralcontrol_state &&
            NEURALCONTROL_EXECUTING == neuralcontrol_state_next) {
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
            // Recurrent memory to zero state
            netlib_reset_states();
            // Compute PWM commands            
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
            TickType_t beforeInference = xTaskGetTickCount();
            uint16_t infer_ctr;
            float16_t dummy[4];
            for (infer_ctr=0; infer_ctr < 10; infer_ctr++) {
                neuralControlComputeNetOutput(dummy);
                if (((float)xTaskGetTickCount() - beforeInference) / (float)M2T(1) > 5.f)
                    break; // Avoid getting killed by watchdog
            }
            if (0 == infer_ctr) infer_ctr = 1;
            inferenceTime = ((float)xTaskGetTickCount() - beforeInference) / (float)M2T(1000) / (float)(infer_ctr);
            DEBUG_PRINT("Avg. Inference Time: %.3f msec.\n", 
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