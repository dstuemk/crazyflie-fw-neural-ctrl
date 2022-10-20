#include "FreeRTOS.h"
#include "queue.h"
#include "remote_control.h"
#include "debug.h"
#include "config.h"
#include "static_mem.h"
#include "task.h"
#include "crtp.h"
#include "commander.h"
#include "rateSupervisor.h"
#include "motors.h"
#include <math.h>
#include <string.h>

static const float deg2rad = 2 * M_PI / 360;

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
    //float16_t gyro_x;   // Added by Sven (Raw Gyro sensor values)
    //float16_t gyro_y;   // Added by Sven (Raw Gyro sensor values)
    //float16_t gyro_z;   // Added by Sven (Raw Gyro sensor values)
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
    float time; // 32 Bits needed
    float dtimeMs;
} remote_drone_state_t;

typedef enum {
    REMOTECONTROL_CMD_HOVER = 0x00,
    REMOTECONTROL_CMD_LAND  = 0x01,
    REMOTECONTROL_CMD_PWM   = 0x02,
    REMOTECONTROL_CMD_START = 0x03,
    REMOTECONTROL_CMD_NONE  = 0x04,
} REMOTECONTROL_CMD_ID;

typedef struct {
    REMOTECONTROL_CMD_ID cmd_id;
    uint8_t              cmd_data[128];
} remote_control_cmd_t;

static const setpoint_t landSetpoint = {.mode.z = modeAbs};
static const setpoint_t hoverSetpoint = {
     .position = {.x = 0.0, .y = 0.0, .z = 1.0},
     .mode = {.x = modeAbs, .y = modeAbs, .z = modeAbs}};
static const setpoint_t resetPIDsetpoint = {
        .thrust = 0.0, 
        .mode = {.pitch = modeAbs, .roll = modeAbs, .yaw = modeAbs}};

typedef enum {
    REMOTECONTROL_ON_GROUND = 0x00,
    REMOTECONTROL_TAKE_OFF  = 0x01,
    REMOTECONTROL_HOVERING  = 0x02,
    REMOTECONTROL_EXECUTING = 0x03,
    REMOTECONTROL_LANDING   = 0x04,
    REMOTECONTROL_INIT      = 0x05,
} REMOTECONTROL_STATE;

static const char *remotecontrol_state_names[] = {
    "ON_GROUND",
    "TAKE_OFF",
    "HOVERING",
    "EXECUTING",
    "LANDING",
    "INIT"
};

static void remoteControlTask(void *);
STATIC_MEM_TASK_ALLOC(remoteControlTask, REMOTECONTROL_TASK_STACKSIZE);

static xQueueHandle kalmanStateQueue;
STATIC_MEM_QUEUE_ALLOC(kalmanStateQueue, 1, sizeof(state_t));

static xQueueHandle pwmBypassQueue;
STATIC_MEM_QUEUE_ALLOC(pwmBypassQueue, 1, sizeof(int));

static rateSupervisor_t remoteRateSupervisor;

static void remoteCreateDroneState(state_t *, remote_drone_state_t *, float);

static bool remoteSafetyCheck(remote_drone_state_t);

void remoteControlTaskInit() {
    kalmanStateQueue = STATIC_MEM_QUEUE_CREATE(kalmanStateQueue);
    pwmBypassQueue = STATIC_MEM_QUEUE_CREATE(pwmBypassQueue);
    STATIC_MEM_TASK_CREATE(remoteControlTask, remoteControlTask, REMOTECONTROL_TASK_NAME, NULL, REMOTECONTROL_TASK_PRI);
}

bool remoteControlTaskTest() {
    return true;
}

static void remoteSendRemoteControlState(REMOTECONTROL_STATE remote_control_state) {
    CRTPPacket packet;
    packet.header = CRTP_HEADER(CRTP_PORT_REMOTECONTROL, 0);
    packet.size = 2;
    packet.data[0] = 0xFF;
    packet.data[1] = remote_control_state;
    crtpSendPacket(&packet);
}

static void remoteSendDroneState(remote_drone_state_t drone_state) {
    CRTPPacket packet;

    const int16_t bytes_per_packet = 29;

    for(uint16_t packet_idx=0; 
      packet_idx < ceilf(((float)sizeof(drone_state)) / bytes_per_packet); 
      packet_idx++) {
        packet.header = CRTP_HEADER(CRTP_PORT_REMOTECONTROL, 0);
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

static void remoteReceiveCommand(remote_control_cmd_t *cmd_buffer) {
    cmd_buffer->cmd_id = REMOTECONTROL_CMD_NONE;

    CRTPPacket packet;
    
    if(crtpReceivePacketWait(CRTP_PORT_REMOTECONTROL, &packet, 1) == pdTRUE) {
        switch(packet.data[0]) {
            case REMOTECONTROL_CMD_START:
                cmd_buffer->cmd_id = REMOTECONTROL_CMD_START;
                break;

            case REMOTECONTROL_CMD_HOVER:
                cmd_buffer->cmd_id = REMOTECONTROL_CMD_HOVER;
                break;
            
            case REMOTECONTROL_CMD_LAND:
                cmd_buffer->cmd_id = REMOTECONTROL_CMD_LAND;
                break;

            case REMOTECONTROL_CMD_PWM:
            {
                // Sanity check on first packet
                bool valid = true;
                float sanity_value = *((float *)&packet.data[1]);
                if (sanity_value != 123.0f) {
                    static uint16_t print_ctr = 0;
                    if (print_ctr++ == 0) DEBUG_PRINT("Receiving invalid PWM signal, sanity value: 123 != %f\n", 
                        (double)sanity_value);
                    print_ctr = print_ctr % 100;
                    valid = false;
                }
                // Receive consecutive packet(s)
                float *pwm_and_t = (float *)cmd_buffer->cmd_data;
                for (uint16_t pwm_idx=0; pwm_idx < 5; pwm_idx++) {
                    pwm_and_t[pwm_idx] = *((float*)(packet.data + 5 + 4*pwm_idx));
                }
                static uint16_t print_pwm_ctr = 0;
                if (print_pwm_ctr++ == 0)
                    DEBUG_PRINT("Received PWM: [%f, %f, %f, %f]   T0: %f\n", 
                        (double)pwm_and_t[0], (double)pwm_and_t[1], 
                        (double)pwm_and_t[2], (double)pwm_and_t[3], (double)pwm_and_t[4]);
                print_pwm_ctr = print_pwm_ctr % 100;
                if(valid) cmd_buffer->cmd_id = REMOTECONTROL_CMD_PWM;
                break;
            }
        }
    }
}

static void remoteSetMotors(float m0, float m1, float m2, float m3) {
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

static void remoteControlTask(void *params) {
    DEBUG_PRINT("Remote-Control task main function is running\n");

    crtpInitTaskQueue(CRTP_PORT_REMOTECONTROL);

    rateSupervisorInit(&remoteRateSupervisor, 
        xTaskGetTickCount(), 
        M2T(1000), 
        REMOTE_CONTROL_RATE-REMOTE_CONTROL_RATE/10, 
        REMOTE_CONTROL_RATE+REMOTE_CONTROL_RATE/10, 1);
    TickType_t lastTime = xTaskGetTickCount();

    REMOTECONTROL_STATE remotecontrol_state = REMOTECONTROL_INIT;

    float reactionTime = -1.f;

    remoteSendRemoteControlState(remotecontrol_state);

    while(1) {
        int pwmBypass = 0; 

        state_t kalmanState;
        xQueuePeek(kalmanStateQueue, &kalmanState, 0);
        
        remote_drone_state_t droneState;
        remoteCreateDroneState(&kalmanState, &droneState, reactionTime);
                
        remote_control_cmd_t control_cmd = { .cmd_id = REMOTECONTROL_CMD_NONE };
        remoteReceiveCommand(&control_cmd);

        REMOTECONTROL_STATE remotecontrol_state_next = remotecontrol_state;

        setpoint_t new_setpoint;

        // State actions
        switch(remotecontrol_state) {
            case REMOTECONTROL_INIT:
                if (REMOTECONTROL_CMD_START == control_cmd.cmd_id)
                    remotecontrol_state_next = REMOTECONTROL_ON_GROUND;
                break;

            case REMOTECONTROL_ON_GROUND:
                remoteSendDroneState(droneState);
                if (REMOTECONTROL_CMD_HOVER == control_cmd.cmd_id)
                    remotecontrol_state_next = REMOTECONTROL_TAKE_OFF;
                else if (REMOTECONTROL_CMD_PWM == control_cmd.cmd_id)
                    remotecontrol_state_next = REMOTECONTROL_EXECUTING;
                break;

            case REMOTECONTROL_TAKE_OFF:
                remoteSendDroneState(droneState);
                new_setpoint = hoverSetpoint;
                commanderSetSetpoint(&new_setpoint, 5);
                if (droneState.z > 0.8) 
                    remotecontrol_state_next = REMOTECONTROL_HOVERING;
                break;

            case REMOTECONTROL_HOVERING:
                remoteSendDroneState(droneState);
                new_setpoint = hoverSetpoint;
                commanderSetSetpoint(&new_setpoint, 5);
                if (REMOTECONTROL_CMD_LAND == control_cmd.cmd_id)
                    remotecontrol_state_next = REMOTECONTROL_LANDING;
                if (REMOTECONTROL_CMD_PWM == control_cmd.cmd_id)
                    remotecontrol_state_next = REMOTECONTROL_EXECUTING;
                break;

            case REMOTECONTROL_LANDING:
                remoteSendDroneState(droneState);
                new_setpoint = landSetpoint;
                commanderSetSetpoint(&new_setpoint, 5);
                if (droneState.z < 0.2) 
                    remotecontrol_state_next = REMOTECONTROL_ON_GROUND;
            break;

            case REMOTECONTROL_EXECUTING:
                remoteSendDroneState(droneState);
                if (!remoteSafetyCheck(droneState)) {
                    DEBUG_PRINT("Remote control lead to instable behaviour\n");
                    remotecontrol_state_next = REMOTECONTROL_LANDING;
                }
                else if (REMOTECONTROL_CMD_HOVER == control_cmd.cmd_id)
                    remotecontrol_state_next = REMOTECONTROL_HOVERING;
                else if (REMOTECONTROL_CMD_LAND == control_cmd.cmd_id)
                    remotecontrol_state_next = REMOTECONTROL_LANDING;
                //else if (REMOTECONTROL_CMD_PWM != control_cmd.cmd_id) {
                //    DEBUG_PRINT("No PWM control command was received\n");
                //    remotecontrol_state_next = REMOTECONTROL_LANDING;
                //}
                break;
        }

        // Transition actions
        if (remotecontrol_state != remotecontrol_state_next) {
            DEBUG_PRINT("Remote-State transition %s -> %s\n", 
                remotecontrol_state_names[remotecontrol_state],
                remotecontrol_state_names[remotecontrol_state_next]);
            remoteSendRemoteControlState(remotecontrol_state_next);
        }
        if (REMOTECONTROL_ON_GROUND == remotecontrol_state &&
            REMOTECONTROL_EXECUTING == remotecontrol_state_next) {
            // Set motors to received PWM signal
            if (REMOTECONTROL_CMD_PWM == control_cmd.cmd_id) {
                float *pwm_and_t = (float*)control_cmd.cmd_data;
                remoteSetMotors(pwm_and_t[0],pwm_and_t[1],pwm_and_t[2],pwm_and_t[3]);
                reactionTime = ((float)xTaskGetTickCount()) / M2T(1000) - pwm_and_t[4];
            }
            // Enable PWM bypass
            pwmBypass = 1;
            xQueueOverwrite(pwmBypassQueue, &pwmBypass);
        }
        if (REMOTECONTROL_HOVERING == remotecontrol_state &&
            REMOTECONTROL_EXECUTING == remotecontrol_state_next) {
            // Set motors to received PWM signal
            if (REMOTECONTROL_CMD_PWM == control_cmd.cmd_id) {
                float *pwm_and_t = (float*)control_cmd.cmd_data;
                remoteSetMotors(pwm_and_t[0],pwm_and_t[1],pwm_and_t[2],pwm_and_t[3]);
                reactionTime = ((float)xTaskGetTickCount()) / M2T(1000) - pwm_and_t[4];
            }
            // Enable PWM bypass
            pwmBypass = 1;
            xQueueOverwrite(pwmBypassQueue, &pwmBypass);
        }
        if (REMOTECONTROL_EXECUTING == remotecontrol_state &&
            REMOTECONTROL_EXECUTING == remotecontrol_state_next) {
            // Set motors to received PWM signal
            if (REMOTECONTROL_CMD_PWM == control_cmd.cmd_id) {
                float *pwm_and_t = (float*)control_cmd.cmd_data;
                remoteSetMotors(pwm_and_t[0],pwm_and_t[1],pwm_and_t[2],pwm_and_t[3]);
                reactionTime = ((float)xTaskGetTickCount()) / M2T(1000) - pwm_and_t[4];
                static uint16_t exec_ctr = 0;
                if (exec_ctr++ == 0) DEBUG_PRINT("Reaction Time: %f sec.\n", (double) reactionTime);
                exec_ctr = exec_ctr % 100;
            }
        }
        if (REMOTECONTROL_EXECUTING == remotecontrol_state &&
            REMOTECONTROL_LANDING == remotecontrol_state_next) {
            // Disable PWM bypass
            pwmBypass = 0;
            xQueueOverwrite(pwmBypassQueue, &pwmBypass);
        }
        if (REMOTECONTROL_EXECUTING == remotecontrol_state &&
            REMOTECONTROL_HOVERING == remotecontrol_state_next) {
            // Disable PWM bypass
            pwmBypass = 0;
            xQueueOverwrite(pwmBypassQueue, &pwmBypass);
        }
        remotecontrol_state = remotecontrol_state_next;

        // Wait remaining time
        vTaskDelayUntil(&lastTime, M2T(1000) / REMOTE_CONTROL_RATE); 

        // Rate supervision
        if (!rateSupervisorValidate(&remoteRateSupervisor, xTaskGetTickCount())) {
            DEBUG_PRINT("WARNING: remote loop rate is off (%lu Hz)\n", 
                rateSupervisorLatestCount(&remoteRateSupervisor));
        }
    }
}

static bool remoteSafetyCheck(remote_drone_state_t droneState) {
    if (droneState.x < -2.5 || droneState.x > 2.5 ||
       droneState.y < -2.5 || droneState.y > 2.5 ||
       droneState.roll < (-30 * deg2rad) || droneState.roll > (30 * deg2rad) ||
       droneState.pitch < (-30 * deg2rad) || droneState.pitch > (30 * deg2rad) ||
       droneState.roll_dot < (-800 * deg2rad) || droneState.roll_dot > (800 * deg2rad) ||
       droneState.pitch_dot < (-800 * deg2rad) || droneState.pitch_dot > (800 * deg2rad))
       return false;
    return true;
}

static void ToQuaternion(float yaw, float pitch, float roll, float *w, float *x, float *y, float *z) {
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

static void remoteCreateDroneState(state_t *kalmanState, remote_drone_state_t *droneState, float reactionTime) {
  // current time:
  droneState->time = ((float)xTaskGetTickCount()) / M2T(1000);
  droneState->dtimeMs = reactionTime*1000.f;

  // xyz and xyz_dot:
  droneState->x = (float16_t)kalmanState->position.x;
  droneState->y = (float16_t)kalmanState->position.y;
  droneState->z = (float16_t)kalmanState->position.z;
  droneState->x_dot = (float16_t)kalmanState->velocity.x;
  droneState->y_dot = (float16_t)kalmanState->velocity.y;
  droneState->z_dot = (float16_t)kalmanState->velocity.z;

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

  //droneState->gyro_x = (float16_t)(kalmanState->attitude_rate.x * deg2rad);
  //droneState->gyro_y = (float16_t)(-kalmanState->attitude_rate.y * deg2rad);
  //droneState->gyro_z = (float16_t)(kalmanState->attitude_rate.z * deg2rad);

  // quaternion abcd:
  float w, x, y, z;
  ToQuaternion((float)droneState->yaw, (float)droneState->pitch, (float)droneState->roll, &w, &x, &y, &z);
  droneState->quaternion_a = (float16_t)x;
  droneState->quaternion_b = (float16_t)y;
  droneState->quaternion_c = (float16_t)z;
  droneState->quaternion_d = (float16_t)w;

}

void remoteControlTaskEnqueueState(state_t state) {
    xQueueOverwrite(kalmanStateQueue, &state);
}

void remoteControlTaskPeekPwmBypass(bool *pwm_bypass) {
    int pwmBypass_int;
    if (pdPASS == xQueuePeek(pwmBypassQueue, &pwmBypass_int, 0))
        *pwm_bypass = pwmBypass_int != 0;
}