#include "logRingBuffer.h"
#include "debug.h"
#include <math.h>
#include "config.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"
#include "task.h"
//#include "neural_control.h"
//#include "neural_com.h"
#include "param.h"
#include "pid.h"
#include "motors.h"
#include "commander.h"
#include "log.h"
#include "rateSupervisor.h"
#include "led.h"
#include "arm_math.h"
#include "filter.h"


#define BUFFER_SIZE 10000 //20000
static float16_t buffer[BUFFER_SIZE];
static uint32_t index_in = 0;
static uint32_t index_out = 0;
static bool different_iteration = false;


// put log into ring buffer
int logRB_in(float16_t * input){
    if(index_in>BUFFER_SIZE){
        index_in = 0;
    }
    if(index_out>BUFFER_SIZE){
        index_out = 0;
    }

    // should not write over what was not read:
    if(index_in==index_out && different_iteration){
        return false;
    }

    buffer[index_in] = *input;
    index_in++;

    if(index_in==BUFFER_SIZE){
        index_in = 0;
        different_iteration = !different_iteration;
    }
    return true;
}

// read log out of ring buffer
int logRB_out(float16_t * output, uint32_t waitTime){
    //DEBUG_PRINT("o(%ld %ld)",index_in,index_out);
    if(index_in>BUFFER_SIZE){
        index_in = 0;
    }
    if(index_out>BUFFER_SIZE){
        index_out = 0;
    }

    // should not read more than written by logRB_in:
    if(index_in==index_out && !different_iteration){
        vTaskDelay(waitTime);
        if(index_in==index_out){
            return false;
        }
    }

    *output = buffer[index_out];
    index_out++;

    if(index_out==BUFFER_SIZE){
        index_out = 0;
        different_iteration = !different_iteration;
    }
    return true;
}