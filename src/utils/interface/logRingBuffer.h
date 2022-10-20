#ifndef _LOGRINGBUFFER_H_
#define _LOGRINGBUFFER_H_

#include <stdlib.h>
#include <stdint.h>

typedef _Float16 float16_t;

int logRB_in(float16_t * input);
int logRB_out(float16_t * output, uint32_t waitTime);

#endif