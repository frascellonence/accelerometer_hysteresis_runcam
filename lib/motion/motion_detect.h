#ifndef MOTION_DETECT_H
#define MOTION_DETECT_H

#include <stdbool.h>

typedef enum {
    ACTIVITY_IDLE = 0,
    ACTIVITY_WALKING,
    ACTIVITY_RUNNING
} activity_state_t;

bool motion_init(void);
void motion_set_sensitivity(float sensitivity);
activity_state_t motion_classify_tick(void);

typedef struct {
    float last_x;
    float last_y;
    float last_z;
    float last_frequency_hz;
    float last_amp_pp;
    int last_peak_count;
    float last_peak_threshold;
} motion_debug_t;

void motion_get_last_sample(float *x, float *y, float *z);
void motion_get_debug(motion_debug_t *out);

#endif


