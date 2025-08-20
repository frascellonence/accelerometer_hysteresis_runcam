#include <stdio.h>
#include "pico/stdlib.h"
#include "motion_detect.h"

int main() {
    stdio_init_all();
    sleep_ms(2000);
    motion_init();
    printf("Activity detect demo.\n");
    while (true) {
        activity_state_t s = motion_classify_tick();
        if (s == ACTIVITY_RUNNING) {
            printf("RUN\n");
        } else if (s == ACTIVITY_WALKING) {
            printf("WALK\n");
        } else {
            printf("IDLE\n");
        }
        sleep_ms(20);
    }
}


