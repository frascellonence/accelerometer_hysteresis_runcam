#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#include "runcam_uart.h"
#include "motion_detect.h"
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

int main() {
    stdio_init_all();
    // Wait up to ~5s for USB CDC to enumerate so logs appear
    sleep_ms(3000);
    for (int i = 0; i < 50 && !stdio_usb_connected(); i++) {
        sleep_ms(100);
    }

    // Init peripherals
    runcam_uart_config_t cam_cfg = {
        .uart_id = uart0,
        .baudrate = 115200,
        .tx_gpio = 0,
        .rx_gpio = 1,
    };
    runcam_init(&cam_cfg);
    // Some cameras require 5-key open to accept control commands; wait 5s after boot per guidance
    sleep_ms(5000);
    runcam_open_5key();
    if (!motion_init()) {
        printf("ERROR: motion_init failed. Check I2C wiring and sensor power.\n");
    } else {
        printf("INFO: motion_init OK.\n");
    }

    // Init LED (supports both Pico and Pico W/2 W)
    #if defined(PICO_DEFAULT_LED_PIN)
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    #endif
    #if defined(CYW43_WL_GPIO_LED_PIN)
    if (cyw43_arch_init() == PICO_OK) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    }
    #endif

    bool recording = false;
    const char *hyst_state = "IDLE";
    uint32_t window_start_ms = to_ms_since_boot(get_absolute_time());
    bool saw_run_in_window = false;
    uint8_t run_window_hist = 0; // bit0 = latest
    uint8_t no_run_secs = 0;     // count of consecutive 1s windows with no RUN
    uint32_t state_entry_ms = to_ms_since_boot(get_absolute_time());
    activity_state_t last_activity = ACTIVITY_IDLE;

    printf("Hysteresis recorder running. Policy: start after 2 windows with RUN, stop after >3s IDLE.\n");

    while (true) {
        activity_state_t s = motion_classify_tick();
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());

        if (s != last_activity) {
            last_activity = s;
            state_entry_ms = now_ms;
        }
        if (s == ACTIVITY_RUNNING) {
            saw_run_in_window = true;
        }

        if (now_ms - window_start_ms >= 1000) {
            run_window_hist = ((run_window_hist << 1) | (saw_run_in_window ? 1 : 0)) & 0x03;
            // Update 3s no-run window counter
            if (!saw_run_in_window) {
                if (no_run_secs < 255) no_run_secs++;
            } else {
                no_run_secs = 0;
            }
            saw_run_in_window = false;
            window_start_ms = now_ms;
            if (!recording && (run_window_hist & 0x03) == 0x03) {
                runcam_start_record();
                recording = true;
                run_window_hist = 0; // reset to avoid immediate retrigger
                printf("START RECORD\n");
                hyst_state = "REC";
            }
        }

        if (recording && no_run_secs >= 3) {
            runcam_stop_record();
            recording = false;
            no_run_secs = 0;
            printf("STOP RECORD\n");
            hyst_state = "IDLE";
        }

        // LED indicates RUN detection
        bool led_on = (s == ACTIVITY_RUNNING);
        #if defined(PICO_DEFAULT_LED_PIN)
        gpio_put(PICO_DEFAULT_LED_PIN, led_on);
        #endif
        #if defined(CYW43_WL_GPIO_LED_PIN)
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
        #endif

        // Periodic debug output
        static uint32_t last_log_ms = 0;
        if (now_ms - last_log_ms >= 500) {
            last_log_ms = now_ms;
            motion_debug_t md = {0};
            motion_get_debug(&md);
            uint32_t state_ms = now_ms - state_entry_ms;
            printf("ACC x=%.3f y=%.3f z=%.3f | act=%s t=%lums | freq=%.2fHz amp=%.3f peaks=%d thr=%.3f | hyst=%s hist=0b%02u rec=%d no_run_s=%u\n",
                   md.last_x, md.last_y, md.last_z,
                   (s==ACTIVITY_RUNNING?"RUN":s==ACTIVITY_WALKING?"WALK":"IDLE"), (unsigned long)state_ms,
                   md.last_frequency_hz, md.last_amp_pp, md.last_peak_count, md.last_peak_threshold,
                   hyst_state, (unsigned)(run_window_hist & 0x03), recording?1:0, (unsigned)no_run_secs);
        }

        sleep_ms(20);
    }
}


