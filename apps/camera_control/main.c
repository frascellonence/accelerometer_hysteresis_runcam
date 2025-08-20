#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "runcam_uart.h"

int main() {
    stdio_init_all();
    sleep_ms(500);

    runcam_uart_config_t cfg = {
        .uart_id = uart0,
        .baudrate = 115200,
        .tx_gpio = 0,
        .rx_gpio = 1,
    };
    runcam_init(&cfg);

    printf("RunCam control demo. Press 1 to start, 2 to stop.\n");
    while (true) {
        int ch = getchar_timeout_us(0);
        if (ch != PICO_ERROR_TIMEOUT) {
            if (ch == '1' || ch == '3') {
                printf("START\n");
                runcam_start_record();
            } else if (ch == '2' || ch == '4') {
                printf("STOP\n");
                runcam_stop_record();
            }
        }
        sleep_ms(10);
    }
}


