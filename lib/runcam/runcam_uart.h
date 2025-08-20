#ifndef RUNCAM_UART_H
#define RUNCAM_UART_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uart_inst_t *uart_id;
    uint32_t baudrate;
    uint tx_gpio;
    uint rx_gpio;
} runcam_uart_config_t;

void runcam_init(const runcam_uart_config_t *config);
void runcam_start_record(void);
void runcam_stop_record(void);
void runcam_open_5key(void);

#ifdef __cplusplus
}
#endif

#endif


