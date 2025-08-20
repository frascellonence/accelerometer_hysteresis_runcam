#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#include "runcam_uart.h"
#include "runcam_crc.h"

// RunCam Device Protocol Header
#define RC_HEADER 0xCC

// Commands
#define RC_CMD_GET_DEVICE_INFO     0x00
#define RC_CMD_CAMERA_CONTROL      0x01
#define RC_CMD_5KEY_CONNECTION     0x04
// Some firmwares use 0x10 for settings/info; keep 0x00 primary
#define RC_CMD_GET_SETTINGS        0x10

// Some firmware variants use 0x01 to toggle start/stop
#define RC_ACTION_STARTSTOP      0x01

static runcam_uart_config_t g_cfg;

static void rc_send_bytes(const uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uart_putc_raw(g_cfg.uart_id, buf[i]);
    }
}

static void rc_send_packet(const uint8_t *payload, size_t len) {
    uint8_t crc = runcam_crc8_dvb_s2(payload, len);
    rc_send_bytes(payload, len);
    uart_putc_raw(g_cfg.uart_id, crc);
}

static void rc_send_control(uint8_t action) {
    uint8_t pkt[3] = { RC_HEADER, RC_CMD_CAMERA_CONTROL, action };
    rc_send_packet(pkt, 3);
}

static void rc_send_5key(uint8_t action) {
    uint8_t pkt[3] = { RC_HEADER, RC_CMD_5KEY_CONNECTION, action };
    rc_send_packet(pkt, 3);
}

void runcam_init(const runcam_uart_config_t *config) {
    g_cfg = *config;
    uart_init(g_cfg.uart_id, g_cfg.baudrate);
    uart_set_format(g_cfg.uart_id, 8, 1, UART_PARITY_NONE);
    gpio_set_function(g_cfg.tx_gpio, GPIO_FUNC_UART);
    gpio_set_function(g_cfg.rx_gpio, GPIO_FUNC_UART);
}

void runcam_start_record(void) {
    rc_send_control(RC_ACTION_STARTSTOP);
}

void runcam_stop_record(void) {
    rc_send_control(RC_ACTION_STARTSTOP);
}

void runcam_open_5key(void) {
    // 1=open, 2=close per protocol variants
    rc_send_5key(0x01);
    // brief delay
    sleep_ms(50);
}


