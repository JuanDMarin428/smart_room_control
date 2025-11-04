#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "main.h"

#ifndef COMMS_RX_BUF_SIZE
#define COMMS_RX_BUF_SIZE 512u
#endif

typedef struct {
    float T, w, c, N;
    bool  valid;
} meas_packet_t;

typedef struct {
    float uh; // heater [0..1]
    float uf; // fan    [0..1]
} ctrl_packet_t;

typedef void (*meas_callback_t)(const meas_packet_t *m);

typedef struct {
    UART_HandleTypeDef *huart;

    volatile uint8_t rx_byte;

    // ring buffer
    volatile size_t head;   // write index (ISR)
    volatile size_t tail;   // read  index (task)
    char rx_buf[COMMS_RX_BUF_SIZE];

    // app callback
    meas_callback_t on_meas;
} comms_t;

#ifdef __cplusplus
extern "C" {
#endif

void comms_init(comms_t *c, UART_HandleTypeDef *huart, meas_callback_t cb);
void comms_uart_rx_callback(comms_t *c, UART_HandleTypeDef *huart);
void comms_poll(comms_t *c);
bool comms_send_ctrl(comms_t *c, const ctrl_packet_t *ctrl);

#ifdef __cplusplus
}
#endif
