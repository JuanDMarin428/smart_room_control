/**
 * @file comms.c
 * @brief Minimal UART communications layer (implementation).
 */

#include "comms.h"
#include "uart_parser.h"
#include <string.h>
#include <stdio.h>

/* -------------------------------------------------------------------------- */
/* Internal helpers                                                           */
/* -------------------------------------------------------------------------- */

static inline size_t advance(size_t idx) {
    return (idx + 1u) % COMMS_RX_BUF_SIZE;
}

static inline bool ring_empty(const comms_t *c) {
    return c->head == c->tail;
}

static inline bool ring_full(const comms_t *c) {
    return advance(c->head) == c->tail;
}

static inline void ring_push_isr(comms_t *c, uint8_t ch) {
    size_t next = advance(c->head);
    if (next == c->tail) {
        /* Overflow: drop oldest byte */
        c->tail = advance(c->tail);
#if COMMS_DEBUG
        c->dbg_overflow++;
#endif
    }
    c->rx_ring[c->head] = ch;
    c->head = next;
}

static inline bool ring_pop(comms_t *c, uint8_t *out) {
    if (ring_empty(c)) return false;
    *out = c->rx_ring[c->tail];
    c->tail = advance(c->tail);
    return true;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void comms_init(comms_t *c, UART_HandleTypeDef *huart, meas_cb_t cb) {
    memset((void*)c, 0, sizeof(*c));
    c->huart = huart;
    c->on_meas = cb;
    (void)HAL_UART_Receive_IT(c->huart, (uint8_t*)&c->rx_byte, 1);
}

void comms_uart_rx_callback(comms_t *c, UART_HandleTypeDef *huart) {
    if (huart != c->huart) return;
    ring_push_isr(c, (uint8_t)c->rx_byte);
    (void)HAL_UART_Receive_IT(c->huart, (uint8_t*)&c->rx_byte, 1);
}

/**
 * @brief Assemble frames '<' ... '>' and process them.
 *        Robust to garbage; resynchronizes on '<' and resets on overflow.
 */
void comms_poll(comms_t *c) {
    static enum { SEEK_LT, IN_FRAME } state = SEEK_LT;
    static char frame[160];
    static size_t flen = 0;

    /* Fast-forward: drop everything until '<' when seeking start */
    if (state == SEEK_LT) {
        while (!ring_empty(c)) {
            uint8_t peek = c->rx_ring[c->tail];
            if (peek == '<') break;
            c->tail = (c->tail + 1u) % COMMS_RX_BUF_SIZE;
        }
    }

    uint8_t ch;
    while (ring_pop(c, &ch)) {
        if (state == SEEK_LT) {
            if (ch == '<') {
                frame[0] = '<';
                flen = 1;
                state = IN_FRAME;
            }
        } else { /* IN_FRAME */
            if (flen + 1 >= sizeof(frame)) {
                state = SEEK_LT;
                flen = 0;
                continue;
            }
            frame[flen++] = (char)ch;

            if (ch == '>') {
                frame[flen] = 0;
                meas_packet_t m;
                if (uart_parse_meas(frame, &m) && c->on_meas) c->on_meas(&m);
                state = SEEK_LT;
                flen = 0;
            } else if (ch == '<') {
                frame[0] = '<';
                flen = 1;
            }
        }
    }
}


HAL_StatusTypeDef comms_send_ctrl(comms_t *c, const ctrl_packet_t *ctrl) {
    char msg[64];
    float uh = ctrl->uh; if (uh < 0) uh = 0; if (uh > 1) uh = 1;
    float uf = ctrl->uf; if (uf < 0) uf = 0; if (uf > 1) uf = 1;

    int n = snprintf(msg, sizeof(msg), "<CTRL, %.3f, %.3f>\r\n", uh, uf);
    if (n < 0) return HAL_ERROR;

    return HAL_UART_Transmit(c->huart, (uint8_t*)msg, (uint16_t)n, HAL_MAX_DELAY);
}
