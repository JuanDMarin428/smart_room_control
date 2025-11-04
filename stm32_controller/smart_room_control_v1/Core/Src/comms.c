#include "comms.h"
#include "uart_parser.h"
#include <string.h>
#include <ctype.h>

/* ---------- Private ---------- */

static inline void start_iRx(comms_t *c) {
    (void)HAL_UART_Receive_IT(c->huart, (uint8_t*)&c->rx_byte, 1);
}

static inline bool ring_is_empty(const comms_t *c) {
    return c->head == c->tail;
}

static inline bool ring_is_full(const comms_t *c) {
    size_t next = (c->head + 1u) % COMMS_RX_BUF_SIZE;
    return next == c->tail;
}

static inline void ring_push_isr(comms_t *c, char ch) {
    size_t next = (c->head + 1u) % COMMS_RX_BUF_SIZE;
    if (next == c->tail) {
        // overflow: drop oldest
        c->tail = (c->tail + 1u) % COMMS_RX_BUF_SIZE;
    }
    c->rx_buf[c->head] = ch;
    c->head = next;
}

/* Extract a line ended by '\n' or '>' (max maxlen-1 chars).
   Returns true if a full line was extracted. */
static bool ring_pop_line(comms_t *c, char *line, size_t maxlen) {
    if (ring_is_empty(c) || maxlen < 2) return false;

    size_t i = 0;
    size_t tail = c->tail;
    bool found = false;

    while (tail != c->head && i + 1 < maxlen) {
        char ch = c->rx_buf[tail];
        line[i++] = ch;
        tail = (tail + 1u) % COMMS_RX_BUF_SIZE;
        if (ch == '\n' || ch == '>') { found = true; break; }
    }

    if (!found) return false;

    line[i] = '\0';
    c->tail = (c->tail + i) % COMMS_RX_BUF_SIZE;
    return true;
}

/* ---------- Public ---------- */

void comms_init(comms_t *c, UART_HandleTypeDef *huart, meas_callback_t cb) {
    memset((void*)c, 0, sizeof(*c));
    c->huart   = huart;
    c->on_meas = cb;
    start_iRx(c);
}

/* Call from your HAL weak callback:
   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
       comms_uart_rx_callback(&comms, huart);
   } */
void comms_uart_rx_callback(comms_t *c, UART_HandleTypeDef *huart) {
    if (!c || huart != c->huart) return;
    ring_push_isr(c, (char)c->rx_byte);
    start_iRx(c);  // re-arm 1-byte IT
}

void comms_poll(comms_t *c) {
    if (!c) return;

    char line[256];
    while (ring_pop_line(c, line, sizeof(line))) {
        meas_packet_t m;
        if (parse_meas_line(line, &m) == 0 && m.valid) {
            if (c->on_meas) c->on_meas(&m);
        }
    }
}

bool comms_send_ctrl(comms_t *c, const ctrl_packet_t *ctrl) {
    if (!c || !ctrl) return false;
    char buf[64];
    int n = format_ctrl_packet(ctrl, buf, sizeof(buf));
    if (n <= 0) return false;

    while (c->huart->gState != HAL_UART_STATE_READY) {
		osDelay(1);
	}

	if (HAL_UART_Transmit(c->huart, (uint8_t*)buf, n, 50) == HAL_OK)
		return true;
	return false;
}
