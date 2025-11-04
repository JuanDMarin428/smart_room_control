/**
 * @file comms.h
 * @brief Minimal UART communications layer (ISR RX ring + ASCII frames).
 *
 * Frames:
 *  - PC -> MCU : "<MEAS, T, w, c, N>\n"
 *  - MCU -> PC : "<CTRL, uh, uf>\r\n"
 *
 * Design:
 *  - RX handled by HAL UART interrupt (byte-by-byte).
 *  - ISR pushes bytes to a single-producer ring buffer.
 *  - comms_poll() assembles frames delimited by '<' and '>' and calls
 *    the user callback when a valid MEAS frame is parsed.
 *  - comms_send_ctrl() formats and transmits CTRL frames.
 *
 * Safety:
 *  - Parser is tolerant to garbage and resynchronizes on '<'.
 *  - Ring buffer overflow drops the oldest byte.
 *
 * @author
 */

#ifndef COMMS_H
#define COMMS_H

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Includes                                                                   */
/* -------------------------------------------------------------------------- */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "stm32f7xx_hal.h"

/* -------------------------------------------------------------------------- */
/* Public Types                                                               */
/* -------------------------------------------------------------------------- */

/**
 * @struct meas_packet_t
 * @brief Parsed values for the MEAS frame.
 */
typedef struct {
    float T;  /**< Temperature */
    float w;  /**< Airflow or slow variable */
    float c;  /**< Load / consumption */
    float N;  /**< Sequence / counter */
} meas_packet_t;

/**
 * @struct ctrl_packet_t
 * @brief Control command values to send to the PC (0..1 range).
 */
typedef struct {
    float uh; /**< Heater command [0..1] */
    float uf; /**< Fan command    [0..1] */
} ctrl_packet_t;

/**
 * @typedef meas_cb_t
 * @brief User callback invoked when a MEAS frame is parsed.
 *
 * @param m Pointer to parsed measurement packet.
 */
typedef void (*meas_cb_t)(const meas_packet_t *m);

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

#ifndef COMMS_RX_BUF_SIZE
#define COMMS_RX_BUF_SIZE 512u
#endif

/* Optional debug (counts, simple printf hooks). */
#ifndef COMMS_DEBUG
#define COMMS_DEBUG 0
#endif

/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

/**
 * @struct comms_t
 * @brief Communications driver state (single instance unless multi-UART).
 */
typedef struct {
    UART_HandleTypeDef *huart;              /**< HAL UART handle */

    volatile uint8_t rx_ring[COMMS_RX_BUF_SIZE]; /**< ISR-managed ring buffer */
    volatile size_t  head;                  /**< Write index (ISR) */
    volatile size_t  tail;                  /**< Read index (poll) */

    volatile uint8_t rx_byte;               /**< Temp byte for HAL receive IT */

    meas_cb_t on_meas;                      /**< User MEAS callback */

#if COMMS_DEBUG
    volatile uint32_t dbg_overflow;         /**< Ring overflow counter */
    volatile uint32_t dbg_frames;           /**< Frames completed */
    volatile uint32_t dbg_bad_meas;         /**< Invalid MEAS frames */
#endif
} comms_t;

/* -------------------------------------------------------------------------- */
/* API                                                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize communications driver and arm the first RX interrupt.
 *
 * @param c       Pointer to driver state.
 * @param huart   UART handle already initialized by MX_USARTx_UART_Init().
 * @param cb      Callback called when a valid MEAS frame is parsed.
 */
void comms_init(comms_t *c, UART_HandleTypeDef *huart, meas_cb_t cb);

/**
 * @brief HAL Rx complete callback hook. Call from HAL_UART_RxCpltCallback().
 *
 * @param c     Pointer to driver state.
 * @param huart UART handle provided by HAL callback.
 */
void comms_uart_rx_callback(comms_t *c, UART_HandleTypeDef *huart);

/**
 * @brief Polling function to assemble frames and parse MEAS packets.
 *
 * Call this frequently from a task (e.g., every 1–10 ms).
 *
 * @param c Pointer to driver state.
 */
void comms_poll(comms_t *c);

/**
 * @brief Send a CTRL frame "<CTRL, uh, uf>\r\n" over UART (blocking).
 *
 * The values are saturated to [0, 1] for safety.
 *
 * @param c     Pointer to driver state.
 * @param ctrl  Pointer to control packet.
 * @return HAL_StatusTypeDef HAL_OK on success.
 */
HAL_StatusTypeDef comms_send_ctrl(comms_t *c, const ctrl_packet_t *ctrl);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_H */
