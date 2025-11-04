/**
 * @file uart_parser.h
 * @brief ASCII frame parser for "<MEAS, T, w, c, N>".
 */

#ifndef UART_PARSER_H
#define UART_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "comms.h"

/**
 * @brief Try to parse a frame "<MEAS, T, w, c, N>".
 *
 * The function expects the input string to include the leading '<' and
 * trailing '>'. Spaces around commas/tokens are tolerated.
 *
 * @param frame Input C-string containing one full frame.
 * @param out   Output parsed structure.
 * @return true on success, false otherwise.
 */
bool uart_parse_meas(const char *frame, meas_packet_t *out);

#ifdef __cplusplus
}
#endif

#endif /* UART_PARSER_H */
