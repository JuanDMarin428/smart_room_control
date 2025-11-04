#pragma once
#include <stddef.h>
#include "comms.h"

#ifdef __cplusplus
extern "C" {
#endif

int parse_meas_line(const char *line, meas_packet_t *out);
int format_ctrl_packet(const ctrl_packet_t *in, char *buf, size_t buflen);

#ifdef __cplusplus
}
#endif
