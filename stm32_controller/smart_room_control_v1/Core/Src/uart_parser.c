/**
 * @file uart_parser.c
 * @brief Implementation of ASCII MEAS frame parser.
 */

#include "uart_parser.h"
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

/* -------------------------------------------------------------------------- */
/* Internal helpers                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Trim ASCII whitespace from both ends (in-place).
 * @param s Mutable C-string.
 * @return Pointer to the first non-space character inside s.
 */
static char *trim(char *s) {
    while (isspace((unsigned char)*s)) s++;
    if (*s == 0) return s;
    char *e = s + strlen(s) - 1;
    while (e > s && isspace((unsigned char)*e)) *e-- = 0;
    return s;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Parse a measurement frame of the form "<MEAS, T, w, c, N>".
 *
 * The function extracts the comma-separated fields and converts them to floats.
 * It is tolerant to extra spaces around commas and tokens.
 * Returns true only if all five tokens ("MEAS", T, w, c, N) are present
 * and valid numbers are parsed.
 *
 * Example:
 *   Input: "<MEAS, 20.315, 0.005890, 781.1, 2>"
 *   Output: out->T=20.315, w=0.005890, c=781.1, N=2
 *
 * @param frame  Input ASCII frame including '<' and '>'.
 * @param out    Output struct with parsed float values.
 * @return true  On successful parsing.
 * @return false On invalid syntax or conversion failure.
 */
bool uart_parse_meas(const char *frame, meas_packet_t *out) {
    /* --- Basic validation --- */
    if (!frame || frame[0] != '<') return false;
    const char *gt = strrchr(frame, '>');
    if (!gt) return false;

    size_t len = (size_t)(gt - frame - 1);
    if (len == 0) return false;
    if (len >= 127) return false;

    /* --- Copy body (between < and >) into a mutable buffer --- */
    char tmp[128];
    memcpy(tmp, frame + 1, len);
    tmp[len] = '\0';

    /* --- Tokenize by commas --- */
    // Expect exactly 5 tokens: "MEAS", T, w, c, N
    char *tok[5] = {0};
    size_t ntok = 0;
    char *p = tmp;
    char *s = NULL;

    while (ntok < 4 && (s = strchr(p, ',')) != NULL) {
        *s = '\0';                  // terminate current token
        tok[ntok++] = trim(p);      // store trimmed token
        p = s + 1;                  // advance past comma
    }
    // Add final token (after last comma)
    tok[ntok++] = trim(p);

    if (ntok != 5) return false;

    /* --- Validate the first token ("MEAS") --- */
    const char *k = tok[0];
    const char *m = "MEAS";
    size_t i = 0;
    for (; m[i] && k[i]; ++i) {
        if (toupper((unsigned char)m[i]) != toupper((unsigned char)k[i]))
            break;
    }
    if (m[i] || k[i]) return false; // mismatch

    /* --- Convert remaining tokens to floats --- */
    char *endp = NULL;
    float T = strtof(tok[1], &endp); if (endp == tok[1]) return false;
    float w = strtof(tok[2], &endp); if (endp == tok[2]) return false;
    float c = strtof(tok[3], &endp); if (endp == tok[3]) return false;
    float N = strtof(tok[4], &endp); if (endp == tok[4]) return false;

    /* --- Assign outputs --- */
    out->T = T;
    out->w = w;
    out->c = c;
    out->N = N;

    return true;
}
