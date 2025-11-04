#include "uart_parser.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>

#if defined(__has_include)
#  if __has_include(<strings.h>)
#    include <strings.h>
#    define local_strncasecmp strncasecmp
#  else
#    define NEED_LOCAL_CI_CMP 1
#  endif
#else
#  define NEED_LOCAL_CI_CMP 1
#endif

#ifdef NEED_LOCAL_CI_CMP
static int local_strncasecmp(const char *a, const char *b, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        unsigned char ca = (unsigned char)tolower((unsigned char)a[i]);
        unsigned char cb = (unsigned char)tolower((unsigned char)b[i]);
        if (ca != cb || ca == '\0' || cb == '\0') return (int)ca - (int)cb;
    }
    return 0;
}
#endif

static const char *trim(const char *s) {
    while (*s && isspace((unsigned char)*s)) s++;
    return s;
}
static void rtrim(char *s) {
    size_t n = strlen(s);
    while (n > 0 && isspace((unsigned char)s[n-1])) { s[n-1] = '\0'; n--; }
}

int parse_meas_line(const char *line, meas_packet_t *out) {
    if (!line || !out) return -1;
    out->valid = false;

    const char *start = strchr(line, '<');
    const char *end   = strrchr(line, '>');
    if (!start || !end || end <= start) return -2;

    size_t len = (size_t)(end - start - 1);
    if (len == 0 || len > 200) return -3;

    char buf[256];
    memcpy(buf, start + 1, len);
    buf[len] = '\0';
    rtrim(buf);

    const char *p = trim(buf);
    if (local_strncasecmp(p, "MEAS", 4) != 0) return -4;
    p += 4;
    while (*p && (isspace((unsigned char)*p) || *p == ',')) p++;

    float vals[4];
    int idx = 0;
    while (*p && idx < 4) {
        char tok[64];
        int ti = 0;
        while (*p && *p != ',') {
            if (ti < (int)sizeof(tok) - 1) tok[ti++] = *p;
            p++;
        }
        tok[ti] = '\0';
        if (*p == ',') p++;

        const char *t = trim(tok);
        char *endptr = NULL;
        float v = strtof(t, &endptr);
        if (endptr == t) return -5;
        vals[idx++] = v;
    }
    if (idx != 4) return -6;

    out->T = vals[0];
    out->w = vals[1];
    out->c = vals[2];
    out->N = vals[3];
    out->valid = true;
    return 0;
}

int format_ctrl_packet(const ctrl_packet_t *in, char *buf, size_t buflen) {
    if (!in || !buf || buflen == 0) return -1;
    float uh = in->uh; if (uh < 0.f) uh = 0.f; if (uh > 1.f) uh = 1.f;
    float uf = in->uf; if (uf < 0.f) uf = 0.f; if (uf > 1.f) uf = 1.f;
    int n = snprintf(buf, buflen, "<CTRL, %.3f, %.3f>\r\n", uh, uf);
    if (n < 0 || (size_t)n >= buflen) return -2;
    return n;
}
