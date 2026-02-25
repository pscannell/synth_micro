#ifndef RTT_IO_H_
#define RTT_IO_H_

#include <stdint.h>

void rtt_init(void);
int rtt_read_line(char *out, int out_len);
int rtt_write(const char *text);
int rtt_printf(const char *fmt, ...);

#endif
