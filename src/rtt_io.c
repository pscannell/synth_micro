#include "rtt_io.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define RTT_MAX_UP_BUFFERS 1
#define RTT_MAX_DOWN_BUFFERS 1
#define RTT_UP_BUFFER_SIZE 1024
#define RTT_DOWN_BUFFER_SIZE 256

typedef struct {
	const char *sName;
	char *pBuffer;
	unsigned SizeOfBuffer;
	volatile unsigned WrOff;
	volatile unsigned RdOff;
	unsigned Flags;
} RTT_RING_BUFFER;

typedef struct {
	char acID[16];
	int MaxNumUpBuffers;
	int MaxNumDownBuffers;
	RTT_RING_BUFFER aUp[RTT_MAX_UP_BUFFERS];
	RTT_RING_BUFFER aDown[RTT_MAX_DOWN_BUFFERS];
} RTT_CB;

volatile RTT_CB _SEGGER_RTT;

static char up_buf[RTT_UP_BUFFER_SIZE];
static char down_buf[RTT_DOWN_BUFFER_SIZE];
static const char up_name[] = "term";
static const char down_name[] = "cmd";

static int rtt_write_bytes(const char *data, unsigned len) {
	volatile RTT_RING_BUFFER *rb = &_SEGGER_RTT.aUp[0];
	unsigned wr = rb->WrOff;
	unsigned rd = rb->RdOff;
	unsigned written = 0;

	for (written = 0; written < len; ++written) {
		unsigned next = wr + 1;
		if (next >= rb->SizeOfBuffer) {
			next = 0;
		}
		if (next == rd) {
			break;
		}
		rb->pBuffer[wr] = data[written];
		wr = next;
	}

	rb->WrOff = wr;
	return (int) written;
}

static int rtt_read_byte(void) {
	volatile RTT_RING_BUFFER *rb = &_SEGGER_RTT.aDown[0];
	unsigned rd = rb->RdOff;
	unsigned wr = rb->WrOff;
	unsigned char c;

	if (rd == wr) {
		return -1;
	}

	c = (unsigned char) rb->pBuffer[rd];
	rd++;
	if (rd >= rb->SizeOfBuffer) {
		rd = 0;
	}
	rb->RdOff = rd;
	return (int) c;
}

void rtt_init(void) {
	memset((void*) &_SEGGER_RTT, 0, sizeof(_SEGGER_RTT));
	memcpy((void*) _SEGGER_RTT.acID, "SEGGER RTT", 10);

	_SEGGER_RTT.MaxNumUpBuffers = RTT_MAX_UP_BUFFERS;
	_SEGGER_RTT.MaxNumDownBuffers = RTT_MAX_DOWN_BUFFERS;

	_SEGGER_RTT.aUp[0].sName = up_name;
	_SEGGER_RTT.aUp[0].pBuffer = up_buf;
	_SEGGER_RTT.aUp[0].SizeOfBuffer = RTT_UP_BUFFER_SIZE;

	_SEGGER_RTT.aDown[0].sName = down_name;
	_SEGGER_RTT.aDown[0].pBuffer = down_buf;
	_SEGGER_RTT.aDown[0].SizeOfBuffer = RTT_DOWN_BUFFER_SIZE;
}

int rtt_read_line(char *out, int out_len) {
	static char line_buf[128];
	static unsigned idx = 0;
	int ch;

	if (out == 0 || out_len < 2) {
		return 0;
	}

	while ((ch = rtt_read_byte()) >= 0) {
		if (ch == '\r') {
			continue;
		}
		if (ch == '\n') {
			line_buf[idx] = '\0';
			snprintf(out, (unsigned) out_len, "%s", line_buf);
			idx = 0;
			return 1;
		}
		if (idx < (sizeof(line_buf) - 1)) {
			line_buf[idx++] = (char) ch;
		} else {
			idx = 0;
		}
	}

	return 0;
}

int rtt_write(const char *text) {
	unsigned len;

	if (text == 0) {
		return 0;
	}
	len = (unsigned) strlen(text);
	return rtt_write_bytes(text, len);
}

int rtt_printf(const char *fmt, ...) {
	char buf[192];
	va_list args;
	int n;

	va_start(args, fmt);
	n = vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	if (n <= 0) {
		return n;
	}
	if ((unsigned) n >= sizeof(buf)) {
		n = (int) sizeof(buf) - 1;
	}
	return rtt_write_bytes(buf, (unsigned) n);
}
