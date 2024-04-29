#pragma once

#ifdef __APPLE__

// These are just stubs to make the code compile (NOT WORK) on macOS

struct serial_struct {
    int type;
    int line;
    unsigned int port;
    int irq;
    int flags;
    int xmit_fifo_size;
    int custom_divisor;
    int baud_base;
    unsigned short close_delay;
    char io_type;
    char reserved_char[1];
    int hub6;
    unsigned short closing_wait;  /* time to wait before closing */
    unsigned short closing_wait2; /* no longer used... */
    unsigned char* iomem_base;
    unsigned short iomem_reg_shift;
    unsigned int port_high;
    unsigned long iomap_base; /* cookie passed into ioremap */
};

constexpr int TIOCGSERIAL = 0x0000541E;
constexpr int TIOCSSERIAL = 0x0000541F;

constexpr int ASYNC_LOW_LATENCY = 0x00000020;

#else

#include_next <linux/serial.h>

#endif