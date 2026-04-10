/*
 * baudrate_shim.c — LD_PRELOAD shim to support arbitrary serial baud rates
 *                   on Linux for MATLAB R2023b.
 *
 * MATLAB's libmwserialsupport.so uses cfsetispeed/cfsetospeed which only
 * accept standard Bxxx constants (max B4000000). This shim intercepts
 * serial::Serial::setBaudRate(unsigned int) and uses the Linux BOTHER
 * mechanism (ioctl TCSETS2) to set arbitrary baud rates.
 *
 * Build:
 *   gcc -shared -fPIC -O2 -o libbaudrate_shim.so baudrate_shim.c -ldl
 *
 * Usage:
 *   LD_PRELOAD=/path/to/libbaudrate_shim.so matlab
 *
 * Copyright 2024 — MIT License
 */

#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>

/* Avoid conflicts between glibc <termios.h> and <linux/termios.h>.
   We define only what we need from the kernel interface. */
#ifndef TCGETS2
#include <asm/ioctls.h>    /* TCGETS2, TCSETS2 */
#endif
#include <asm/termbits.h>  /* struct termios2, BOTHER, CBAUD */

/* Max standard baud rate supported by Linux cfsetospeed */
#define MAX_STANDARD_BAUD  4000000

/*
 * Intercept serial::Serial::setBaudRate(unsigned int)
 * Mangled symbol: _ZN6serial6Serial11setBaudRateEj
 *
 * From disassembly of libmwserialsupport.so:
 *   - this + 0x38 = stored baud rate (uint32_t)
 *   - this + 0x68 = file descriptor  (int32_t)
 */
void _ZN6serial6Serial11setBaudRateEj(void *self, unsigned int baudrate)
{
    typedef void (*orig_fn_t)(void *, unsigned int);
    static orig_fn_t orig = NULL;

    if (!orig) {
        orig = (orig_fn_t)dlsym(RTLD_NEXT,
                                "_ZN6serial6Serial11setBaudRateEj");
        if (!orig) {
            fprintf(stderr, "[baudrate_shim] FATAL: cannot resolve original "
                    "serial::Serial::setBaudRate: %s\n", dlerror());
            return;
        }
    }

    /* Standard baud rates — let the original handle them */
    if (baudrate <= MAX_STANDARD_BAUD) {
        orig(self, baudrate);
        return;
    }

    /* Non-standard baud rate (e.g., 12 000 000).
       Store it, then use BOTHER + TCSETS2 to set it directly. */
    fprintf(stderr, "[baudrate_shim] Setting non-standard baud rate: %u\n",
            baudrate);

    /* Write baud rate into the object (same as original does first thing) */
    *(uint32_t *)((char *)self + 0x38) = baudrate;

    /* Read the file descriptor */
    int fd = *(int32_t *)((char *)self + 0x68);
    if (fd < 0) {
        /* Port not open yet — baud rate is stored, will be applied later
           when the port opens and setBaudRate is called again. */
        return;
    }

    /* Use kernel ioctl to set arbitrary baud rate */
    struct termios2 tio;
    memset(&tio, 0, sizeof(tio));

    if (ioctl(fd, TCGETS2, &tio) < 0) {
        fprintf(stderr, "[baudrate_shim] TCGETS2 failed: %s\n",
                strerror(errno));
        /* Fallback: call original with 4000000 */
        orig(self, MAX_STANDARD_BAUD);
        return;
    }

    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = baudrate;
    tio.c_ospeed = baudrate;

    if (ioctl(fd, TCSETS2, &tio) < 0) {
        fprintf(stderr, "[baudrate_shim] TCSETS2 failed for %u baud: %s\n",
                baudrate, strerror(errno));
        /* Fallback: call original with 4000000 (fine for USB CDC) */
        fprintf(stderr, "[baudrate_shim] Falling back to %d baud "
                "(OK for USB CDC virtual COM ports)\n", MAX_STANDARD_BAUD);
        orig(self, MAX_STANDARD_BAUD);
        return;
    }

    fprintf(stderr, "[baudrate_shim] Successfully set baud rate to %u\n",
            baudrate);
}
