/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Test-only sysfs/XDMA shim. Never accesses real PCI functions. */
#define _GNU_SOURCE
#define _LARGEFILE64_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include "hw/misc/scope_fpga_proxy_abi.h"

static const char *redirect(const char *path, char *buf)
{
    const char *prefix = "/sys/bus/pci/devices/fffe:";
    const char *root = getenv("SCOPE_TEST_SYSFS");

    if (root && path && !strncmp(path, prefix, strlen(prefix))) {
        snprintf(buf, 4096, "%s/%s", root,
                 path + strlen("/sys/bus/pci/devices/"));
        return buf;
    }
    return path;
}

#define PATH_WRAPPER(name, result, args, callargs) \
result name args { \
    char buf[4096]; \
    static result (*real_fn) args; \
    if (!real_fn) real_fn = dlsym(RTLD_NEXT, #name); \
    path = redirect(path, buf); \
    return real_fn callargs; \
}

PATH_WRAPPER(fopen, FILE *, (const char *path, const char *mode), (path, mode))
PATH_WRAPPER(fopen64, FILE *, (const char *path, const char *mode), (path, mode))
PATH_WRAPPER(stat, int, (const char *path, struct stat *st), (path, st))
PATH_WRAPPER(stat64, int, (const char *path, struct stat64 *st), (path, st))
PATH_WRAPPER(access, int, (const char *path, int mode), (path, mode))

#define OPEN_WRAPPER(name) \
int name(const char *path, int flags, ...) { \
    char buf[4096]; \
    mode_t mode = 0; \
    static int (*real_fn)(const char *, int, ...); \
    if (flags & O_CREAT) { \
        va_list ap; va_start(ap, flags); \
        mode = va_arg(ap, int); va_end(ap); \
    } \
    if (!real_fn) real_fn = dlsym(RTLD_NEXT, #name); \
    return real_fn(redirect(path, buf), flags, mode); \
}
OPEN_WRAPPER(open)
OPEN_WRAPPER(open64)

int ioctl(int fd, unsigned long request, ...)
{
    static int (*real_fn)(int, unsigned long, ...);
    void *arg;
    va_list ap;
    char link[64], path[4096];
    ssize_t len;
    const char *ctrl = getenv("SCOPE_TEST_CTRL");

    snprintf(link, sizeof(link), "/proc/self/fd/%d", fd);
    len = readlink(link, path, sizeof(path) - 1);
    if (len > 0) {
        path[len] = 0;
        if (ctrl && !strcmp(path, ctrl)) {
            const char *mode = getenv("SCOPE_TEST_RING_MODE");
            const char *trace = getenv("SCOPE_TEST_IOCTL_LOG");
            if (request == XDMA_IOC_DMA32_DB_FREE) {
                if (trace) {
                    FILE *log = fopen(trace, "a");
                    if (log) {
                        fprintf(log, "FREE\n");
                        fclose(log);
                    }
                }
                return 0;
            }
            if (request == XDMA_IOC_DMA32_DB_QUERY) {
                if (mode && !strcmp(mode, "stale")) {
                    va_start(ap, request);
                    struct scope_xdma_dma32_doorbell *db = va_arg(ap, void *);
                    va_end(ap);
                    db->size = 65536;
                    db->dma_addr = 0x40000000;
                    return 0;
                }
                errno = ENODATA;
                return -1;
            }
            if (request == XDMA_IOC_DMA32_DB_ALLOC) {
                if (mode && (!strcmp(mode, "busy") ||
                             !strcmp(mode, "stale"))) {
                    errno = EBUSY;
                    return -1;
                }
                va_start(ap, request);
                struct scope_xdma_dma32_doorbell *db = va_arg(ap, void *);
                va_end(ap);
                db->dma_addr = 0x40000000;
                return 0;
            }
        }
    }
    va_start(ap, request);
    arg = va_arg(ap, void *);
    va_end(ap);
    if (!real_fn) real_fn = dlsym(RTLD_NEXT, "ioctl");
    return real_fn(fd, request, arg);
}
