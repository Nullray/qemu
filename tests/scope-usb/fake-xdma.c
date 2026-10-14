/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Test-only sysfs/XDMA shim. Never accesses real PCI functions. */
#define _GNU_SOURCE
#define _LARGEFILE64_SOURCE
#include <dlfcn.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <linux/if_tun.h>
#include <errno.h>
#include "hw/misc/scope_fpga_proxy_abi.h"
#include "hw/misc/scope_qdma_hostbuf_uapi.h"
#include "hw/misc/scope_qdma_bridge_uapi.h"

static int fake_tap_fd = -1;

/* Model the aligned PCIe DMA counter publication, without Python pack_into's
 * transient zeroing/byte stores becoming visible to the QEMU polling thread.
 */
void scope_test_store32(uint32_t *where, uint32_t value)
{
    __atomic_store_n(where, value, __ATOMIC_RELEASE);
}
void scope_test_store16(uint16_t *where, uint16_t value)
{
    __atomic_store_n(where, value, __ATOMIC_RELEASE);
}

ssize_t write(int fd, const void *buffer, size_t size)
{
    static ssize_t (*real_fn)(int, const void *, size_t);
    static int blocked_once;
    if (fd == fake_tap_fd && !blocked_once && getenv("SCOPE_TEST_TAP_EAGAIN")) {
        blocked_once = 1;
        errno = EAGAIN;
        return -1;
    }
    if (!real_fn) real_fn = dlsym(RTLD_NEXT, "write");
    return real_fn(fd, buffer, size);
}

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
    if (!strcmp(path, "/dev/net/tun") && getenv("SCOPE_TEST_TAP_FD")) { \
        fake_tap_fd = dup(atoi(getenv("SCOPE_TEST_TAP_FD"))); \
        fcntl(fake_tap_fd, F_SETFL, O_NONBLOCK); \
        return fake_tap_fd; \
    } \
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
    const char *cq = getenv("SCOPE_TEST_QDMA");

    if (fd == fake_tap_fd && request == TUNSETIFF) return 0;

    snprintf(link, sizeof(link), "/proc/self/fd/%d", fd);
    len = readlink(link, path, sizeof(path) - 1);
    if (len > 0) {
        path[len] = 0;
        if (cq && !strcmp(path, cq)) {
            uint32_t value;
            if (request == SCOPE_QDMA_ARM || request == SCOPE_BRIDGE_ARM) {
                value = 1;
                return pwrite(fd, &value, sizeof(value), 4084) == sizeof(value) ? 0 : -1;
            }
            va_start(ap, request);
            arg = va_arg(ap, void *);
            va_end(ap);
            if (request == SCOPE_BRIDGE_START) {
                struct scope_bridge_start *start = arg;
                if (start->abi != SCOPE_BRIDGE_ABI || start->entries != 8 || start->reserved) {
                    errno = EINVAL; return -1;
                }
                return 0;
            }
            if (request == SCOPE_BRIDGE_INFO) {
                struct scope_bridge_info info = {
                    .abi = SCOPE_BRIDGE_ABI, .entries = 8, .slot_bytes = 64,
                    .header_bytes = 64, .packet_bytes = 32, .features = 3,
                    .mmap_bytes = 4096,
                };
                memcpy(arg, &info, sizeof(info)); return 0;
            }
            if (request == SCOPE_BRIDGE_ACK) {
                struct scope_bridge_ack *ack = arg;
                uint32_t produced;
                if (pread(fd, &value, sizeof(value), 4080) != sizeof(value) ||
                    pread(fd, &produced, sizeof(produced), 0) != sizeof(produced)) return -1;
                if (!ack->entries || ack->reserved || ack->entries > 8 ||
                    ack->entries > produced - value) { errno = EINVAL; return -1; }
                value += ack->entries;
                return pwrite(fd, &value, sizeof(value), 4080) == sizeof(value) ? 0 : -1;
            }
            if (request == SCOPE_QDMA_START) return 0;
            if (request == SCOPE_QDMA_INFO) {
                struct scope_qdma_info info = {
                    .abi = SCOPE_QDMA_ABI, .entries = 8, .entry_bytes = 64,
                    .packet_offset = 8, .packet_bytes = 32,
                    .initial_color = 1, .mmap_bytes = 4096,
                };
                memcpy(arg, &info, sizeof(info));
                return 0;
            }
            if (request == SCOPE_QDMA_ACK) {
                struct scope_qdma_ack *ack = arg;
                uint32_t produced;
                if (pread(fd, &value, sizeof(value), 4080) != sizeof(value) ||
                    pread(fd, &produced, sizeof(produced), 4088) != sizeof(produced)) return -1;
                if (!ack->entries || ack->entries > 7 || ack->entries > produced - value) {
                    errno = EINVAL; return -1;
                }
                value += ack->entries;
                return pwrite(fd, &value, sizeof(value), 4080) == sizeof(value) ? 0 : -1;
            }
            errno = ENOTTY; return -1;
        }
        if (ctrl && !strcmp(path, ctrl)) {
            if (request == XDMA_IOC_DMA32_DB_FREE) return 0;
            if (request == XDMA_IOC_DMA32_DB_ALLOC) {
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
