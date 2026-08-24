/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef SCOPE_XDMA_PEER_UAPI_H
#define SCOPE_XDMA_PEER_UAPI_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define XDMA_PEER_IOC_MAGIC 'P'
#define XDMA_PEER_ABI_VERSION 1U

#define XDMA_PEER_F_READ  (1U << 0)
#define XDMA_PEER_F_WRITE (1U << 1)

struct xdma_peer_export {
    __u32 abi_version;
    __u32 flags;
    __u64 bar_offset;
    __u64 length;
    __u64 guest_iova;
    __u64 generation;
    __s32 dmabuf_fd;
    __u32 reserved;
};

#define XDMA_PEER_IOC_EXPORT \
    _IOWR(XDMA_PEER_IOC_MAGIC, 0, struct xdma_peer_export)

#endif
