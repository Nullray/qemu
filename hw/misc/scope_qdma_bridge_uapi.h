/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef SCOPE_QDMA_BRIDGE_UAPI_H
#define SCOPE_QDMA_BRIDGE_UAPI_H
#include <linux/types.h>
#include <linux/ioctl.h>

#define SCOPE_BRIDGE_ABI 1U
#define SCOPE_BRIDGE_MAGIC 0x53434231U
#define SCOPE_BRIDGE_CSR_OFFSET 0x240000U
#define SCOPE_BRIDGE_HEADER_BYTES 64U
#define SCOPE_BRIDGE_SLOT_BYTES 64U
#define SCOPE_BRIDGE_PACKET_BYTES 32U
#define SCOPE_BRIDGE_FEATURE_COHERENT 1U
#define SCOPE_BRIDGE_FEATURE_FULL_DMA_ADDRESS 2U
struct scope_bridge_start {
    __u32 abi;
    __u32 entries;
    __u64 reserved;
};
struct scope_bridge_info {
    __u32 abi;
    __u32 entries;
    __u32 slot_bytes;
    __u32 header_bytes;
    __u32 packet_bytes;
    __u32 features;
    __u64 mmap_bytes;
    __u64 reserved;
};
struct scope_bridge_ack { __u32 entries; __u32 reserved; };
#define SCOPE_BRIDGE_START _IOW('B', 0x70, struct scope_bridge_start)
#define SCOPE_BRIDGE_INFO _IOR('B', 0x71, struct scope_bridge_info)
#define SCOPE_BRIDGE_ACK _IOW('B', 0x72, struct scope_bridge_ack)
#define SCOPE_BRIDGE_ARM _IO('B', 0x73)
#endif
