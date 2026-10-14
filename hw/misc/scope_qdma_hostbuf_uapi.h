/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef SCOPE_QDMA_HOSTBUF_UAPI_H
#define SCOPE_QDMA_HOSTBUF_UAPI_H
#include <linux/types.h>
#include <linux/ioctl.h>

#define SCOPE_QDMA_ABI 1U
#define SCOPE_QDMA_CSR_MAGIC 0x53435131U
#define SCOPE_QDMA_CSR_OFFSET 0x240000U
#define SCOPE_QDMA_ENTRY_BYTES 64U
#define SCOPE_QDMA_PACKET_OFFSET 8U
#define SCOPE_QDMA_PACKET_BYTES 32U
#define SCOPE_QDMA_COLOR_MASK 2U
#define SCOPE_QDMA_ERROR_MASK 4U

struct scope_qdma_start {
    __u32 abi;
    __u32 qidx;
    __u32 ring_size_index;
    __u32 reserved;
};
struct scope_qdma_info {
    __u32 abi;
    __u32 entries;
    __u32 entry_bytes;
    __u32 packet_offset;
    __u32 packet_bytes;
    __u32 hardware_qid;
    __u32 initial_cidx;
    __u32 initial_color;
    __u64 mmap_bytes;
};
struct scope_qdma_ack {
    __u32 entries;
    __u32 reserved;
};
#define SCOPE_QDMA_START _IOW('Q', 0x70, struct scope_qdma_start)
#define SCOPE_QDMA_INFO _IOR('Q', 0x71, struct scope_qdma_info)
#define SCOPE_QDMA_ACK _IOW('Q', 0x72, struct scope_qdma_ack)
#define SCOPE_QDMA_ARM _IO('Q', 0x73)
#endif
