/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef SCOPE_REMOTE_PROTO_H
#define SCOPE_REMOTE_PROTO_H

#include <stdint.h>

#define SCOPE_REMOTE_MAGIC              0x4d445253U /* "SRDM" */
#define SCOPE_REMOTE_PROTO_MAJOR        1U
#define SCOPE_REMOTE_PROTO_MINOR        2U
#define SCOPE_REMOTE_MAX_MESSAGE        4096U
#define SCOPE_REMOTE_MAX_DEVICE_ID      64U
#define SCOPE_REMOTE_MAX_SEGMENTS       64U

#define SCOPE_REMOTE_FEATURE_NVME       (1ULL << 0)
#define SCOPE_REMOTE_FEATURE_PEER_MR    (1ULL << 1)
#define SCOPE_REMOTE_FEATURE_ZERO_COPY  (1ULL << 2)
#define SCOPE_REMOTE_FEATURE_HOST_A_STAGING (1ULL << 3)
#define SCOPE_REMOTE_FEATURE_IXGBE_PACKET   (1ULL << 4)
#define SCOPE_REMOTE_DIRECT_FEATURES  \
    (SCOPE_REMOTE_FEATURE_NVME | SCOPE_REMOTE_FEATURE_PEER_MR | \
     SCOPE_REMOTE_FEATURE_ZERO_COPY)
#define SCOPE_REMOTE_STAGING_FEATURES \
    (SCOPE_REMOTE_FEATURE_NVME | SCOPE_REMOTE_FEATURE_PEER_MR | \
     SCOPE_REMOTE_FEATURE_HOST_A_STAGING)
#define SCOPE_REMOTE_IXGBE_FEATURES \
    (SCOPE_REMOTE_FEATURE_PEER_MR | SCOPE_REMOTE_FEATURE_HOST_A_STAGING | \
     SCOPE_REMOTE_FEATURE_IXGBE_PACKET)

#define SCOPE_REMOTE_ETHER_MAX_FRAME    2048U

enum scope_remote_device_type {
    SCOPE_REMOTE_DEVICE_NVME = 1,
    SCOPE_REMOTE_DEVICE_IXGBE_PACKET = 2,
};

enum scope_remote_opcode {
    SCOPE_REMOTE_OP_HELLO = 1,
    SCOPE_REMOTE_OP_HELLO_RSP,
    SCOPE_REMOTE_OP_DEVICE_OPEN,
    SCOPE_REMOTE_OP_DEVICE_OPEN_RSP,
    SCOPE_REMOTE_OP_BAR_READ,
    SCOPE_REMOTE_OP_BAR_READ_RSP,
    SCOPE_REMOTE_OP_BAR_WRITE,
    SCOPE_REMOTE_OP_BAR_WRITE_RSP,
    SCOPE_REMOTE_OP_NVME_ENABLE,
    SCOPE_REMOTE_OP_NVME_DISABLE,
    SCOPE_REMOTE_OP_NVME_SUBMIT,
    SCOPE_REMOTE_OP_NVME_COMPLETE,
    SCOPE_REMOTE_OP_RESET,
    SCOPE_REMOTE_OP_RESET_RSP,
    SCOPE_REMOTE_OP_KEEPALIVE,
    SCOPE_REMOTE_OP_ERROR,
    SCOPE_REMOTE_OP_CLOSE,
    SCOPE_REMOTE_OP_IXGBE_TX,
    SCOPE_REMOTE_OP_IXGBE_TX_COMPLETE,
    SCOPE_REMOTE_OP_IXGBE_RX,
    SCOPE_REMOTE_OP_IXGBE_LINK,
};

enum scope_remote_status {
    SCOPE_REMOTE_STATUS_OK = 0,
    SCOPE_REMOTE_STATUS_INVAL = -1,
    SCOPE_REMOTE_STATUS_VERSION = -2,
    SCOPE_REMOTE_STATUS_SESSION = -3,
    SCOPE_REMOTE_STATUS_DEVICE = -4,
    SCOPE_REMOTE_STATUS_RANGE = -5,
    SCOPE_REMOTE_STATUS_BUSY = -6,
    SCOPE_REMOTE_STATUS_IO = -7,
    SCOPE_REMOTE_STATUS_TIMEOUT = -8,
    SCOPE_REMOTE_STATUS_UNSUPPORTED = -9,
    SCOPE_REMOTE_STATUS_FAILED = -10,
};

enum scope_remote_io_direction {
    SCOPE_REMOTE_IO_NONE = 0,
    SCOPE_REMOTE_IO_TO_DEVICE = 1,
    SCOPE_REMOTE_IO_FROM_DEVICE = 2,
};

/* All multi-byte fields are little-endian on the wire. */
struct scope_remote_hdr {
    uint32_t magic;
    uint16_t major;
    uint16_t minor;
    uint16_t opcode;
    uint16_t flags;
    uint32_t payload_len;
    uint64_t session_id;
    uint64_t request_id;
    uint64_t generation;
    int32_t status;
    uint32_t reserved;
} __attribute__((packed));

struct scope_remote_hello {
    uint64_t features;
    uint64_t peer_base;
    uint64_t peer_length;
    uint32_t peer_rkey;
    uint32_t max_inflight;
    uint32_t max_segments;
    uint32_t reserved;
} __attribute__((packed));

struct scope_remote_device_open {
    char device_id[SCOPE_REMOTE_MAX_DEVICE_ID];
    uint32_t namespace_id;
    uint32_t device_type;
} __attribute__((packed));

struct scope_remote_device_info {
    uint16_t vendor_id;
    uint16_t device_id;
    uint32_t class_code;
    uint64_t bar0_size;
    uint64_t nvme_cap;
    uint32_t nvme_vs;
    uint32_t controller_page_size;
    uint64_t namespace_blocks;
    uint32_t lba_shift;
    uint32_t max_transfer_bytes;
    uint32_t max_inflight;
    uint32_t device_type;
    uint32_t mtu;
    uint8_t mac[6];
    uint8_t link_up;
    uint8_t reserved;
} __attribute__((packed));

struct scope_remote_ether_frame {
    uint32_t length;
    uint32_t flags;
    uint8_t data[SCOPE_REMOTE_ETHER_MAX_FRAME];
} __attribute__((packed));

struct scope_remote_ether_complete {
    uint32_t length;
    uint32_t reserved;
} __attribute__((packed));

struct scope_remote_link_state {
    uint32_t link_up;
    uint32_t speed_mbps;
} __attribute__((packed));

struct scope_remote_bar {
    uint32_t offset;
    uint32_t size;
    uint64_t value;
    uint32_t wstrb;
    uint32_t reserved;
} __attribute__((packed));

struct scope_remote_segment {
    uint64_t guest_pa;
    uint32_t length;
    uint32_t reserved;
} __attribute__((packed));

struct scope_remote_nvme_submit {
    uint8_t command[64];
    uint32_t qid;
    uint32_t direction;
    uint32_t data_len;
    uint32_t segment_count;
    struct scope_remote_segment segments[SCOPE_REMOTE_MAX_SEGMENTS];
} __attribute__((packed));

struct scope_remote_nvme_complete {
    uint32_t result;
    uint16_t sq_head;
    uint16_t sq_id;
    uint16_t cid;
    uint16_t status;
    uint32_t transferred;
    uint32_t reserved;
} __attribute__((packed));

struct scope_remote_message {
    struct scope_remote_hdr hdr;
    uint8_t payload[SCOPE_REMOTE_MAX_MESSAGE - sizeof(struct scope_remote_hdr)];
} __attribute__((packed));

_Static_assert(sizeof(struct scope_remote_hdr) == 48,
               "scope remote header ABI changed");
_Static_assert(sizeof(struct scope_remote_message) == SCOPE_REMOTE_MAX_MESSAGE,
               "scope remote message ABI changed");
_Static_assert(sizeof(struct scope_remote_nvme_submit) <=
               sizeof(((struct scope_remote_message *)0)->payload),
               "NVMe submit exceeds control message");
_Static_assert(sizeof(struct scope_remote_ether_frame) <=
               sizeof(((struct scope_remote_message *)0)->payload),
               "Ethernet frame exceeds control message");

#endif
