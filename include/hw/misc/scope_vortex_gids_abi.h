/* SPDX-License-Identifier: Apache-2.0 */
#ifndef HW_MISC_SCOPE_VORTEX_GIDS_ABI_H
#define HW_MISC_SCOPE_VORTEX_GIDS_ABI_H

#include <stddef.h>
#include <stdint.h>

#define SCOPE_GIDS_ABI_MAGIC              UINT32_C(0x53444947) /* "GIDS" */
#define SCOPE_GIDS_ABI_MAJOR              1U
#define SCOPE_GIDS_ABI_MINOR              0U
#define SCOPE_GIDS_ENTRY_SIZE             64U
#define SCOPE_GIDS_QUEUE_HEADER_SIZE      64U
#define SCOPE_GIDS_QUEUE_ALIGN            4096U
#define SCOPE_GIDS_MIN_DEPTH              2U
#define SCOPE_GIDS_MAX_DEPTH              1024U

#define SCOPE_GIDS_FEATURE_READ           (UINT32_C(1) << 0)
#define SCOPE_GIDS_FEATURE_WRITE          (UINT32_C(1) << 1)
#define SCOPE_GIDS_FEATURE_BATCHED        (UINT32_C(1) << 2)
#define SCOPE_GIDS_FEATURE_DIRECT_P2P     (UINT32_C(1) << 3)
#define SCOPE_GIDS_FEATURE_SINGLE_PRODUCER (UINT32_C(1) << 4)

enum scope_gids_queue_state {
    SCOPE_GIDS_QUEUE_RESET = 0,
    SCOPE_GIDS_QUEUE_READY = 1,
    SCOPE_GIDS_QUEUE_QUIESCING = 2,
    SCOPE_GIDS_QUEUE_FAILED = 3,
};

enum scope_gids_opcode {
    SCOPE_GIDS_OP_READ = 1,
    SCOPE_GIDS_OP_WRITE = 2,
};

enum scope_gids_status {
    SCOPE_GIDS_STATUS_SUCCESS = 0,
    SCOPE_GIDS_STATUS_INVALID_OPCODE = 1,
    SCOPE_GIDS_STATUS_INVALID_NAMESPACE = 2,
    SCOPE_GIDS_STATUS_LBA_RANGE = 3,
    SCOPE_GIDS_STATUS_BUFFER_RANGE = 4,
    SCOPE_GIDS_STATUS_STALE_GENERATION = 5,
    SCOPE_GIDS_STATUS_DUPLICATE_CID = 6,
    SCOPE_GIDS_STATUS_QUEUE_FAILED = 7,
    SCOPE_GIDS_STATUS_TIMEOUT = 8,
    SCOPE_GIDS_STATUS_TRANSPORT = 9,
    SCOPE_GIDS_STATUS_NVME = 10,
};

#define SCOPE_GIDS_CQE_F_NVME_STATUS_VALID (UINT16_C(1) << 0)
#define SCOPE_GIDS_CQE_F_FATAL             (UINT16_C(1) << 1)

/*
 * All fields are little-endian on the wire.  The current XiangShan, U280 and
 * x86 hosts are little-endian, so producers and consumers may use native
 * scalar accesses after validating ABI_MAJOR.  Reserved fields must be zero.
 */
struct scope_gids_queue_header {
    uint32_t magic;
    uint16_t abi_major;
    uint16_t abi_minor;
    uint16_t header_size;
    uint16_t sqe_size;
    uint16_t cqe_size;
    uint16_t reserved0;
    uint32_t depth;
    uint32_t features;
    uint32_t state;
    uint32_t generation;
    uint32_t sq_producer;
    uint32_t sq_consumer;
    uint32_t cq_producer;
    uint32_t cq_consumer;
    uint32_t fatal_error;
    uint32_t reserved1;
    uint64_t session_cookie;
};

struct scope_gids_sqe {
    uint32_t ready_sequence;
    uint32_t logical_cid;
    uint8_t opcode;
    uint8_t flags;
    uint16_t reserved0;
    uint32_t nsid;
    uint64_t slba;
    uint32_t block_count;
    uint32_t buffer_handle;
    uint64_t buffer_generation;
    uint64_t buffer_offset;
    uint32_t byte_count;
    uint32_t reserved1;
    uint64_t user_token;
};

struct scope_gids_cqe {
    uint32_t ready_sequence;
    uint32_t logical_cid;
    uint16_t status;
    uint16_t flags;
    uint16_t nvme_status;
    uint16_t reserved0;
    uint32_t completed_bytes;
    uint32_t reserved1;
    uint64_t user_token;
    uint64_t submit_timestamp;
    uint64_t complete_timestamp;
    uint32_t generation;
    uint32_t reserved2;
    uint64_t reserved3;
};

#if defined(__cplusplus)
#define SCOPE_GIDS_STATIC_ASSERT static_assert
#else
#define SCOPE_GIDS_STATIC_ASSERT _Static_assert
#endif

SCOPE_GIDS_STATIC_ASSERT(sizeof(struct scope_gids_queue_header) ==
                         SCOPE_GIDS_QUEUE_HEADER_SIZE,
                         "GIDS queue header ABI");
SCOPE_GIDS_STATIC_ASSERT(sizeof(struct scope_gids_sqe) == SCOPE_GIDS_ENTRY_SIZE,
                         "GIDS SQE ABI");
SCOPE_GIDS_STATIC_ASSERT(sizeof(struct scope_gids_cqe) == SCOPE_GIDS_ENTRY_SIZE,
                         "GIDS CQE ABI");
SCOPE_GIDS_STATIC_ASSERT(offsetof(struct scope_gids_sqe, slba) == 16,
                         "GIDS SQE SLBA offset");
SCOPE_GIDS_STATIC_ASSERT(offsetof(struct scope_gids_sqe, buffer_generation) == 32,
                         "GIDS SQE generation offset");
SCOPE_GIDS_STATIC_ASSERT(offsetof(struct scope_gids_cqe, user_token) == 24,
                         "GIDS CQE token offset");

#undef SCOPE_GIDS_STATIC_ASSERT

#endif
