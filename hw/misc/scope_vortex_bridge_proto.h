/* SPDX-License-Identifier: Apache-2.0 */
#ifndef HW_MISC_SCOPE_VORTEX_BRIDGE_PROTO_H
#define HW_MISC_SCOPE_VORTEX_BRIDGE_PROTO_H

#include <stdint.h>

#define SCOPE_VORTEX_RPC_MAGIC       0x56585250U
#define SCOPE_VORTEX_RPC_VERSION     4U
#define SCOPE_VORTEX_RPC_CAP_PEER_MAP (1U << 0)
#define SCOPE_VORTEX_RPC_CAP_GDS_P2P  (1U << 1)
#define SCOPE_VORTEX_RPC_CAP_GIDS_QUEUE (1U << 2)
#define SCOPE_VORTEX_RPC_CAP_GIDS_QUEUE_STATE (1U << 3)
#define SCOPE_VORTEX_RPC_GIDS_MAX_BATCH 32U
#define SCOPE_VORTEX_GDS_CAP_HBM0     (1U << 0)
#define SCOPE_VORTEX_HELLO_DIRECT_P2P (1U << 0)
#define SCOPE_VORTEX_RPC_MAX_PAYLOAD (16U * 1024U * 1024U)

enum scope_vortex_rpc_op {
    SCOPE_VORTEX_RPC_HELLO = 1,
    SCOPE_VORTEX_RPC_CP_READ,
    SCOPE_VORTEX_RPC_CP_WRITE,
    SCOPE_VORTEX_RPC_MEM_ALLOC,
    SCOPE_VORTEX_RPC_MEM_FREE,
    SCOPE_VORTEX_RPC_MEM_READ,
    SCOPE_VORTEX_RPC_MEM_WRITE,
    SCOPE_VORTEX_RPC_PEER_CAPS,
    SCOPE_VORTEX_RPC_PEER_MAP,
    SCOPE_VORTEX_RPC_PEER_UNMAP,
    SCOPE_VORTEX_RPC_GDS_CAPS,
    SCOPE_VORTEX_RPC_GDS_ALLOC,
    SCOPE_VORTEX_RPC_GDS_FREE,
    SCOPE_VORTEX_RPC_GIDS_QUEUE_CREATE,
    SCOPE_VORTEX_RPC_GIDS_QUEUE_DESTROY,
    SCOPE_VORTEX_RPC_GIDS_FETCH_SQ,
    SCOPE_VORTEX_RPC_GIDS_COMMIT_SQ,
    SCOPE_VORTEX_RPC_GIDS_POST_CQ,
    SCOPE_VORTEX_RPC_GIDS_QUEUE_STATUS,
    SCOPE_VORTEX_RPC_GIDS_QUEUE_SET_STATE,
};

struct scope_vortex_rpc_hello_req {
    uint32_t flags;
    uint32_t reserved;
} QEMU_PACKED;

struct scope_vortex_rpc_hello_rsp {
    uint32_t version;
    uint32_t capabilities;
} QEMU_PACKED;

struct scope_vortex_rpc_peer_caps {
    uint32_t flags;
    uint32_t reserved;
    uint64_t host_base;
    uint64_t control_size;
    uint64_t peer_base;
    uint64_t peer_size;
    uint64_t slot_size;
    uint64_t generation;
} QEMU_PACKED;

struct scope_vortex_rpc_peer_map_req {
    uint16_t domain;
    uint8_t bus;
    uint8_t devfn;
    uint8_t bar;
    uint8_t reserved[7];
    uint64_t bar_offset;
    uint64_t window_size;
} QEMU_PACKED;

struct scope_vortex_rpc_peer_map_rsp {
    uint64_t cp_peer_base;
    uint64_t window_size;
    uint64_t slot_size;
    uint64_t generation;
} QEMU_PACKED;

struct scope_vortex_rpc_peer_unmap {
    uint64_t generation;
} QEMU_PACKED;

struct scope_vortex_rpc_gds_caps_req {
    uint16_t domain;
    uint8_t bus;
    uint8_t devfn;
    uint32_t reserved;
} QEMU_PACKED;

struct scope_vortex_rpc_gds_caps_rsp {
    uint32_t flags;
    uint32_t alignment;
    uint64_t max_size;
} QEMU_PACKED;

struct scope_vortex_rpc_gds_alloc_req {
    uint64_t size;
    uint64_t alignment;
} QEMU_PACKED;

struct scope_vortex_rpc_gds_alloc_rsp {
    uint32_t handle;
    uint32_t flags;
    uint64_t hbm_addr;
    uint64_t p2p_bus_addr;
    uint64_t size;
    uint64_t generation;
} QEMU_PACKED;

struct scope_vortex_rpc_gds_free {
    uint32_t handle;
    uint32_t reserved;
    uint64_t generation;
} QEMU_PACKED;

struct scope_vortex_rpc_gids_queue_create_req {
    uint32_t depth;
    uint32_t features;
    uint64_t payload_size;
    uint64_t session_cookie;
} QEMU_PACKED;

struct scope_vortex_rpc_gids_queue_create_rsp {
    uint32_t handle;
    uint32_t depth;
    uint64_t generation;
    uint64_t hbm_addr;
    uint64_t p2p_bus_addr;
    uint64_t total_size;
    uint64_t sq_offset;
    uint64_t cq_offset;
    uint64_t payload_offset;
    uint64_t payload_size;
} QEMU_PACKED;

struct scope_vortex_rpc_gids_batch_req {
    uint32_t handle;
    uint32_t max_entries;
    uint64_t generation;
    uint32_t start_counter;
    uint32_t reserved;
} QEMU_PACKED;

struct scope_vortex_rpc_gids_batch_rsp {
    uint32_t producer;
    uint32_t consumer;
    uint32_t count;
    uint32_t reserved;
    uint64_t generation;
} QEMU_PACKED;

struct scope_vortex_rpc_gids_commit_req {
    uint32_t handle;
    uint32_t count;
    uint64_t generation;
    uint32_t start_counter;
    uint32_t reserved;
} QEMU_PACKED;

struct scope_vortex_rpc_gids_status_req {
    uint32_t handle;
    uint32_t reserved;
    uint64_t generation;
} QEMU_PACKED;

struct scope_vortex_rpc_gids_state_req {
    uint32_t handle;
    uint32_t state;
    uint64_t generation;
    uint32_t fatal_error;
    uint32_t reserved;
} QEMU_PACKED;

struct scope_vortex_rpc_header {
    uint32_t magic;
    uint16_t version;
    uint16_t opcode;
    uint32_t request_id;
    uint32_t payload_len;
    int32_t status;
} QEMU_PACKED;

struct scope_vortex_rpc_reg {
    uint32_t offset;
    uint32_t value;
} QEMU_PACKED;

struct scope_vortex_rpc_alloc_req {
    uint64_t size;
} QEMU_PACKED;

struct scope_vortex_rpc_alloc_rsp {
    uint32_t handle;
    uint32_t reserved;
    uint64_t cp_addr;
    uint64_t size;
} QEMU_PACKED;

struct scope_vortex_rpc_mem {
    uint32_t handle;
    uint32_t length;
    uint64_t offset;
} QEMU_PACKED;

_Static_assert(sizeof(struct scope_vortex_rpc_header) == 20, "RPC header ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_hello_req) == 8, "RPC hello request ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_hello_rsp) == 8, "RPC hello response ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_peer_caps) == 56, "RPC peer caps ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_peer_map_req) == 28, "RPC peer map request ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_peer_map_rsp) == 32, "RPC peer map response ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_peer_unmap) == 8, "RPC peer unmap ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_gds_caps_req) == 8, "RPC GDS caps request ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_gds_caps_rsp) == 16, "RPC GDS caps response ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_gds_alloc_req) == 16, "RPC GDS alloc request ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_gds_alloc_rsp) == 40, "RPC GDS alloc response ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_gds_free) == 16, "RPC GDS free ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_gids_queue_create_req) == 24, "RPC GIDS create request ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_gids_queue_create_rsp) == 72, "RPC GIDS create response ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_gids_batch_req) == 24, "RPC GIDS batch request ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_gids_batch_rsp) == 24, "RPC GIDS batch response ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_gids_commit_req) == 24, "RPC GIDS commit request ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_gids_status_req) == 16, "RPC GIDS status request ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_gids_state_req) == 24, "RPC GIDS state request ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_reg) == 8, "RPC register ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_alloc_req) == 8, "RPC alloc request ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_alloc_rsp) == 24, "RPC alloc response ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_mem) == 16, "RPC memory ABI");

#endif
