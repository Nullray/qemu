/* SPDX-License-Identifier: Apache-2.0 */
#ifndef HW_MISC_SCOPE_VORTEX_BRIDGE_PROTO_H
#define HW_MISC_SCOPE_VORTEX_BRIDGE_PROTO_H

#include <stdint.h>

#define SCOPE_VORTEX_RPC_MAGIC       0x56585250U
#define SCOPE_VORTEX_RPC_VERSION     2U
#define SCOPE_VORTEX_RPC_CAP_PEER_MAP (1U << 0)
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
_Static_assert(sizeof(struct scope_vortex_rpc_reg) == 8, "RPC register ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_alloc_req) == 8, "RPC alloc request ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_alloc_rsp) == 24, "RPC alloc response ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_mem) == 16, "RPC memory ABI");

#endif
