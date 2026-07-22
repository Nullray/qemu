/* SPDX-License-Identifier: Apache-2.0 */
#ifndef HW_MISC_SCOPE_VORTEX_BRIDGE_PROTO_H
#define HW_MISC_SCOPE_VORTEX_BRIDGE_PROTO_H

#include <stdint.h>

#define SCOPE_VORTEX_RPC_MAGIC       0x56585250U
#define SCOPE_VORTEX_RPC_VERSION     1U
#define SCOPE_VORTEX_RPC_MAX_PAYLOAD (16U * 1024U * 1024U)

enum scope_vortex_rpc_op {
    SCOPE_VORTEX_RPC_HELLO = 1,
    SCOPE_VORTEX_RPC_CP_READ,
    SCOPE_VORTEX_RPC_CP_WRITE,
    SCOPE_VORTEX_RPC_MEM_ALLOC,
    SCOPE_VORTEX_RPC_MEM_FREE,
    SCOPE_VORTEX_RPC_MEM_READ,
    SCOPE_VORTEX_RPC_MEM_WRITE,
};

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
_Static_assert(sizeof(struct scope_vortex_rpc_reg) == 8, "RPC register ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_alloc_req) == 8, "RPC alloc request ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_alloc_rsp) == 24, "RPC alloc response ABI");
_Static_assert(sizeof(struct scope_vortex_rpc_mem) == 16, "RPC memory ABI");

#endif
