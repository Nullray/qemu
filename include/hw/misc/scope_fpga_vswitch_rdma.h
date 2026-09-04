/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_MISC_SCOPE_FPGA_VSWITCH_RDMA_H
#define HW_MISC_SCOPE_FPGA_VSWITCH_RDMA_H

#include "qapi/error.h"
#include "hw/misc/scope_remote_proto.h"

typedef struct ScopeRemoteState ScopeRemoteState;

typedef enum ScopeRemoteMemoryMode {
    SCOPE_REMOTE_MEMORY_PEER_DMABUF = 0,
    SCOPE_REMOTE_MEMORY_HOST_STAGING,
    SCOPE_REMOTE_MEMORY_INLINE,
} ScopeRemoteMemoryMode;

typedef struct ScopeRemoteConfig {
    const char *host;
    const char *service;
    const char *device_id;
    const char *rdma_device;
    const char *peer_memory_device;
    ScopeRemoteMemoryMode memory_mode;
    uint32_t host_staging_size;
    uint32_t namespace_id;
    uint32_t connect_timeout_ms;
    uint32_t request_timeout_ms;
    uint64_t guest_ddr_base;
    uint64_t guest_ddr_size;
    uint64_t coherent_alias_base;
    uint32_t device_type;
    bool ixgbe_shadow_ring;
} ScopeRemoteConfig;

typedef enum ScopeRemoteEventType {
    SCOPE_REMOTE_EVENT_NONE = 0,
    SCOPE_REMOTE_EVENT_BAR_RESPONSE,
    SCOPE_REMOTE_EVENT_NVME_COMPLETE,
    SCOPE_REMOTE_EVENT_IXGBE_TX_COMPLETE,
    SCOPE_REMOTE_EVENT_IXGBE_RX,
    SCOPE_REMOTE_EVENT_IXGBE_LINK,
    SCOPE_REMOTE_EVENT_FAILED,
} ScopeRemoteEventType;

typedef struct ScopeRemoteEvent {
    ScopeRemoteEventType type;
    uint64_t request_id;
    int32_t status;
    union {
        struct scope_remote_bar bar;
        struct scope_remote_nvme_complete nvme;
        struct scope_remote_ether_frame frame;
        struct scope_remote_ether_complete tx_complete;
        struct scope_remote_link_state link;
    } u;
} ScopeRemoteEvent;

#ifdef CONFIG_RDMA
ScopeRemoteState *scope_remote_connect(const ScopeRemoteConfig *config,
                                       struct scope_remote_device_info *info,
                                       Error **errp);
void scope_remote_disconnect(ScopeRemoteState *remote);
bool scope_remote_poll(ScopeRemoteState *remote, ScopeRemoteEvent *event,
                       Error **errp);
bool scope_remote_submit_bar(ScopeRemoteState *remote, bool write,
                             const struct scope_remote_bar *bar,
                             uint64_t request_id, Error **errp);
bool scope_remote_submit_nvme(ScopeRemoteState *remote,
                              const struct scope_remote_nvme_submit *submit,
                              uint64_t request_id, Error **errp);
bool scope_remote_submit_ixgbe_tx(ScopeRemoteState *remote,
                                  const uint8_t *data, uint32_t length,
                                  uint64_t request_id, Error **errp);
uint64_t scope_remote_session_id(const ScopeRemoteState *remote);
uint64_t scope_remote_generation(const ScopeRemoteState *remote);
bool scope_remote_uses_host_staging(const ScopeRemoteState *remote);
void *scope_remote_staging_data(ScopeRemoteState *remote);
uint64_t scope_remote_payload_base(const ScopeRemoteState *remote);
uint32_t scope_remote_payload_size(const ScopeRemoteState *remote);
uint32_t scope_remote_request_timeout_ms(const ScopeRemoteState *remote);
#else
static inline ScopeRemoteState *
scope_remote_connect(const ScopeRemoteConfig *config,
                     struct scope_remote_device_info *info, Error **errp)
{
    error_setg(errp,
               "remote-rdma requires QEMU configured with --enable-rdma");
    return NULL;
}

static inline void scope_remote_disconnect(ScopeRemoteState *remote)
{
}

static inline bool scope_remote_poll(ScopeRemoteState *remote,
                                     ScopeRemoteEvent *event, Error **errp)
{
    return false;
}

static inline bool scope_remote_submit_bar(
    ScopeRemoteState *remote, bool write, const struct scope_remote_bar *bar,
    uint64_t request_id, Error **errp)
{
    error_setg(errp, "remote-rdma is not compiled in");
    return false;
}

static inline bool scope_remote_submit_nvme(
    ScopeRemoteState *remote, const struct scope_remote_nvme_submit *submit,
    uint64_t request_id, Error **errp)
{
    error_setg(errp, "remote-rdma is not compiled in");
    return false;
}

static inline bool scope_remote_submit_ixgbe_tx(
    ScopeRemoteState *remote, const uint8_t *data, uint32_t length,
    uint64_t request_id, Error **errp)
{
    error_setg(errp, "remote-rdma is not compiled in");
    return false;
}

static inline uint64_t scope_remote_session_id(const ScopeRemoteState *remote)
{
    return 0;
}

static inline uint64_t scope_remote_generation(const ScopeRemoteState *remote)
{
    return 0;
}

static inline bool
scope_remote_uses_host_staging(const ScopeRemoteState *remote)
{
    return false;
}

static inline void *scope_remote_staging_data(ScopeRemoteState *remote)
{
    return NULL;
}

static inline uint64_t
scope_remote_payload_base(const ScopeRemoteState *remote)
{
    return 0;
}

static inline uint32_t
scope_remote_payload_size(const ScopeRemoteState *remote)
{
    return 0;
}

static inline uint32_t
scope_remote_request_timeout_ms(const ScopeRemoteState *remote)
{
    return 0;
}
#endif

#endif
