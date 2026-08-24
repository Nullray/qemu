/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * RDMA transport for scope-fpga-vswitch remote backends.
 *
 * This file deliberately knows nothing about the vSwitch manager's private
 * structures.  It owns the RC QP, the XDMA peer dma-buf MR and the wire
 * protocol.  Device semantics remain in the NVMe backend.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "qemu/memalign.h"
#include "qapi/error.h"
#include "hw/misc/scope_fpga_vswitch_rdma.h"
#include "hw/misc/scope_xdma_peer_uapi.h"

#include <infiniband/verbs.h>
#include <rdma/rdma_cma.h>
#include <rdma/rdma_verbs.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define SCOPE_REMOTE_DEFAULT_CONNECT_TIMEOUT_MS 5000U
#define SCOPE_REMOTE_DEFAULT_REQUEST_TIMEOUT_MS 5000U
#define SCOPE_REMOTE_TX_QUEUE_DEPTH 128U
#define SCOPE_REMOTE_DEFAULT_HOST_STAGING (4U * 1024U * 1024U)
#define SCOPE_REMOTE_MAX_HOST_STAGING     (64U * 1024U * 1024U)

typedef struct ScopeRemoteTxItem {
    struct scope_remote_message msg;
    size_t length;
} ScopeRemoteTxItem;

struct ScopeRemoteState {
    struct rdma_cm_id *id;
    struct ibv_mr *tx_mr;
    struct ibv_mr *rx_mr;
    struct ibv_mr *peer_mr;
    void *host_staging;
    ScopeRemoteMemoryMode memory_mode;
    uint64_t payload_base;
    uint32_t payload_size;
    struct scope_remote_message *tx;
    struct scope_remote_message *rx;
    int peer_fd;
    int dmabuf_fd;
    bool recv_posted;
    bool tx_inflight;
    GQueue tx_pending;
    bool failed;
    uint64_t session_id;
    uint64_t generation;
    uint64_t next_request_id;
    uint32_t connect_timeout_ms;
    uint32_t request_timeout_ms;
};

static uint64_t scope_remote_now_ms(void)
{
    return g_get_monotonic_time() / 1000;
}

static void scope_remote_init_hdr(ScopeRemoteState *r,
                                  struct scope_remote_message *msg,
                                  uint16_t opcode, uint32_t payload_len,
                                  uint64_t request_id)
{
    memset(msg, 0, sizeof(*msg));
    msg->hdr.magic = cpu_to_le32(SCOPE_REMOTE_MAGIC);
    msg->hdr.major = cpu_to_le16(SCOPE_REMOTE_PROTO_MAJOR);
    msg->hdr.minor = cpu_to_le16(SCOPE_REMOTE_PROTO_MINOR);
    msg->hdr.opcode = cpu_to_le16(opcode);
    msg->hdr.payload_len = cpu_to_le32(payload_len);
    msg->hdr.session_id = cpu_to_le64(r->session_id);
    msg->hdr.request_id = cpu_to_le64(request_id);
    msg->hdr.generation = cpu_to_le64(r->generation);
}

static bool scope_remote_validate_message(ScopeRemoteState *r,
                                          const struct scope_remote_message *msg,
                                          uint16_t expected_opcode,
                                          uint64_t expected_request,
                                          Error **errp)
{
    uint32_t payload_len = le32_to_cpu(msg->hdr.payload_len);

    if (le32_to_cpu(msg->hdr.magic) != SCOPE_REMOTE_MAGIC ||
        le16_to_cpu(msg->hdr.major) != SCOPE_REMOTE_PROTO_MAJOR ||
        le16_to_cpu(msg->hdr.minor) > SCOPE_REMOTE_PROTO_MINOR ||
        payload_len > sizeof(msg->payload) ||
        le32_to_cpu(msg->hdr.reserved) != 0) {
        error_setg(errp, "remote-rdma received an invalid protocol header");
        return false;
    }
    if (le64_to_cpu(msg->hdr.session_id) != r->session_id ||
        le64_to_cpu(msg->hdr.generation) != r->generation) {
        error_setg(errp, "remote-rdma session/generation mismatch");
        return false;
    }
    if (expected_opcode && le16_to_cpu(msg->hdr.opcode) != expected_opcode) {
        error_setg(errp, "remote-rdma expected opcode %u, received %u",
                   expected_opcode, le16_to_cpu(msg->hdr.opcode));
        return false;
    }
    if (expected_request &&
        le64_to_cpu(msg->hdr.request_id) != expected_request) {
        error_setg(errp, "remote-rdma request id mismatch");
        return false;
    }
    if (expected_opcode &&
        (int32_t)le32_to_cpu(msg->hdr.status) != SCOPE_REMOTE_STATUS_OK) {
        error_setg(errp, "remote-rdma peer returned status %d",
                   (int32_t)le32_to_cpu(msg->hdr.status));
        return false;
    }
    return true;
}

static bool scope_remote_poll_wc(struct ibv_cq *cq, struct ibv_wc *wc,
                                 uint64_t deadline_ms, Error **errp)
{
    int ret;

    do {
        ret = ibv_poll_cq(cq, 1, wc);
        if (ret < 0) {
            error_setg_errno(errp, errno, "remote-rdma CQ poll failed");
            return false;
        }
        if (ret == 1) {
            if (wc->status != IBV_WC_SUCCESS) {
                error_setg(errp, "remote-rdma work completion failed: %s",
                           ibv_wc_status_str(wc->status));
                return false;
            }
            return true;
        }
        g_usleep(50);
    } while (scope_remote_now_ms() < deadline_ms);

    error_setg(errp, "remote-rdma work completion timed out");
    return false;
}

static bool scope_remote_post_recv(ScopeRemoteState *r, Error **errp)
{
    if (rdma_post_recv(r->id, r->rx, r->rx, sizeof(*r->rx), r->rx_mr)) {
        error_setg_errno(errp, errno, "remote-rdma failed to post receive");
        return false;
    }
    r->recv_posted = true;
    return true;
}

static bool scope_remote_send(ScopeRemoteState *r, size_t length, Error **errp)
{
    if (r->failed || r->tx_inflight) {
        error_setg(errp, "remote-rdma send queue is unavailable");
        return false;
    }
    if (length > sizeof(*r->tx)) {
        error_setg(errp, "remote-rdma control message is too large");
        return false;
    }
    if (rdma_post_send(r->id, r->tx, r->tx, length, r->tx_mr,
                       IBV_SEND_SIGNALED)) {
        error_setg_errno(errp, errno, "remote-rdma failed to post send");
        return false;
    }
    r->tx_inflight = true;
    return true;
}

static bool scope_remote_wait_send(ScopeRemoteState *r, uint64_t deadline_ms,
                                   Error **errp)
{
    struct ibv_wc wc;

    if (!r->tx_inflight) {
        return true;
    }
    if (!scope_remote_poll_wc(r->id->send_cq, &wc, deadline_ms, errp)) {
        r->failed = true;
        return false;
    }
    r->tx_inflight = false;
    return true;
}

static bool scope_remote_wait_recv(ScopeRemoteState *r, uint64_t deadline_ms,
                                   Error **errp)
{
    struct ibv_wc wc;

    if (!r->recv_posted) {
        error_setg(errp, "remote-rdma has no posted receive");
        return false;
    }
    if (!scope_remote_poll_wc(r->id->recv_cq, &wc, deadline_ms, errp)) {
        r->failed = true;
        return false;
    }
    r->recv_posted = false;
    return true;
}

static bool scope_remote_exchange(ScopeRemoteState *r, size_t tx_len,
                                  uint16_t response_opcode,
                                  uint64_t request_id, Error **errp)
{
    uint64_t deadline = scope_remote_now_ms() + r->request_timeout_ms;

    if (!scope_remote_send(r, tx_len, errp) ||
        !scope_remote_wait_send(r, deadline, errp) ||
        !scope_remote_wait_recv(r, deadline, errp) ||
        !scope_remote_validate_message(r, r->rx, response_opcode,
                                       request_id, errp)) {
        return false;
    }
    return true;
}

static bool scope_remote_register_payload_mr(ScopeRemoteState *r,
                                             const ScopeRemoteConfig *config,
                                             Error **errp)
{
    struct xdma_peer_export export = {
        .abi_version = XDMA_PEER_ABI_VERSION,
        .flags = XDMA_PEER_F_READ | XDMA_PEER_F_WRITE,
        .bar_offset = config->coherent_alias_base + config->guest_ddr_base,
        .length = config->guest_ddr_size,
        .guest_iova = config->guest_ddr_base,
        .dmabuf_fd = -1,
    };
    int access = IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_READ |
                 IBV_ACCESS_REMOTE_WRITE;

    r->memory_mode = config->memory_mode;
    if (r->memory_mode == SCOPE_REMOTE_MEMORY_HOST_STAGING) {
        uint32_t size = config->host_staging_size ?
            config->host_staging_size : SCOPE_REMOTE_DEFAULT_HOST_STAGING;

        if (size < 4096 || size > SCOPE_REMOTE_MAX_HOST_STAGING ||
            (size & 4095U)) {
            error_setg(errp, "remote-rdma host staging size must be a "
                       "4 KiB multiple in [4 KiB, 64 MiB]");
            return false;
        }
        r->host_staging = qemu_memalign(4096, size);
        memset(r->host_staging, 0, size);
        r->peer_mr = ibv_reg_mr(r->id->pd, r->host_staging, size, access);
        if (!r->peer_mr) {
            error_setg_errno(errp, errno,
                             "RNIC rejected Host A staging memory MR");
            return false;
        }
        r->payload_base = (uintptr_t)r->host_staging;
        r->payload_size = size;
        r->generation = ((uint64_t)g_random_int() << 32) | g_random_int();
        if (!r->generation) {
            r->generation = 1;
        }
        return true;
    }

    if (r->memory_mode != SCOPE_REMOTE_MEMORY_PEER_DMABUF) {
        error_setg(errp, "remote-rdma has an invalid memory mode");
        return false;
    }

    r->peer_fd = open(config->peer_memory_device, O_RDWR | O_CLOEXEC);
    if (r->peer_fd < 0) {
        error_setg_errno(errp, errno, "cannot open XDMA peer-memory device %s",
                         config->peer_memory_device);
        return false;
    }
    if (ioctl(r->peer_fd, XDMA_PEER_IOC_EXPORT, &export)) {
        error_setg_errno(errp, errno,
                         "XDMA peer-memory export failed (CONFIG_PCI_P2PDMA "
                         "and a P2P-capable RNIC are required)");
        return false;
    }
    if (export.abi_version != XDMA_PEER_ABI_VERSION ||
        export.dmabuf_fd < 0 || export.length != config->guest_ddr_size ||
        export.guest_iova != config->guest_ddr_base ||
        export.bar_offset != config->coherent_alias_base +
                             config->guest_ddr_base ||
        !export.generation) {
        if (export.dmabuf_fd >= 0) {
            close(export.dmabuf_fd);
        }
        error_setg(errp, "XDMA peer-memory metadata does not match guest DDR");
        return false;
    }

    r->dmabuf_fd = export.dmabuf_fd;
    r->generation = export.generation;
    r->peer_mr = ibv_reg_dmabuf_mr(r->id->pd, 0, export.length,
                                   export.guest_iova, r->dmabuf_fd, access);
    if (!r->peer_mr) {
        error_setg_errno(errp, errno,
                         "RNIC rejected the XDMA peer dma-buf MR");
        return false;
    }
    r->payload_base = export.guest_iova;
    r->payload_size = export.length;
    return true;
}

static bool scope_remote_handshake(ScopeRemoteState *r,
                                   const ScopeRemoteConfig *config,
                                   struct scope_remote_device_info *info,
                                   Error **errp)
{
    struct scope_remote_hello *hello;
    struct scope_remote_hello *hello_rsp;
    struct scope_remote_device_open *open_req;
    struct scope_remote_device_info *open_rsp;
    uint64_t request_id;
    uint64_t features;
    uint64_t required_features;

    if (config->device_type == SCOPE_REMOTE_DEVICE_IXGBE_PACKET) {
        if (r->memory_mode != SCOPE_REMOTE_MEMORY_HOST_STAGING) {
            error_setg(errp, "remote ixgbe packet transport requires "
                       "host-staging mode");
            return false;
        }
        required_features = SCOPE_REMOTE_IXGBE_FEATURES;
    } else {
        required_features =
            r->memory_mode == SCOPE_REMOTE_MEMORY_HOST_STAGING ?
            SCOPE_REMOTE_STAGING_FEATURES : SCOPE_REMOTE_DIRECT_FEATURES;
    }

    request_id = r->next_request_id++;
    scope_remote_init_hdr(r, r->tx, SCOPE_REMOTE_OP_HELLO,
                          sizeof(*hello), request_id);
    hello = (void *)r->tx->payload;
    hello->features = cpu_to_le64(required_features);
    hello->peer_base = cpu_to_le64(r->payload_base);
    hello->peer_length = cpu_to_le64(r->payload_size);
    hello->peer_rkey = cpu_to_le32(r->peer_mr->rkey);
    hello->max_inflight = cpu_to_le32(64);
    hello->max_segments = cpu_to_le32(SCOPE_REMOTE_MAX_SEGMENTS);
    if (!scope_remote_exchange(r, sizeof(r->tx->hdr) + sizeof(*hello),
                               SCOPE_REMOTE_OP_HELLO_RSP, request_id, errp)) {
        return false;
    }
    if (le32_to_cpu(r->rx->hdr.payload_len) != sizeof(*hello_rsp)) {
        error_setg(errp, "remote-rdma HELLO response has invalid length");
        return false;
    }
    hello_rsp = (void *)r->rx->payload;
    features = le64_to_cpu(hello_rsp->features);
    if ((features & required_features) != required_features) {
        error_setg(errp, "remote-rdma agent lacks required payload-memory "
                   "features for the selected mode");
        return false;
    }
    if (!scope_remote_post_recv(r, errp)) {
        return false;
    }

    request_id = r->next_request_id++;
    scope_remote_init_hdr(r, r->tx, SCOPE_REMOTE_OP_DEVICE_OPEN,
                          sizeof(*open_req), request_id);
    open_req = (void *)r->tx->payload;
    pstrcpy(open_req->device_id, sizeof(open_req->device_id),
            config->device_id);
    open_req->namespace_id = cpu_to_le32(config->namespace_id);
    open_req->device_type = cpu_to_le32(config->device_type);
    if (!scope_remote_exchange(r, sizeof(r->tx->hdr) + sizeof(*open_req),
                               SCOPE_REMOTE_OP_DEVICE_OPEN_RSP,
                               request_id, errp)) {
        return false;
    }
    if (le32_to_cpu(r->rx->hdr.payload_len) != sizeof(*open_rsp)) {
        error_setg(errp, "remote-rdma DEVICE_OPEN response has invalid length");
        return false;
    }
    open_rsp = (void *)r->rx->payload;
    *info = (struct scope_remote_device_info) {
        .vendor_id = le16_to_cpu(open_rsp->vendor_id),
        .device_id = le16_to_cpu(open_rsp->device_id),
        .class_code = le32_to_cpu(open_rsp->class_code),
        .bar0_size = le64_to_cpu(open_rsp->bar0_size),
        .nvme_cap = le64_to_cpu(open_rsp->nvme_cap),
        .nvme_vs = le32_to_cpu(open_rsp->nvme_vs),
        .controller_page_size =
            le32_to_cpu(open_rsp->controller_page_size),
        .namespace_blocks = le64_to_cpu(open_rsp->namespace_blocks),
        .lba_shift = le32_to_cpu(open_rsp->lba_shift),
        .max_transfer_bytes = le32_to_cpu(open_rsp->max_transfer_bytes),
        .max_inflight = le32_to_cpu(open_rsp->max_inflight),
        .device_type = le32_to_cpu(open_rsp->device_type),
        .mtu = le32_to_cpu(open_rsp->mtu),
        .link_up = open_rsp->link_up,
    };
    memcpy(info->mac, open_rsp->mac, sizeof(info->mac));
    if (info->device_type != config->device_type) {
        error_setg(errp, "remote-rdma agent returned the wrong device type");
        return false;
    }
    if (config->device_type == SCOPE_REMOTE_DEVICE_IXGBE_PACKET) {
        if (info->vendor_id != 0x8086 || info->device_id != 0x10fb ||
            info->class_code != 0x020000 || info->bar0_size != 0x20000 ||
            info->mtu < 576 || info->mtu > 1500 ||
            (info->mac[0] & 1) || !memcmp(info->mac, "\0\0\0\0\0\0", 6)) {
            error_setg(errp,
                       "remote-rdma agent returned invalid 82599 capabilities");
            return false;
        }
    } else if (!info->bar0_size ||
               (info->bar0_size & (info->bar0_size - 1)) ||
               !info->nvme_cap || info->controller_page_size < 4096 ||
               info->lba_shift < 9 || info->lba_shift > 30 ||
               !info->max_transfer_bytes || !info->max_inflight) {
        error_setg(errp, "remote-rdma agent returned invalid NVMe capabilities");
        return false;
    }
    return scope_remote_post_recv(r, errp);
}

ScopeRemoteState *scope_remote_connect(const ScopeRemoteConfig *config,
                                       struct scope_remote_device_info *info,
                                       Error **errp)
{
    struct rdma_addrinfo hints = {
        .ai_port_space = RDMA_PS_TCP,
        .ai_qp_type = IBV_QPT_RC,
    };
    struct rdma_addrinfo *res = NULL;
    struct ibv_qp_init_attr qp_attr = {
        .cap = {
            .max_send_wr = 128,
            .max_recv_wr = 128,
            .max_send_sge = 1,
            .max_recv_sge = 1,
        },
        .qp_type = IBV_QPT_RC,
        .sq_sig_all = 0,
    };
    struct rdma_conn_param conn = {
        .responder_resources = 4,
        .initiator_depth = 4,
        .retry_count = 7,
        .rnr_retry_count = 7,
    };
    ScopeRemoteState *r = g_new0(ScopeRemoteState, 1);
    int ret;

    r->peer_fd = -1;
    r->dmabuf_fd = -1;
    g_queue_init(&r->tx_pending);
    r->connect_timeout_ms = config->connect_timeout_ms ?
        config->connect_timeout_ms : SCOPE_REMOTE_DEFAULT_CONNECT_TIMEOUT_MS;
    r->request_timeout_ms = config->request_timeout_ms ?
        config->request_timeout_ms : SCOPE_REMOTE_DEFAULT_REQUEST_TIMEOUT_MS;
    r->session_id = ((uint64_t)g_random_int() << 32) | g_random_int();
    if (!r->session_id) {
        r->session_id = 1;
    }
    r->next_request_id = 1;

    ret = rdma_getaddrinfo((char *)config->host, (char *)config->service,
                           &hints, &res);
    if (ret) {
        error_setg(errp, "cannot resolve RDMA endpoint %s:%s: %s",
                   config->host, config->service, gai_strerror(ret));
        goto fail;
    }
    ret = rdma_create_ep(&r->id, res, NULL, &qp_attr);
    rdma_freeaddrinfo(res);
    res = NULL;
    if (ret) {
        error_setg_errno(errp, errno, "cannot create RDMA endpoint");
        goto fail;
    }
    if (config->rdma_device && config->rdma_device[0] &&
        strcmp(ibv_get_device_name(r->id->verbs->device),
               config->rdma_device)) {
        error_setg(errp, "RDMA route selected %s, expected %s",
                   ibv_get_device_name(r->id->verbs->device),
                   config->rdma_device);
        goto fail;
    }

    r->tx = qemu_memalign(64, sizeof(*r->tx));
    r->rx = qemu_memalign(64, sizeof(*r->rx));
    memset(r->tx, 0, sizeof(*r->tx));
    memset(r->rx, 0, sizeof(*r->rx));
    r->tx_mr = rdma_reg_msgs(r->id, r->tx, sizeof(*r->tx));
    r->rx_mr = rdma_reg_msgs(r->id, r->rx, sizeof(*r->rx));
    if (!r->tx_mr || !r->rx_mr) {
        error_setg_errno(errp, errno,
                         "cannot register RDMA control message buffers");
        goto fail;
    }
    if (!scope_remote_register_payload_mr(r, config, errp) ||
        !scope_remote_post_recv(r, errp)) {
        goto fail;
    }
    if (rdma_connect(r->id, &conn)) {
        error_setg_errno(errp, errno, "cannot connect RDMA endpoint %s:%s",
                         config->host, config->service);
        goto fail;
    }
    if (!scope_remote_handshake(r, config, info, errp)) {
        goto fail;
    }
    return r;

fail:
    if (res) {
        rdma_freeaddrinfo(res);
    }
    scope_remote_disconnect(r);
    return NULL;
}

void scope_remote_disconnect(ScopeRemoteState *r)
{
    if (!r) {
        return;
    }
    if (r->id && r->id->qp) {
        rdma_disconnect(r->id);
    }
    if (r->peer_mr) {
        ibv_dereg_mr(r->peer_mr);
    }
    if (r->tx_mr) {
        rdma_dereg_mr(r->tx_mr);
    }
    if (r->rx_mr) {
        rdma_dereg_mr(r->rx_mr);
    }
    if (r->id) {
        rdma_destroy_ep(r->id);
    }
    if (r->dmabuf_fd >= 0) {
        close(r->dmabuf_fd);
    }
    if (r->peer_fd >= 0) {
        close(r->peer_fd);
    }
    g_queue_clear_full(&r->tx_pending, g_free);
    qemu_vfree(r->tx);
    qemu_vfree(r->rx);
    qemu_vfree(r->host_staging);
    g_free(r);
}

bool scope_remote_uses_host_staging(const ScopeRemoteState *r)
{
    return r && r->memory_mode == SCOPE_REMOTE_MEMORY_HOST_STAGING;
}

void *scope_remote_staging_data(ScopeRemoteState *r)
{
    return scope_remote_uses_host_staging(r) ? r->host_staging : NULL;
}

uint64_t scope_remote_payload_base(const ScopeRemoteState *r)
{
    return r ? r->payload_base : 0;
}

uint32_t scope_remote_payload_size(const ScopeRemoteState *r)
{
    return r ? r->payload_size : 0;
}

uint32_t scope_remote_request_timeout_ms(const ScopeRemoteState *r)
{
    return r ? r->request_timeout_ms : 0;
}

static bool scope_remote_submit_message(ScopeRemoteState *r, uint16_t opcode,
                                        const void *payload, size_t payload_len,
                                        uint64_t request_id, Error **errp)
{
    ScopeRemoteTxItem *item;

    if (payload_len > sizeof(r->tx->payload)) {
        error_setg(errp, "remote-rdma payload is too large");
        return false;
    }
    if (r->failed || g_queue_get_length(&r->tx_pending) >=
                     SCOPE_REMOTE_TX_QUEUE_DEPTH) {
        error_setg(errp, "remote-rdma transmit queue is full or failed");
        return false;
    }
    item = g_new0(ScopeRemoteTxItem, 1);
    scope_remote_init_hdr(r, &item->msg, opcode, payload_len, request_id);
    memcpy(item->msg.payload, payload, payload_len);
    item->length = sizeof(item->msg.hdr) + payload_len;
    g_queue_push_tail(&r->tx_pending, item);

    if (!r->tx_inflight) {
        item = g_queue_pop_head(&r->tx_pending);
        memcpy(r->tx, &item->msg, item->length);
        if (!scope_remote_send(r, item->length, errp)) {
            g_free(item);
            r->failed = true;
            return false;
        }
        g_free(item);
    }
    return true;
}

bool scope_remote_submit_bar(ScopeRemoteState *r, bool write,
                             const struct scope_remote_bar *bar,
                             uint64_t request_id, Error **errp)
{
    struct scope_remote_bar wire = {
        .offset = cpu_to_le32(bar->offset),
        .size = cpu_to_le32(bar->size),
        .value = cpu_to_le64(bar->value),
        .wstrb = cpu_to_le32(bar->wstrb),
    };

    return scope_remote_submit_message(r,
        write ? SCOPE_REMOTE_OP_BAR_WRITE : SCOPE_REMOTE_OP_BAR_READ,
        &wire, sizeof(wire), request_id, errp);
}

bool scope_remote_submit_nvme(ScopeRemoteState *r,
                              const struct scope_remote_nvme_submit *submit,
                              uint64_t request_id, Error **errp)
{
    struct scope_remote_nvme_submit wire = *submit;
    unsigned int i;

    wire.qid = cpu_to_le32(submit->qid);
    wire.direction = cpu_to_le32(submit->direction);
    wire.data_len = cpu_to_le32(submit->data_len);
    wire.segment_count = cpu_to_le32(submit->segment_count);
    for (i = 0; i < submit->segment_count &&
                i < SCOPE_REMOTE_MAX_SEGMENTS; i++) {
        wire.segments[i].guest_pa = cpu_to_le64(submit->segments[i].guest_pa);
        wire.segments[i].length = cpu_to_le32(submit->segments[i].length);
        wire.segments[i].reserved = 0;
    }
    return scope_remote_submit_message(r, SCOPE_REMOTE_OP_NVME_SUBMIT,
                                       &wire, sizeof(wire), request_id, errp);
}

bool scope_remote_submit_ixgbe_tx(ScopeRemoteState *r,
                                  const uint8_t *data, uint32_t length,
                                  uint64_t request_id, Error **errp)
{
    struct scope_remote_ether_frame wire = { 0 };

    if (!length || length > SCOPE_REMOTE_ETHER_MAX_FRAME) {
        error_setg(errp, "remote ixgbe frame length %u is invalid", length);
        return false;
    }
    wire.length = cpu_to_le32(length);
    memcpy(wire.data, data, length);
    return scope_remote_submit_message(r, SCOPE_REMOTE_OP_IXGBE_TX,
                                       &wire, sizeof(wire), request_id, errp);
}

bool scope_remote_poll(ScopeRemoteState *r, ScopeRemoteEvent *event,
                       Error **errp)
{
    struct ibv_wc wc;
    uint32_t payload_len;
    uint16_t opcode;
    int ret;

    memset(event, 0, sizeof(*event));
    if (!r || r->failed) {
        return false;
    }
    if (r->tx_inflight) {
        ret = ibv_poll_cq(r->id->send_cq, 1, &wc);
        if (ret < 0 || (ret == 1 && wc.status != IBV_WC_SUCCESS)) {
            error_setg(errp, "remote-rdma send completion failed");
            r->failed = true;
            event->type = SCOPE_REMOTE_EVENT_FAILED;
            return true;
        }
        if (ret == 1) {
            r->tx_inflight = false;
            if (!g_queue_is_empty(&r->tx_pending)) {
                ScopeRemoteTxItem *item =
                    g_queue_pop_head(&r->tx_pending);

                memcpy(r->tx, &item->msg, item->length);
                if (!scope_remote_send(r, item->length, errp)) {
                    g_free(item);
                    r->failed = true;
                    event->type = SCOPE_REMOTE_EVENT_FAILED;
                    return true;
                }
                g_free(item);
            }
        }
    }

    ret = ibv_poll_cq(r->id->recv_cq, 1, &wc);
    if (ret < 0) {
        error_setg_errno(errp, errno, "remote-rdma receive CQ poll failed");
        r->failed = true;
        event->type = SCOPE_REMOTE_EVENT_FAILED;
        return true;
    }
    if (!ret) {
        return false;
    }
    r->recv_posted = false;
    if (wc.status != IBV_WC_SUCCESS ||
        !scope_remote_validate_message(r, r->rx, 0, 0, errp)) {
        r->failed = true;
        event->type = SCOPE_REMOTE_EVENT_FAILED;
        return true;
    }

    event->request_id = le64_to_cpu(r->rx->hdr.request_id);
    event->status = (int32_t)le32_to_cpu(r->rx->hdr.status);
    opcode = le16_to_cpu(r->rx->hdr.opcode);
    payload_len = le32_to_cpu(r->rx->hdr.payload_len);
    if (opcode == SCOPE_REMOTE_OP_BAR_READ_RSP ||
        opcode == SCOPE_REMOTE_OP_BAR_WRITE_RSP) {
        const struct scope_remote_bar *bar = (const void *)r->rx->payload;

        if (payload_len != sizeof(*bar)) {
            error_setg(errp, "remote-rdma BAR response has invalid length");
            goto invalid_message;
        }
        event->type = SCOPE_REMOTE_EVENT_BAR_RESPONSE;
        event->u.bar.offset = le32_to_cpu(bar->offset);
        event->u.bar.size = le32_to_cpu(bar->size);
        event->u.bar.value = le64_to_cpu(bar->value);
        event->u.bar.wstrb = le32_to_cpu(bar->wstrb);
    } else if (opcode == SCOPE_REMOTE_OP_NVME_COMPLETE) {
        const struct scope_remote_nvme_complete *c =
            (const void *)r->rx->payload;

        if (payload_len != sizeof(*c)) {
            error_setg(errp,
                       "remote-rdma NVMe completion has invalid length");
            goto invalid_message;
        }
        event->type = SCOPE_REMOTE_EVENT_NVME_COMPLETE;
        event->u.nvme.result = le32_to_cpu(c->result);
        event->u.nvme.sq_head = le16_to_cpu(c->sq_head);
        event->u.nvme.sq_id = le16_to_cpu(c->sq_id);
        event->u.nvme.cid = le16_to_cpu(c->cid);
        event->u.nvme.status = le16_to_cpu(c->status);
        event->u.nvme.transferred = le32_to_cpu(c->transferred);
    } else if (opcode == SCOPE_REMOTE_OP_IXGBE_TX_COMPLETE) {
        const struct scope_remote_ether_complete *c =
            (const void *)r->rx->payload;

        if (payload_len != sizeof(*c)) {
            error_setg(errp, "remote-rdma IXGBE TX completion has invalid length");
            goto invalid_message;
        }
        event->type = SCOPE_REMOTE_EVENT_IXGBE_TX_COMPLETE;
        event->u.tx_complete.length = le32_to_cpu(c->length);
    } else if (opcode == SCOPE_REMOTE_OP_IXGBE_RX) {
        const struct scope_remote_ether_frame *frame =
            (const void *)r->rx->payload;
        uint32_t length;

        if (payload_len != sizeof(*frame)) {
            error_setg(errp, "remote-rdma IXGBE RX has invalid length");
            goto invalid_message;
        }
        length = le32_to_cpu(frame->length);
        if (!length || length > SCOPE_REMOTE_ETHER_MAX_FRAME) {
            error_setg(errp, "remote-rdma IXGBE RX frame length is invalid");
            goto invalid_message;
        }
        event->type = SCOPE_REMOTE_EVENT_IXGBE_RX;
        event->u.frame.length = length;
        event->u.frame.flags = le32_to_cpu(frame->flags);
        memcpy(event->u.frame.data, frame->data, length);
    } else if (opcode == SCOPE_REMOTE_OP_IXGBE_LINK) {
        const struct scope_remote_link_state *link =
            (const void *)r->rx->payload;

        if (payload_len != sizeof(*link)) {
            error_setg(errp, "remote-rdma IXGBE link event has invalid length");
            goto invalid_message;
        }
        event->type = SCOPE_REMOTE_EVENT_IXGBE_LINK;
        event->u.link.link_up = le32_to_cpu(link->link_up);
        event->u.link.speed_mbps = le32_to_cpu(link->speed_mbps);
    } else {
        error_setg(errp, "remote-rdma received unexpected opcode %u", opcode);
        goto invalid_message;
    }
    if (!scope_remote_post_recv(r, errp)) {
        r->failed = true;
        event->type = SCOPE_REMOTE_EVENT_FAILED;
    }
    return true;

invalid_message:
    r->failed = true;
    event->type = SCOPE_REMOTE_EVENT_FAILED;
    return true;
}

uint64_t scope_remote_session_id(const ScopeRemoteState *r)
{
    return r ? r->session_id : 0;
}

uint64_t scope_remote_generation(const ScopeRemoteState *r)
{
    return r ? r->generation : 0;
}
