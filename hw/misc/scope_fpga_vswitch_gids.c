/*
 * SCOPE Vortex GPU-initiated direct-storage service.
 *
 * This file is textually included by scope_fpga_vswitch.c.  It deliberately
 * owns a private physical NVMe controller and never shares guest queue state.
 */

#define SCOPE_GIDS_ADMIN_DEPTH          32U
#define SCOPE_GIDS_IO_QID               1U
#define SCOPE_GIDS_POLL_BUDGET          16U
#define SCOPE_GIDS_ADMIN_TIMEOUT_US     5000000U
#define SCOPE_GIDS_READY_TIMEOUT_US     5000000U
#define SCOPE_GIDS_IO_TIMEOUT_US        30000000U
#define SCOPE_GIDS_CTRL_ASQ             0x00000U
#define SCOPE_GIDS_CTRL_ACQ             0x10000U
#define SCOPE_GIDS_CTRL_IOSQ            0x20000U
#define SCOPE_GIDS_CTRL_IOCQ            0x30000U
#define SCOPE_GIDS_CTRL_IDENTIFY        0x40000U
#define SCOPE_GIDS_CTRL_PRP_LISTS       0x50000U
#define SCOPE_GIDS_MAX_IO_BYTES         (512U * 4096U)

typedef struct ScopeGidsInflight {
    bool valid;
    uint32_t logical_cid;
    uint64_t user_token;
    uint32_t byte_count;
    uint64_t submit_timestamp;
    int64_t deadline_us;
} ScopeGidsInflight;

struct ScopeGidsService {
    ScopeProxyState *manager;
    ScopeVortexState *vortex;
    ScopeBackend *owner;
    ScopeBackend physical;

    struct scope_vortex_rpc_gids_queue_create_rsp queue;
    uint32_t logical_sq_consumer;
    uint32_t logical_cq_producer;
    uint64_t session_cookie;

    uint64_t control_base;
    uint64_t control_size;
    uint64_t asq_dma;
    uint64_t acq_dma;
    uint64_t iosq_dma;
    uint64_t iocq_dma;
    uint64_t identify_dma;
    uint32_t queue_depth;
    uint32_t lba_shift;
    uint64_t namespace_blocks;

    uint16_t admin_tail;
    uint16_t admin_head;
    bool admin_phase;
    uint16_t io_tail;
    uint16_t io_head;
    bool io_phase;
    uint32_t io_outstanding;
    ScopeGidsInflight inflight[SCOPE_GIDS_MAX_DEPTH];

    struct scope_gids_cqe pending[SCOPE_GIDS_MAX_DEPTH];
    uint32_t pending_head;
    uint32_t pending_count;
    bool queue_created;
    bool controller_enabled;
    bool failed;
    bool failure_notified;
    uint32_t fatal_error;
    uint64_t logical_accepted;
    uint64_t physical_submitted;
    uint64_t physical_completed;
    uint64_t logical_completed;
    uint64_t read_payload_bytes;
    uint64_t write_payload_bytes;
    uint64_t fetch_batches;
    uint64_t post_batches;
    uint64_t rejected_requests;
};

static ScopeBackend *scope_gids_activate(ScopeGidsService *g)
{
    ScopeBackend *saved = g->manager->active;

    g->manager->active = &g->physical;
    return saved;
}

static void scope_gids_restore_active(ScopeGidsService *g, ScopeBackend *saved)
{
    g->manager->active = saved;
}

static bool scope_gids_rpc_create(ScopeGidsService *g)
{
    struct scope_vortex_rpc_gids_queue_create_req req = {
        .depth = g->owner->gids_queue_depth,
        .features = SCOPE_GIDS_FEATURE_READ |
                    SCOPE_GIDS_FEATURE_WRITE |
                    SCOPE_GIDS_FEATURE_BATCHED |
                    SCOPE_GIDS_FEATURE_DIRECT_P2P |
                    SCOPE_GIDS_FEATURE_SINGLE_PRODUCER,
        .payload_size = g->owner->gids_payload_size,
        .session_cookie = g->session_cookie,
    };

    if (!(g->vortex->bridge_caps & SCOPE_VORTEX_RPC_CAP_GIDS_QUEUE) ||
        !scope_vortex_rpc(g->vortex, SCOPE_VORTEX_RPC_GIDS_QUEUE_CREATE,
                          &req, sizeof(req), &g->queue, sizeof(g->queue)) ||
        g->queue.depth != req.depth ||
        g->queue.generation == 0 ||
        g->queue.payload_size < req.payload_size ||
        g->queue.payload_offset > g->queue.total_size ||
        g->queue.payload_size > g->queue.total_size - g->queue.payload_offset) {
        return false;
    }
    g->queue_created = true;
    return true;
}

static void scope_gids_rpc_destroy(ScopeGidsService *g)
{
    struct scope_vortex_rpc_gids_status_req req;

    if (!g->queue_created || g->vortex->socket_fd < 0) {
        return;
    }
    req.handle = g->queue.handle;
    req.reserved = 0;
    req.generation = g->queue.generation;
    if (!scope_vortex_rpc(g->vortex, SCOPE_VORTEX_RPC_GIDS_QUEUE_DESTROY,
                          &req, sizeof(req), NULL, 0)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE GIDS: queue destroy failed handle=%u\n",
                      req.handle);
    }
    g->queue_created = false;
}

static void scope_gids_service_fail(ScopeGidsService *g, uint32_t error)
{
    struct scope_vortex_rpc_gids_state_req req;

    if (!g || g->failed) {
        return;
    }
    g->failed = true;
    g->fatal_error = error ? error : SCOPE_GIDS_STATUS_QUEUE_FAILED;
    if (!g->queue_created || g->vortex->socket_fd < 0 ||
        !(g->vortex->bridge_caps &
          SCOPE_VORTEX_RPC_CAP_GIDS_QUEUE_STATE)) {
        return;
    }
    req.handle = g->queue.handle;
    req.state = SCOPE_GIDS_QUEUE_FAILED;
    req.generation = g->queue.generation;
    req.fatal_error = g->fatal_error;
    req.reserved = 0;
    if (scope_vortex_rpc(g->vortex,
                         SCOPE_VORTEX_RPC_GIDS_QUEUE_SET_STATE,
                         &req, sizeof(req), NULL, 0)) {
        g->failure_notified = true;
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE GIDS: failed to publish queue failure "
                      "handle=%u error=%u\n",
                      req.handle, req.fatal_error);
    }
}

static bool scope_gids_guest_zero(ScopeGidsService *g, uint64_t guest,
                                  size_t bytes)
{
    uint8_t zero[4096] = { 0 };

    while (bytes) {
        size_t chunk = MIN(bytes, sizeof(zero));

        if (!scope_guest_mem_write(g->manager, guest, zero, chunk)) {
            return false;
        }
        guest += chunk;
        bytes -= chunk;
    }
    return true;
}

static uint32_t scope_gids_bar_read32(ScopeGidsService *g, uint32_t off)
{
    return *(volatile uint32_t *)((uint8_t *)g->physical.real_bar0_map + off);
}

static void scope_gids_bar_write32(ScopeGidsService *g, uint32_t off,
                                   uint32_t value)
{
    *(volatile uint32_t *)((uint8_t *)g->physical.real_bar0_map + off) = value;
    smp_mb();
}

static void scope_gids_bar_write64(ScopeGidsService *g, uint32_t off,
                                   uint64_t value)
{
    scope_gids_bar_write32(g, off, value);
    scope_gids_bar_write32(g, off + 4U, value >> 32);
}

static uint32_t scope_gids_db_offset(ScopeGidsService *g, uint16_t qid,
                                     bool completion)
{
    return SCOPE_NVME_DOORBELL_BASE +
           (2U * qid + (completion ? 1U : 0U)) *
           g->physical.doorbell_stride;
}

static bool scope_gids_wait_ready(ScopeGidsService *g, bool ready,
                                  int64_t timeout_us)
{
    int64_t deadline = g_get_monotonic_time() + timeout_us;

    do {
        uint32_t csts = scope_gids_bar_read32(g, NVME_REG_CSTS);

        if (NVME_CSTS_CFS(csts)) {
            return false;
        }
        if (!!NVME_CSTS_RDY(csts) == ready) {
            return true;
        }
        g_usleep(1000);
    } while (g_get_monotonic_time() < deadline);
    return false;
}

static bool scope_gids_admin_submit(ScopeGidsService *g, const NvmeCmd *cmd,
                                    NvmeCqe *completion)
{
    uint64_t sq_pa = g->control_base + SCOPE_GIDS_CTRL_ASQ +
                     (uint64_t)g->admin_tail * sizeof(*cmd);
    int64_t deadline = g_get_monotonic_time() + SCOPE_GIDS_ADMIN_TIMEOUT_US;

    if (!scope_guest_mem_write(g->manager, sq_pa, cmd, sizeof(*cmd))) {
        return false;
    }
    smp_wmb();
    g->admin_tail = (g->admin_tail + 1U) % SCOPE_GIDS_ADMIN_DEPTH;
    scope_gids_bar_write32(g, scope_gids_db_offset(g, 0, false),
                           g->admin_tail);

    do {
        NvmeCqe cqe;
        uint64_t cq_pa = g->control_base + SCOPE_GIDS_CTRL_ACQ +
                         (uint64_t)g->admin_head * sizeof(cqe);

        if (!scope_guest_mem_read(g->manager, cq_pa, &cqe, sizeof(cqe))) {
            return false;
        }
        if (!!(le16_to_cpu(cqe.status) & 1U) == g->admin_phase) {
            if (completion) {
                *completion = cqe;
            }
            g->admin_head++;
            if (g->admin_head == SCOPE_GIDS_ADMIN_DEPTH) {
                g->admin_head = 0;
                g->admin_phase = !g->admin_phase;
            }
            scope_gids_bar_write32(g, scope_gids_db_offset(g, 0, true),
                                   g->admin_head);
            return (le16_to_cpu(cqe.status) >> 1) == NVME_SUCCESS;
        }
        g_usleep(50);
    } while (g_get_monotonic_time() < deadline);
    return false;
}

static bool scope_gids_identify_namespace(ScopeGidsService *g)
{
    NvmeIdentify identify = { 0 };
    NvmeIdNs idns;
    unsigned int format;

    identify.opcode = NVME_ADM_CMD_IDENTIFY;
    identify.cid = cpu_to_le16(1);
    identify.nsid = cpu_to_le32(g->owner->gids_nsid);
    identify.prp1 = cpu_to_le64(g->identify_dma);
    identify.cns = 0;
    if (!scope_gids_guest_zero(g, g->control_base + SCOPE_GIDS_CTRL_IDENTIFY,
                               sizeof(idns)) ||
        !scope_gids_admin_submit(g, (NvmeCmd *)&identify, NULL) ||
        !scope_guest_mem_read(g->manager,
                              g->control_base + SCOPE_GIDS_CTRL_IDENTIFY,
                              &idns, sizeof(idns))) {
        return false;
    }
    format = NVME_ID_NS_FLBAS_INDEX(idns.flbas);
    if (format > idns.nlbaf || idns.lbaf[format].ds < SCOPE_NVME_MIN_LBA_SHIFT ||
        idns.lbaf[format].ds > SCOPE_NVME_MAX_LBA_SHIFT ||
        NVME_ID_NS_FLBAS_EXTENDED(idns.flbas)) {
        return false;
    }
    g->lba_shift = idns.lbaf[format].ds;
    g->namespace_blocks = le64_to_cpu(idns.nsze);
    return g->namespace_blocks != 0;
}

static bool scope_gids_create_io_queues(ScopeGidsService *g)
{
    NvmeCreateCq cq = { 0 };
    NvmeCreateSq sq = { 0 };

    cq.opcode = NVME_ADM_CMD_CREATE_CQ;
    cq.cid = cpu_to_le16(2);
    cq.prp1 = cpu_to_le64(g->iocq_dma);
    cq.cqid = cpu_to_le16(SCOPE_GIDS_IO_QID);
    cq.qsize = cpu_to_le16(g->queue_depth - 1U);
    cq.cq_flags = cpu_to_le16(NVME_CQ_PC);
    if (!scope_gids_admin_submit(g, (NvmeCmd *)&cq, NULL)) {
        return false;
    }

    sq.opcode = NVME_ADM_CMD_CREATE_SQ;
    sq.cid = cpu_to_le16(3);
    sq.prp1 = cpu_to_le64(g->iosq_dma);
    sq.sqid = cpu_to_le16(SCOPE_GIDS_IO_QID);
    sq.qsize = cpu_to_le16(g->queue_depth - 1U);
    sq.sq_flags = cpu_to_le16(NVME_SQ_PC |
                              (NVME_SQ_PRIO_NORMAL << 1));
    sq.cqid = cpu_to_le16(SCOPE_GIDS_IO_QID);
    return scope_gids_admin_submit(g, (NvmeCmd *)&sq, NULL);
}

static bool scope_gids_init_physical(ScopeGidsService *g, Error **errp)
{
    ScopeProxyState *s = g->manager;
    ScopeBackend *saved;
    uint64_t required;
    uint64_t cap;
    uint32_t cc;
    bool ok = false;

    memset(&g->physical, 0, sizeof(g->physical));
    g->physical.type = SCOPE_BACKEND_NVME;
    g->physical.transport = SCOPE_TRANSPORT_LOCAL_P2P;
    g->physical.real_bar_fd = -1;
    g->physical.real_host_bdf = g_strdup(g->owner->gids_nvme_bdf);
    g->control_base = g->owner->gids_control_base;
    g->control_size = g->owner->gids_control_size;
    g->queue_depth = g->owner->gids_queue_depth;
    required = SCOPE_GIDS_CTRL_PRP_LISTS +
               (uint64_t)g->queue_depth * SCOPE_NVME_DEFAULT_CTRL_PAGE_SIZE;
    if (required > g->control_size) {
        error_setg(errp,
                   "GIDS control region 0x%" PRIx64
                   " is smaller than required 0x%" PRIx64,
                   g->control_size, required);
        return false;
    }

    saved = scope_gids_activate(g);
    if (!scope_parse_real_bar0(s, errp) ||
        !scope_enable_real_pci_bus_master(s, errp) ||
        !scope_init_nvme_capability_cache(s, errp) ||
        !scope_real_nvme_disable(s, "GIDS exclusive controller", errp)) {
        goto out;
    }
    cap = g->physical.nvme_cap;
    if (NVME_CAP_MPSMIN(cap) != 0 ||
        SCOPE_GIDS_ADMIN_DEPTH > NVME_CAP_MQES(cap) + 1U ||
        g->queue_depth > NVME_CAP_MQES(cap) + 1U) {
        error_setg(errp,
                   "GIDS controller requires 4 KiB pages and queue depth %u "
                   "(CAP=0x%016" PRIx64 ")",
                   g->queue_depth, cap);
        goto out;
    }
    if (!scope_translate_guest_pa_for_real_dma(s,
            g->control_base + SCOPE_GIDS_CTRL_ASQ, 4096, &g->asq_dma) ||
        !scope_translate_guest_pa_for_real_dma(s,
            g->control_base + SCOPE_GIDS_CTRL_ACQ, 4096, &g->acq_dma) ||
        !scope_translate_guest_pa_for_real_dma(s,
            g->control_base + SCOPE_GIDS_CTRL_IOSQ, 4096, &g->iosq_dma) ||
        !scope_translate_guest_pa_for_real_dma(s,
            g->control_base + SCOPE_GIDS_CTRL_IOCQ, 4096, &g->iocq_dma) ||
        !scope_translate_guest_pa_for_real_dma(s,
            g->control_base + SCOPE_GIDS_CTRL_IDENTIFY, 4096,
            &g->identify_dma) ||
        !scope_gids_guest_zero(g, g->control_base, required)) {
        error_setg(errp, "GIDS control region is not DMA-visible");
        goto out;
    }

    scope_gids_bar_write32(g, NVME_REG_INTMS, UINT32_MAX);
    scope_gids_bar_write32(g, NVME_REG_AQA,
                           ((SCOPE_GIDS_ADMIN_DEPTH - 1U) << 16) |
                           (SCOPE_GIDS_ADMIN_DEPTH - 1U));
    scope_gids_bar_write64(g, NVME_REG_ASQ, g->asq_dma);
    scope_gids_bar_write64(g, NVME_REG_ACQ, g->acq_dma);
    g->admin_phase = true;
    g->io_phase = true;
    cc = 1U | (6U << 16) | (4U << 20);
    scope_gids_bar_write32(g, NVME_REG_CC, cc);
    if (!scope_gids_wait_ready(g, true, SCOPE_GIDS_READY_TIMEOUT_US)) {
        error_setg(errp, "GIDS NVMe controller did not become ready");
        goto out;
    }
    g->controller_enabled = true;
    if (!scope_gids_identify_namespace(g) ||
        !scope_gids_create_io_queues(g)) {
        error_setg(errp, "GIDS NVMe namespace or I/O queue initialization failed");
        goto out;
    }
    if (g->owner->gids_slba_base >= g->namespace_blocks ||
        g->owner->gids_block_count >
            g->namespace_blocks - g->owner->gids_slba_base) {
        error_setg(errp,
                   "GIDS LBA partition [0x%" PRIx64 ",+0x%" PRIx64
                   "] exceeds namespace size 0x%" PRIx64,
                   g->owner->gids_slba_base, g->owner->gids_block_count,
                   g->namespace_blocks);
        goto out;
    }
    ok = true;
out:
    scope_gids_restore_active(g, saved);
    return ok;
}

static bool scope_gids_pending_push(ScopeGidsService *g,
                                    const struct scope_gids_cqe *cqe)
{
    uint32_t tail;

    if (g->pending_count == g->queue_depth) {
        return false;
    }
    tail = (g->pending_head + g->pending_count) & (g->queue_depth - 1U);
    g->pending[tail] = *cqe;
    g->pending_count++;
    return true;
}

static void scope_gids_build_error_cqe(ScopeGidsService *g,
                                       const struct scope_gids_sqe *sqe,
                                       uint16_t status,
                                       struct scope_gids_cqe *cqe)
{
    memset(cqe, 0, sizeof(*cqe));
    cqe->logical_cid = sqe->logical_cid;
    cqe->status = status;
    cqe->user_token = sqe->user_token;
    cqe->submit_timestamp = g_get_monotonic_time() * 1000ULL;
    cqe->complete_timestamp = cqe->submit_timestamp;
    cqe->generation = g->queue.generation;
}

static uint16_t scope_gids_validate_sqe(ScopeGidsService *g,
                                        const struct scope_gids_sqe *sqe,
                                        uint64_t *payload_dma)
{
    uint64_t blocks = sqe->block_count;
    uint64_t expected;
    uint64_t slba;

    if (sqe->opcode != SCOPE_GIDS_OP_READ &&
        sqe->opcode != SCOPE_GIDS_OP_WRITE) {
        return SCOPE_GIDS_STATUS_INVALID_OPCODE;
    }
    if (sqe->reserved0 || sqe->reserved1 ||
        sqe->nsid != g->owner->gids_nsid) {
        return SCOPE_GIDS_STATUS_INVALID_NAMESPACE;
    }
    if (!blocks || blocks > UINT16_MAX + 1ULL ||
        blocks > (UINT64_MAX >> g->lba_shift)) {
        return SCOPE_GIDS_STATUS_LBA_RANGE;
    }
    expected = blocks << g->lba_shift;
    if (expected != sqe->byte_count || expected > SCOPE_GIDS_MAX_IO_BYTES ||
        sqe->slba >= g->owner->gids_block_count ||
        blocks > g->owner->gids_block_count - sqe->slba) {
        return SCOPE_GIDS_STATUS_LBA_RANGE;
    }
    if (sqe->buffer_handle != g->queue.handle ||
        sqe->buffer_generation != g->queue.generation ||
        sqe->buffer_offset > g->queue.payload_size ||
        (sqe->buffer_offset & ((1ULL << g->lba_shift) - 1U)) ||
        expected > g->queue.payload_size - sqe->buffer_offset) {
        return SCOPE_GIDS_STATUS_BUFFER_RANGE;
    }
    for (uint32_t i = 0; i < g->queue_depth; i++) {
        if (g->inflight[i].valid &&
            g->inflight[i].logical_cid == sqe->logical_cid) {
            return SCOPE_GIDS_STATUS_DUPLICATE_CID;
        }
    }
    slba = g->owner->gids_slba_base + sqe->slba;
    if (slba < g->owner->gids_slba_base) {
        return SCOPE_GIDS_STATUS_LBA_RANGE;
    }
    *payload_dma = g->queue.p2p_bus_addr + g->queue.payload_offset;
    if (*payload_dma < g->queue.p2p_bus_addr ||
        sqe->buffer_offset > UINT64_MAX - *payload_dma) {
        return SCOPE_GIDS_STATUS_BUFFER_RANGE;
    }
    *payload_dma += sqe->buffer_offset;
    if (*payload_dma > UINT64_MAX - expected) {
        return SCOPE_GIDS_STATUS_BUFFER_RANGE;
    }
    return SCOPE_GIDS_STATUS_SUCCESS;
}

static bool scope_gids_build_prps(ScopeGidsService *g, uint16_t cid,
                                  uint64_t data_dma, uint32_t bytes,
                                  uint64_t *prp1, uint64_t *prp2)
{
    const uint64_t page = SCOPE_NVME_DEFAULT_CTRL_PAGE_SIZE;
    uint64_t first = MIN((uint64_t)bytes, page - (data_dma & (page - 1U)));
    uint64_t remaining = bytes - first;
    uint64_t next = data_dma + first;
    uint64_t list_guest;
    uint64_t list_dma;
    uint64_t entries[512];
    uint32_t count;

    *prp1 = data_dma;
    *prp2 = 0;
    if (!remaining) {
        return true;
    }
    if (remaining <= page) {
        *prp2 = next;
        return true;
    }
    count = DIV_ROUND_UP(remaining, page);
    if (count > ARRAY_SIZE(entries)) {
        return false;
    }
    for (uint32_t i = 0; i < count; i++) {
        entries[i] = cpu_to_le64(next + (uint64_t)i * page);
    }
    list_guest = g->control_base + SCOPE_GIDS_CTRL_PRP_LISTS +
                 (uint64_t)cid * page;
    if (!scope_guest_mem_write(g->manager, list_guest, entries,
                               count * sizeof(entries[0]))) {
        return false;
    }
    {
        ScopeBackend *saved = scope_gids_activate(g);
        bool ok = scope_translate_guest_pa_for_real_dma(g->manager, list_guest,
                                                         page, &list_dma);
        scope_gids_restore_active(g, saved);
        if (!ok) {
            return false;
        }
    }
    *prp2 = list_dma;
    return true;
}

static bool scope_gids_issue(ScopeGidsService *g,
                             const struct scope_gids_sqe *sqe,
                             uint64_t payload_dma)
{
    NvmeRwCmd cmd = { 0 };
    ScopeGidsInflight *inflight;
    uint16_t cid = g->io_tail;
    uint64_t prp1, prp2;
    uint64_t sq_pa;

    if (g->io_outstanding >= g->queue_depth - 1U ||
        g->inflight[cid].valid ||
        !scope_gids_build_prps(g, cid, payload_dma, sqe->byte_count,
                               &prp1, &prp2)) {
        return false;
    }
    cmd.opcode = sqe->opcode == SCOPE_GIDS_OP_READ ?
                 NVME_CMD_READ : NVME_CMD_WRITE;
    cmd.cid = cpu_to_le16(cid);
    cmd.nsid = cpu_to_le32(g->owner->gids_nsid);
    cmd.dptr.prp1 = cpu_to_le64(prp1);
    cmd.dptr.prp2 = cpu_to_le64(prp2);
    cmd.slba = cpu_to_le64(g->owner->gids_slba_base + sqe->slba);
    cmd.nlb = cpu_to_le16(sqe->block_count - 1U);
    sq_pa = g->control_base + SCOPE_GIDS_CTRL_IOSQ +
            (uint64_t)g->io_tail * sizeof(cmd);
    if (!scope_guest_mem_write(g->manager, sq_pa, &cmd, sizeof(cmd))) {
        return false;
    }
    inflight = &g->inflight[cid];
    inflight->valid = true;
    inflight->logical_cid = sqe->logical_cid;
    inflight->user_token = sqe->user_token;
    inflight->byte_count = sqe->byte_count;
    inflight->submit_timestamp = g_get_monotonic_time() * 1000ULL;
    inflight->deadline_us = g_get_monotonic_time() +
                            SCOPE_GIDS_IO_TIMEOUT_US;
    g->io_tail = (g->io_tail + 1U) & (g->queue_depth - 1U);
    g->io_outstanding++;
    g->physical_submitted++;
    if (sqe->opcode == SCOPE_GIDS_OP_READ) {
        g->read_payload_bytes += sqe->byte_count;
    } else {
        g->write_payload_bytes += sqe->byte_count;
    }
    smp_wmb();
    scope_gids_bar_write32(g,
                           scope_gids_db_offset(g, SCOPE_GIDS_IO_QID, false),
                           g->io_tail);
    return true;
}

static bool scope_gids_poll_physical(ScopeGidsService *g)
{
    bool progressed = false;

    for (uint32_t n = 0; n < SCOPE_GIDS_POLL_BUDGET; n++) {
        NvmeCqe cqe;
        ScopeGidsInflight *inflight;
        struct scope_gids_cqe logical = { 0 };
        uint16_t cid;
        uint16_t status;
        uint64_t cq_pa;

        if (g->pending_count == g->queue_depth) {
            break;
        }
        cq_pa = g->control_base + SCOPE_GIDS_CTRL_IOCQ +
                (uint64_t)g->io_head * sizeof(cqe);
        if (!scope_guest_mem_read(g->manager, cq_pa, &cqe, sizeof(cqe)) ||
            !!(le16_to_cpu(cqe.status) & 1U) != g->io_phase) {
            break;
        }
        cid = le16_to_cpu(cqe.cid);
        if (cid >= g->queue_depth || !g->inflight[cid].valid) {
            scope_gids_service_fail(g, SCOPE_GIDS_STATUS_NVME);
            break;
        }
        inflight = &g->inflight[cid];
        status = le16_to_cpu(cqe.status) >> 1;
        logical.logical_cid = inflight->logical_cid;
        logical.status = status == NVME_SUCCESS ?
                         SCOPE_GIDS_STATUS_SUCCESS : SCOPE_GIDS_STATUS_NVME;
        logical.flags = SCOPE_GIDS_CQE_F_NVME_STATUS_VALID;
        logical.nvme_status = status;
        logical.completed_bytes = status == NVME_SUCCESS ?
                                  inflight->byte_count : 0;
        logical.user_token = inflight->user_token;
        logical.submit_timestamp = inflight->submit_timestamp;
        logical.complete_timestamp = g_get_monotonic_time() * 1000ULL;
        logical.generation = g->queue.generation;
        if (!scope_gids_pending_push(g, &logical)) {
            break;
        }
        memset(inflight, 0, sizeof(*inflight));
        g->io_outstanding--;
        g->physical_completed++;
        g->io_head++;
        if (g->io_head == g->queue_depth) {
            g->io_head = 0;
            g->io_phase = !g->io_phase;
        }
        scope_gids_bar_write32(g,
                               scope_gids_db_offset(g, SCOPE_GIDS_IO_QID,
                                                    true),
                               g->io_head);
        progressed = true;
    }
    return progressed;
}

static bool scope_gids_check_timeouts(ScopeGidsService *g)
{
    int64_t now_us;

    if (!g->io_outstanding) {
        return false;
    }
    now_us = g_get_monotonic_time();
    for (uint32_t cid = 0; cid < g->queue_depth; cid++) {
        ScopeGidsInflight *inflight = &g->inflight[cid];

        if (inflight->valid && now_us >= inflight->deadline_us) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "SCOPE GIDS: physical I/O timeout cid=%u "
                          "logical_cid=%u outstanding=%u\n",
                          cid, inflight->logical_cid, g->io_outstanding);
            scope_gids_service_fail(g, SCOPE_GIDS_STATUS_TIMEOUT);
            return true;
        }
    }
    return false;
}

static bool scope_gids_post_completions(ScopeGidsService *g)
{
    uint8_t payload[sizeof(struct scope_vortex_rpc_gids_commit_req) +
                    SCOPE_VORTEX_RPC_GIDS_MAX_BATCH *
                    sizeof(struct scope_gids_cqe)];
    struct scope_vortex_rpc_gids_commit_req req = { 0 };
    uint32_t count;

    if (!g->pending_count) {
        return false;
    }
    count = MIN(g->pending_count, SCOPE_VORTEX_RPC_GIDS_MAX_BATCH);
    req.handle = g->queue.handle;
    req.count = count;
    req.generation = g->queue.generation;
    req.start_counter = g->logical_cq_producer;
    memcpy(payload, &req, sizeof(req));
    for (uint32_t i = 0; i < count; i++) {
        uint32_t slot = (g->pending_head + i) & (g->queue_depth - 1U);

        memcpy(payload + sizeof(req) + i * sizeof(g->pending[slot]),
               &g->pending[slot], sizeof(g->pending[slot]));
    }
    if (!scope_vortex_rpc(g->vortex, SCOPE_VORTEX_RPC_GIDS_POST_CQ,
                          payload, sizeof(req) + count * sizeof(g->pending[0]),
                          NULL, 0)) {
        return false;
    }
    g->pending_head = (g->pending_head + count) & (g->queue_depth - 1U);
    g->pending_count -= count;
    g->logical_cq_producer += count;
    g->logical_completed += count;
    g->post_batches++;
    return true;
}

static bool scope_gids_fetch_and_issue(ScopeGidsService *g)
{
    struct {
        struct scope_vortex_rpc_gids_batch_rsp header;
        struct scope_gids_sqe entries[SCOPE_VORTEX_RPC_GIDS_MAX_BATCH];
    } response;
    struct scope_vortex_rpc_gids_batch_req req = {
        .handle = g->queue.handle,
        .max_entries = SCOPE_VORTEX_RPC_GIDS_MAX_BATCH,
        .generation = g->queue.generation,
        .start_counter = g->logical_sq_consumer,
    };
    uint32_t response_len = 0;
    uint32_t accepted = 0;

    if (g->pending_count == g->queue_depth ||
        g->io_outstanding >= g->queue_depth - 1U ||
        !scope_vortex_rpc_variable(g->vortex,
                                   SCOPE_VORTEX_RPC_GIDS_FETCH_SQ,
                                   &req, sizeof(req), &response,
                                   sizeof(response), &response_len)) {
        return false;
    }
    if (response_len < sizeof(response.header) ||
        response.header.generation != g->queue.generation ||
        response.header.consumer != g->logical_sq_consumer ||
        response.header.count > SCOPE_VORTEX_RPC_GIDS_MAX_BATCH ||
        response_len != sizeof(response.header) +
                        response.header.count * sizeof(response.entries[0])) {
        scope_gids_service_fail(g, SCOPE_GIDS_STATUS_TRANSPORT);
        return false;
    }
    if (response.header.count) {
        g->fetch_batches++;
    }
    for (uint32_t i = 0; i < response.header.count; i++) {
        const struct scope_gids_sqe *sqe = &response.entries[i];
        struct scope_gids_cqe error_cqe;
        uint64_t payload_dma = 0;
        uint16_t status;

        if (sqe->ready_sequence != g->logical_sq_consumer + accepted + 1U) {
            break;
        }
        status = scope_gids_validate_sqe(g, sqe, &payload_dma);
        if (status != SCOPE_GIDS_STATUS_SUCCESS) {
            g->rejected_requests++;
            scope_gids_build_error_cqe(g, sqe, status, &error_cqe);
            if (!scope_gids_pending_push(g, &error_cqe)) {
                break;
            }
        } else if (!scope_gids_issue(g, sqe, payload_dma)) {
            break;
        }
        accepted++;
        g->logical_accepted++;
    }
    if (accepted) {
        struct scope_vortex_rpc_gids_commit_req commit = {
            .handle = g->queue.handle,
            .count = accepted,
            .generation = g->queue.generation,
            .start_counter = g->logical_sq_consumer,
        };

        if (!scope_vortex_rpc(g->vortex, SCOPE_VORTEX_RPC_GIDS_COMMIT_SQ,
                              &commit, sizeof(commit), NULL, 0)) {
            scope_gids_service_fail(g, SCOPE_GIDS_STATUS_TRANSPORT);
            return false;
        }
        g->logical_sq_consumer += accepted;
        return true;
    }
    return false;
}

static bool scope_gids_service_poll(ScopeGidsService *g)
{
    bool progressed = false;

    if (!g || g->failed || !g->queue_created || !g->controller_enabled) {
        return false;
    }
    progressed = scope_gids_poll_physical(g) || progressed;
    if (scope_gids_check_timeouts(g)) {
        return true;
    }
    progressed = scope_gids_post_completions(g) || progressed;
    progressed = scope_gids_fetch_and_issue(g) || progressed;
    return progressed;
}

static bool scope_gids_service_bar_read(ScopeGidsService *g, uint32_t off,
                                        uint32_t *value)
{
    uint64_t header_hbm;

    if (off < SCOPE_VX_GIDS_CAPS || off > SCOPE_VX_GIDS_BLOCKS_HI) {
        return false;
    }
    if (!g || !g->queue_created) {
        *value = 0;
        return true;
    }
    header_hbm = g->queue.hbm_addr;
    switch (off) {
    case SCOPE_VX_GIDS_CAPS:
        *value = SCOPE_GIDS_FEATURE_READ | SCOPE_GIDS_FEATURE_WRITE |
                 SCOPE_GIDS_FEATURE_BATCHED |
                 SCOPE_GIDS_FEATURE_DIRECT_P2P |
                 SCOPE_GIDS_FEATURE_SINGLE_PRODUCER;
        return true;
    case SCOPE_VX_GIDS_STATE:
        *value = g->failed ? SCOPE_GIDS_QUEUE_FAILED :
                             SCOPE_GIDS_QUEUE_READY;
        return true;
    case SCOPE_VX_GIDS_ERROR: *value = g->fatal_error; return true;
    case SCOPE_VX_GIDS_HANDLE: *value = g->queue.handle; return true;
    case SCOPE_VX_GIDS_DEPTH: *value = g->queue.depth; return true;
    case SCOPE_VX_GIDS_GENERATION_LO: *value = g->queue.generation; return true;
    case SCOPE_VX_GIDS_GENERATION_HI: *value = g->queue.generation >> 32; return true;
    case SCOPE_VX_GIDS_HEADER_LO: *value = header_hbm; return true;
    case SCOPE_VX_GIDS_HEADER_HI: *value = header_hbm >> 32; return true;
    case SCOPE_VX_GIDS_SQ_LO: *value = header_hbm + g->queue.sq_offset; return true;
    case SCOPE_VX_GIDS_SQ_HI: *value = (header_hbm + g->queue.sq_offset) >> 32; return true;
    case SCOPE_VX_GIDS_CQ_LO: *value = header_hbm + g->queue.cq_offset; return true;
    case SCOPE_VX_GIDS_CQ_HI: *value = (header_hbm + g->queue.cq_offset) >> 32; return true;
    case SCOPE_VX_GIDS_PAYLOAD_LO:
        *value = header_hbm + g->queue.payload_offset;
        return true;
    case SCOPE_VX_GIDS_PAYLOAD_HI:
        *value = (header_hbm + g->queue.payload_offset) >> 32;
        return true;
    case SCOPE_VX_GIDS_PAYLOAD_SIZE_LO: *value = g->queue.payload_size; return true;
    case SCOPE_VX_GIDS_PAYLOAD_SIZE_HI: *value = g->queue.payload_size >> 32; return true;
    case SCOPE_VX_GIDS_NSID: *value = g->owner->gids_nsid; return true;
    case SCOPE_VX_GIDS_LBA_SHIFT: *value = g->lba_shift; return true;
    case SCOPE_VX_GIDS_BLOCKS_LO: *value = g->owner->gids_block_count; return true;
    case SCOPE_VX_GIDS_BLOCKS_HI: *value = g->owner->gids_block_count >> 32; return true;
    default: return false;
    }
}

static bool scope_gids_service_realize(ScopeProxyState *s,
                                       ScopeVortexState *v, Error **errp)
{
    ScopeBackend *be = v->backend;
    ScopeGidsService *g;
    struct scope_vortex_rpc_gds_caps_req caps_req = { 0 };
    struct scope_vortex_rpc_gds_caps_rsp caps_rsp = { 0 };
    uint16_t domain;
    uint8_t bus;
    uint8_t devfn;

    if (!be->gids_enabled) {
        return true;
    }
    if ((v->bridge_caps & (SCOPE_VORTEX_RPC_CAP_GIDS_QUEUE |
                           SCOPE_VORTEX_RPC_CAP_GIDS_QUEUE_STATE)) !=
        (SCOPE_VORTEX_RPC_CAP_GIDS_QUEUE |
         SCOPE_VORTEX_RPC_CAP_GIDS_QUEUE_STATE) ||
        !scope_vortex_parse_host_bdf(be->gids_nvme_bdf,
                                     &domain, &bus, &devfn)) {
        error_setg(errp,
                   "Vortex bridge lacks GIDS queue/state support");
        return false;
    }
    caps_req.domain = domain;
    caps_req.bus = bus;
    caps_req.devfn = devfn;
    if (!scope_vortex_rpc_gds_caps(v, &caps_req, &caps_rsp) ||
        !(caps_rsp.flags & SCOPE_VORTEX_GDS_CAP_HBM0) ||
        caps_rsp.alignment > 4096U ||
        caps_rsp.max_size < be->gids_payload_size) {
        error_setg(errp,
                   "Vortex bridge cannot allocate the requested GIDS HBM arena");
        return false;
    }

    g = g_new0(ScopeGidsService, 1);
    g->manager = s;
    g->vortex = v;
    g->owner = be;
    g->session_cookie = ((uint64_t)g_random_int() << 32) | g_random_int();
    if (!g->session_cookie) {
        g->session_cookie = 1;
    }
    be->gids = g;
    if (!scope_gids_init_physical(g, errp) || !scope_gids_rpc_create(g)) {
        if (!*errp) {
            error_setg(errp, "failed to create GIDS logical queue");
        }
        scope_gids_service_cleanup(g);
        return false;
    }
    SCOPE_PRINTF("[SCOPE GIDS] ready vortex_backend=%u nvme=%s nsid=%u "
                 "lba=[0x%" PRIx64 ",+0x%" PRIx64 "] depth=%u "
                 "payload=0x%" PRIx64 " hbm=0x%" PRIx64
                 " p2p=0x%" PRIx64 "\n",
                 be->id, be->gids_nvme_bdf, be->gids_nsid,
                 be->gids_slba_base, be->gids_block_count, g->queue_depth,
                 g->queue.payload_size, g->queue.hbm_addr,
                 g->queue.p2p_bus_addr + g->queue.payload_offset);
    return true;
}

static void scope_gids_service_cleanup(ScopeGidsService *g)
{
    ScopeBackend *saved;

    if (!g) {
        return;
    }
    SCOPE_PRINTF("[SCOPE GIDS][SUMMARY] accepted=%" PRIu64
                 " submitted=%" PRIu64 " physical_completed=%" PRIu64
                 " logical_completed=%" PRIu64 " read_bytes=%" PRIu64
                 " write_bytes=%" PRIu64 " fetch_batches=%" PRIu64
                 " post_batches=%" PRIu64 " rejected=%" PRIu64
                 " failed=%u fatal_error=%u\n",
                 g->logical_accepted, g->physical_submitted,
                 g->physical_completed, g->logical_completed,
                 g->read_payload_bytes, g->write_payload_bytes,
                 g->fetch_batches, g->post_batches, g->rejected_requests,
                 g->failed, g->fatal_error);
    scope_gids_rpc_destroy(g);
    saved = scope_gids_activate(g);
    if (g->controller_enabled) {
        scope_real_nvme_disable(g->manager, "GIDS teardown", NULL);
        g->controller_enabled = false;
    }
    scope_gids_restore_active(g, saved);
    scope_restore_real_pci_command(&g->physical);
    if (g->physical.real_bar0_map) {
        munmap(g->physical.real_bar0_map, g->physical.real_bar0_size);
    }
    if (g->physical.real_bar_fd >= 0) {
        close(g->physical.real_bar_fd);
    }
    g_free(g->physical.real_host_bdf);
    g->owner->gids = NULL;
    g_free(g);
}
