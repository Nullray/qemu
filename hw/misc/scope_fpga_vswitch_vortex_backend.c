/*
 * Vortex Command Processor backend for scope-fpga-vswitch.
 *
 * BAR0 is a virtual 4 KiB CP register file.  The guest owns a coherent
 * command ring in XiangShan DDR.  A dedicated worker either stages payloads
 * through XRT BOs (mediated compatibility mode), or maps a validated NM37
 * BAR2 window and patches only the host operand (direct-P2P mode).  Control
 * commands still reach the physical Vortex CP through the bridge RPC.
 */

#define SCOPE_VX_CP_CTRL               0x000U
#define SCOPE_VX_CP_DEV_CAPS           0x008U
#define SCOPE_VX_CP_CYCLE_LO           0x010U
#define SCOPE_VX_CP_CYCLE_HI           0x014U
#define SCOPE_VX_CP_GPU_CAPS_LO        0x018U
#define SCOPE_VX_CP_GPU_CAPS_HI        0x01cU
#define SCOPE_VX_CP_ISA_CAPS_LO        0x020U
#define SCOPE_VX_CP_ISA_CAPS_HI        0x024U
#define SCOPE_VX_CP_Q_RING_LO          0x100U
#define SCOPE_VX_CP_Q_RING_HI          0x104U
#define SCOPE_VX_CP_Q_HEAD_LO          0x108U
#define SCOPE_VX_CP_Q_HEAD_HI          0x10cU
#define SCOPE_VX_CP_Q_CMPL_LO          0x110U
#define SCOPE_VX_CP_Q_CMPL_HI          0x114U
#define SCOPE_VX_CP_Q_RING_LOG2        0x118U
#define SCOPE_VX_CP_Q_CONTROL          0x11cU
#define SCOPE_VX_CP_Q_TAIL_LO          0x120U
#define SCOPE_VX_CP_Q_TAIL_HI          0x124U
#define SCOPE_VX_CP_Q_SEQNUM           0x128U
#define SCOPE_VX_CP_Q_ERROR            0x12cU
#define SCOPE_VX_CP_Q_LAST_DCR         0x130U

#define SCOPE_VX_CP_RING_LOG2          16U
#define SCOPE_VX_CP_RING_SIZE          (1U << SCOPE_VX_CP_RING_LOG2)
#define SCOPE_VX_CP_CL_SIZE            64U
#define SCOPE_VX_CP_MAX_LINES          (SCOPE_VX_CP_RING_SIZE / SCOPE_VX_CP_CL_SIZE)
#define SCOPE_VX_CP_TIMEOUT_US         120000000LL
#define SCOPE_VX_CP_POLL_US            50U
#define SCOPE_VX_RPC_IO_CHUNK          (64U * 1024U)
#define SCOPE_VX_RPC_SOCKET_TIMEOUT_SEC 5
#define SCOPE_VX_GUEST_VISIBLE_TIMEOUT_US 5000000LL
#define SCOPE_VX_PEER_SLOT_SIZE       (4ULL * 1024ULL * 1024ULL)
#define SCOPE_VX_PEER_WINDOW_SIZE     (64ULL * 1024ULL * 1024ULL)
#define SCOPE_VX_P2P_GUEST_DDR_BASE   UINT64_C(0x80000000)
#define SCOPE_VX_P2P_GUEST_DDR_SIZE   UINT64_C(0x80000000)
#define SCOPE_VX_P2P_ALIAS_OFFSET     UINT64_C(0x100000000)
#define SCOPE_VX_P2P_BAR_INDEX        2

#define SCOPE_VX_OP_NOP                0x00U
#define SCOPE_VX_OP_MEM_WRITE          0x01U
#define SCOPE_VX_OP_MEM_READ           0x02U
#define SCOPE_VX_OP_MEM_COPY           0x03U
#define SCOPE_VX_OP_DCR_WRITE          0x04U
#define SCOPE_VX_OP_DCR_READ           0x05U
#define SCOPE_VX_OP_LAUNCH             0x06U
#define SCOPE_VX_OP_FENCE              0x07U
#define SCOPE_VX_OP_EVENT_SIGNAL       0x08U
#define SCOPE_VX_OP_EVENT_WAIT         0x09U
#define SCOPE_VX_OP_CACHE_FLUSH        0x0aU
#define SCOPE_VX_OP_LAUNCH_QMD         0x0bU
#define SCOPE_VX_OP_DRAW               0x0cU
#define SCOPE_VX_FLAG_PROFILE          0x01U

typedef struct ScopeVortexJob {
    uint64_t guest_tail;
    uint32_t target_seq;
    uint32_t line_count;
    uint8_t lines[];
} ScopeVortexJob;

typedef struct ScopeVortexTransfer {
    uint32_t handle;
    uint64_t cp_addr;
    uint64_t guest_pa;
    uint32_t size;
    bool download;
} ScopeVortexTransfer;

struct ScopeVortexState {
    ScopeProxyState *manager;
    ScopeBackend *backend;
    int socket_fd;
    uint32_t rpc_request_id;
    GAsyncQueue *jobs;
    QemuThread worker;
    bool worker_started;
    int worker_stop;

    uint32_t caps[7];
    uint32_t cp_ctrl;
    uint64_t guest_ring_base;
    uint64_t guest_head_addr;
    uint64_t guest_cmpl_addr;
    uint32_t guest_ring_log2;
    uint32_t guest_q_control;
    uint32_t guest_tail_lo;
    uint64_t submitted_tail;
    uint64_t pending_tail;
    bool pending_tail_valid;
    int64_t pending_tail_deadline_us;
    uint32_t submitted_seq;
    uint32_t retired_seq;
    uint32_t q_error;
    uint32_t last_dcr_rsp;
    bool failed;

    uint32_t physical_ring_handle;
    uint32_t physical_head_handle;
    uint32_t physical_cmpl_handle;
    uint64_t physical_ring_addr;
    uint64_t physical_head_addr;
    uint64_t physical_cmpl_addr;
    uint64_t physical_tail;
    uint32_t physical_seq;

    bool direct_p2p;
    struct scope_vortex_rpc_peer_caps peer_caps;
    bool peer_window_valid;
    uint64_t peer_window_guest_base;
    uint64_t peer_generation;
    uint64_t payload_cpu_bytes;
};

static const char *scope_vortex_opcode_name(uint8_t opcode)
{
    switch (opcode) {
    case SCOPE_VX_OP_NOP: return "NOP";
    case SCOPE_VX_OP_MEM_WRITE: return "MEM_WRITE";
    case SCOPE_VX_OP_MEM_READ: return "MEM_READ";
    case SCOPE_VX_OP_MEM_COPY: return "MEM_COPY";
    case SCOPE_VX_OP_DCR_WRITE: return "DCR_WRITE";
    case SCOPE_VX_OP_DCR_READ: return "DCR_READ";
    case SCOPE_VX_OP_LAUNCH: return "LAUNCH";
    case SCOPE_VX_OP_FENCE: return "FENCE";
    case SCOPE_VX_OP_EVENT_SIGNAL: return "EVENT_SIGNAL";
    case SCOPE_VX_OP_EVENT_WAIT: return "EVENT_WAIT";
    case SCOPE_VX_OP_CACHE_FLUSH: return "CACHE_FLUSH";
    case SCOPE_VX_OP_LAUNCH_QMD: return "LAUNCH_QMD";
    case SCOPE_VX_OP_DRAW: return "DRAW";
    default: return "UNKNOWN";
    }
}

static void scope_vortex_trace(ScopeVortexState *v, const char *fmt, ...)
    G_GNUC_PRINTF(2, 3);

static void scope_vortex_trace(ScopeVortexState *v, const char *fmt, ...)
{
    va_list ap;

    if (!v->manager->vortex_log) {
        return;
    }
    fprintf(stderr, "[SCOPE VORTEX] backend=%u ", v->backend->id);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fflush(stderr);
}

static bool scope_vortex_io_full(int fd, void *buf, size_t len, bool write_op)
{
    uint8_t *p = buf;

    while (len) {
        ssize_t n = write_op ? send(fd, p, len, MSG_NOSIGNAL) : recv(fd, p, len, 0);
        if (n == 0) {
            return false;
        }
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        p += n;
        len -= n;
    }
    return true;
}

static bool scope_vortex_rpc(ScopeVortexState *v, uint16_t opcode,
                             const void *request, uint32_t request_len,
                             void *response, uint32_t response_len)
{
    struct scope_vortex_rpc_header req = {
        .magic = SCOPE_VORTEX_RPC_MAGIC,
        .version = SCOPE_VORTEX_RPC_VERSION,
        .opcode = opcode,
        .request_id = ++v->rpc_request_id,
        .payload_len = request_len,
    };
    struct scope_vortex_rpc_header rsp;

    if (!scope_vortex_io_full(v->socket_fd, &req, sizeof(req), true) ||
        (request_len && !scope_vortex_io_full(v->socket_fd, (void *)request,
                                              request_len, true)) ||
        !scope_vortex_io_full(v->socket_fd, &rsp, sizeof(rsp), false)) {
        return false;
    }
    if (rsp.magic != SCOPE_VORTEX_RPC_MAGIC ||
        rsp.version != SCOPE_VORTEX_RPC_VERSION ||
        rsp.opcode != opcode || rsp.request_id != req.request_id ||
        rsp.status != 0 || rsp.payload_len != response_len) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE VORTEX: RPC op=%u status=%d payload=%u expected=%u\n",
                      opcode, rsp.status, rsp.payload_len, response_len);
        return false;
    }
    return !response_len || scope_vortex_io_full(v->socket_fd, response,
                                                  response_len, false);
}

static bool scope_vortex_rpc_cp_read(ScopeVortexState *v, uint32_t off,
                                     uint32_t *value)
{
    struct scope_vortex_rpc_reg req = { .offset = off };
    struct scope_vortex_rpc_reg rsp;

    if (!scope_vortex_rpc(v, SCOPE_VORTEX_RPC_CP_READ, &req, sizeof(req),
                          &rsp, sizeof(rsp))) {
        return false;
    }
    *value = rsp.value;
    return true;
}

static bool scope_vortex_rpc_cp_write(ScopeVortexState *v, uint32_t off,
                                      uint32_t value)
{
    struct scope_vortex_rpc_reg req = { .offset = off, .value = value };

    return scope_vortex_rpc(v, SCOPE_VORTEX_RPC_CP_WRITE, &req, sizeof(req),
                            NULL, 0);
}

static bool scope_vortex_rpc_free(ScopeVortexState *v, uint32_t handle);

static bool scope_vortex_rpc_alloc(ScopeVortexState *v, uint64_t size,
                                   uint32_t *handle, uint64_t *cp_addr)
{
    struct scope_vortex_rpc_alloc_req req = { .size = size };
    struct scope_vortex_rpc_alloc_rsp rsp;

    if (!scope_vortex_rpc(v, SCOPE_VORTEX_RPC_MEM_ALLOC, &req, sizeof(req),
                          &rsp, sizeof(rsp))) {
        return false;
    }
    if (!rsp.handle || rsp.size < size) {
        if (rsp.handle) {
            scope_vortex_rpc_free(v, rsp.handle);
        }
        return false;
    }
    *handle = rsp.handle;
    *cp_addr = rsp.cp_addr;
    return true;
}

static bool scope_vortex_rpc_free(ScopeVortexState *v, uint32_t handle)
{
    return !handle || scope_vortex_rpc(v, SCOPE_VORTEX_RPC_MEM_FREE,
                                       &handle, sizeof(handle), NULL, 0);
}

static bool scope_vortex_rpc_mem_write(ScopeVortexState *v, uint32_t handle,
                                       uint64_t off, const void *buf,
                                       uint32_t len)
{
    const uint8_t *src = buf;

    while (len) {
        uint32_t chunk = MIN(len, SCOPE_VX_RPC_IO_CHUNK);
        struct scope_vortex_rpc_mem hdr = {
            .handle = handle, .length = chunk, .offset = off,
        };
        g_autofree uint8_t *payload = g_malloc(sizeof(hdr) + chunk);

        memcpy(payload, &hdr, sizeof(hdr));
        memcpy(payload + sizeof(hdr), src, chunk);
        if (!scope_vortex_rpc(v, SCOPE_VORTEX_RPC_MEM_WRITE, payload,
                              sizeof(hdr) + chunk, NULL, 0)) {
            return false;
        }
        src += chunk;
        off += chunk;
        len -= chunk;
    }
    return true;
}

static bool scope_vortex_rpc_mem_read(ScopeVortexState *v, uint32_t handle,
                                      uint64_t off, void *buf, uint32_t len)
{
    uint8_t *dst = buf;

    while (len) {
        uint32_t chunk = MIN(len, SCOPE_VX_RPC_IO_CHUNK);
        struct scope_vortex_rpc_mem req = {
            .handle = handle, .length = chunk, .offset = off,
        };

        if (!scope_vortex_rpc(v, SCOPE_VORTEX_RPC_MEM_READ,
                              &req, sizeof(req), dst, chunk)) {
            return false;
        }
        dst += chunk;
        off += chunk;
        len -= chunk;
    }
    return true;
}

static bool scope_vortex_rpc_peer_caps(ScopeVortexState *v)
{
    return scope_vortex_rpc(v, SCOPE_VORTEX_RPC_PEER_CAPS, NULL, 0,
                            &v->peer_caps, sizeof(v->peer_caps));
}

static bool scope_vortex_rpc_peer_map(
    ScopeVortexState *v, const struct scope_vortex_rpc_peer_map_req *req,
    struct scope_vortex_rpc_peer_map_rsp *rsp)
{
    return scope_vortex_rpc(v, SCOPE_VORTEX_RPC_PEER_MAP, req, sizeof(*req),
                            rsp, sizeof(*rsp));
}

static bool scope_vortex_rpc_peer_unmap(ScopeVortexState *v)
{
    struct scope_vortex_rpc_peer_unmap req = {
        .generation = v->peer_generation,
    };

    if (!v->peer_window_valid) {
        return true;
    }
    if (!scope_vortex_rpc(v, SCOPE_VORTEX_RPC_PEER_UNMAP,
                          &req, sizeof(req), NULL, 0)) {
        return false;
    }
    v->peer_window_valid = false;
    v->peer_generation = 0;
    return true;
}

static bool scope_vortex_connect(ScopeVortexState *v, const char *path,
                                 Error **errp)
{
    struct sockaddr_un addr = { .sun_family = AF_UNIX };
    struct timeval timeout = { .tv_sec = SCOPE_VX_RPC_SOCKET_TIMEOUT_SEC };
    struct scope_vortex_rpc_hello_req hello_req = {
        .flags = v->direct_p2p ? SCOPE_VORTEX_HELLO_DIRECT_P2P : 0,
    };
    struct scope_vortex_rpc_hello_rsp hello_rsp;

    if (strlen(path) >= sizeof(addr.sun_path)) {
        error_setg(errp, "Vortex bridge socket path is too long");
        return false;
    }
    v->socket_fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (v->socket_fd < 0) {
        error_setg_errno(errp, errno, "cannot create Vortex bridge socket");
        return false;
    }
    g_strlcpy(addr.sun_path, path, sizeof(addr.sun_path));
    if (connect(v->socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        error_setg_errno(errp, errno, "cannot connect Vortex bridge %s", path);
        return false;
    }
    if (setsockopt(v->socket_fd, SOL_SOCKET, SO_RCVTIMEO,
                   &timeout, sizeof(timeout)) < 0 ||
        setsockopt(v->socket_fd, SOL_SOCKET, SO_SNDTIMEO,
                   &timeout, sizeof(timeout)) < 0) {
        error_setg_errno(errp, errno,
                         "cannot set Vortex bridge socket timeout");
        return false;
    }
    if (!scope_vortex_rpc(v, SCOPE_VORTEX_RPC_HELLO,
                          &hello_req, sizeof(hello_req),
                          &hello_rsp, sizeof(hello_rsp)) ||
        hello_rsp.version != SCOPE_VORTEX_RPC_VERSION ||
        (v->direct_p2p &&
         !(hello_rsp.capabilities & SCOPE_VORTEX_RPC_CAP_PEER_MAP))) {
        error_setg(errp, "Vortex bridge protocol handshake failed");
        return false;
    }
    return true;
}

static bool scope_vortex_device_range_valid(ScopeVortexState *v,
                                            uint64_t addr, uint64_t len)
{
    uint64_t dev_caps = v->caps[3] | ((uint64_t)v->caps[4] << 32);
    uint32_t banks_log2 = (dev_caps >> 34) & 0x7U;
    uint32_t bank_size_log2 = 20U + ((dev_caps >> 37) & 0x1fU);
    uint64_t banks = UINT64_C(1) << banks_log2;
    uint64_t bank_size;
    uint64_t total;
    uint64_t aperture_addr;

    if (!len || bank_size_log2 >= 64U) {
        return false;
    }
    bank_size = UINT64_C(1) << bank_size_log2;
    if (banks > UINT64_MAX / bank_size) {
        return false;
    }
    total = banks * bank_size;

    /*
     * The RTL narrows a Vortex logical address to
     * VX_CFG_PLATFORM_MEMORY_ADDR_WIDTH before driving m_axi_mem.  In the
     * single-bank U280 image that width is 28 bits, so the conventional
     * kernel VMA 0x80000000 reaches HBM offset 0.  Validate the same narrowed
     * aperture here; checking the untruncated 64-bit VMA incorrectly rejects
     * every kernel image linked at Vortex's standard high address.
     *
     * banks and bank_size are both powers of two, hence total is a power of
     * two after the overflow check above.
     */
    aperture_addr = addr & (total - 1U);
    return len <= total - aperture_addr;
}

static bool scope_vortex_parse_host_bdf(const char *text, uint16_t *domain,
                                        uint8_t *bus, uint8_t *devfn)
{
    unsigned int dom;
    unsigned int b;
    unsigned int dev;
    unsigned int fn;
    char tail;

    if (!text || sscanf(text, "%x:%x:%x.%x%c", &dom, &b, &dev, &fn,
                        &tail) != 4 || dom > UINT16_MAX || b > UINT8_MAX ||
        dev > 31U || fn > 7U) {
        return false;
    }
    *domain = dom;
    *bus = b;
    *devfn = (dev << 3) | fn;
    return true;
}

static bool scope_vortex_validate_nonmem_line(ScopeVortexState *v,
                                               const uint8_t *line)
{
    uint8_t opcode = line[0];
    uint8_t flags = line[1];
    uint64_t size;

    if (flags & SCOPE_VX_FLAG_PROFILE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE VORTEX: profiled CP commands are not supported\n");
        return false;
    }
    switch (opcode) {
    case SCOPE_VX_OP_NOP:
    case SCOPE_VX_OP_DCR_WRITE:
    case SCOPE_VX_OP_DCR_READ:
    case SCOPE_VX_OP_LAUNCH:
    case SCOPE_VX_OP_FENCE:
    case SCOPE_VX_OP_CACHE_FLUSH:
        return true;
    case SCOPE_VX_OP_MEM_COPY:
        size = ldq_le_p(line + 20);
        return scope_vortex_device_range_valid(v, ldq_le_p(line + 4), size) &&
               scope_vortex_device_range_valid(v, ldq_le_p(line + 12), size);
    case SCOPE_VX_OP_EVENT_SIGNAL:
    case SCOPE_VX_OP_EVENT_WAIT:
    case SCOPE_VX_OP_LAUNCH_QMD:
    case SCOPE_VX_OP_DRAW:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE VORTEX: physical RTL CP does not support opcode 0x%02x\n",
                      opcode);
        return false;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE VORTEX: invalid direct-P2P opcode 0x%02x\n",
                      opcode);
        return false;
    }
}

static bool scope_vortex_map_peer_window(ScopeVortexState *v, uint8_t opcode,
                                         uint64_t guest_pa, uint64_t size,
                                         uint64_t *cp_addr)
{
    ScopeProxyState *s = v->manager;
    struct scope_vortex_rpc_peer_map_req req = { 0 };
    struct scope_vortex_rpc_peer_map_rsp rsp;
    uint64_t rounded;
    uint64_t guest_end;
    uint64_t window_guest_base;
    uint64_t bar_offset;
    uint64_t peer_target;
    int64_t map_started_us;
    uint16_t domain;
    uint8_t bus;
    uint8_t devfn;

    if (!size || size > SCOPE_VORTEX_RPC_MAX_PAYLOAD ||
        size > UINT64_MAX - (SCOPE_VX_CP_CL_SIZE - 1U)) {
        return false;
    }
    rounded = ROUND_UP(size, SCOPE_VX_CP_CL_SIZE);
    if (!scope_guest_pa_in_ddr_window(s, guest_pa, rounded) ||
        s->guest_ddr_base > UINT64_MAX - s->guest_ddr_size) {
        return false;
    }
    guest_end = s->guest_ddr_base + s->guest_ddr_size;
    if (s->guest_ddr_size < SCOPE_VX_PEER_WINDOW_SIZE) {
        return false;
    }
    window_guest_base = QEMU_ALIGN_DOWN(guest_pa, SCOPE_VX_PEER_SLOT_SIZE);
    if (window_guest_base > guest_end - SCOPE_VX_PEER_WINDOW_SIZE) {
        window_guest_base = guest_end - SCOPE_VX_PEER_WINDOW_SIZE;
    }
    if (guest_pa < window_guest_base ||
        rounded > SCOPE_VX_PEER_WINDOW_SIZE - (guest_pa - window_guest_base) ||
        !scope_guest_range_to_coherent_bar_offset(
            s, window_guest_base, SCOPE_VX_PEER_WINDOW_SIZE, &bar_offset) ||
        !scope_translate_guest_pa_for_real_dma(s, guest_pa, rounded,
                                               &peer_target)) {
        return false;
    }

    if (!v->peer_window_valid ||
        v->peer_window_guest_base != window_guest_base) {
        if (!scope_vortex_parse_host_bdf(s->fpga_host_bdf, &domain,
                                         &bus, &devfn) ||
            s->fpga_bypass_bar_index < 0 ||
            s->fpga_bypass_bar_index > UINT8_MAX) {
            return false;
        }
        req.domain = domain;
        req.bus = bus;
        req.devfn = devfn;
        req.bar = s->fpga_bypass_bar_index;
        req.bar_offset = bar_offset;
        req.window_size = SCOPE_VX_PEER_WINDOW_SIZE;
        map_started_us = g_get_monotonic_time();
        if (!scope_vortex_rpc_peer_map(v, &req, &rsp) ||
            rsp.cp_peer_base != v->peer_caps.peer_base ||
            rsp.window_size != SCOPE_VX_PEER_WINDOW_SIZE ||
            rsp.slot_size != SCOPE_VX_PEER_SLOT_SIZE || !rsp.generation) {
            return false;
        }
        v->peer_window_valid = true;
        v->peer_window_guest_base = window_guest_base;
        v->peer_generation = rsp.generation;
        scope_vortex_trace(v,
                           "path=direct-p2p peer-map window_guest_base=0x%" PRIx64
                           " bar_offset=0x%" PRIx64 " generation=%" PRIu64
                           " map_latency_us=%" PRId64 "\n",
                           window_guest_base, bar_offset, rsp.generation,
                           g_get_monotonic_time() - map_started_us);
    }

    *cp_addr = v->peer_caps.peer_base + guest_pa - window_guest_base;
    scope_vortex_trace(v,
                       "path=direct-p2p direction=%s guest_pa=0x%" PRIx64
                       " bar_offset=0x%" PRIx64 " peer_target=0x%" PRIx64
                       " window_guest_base=0x%" PRIx64
                       " cp_peer_addr=0x%" PRIx64 " generation=%" PRIu64
                       " bytes=%" PRIu64 " payload_cpu_bytes=0\n",
                       opcode == SCOPE_VX_OP_MEM_WRITE ? "PCIe-MRd" : "PCIe-MWr",
                       guest_pa,
                       bar_offset + guest_pa - window_guest_base,
                       peer_target, window_guest_base, *cp_addr,
                       v->peer_generation, size);
    return true;
}

static bool scope_vortex_patch_direct_mem(ScopeVortexState *v, uint8_t *line)
{
    uint8_t opcode = line[0];
    uint64_t guest_pa;
    uint64_t device_addr;
    uint64_t size;
    uint64_t rounded;
    uint64_t cp_addr;

    if ((opcode != SCOPE_VX_OP_MEM_WRITE && opcode != SCOPE_VX_OP_MEM_READ) ||
        (line[1] & SCOPE_VX_FLAG_PROFILE)) {
        return false;
    }
    guest_pa = ldq_le_p(line + (opcode == SCOPE_VX_OP_MEM_WRITE ? 12 : 4));
    device_addr = ldq_le_p(line + (opcode == SCOPE_VX_OP_MEM_WRITE ? 4 : 12));
    size = ldq_le_p(line + 20);
    if (!size || size > UINT64_MAX - (SCOPE_VX_CP_CL_SIZE - 1U)) {
        return false;
    }
    rounded = ROUND_UP(size, SCOPE_VX_CP_CL_SIZE);
    if (!scope_vortex_device_range_valid(v, device_addr, rounded) ||
        !scope_vortex_map_peer_window(v, opcode, guest_pa, size, &cp_addr)) {
        return false;
    }
    stq_le_p(line + (opcode == SCOPE_VX_OP_MEM_WRITE ? 12 : 4), cp_addr);
    return true;
}

static bool scope_vortex_patch_line(ScopeVortexState *v, uint8_t *line,
                                    ScopeVortexTransfer *xfer)
{
    ScopeProxyState *s = v->manager;
    uint8_t opcode = line[0];
    uint8_t flags = line[1];
    uint64_t guest_pa;
    uint64_t size64;
    uint64_t device_addr;
    uint64_t alloc_size;
    g_autofree uint8_t *payload = NULL;

    memset(xfer, 0, sizeof(*xfer));
    if (flags & SCOPE_VX_FLAG_PROFILE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE VORTEX: profiled CP commands are not supported\n");
        return false;
    }
    switch (opcode) {
    case SCOPE_VX_OP_MEM_WRITE:
    case SCOPE_VX_OP_MEM_READ:
        guest_pa = ldq_le_p(line + (opcode == SCOPE_VX_OP_MEM_WRITE ? 12 : 4));
        device_addr = ldq_le_p(line +
                               (opcode == SCOPE_VX_OP_MEM_WRITE ? 4 : 12));
        size64 = ldq_le_p(line + 20);
        alloc_size = ROUND_UP(size64, SCOPE_VX_CP_CL_SIZE);
        if (!size64 || alloc_size < size64 ||
            alloc_size > SCOPE_VORTEX_RPC_MAX_PAYLOAD ||
            !scope_guest_pa_in_ddr_window(s, guest_pa, size64) ||
            !scope_vortex_device_range_valid(v, device_addr, size64) ||
            !scope_vortex_rpc_alloc(v, alloc_size,
                                    &xfer->handle, &xfer->cp_addr)) {
            return false;
        }
        xfer->guest_pa = guest_pa;
        xfer->size = size64;
        xfer->download = opcode == SCOPE_VX_OP_MEM_READ;
        if (!xfer->download) {
            payload = g_malloc(xfer->size);
            if (!scope_guest_mem_read(s, guest_pa, payload, xfer->size) ||
                !scope_vortex_rpc_mem_write(v, xfer->handle, 0,
                                            payload, xfer->size)) {
                scope_vortex_rpc_free(v, xfer->handle);
                xfer->handle = 0;
                return false;
            }
            v->payload_cpu_bytes += xfer->size;
        }
        stq_le_p(line + (xfer->download ? 4 : 12), xfer->cp_addr);
        scope_vortex_trace(v,
                           "%s guest_pa=0x%" PRIx64
                           " device=0x%" PRIx64 " bytes=%u xrt_cp=0x%" PRIx64 "\n",
                           xfer->download ? "download" : "upload",
                           guest_pa, device_addr, xfer->size, xfer->cp_addr);
        return true;
    case SCOPE_VX_OP_EVENT_SIGNAL:
    case SCOPE_VX_OP_EVENT_WAIT:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE VORTEX: device event command 0x%02x is unsupported\n",
                      opcode);
        return false;
    case SCOPE_VX_OP_NOP:
        return true;
    case SCOPE_VX_OP_MEM_COPY:
        size64 = ldq_le_p(line + 20);
        return scope_vortex_device_range_valid(v, ldq_le_p(line + 4), size64) &&
               scope_vortex_device_range_valid(v, ldq_le_p(line + 12), size64);
    case SCOPE_VX_OP_DCR_WRITE:
    case SCOPE_VX_OP_DCR_READ:
    case SCOPE_VX_OP_LAUNCH:
    case SCOPE_VX_OP_FENCE:
    case SCOPE_VX_OP_CACHE_FLUSH:
        return true;
    case SCOPE_VX_OP_LAUNCH_QMD:
    case SCOPE_VX_OP_DRAW:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE VORTEX: physical RTL CP does not support opcode 0x%02x\n",
                      opcode);
        return false;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE VORTEX: unknown CP opcode 0x%02x\n", opcode);
        return false;
    }
}

static bool scope_vortex_finish_transfer(ScopeVortexState *v,
                                         ScopeVortexTransfer *xfer)
{
    g_autofree uint8_t *payload = NULL;
    bool ok = true;

    if (!xfer->handle) {
        return true;
    }
    if (xfer->download) {
        payload = g_malloc(xfer->size);
        ok = scope_vortex_rpc_mem_read(v, xfer->handle, 0, payload,
                                       xfer->size) &&
             scope_guest_mem_write(v->manager, xfer->guest_pa, payload,
                                   xfer->size);
        v->payload_cpu_bytes += xfer->size;
    }
    return scope_vortex_rpc_free(v, xfer->handle) && ok;
}

static bool scope_vortex_execute_job_mediated(ScopeVortexState *v,
                                              ScopeVortexJob *job)
{
    g_autofree ScopeVortexTransfer *xfers =
        g_new0(ScopeVortexTransfer, job->line_count);
    uint32_t i;
    uint32_t target;
    uint32_t cycle_lo = 0;
    uint32_t cycle_hi = 0;
    uint32_t last_dcr_rsp = 0;
    int64_t deadline;
    int64_t started_us = g_get_monotonic_time();
    uint64_t new_tail = v->physical_tail;
    bool cycle_valid = false;
    bool last_dcr_valid = false;
    bool ok = true;

    scope_vortex_trace(v,
                       "job start guest_tail=0x%" PRIx64
                       " lines=%u virtual_target=%u physical_seq=%u\n",
                       job->guest_tail, job->line_count, job->target_seq,
                       v->physical_seq);

    for (i = 0; i < job->line_count; i++) {
        uint8_t *line = job->lines + i * SCOPE_VX_CP_CL_SIZE;
        uint64_t ring_off = new_tail & (SCOPE_VX_CP_RING_SIZE - 1U);
        uint8_t opcode = line[0];

        scope_vortex_trace(v,
                           "cmd=%u op=%s(0x%02x) arg0=0x%" PRIx64
                           " arg1=0x%" PRIx64 " arg2=0x%" PRIx64 "\n",
                           i, scope_vortex_opcode_name(opcode), opcode,
                           ldq_le_p(line + 4), ldq_le_p(line + 12),
                           ldq_le_p(line + 20));
        if (!scope_vortex_patch_line(v, line, &xfers[i]) ||
            !scope_vortex_rpc_mem_write(v, v->physical_ring_handle, ring_off,
                                        line, SCOPE_VX_CP_CL_SIZE)) {
            ok = false;
            break;
        }
        new_tail += SCOPE_VX_CP_CL_SIZE;
    }
    if (ok) {
        v->physical_tail = new_tail;
        target = v->physical_seq + job->line_count;
        ok = scope_vortex_rpc_cp_write(v, SCOPE_VX_CP_Q_TAIL_LO,
                                       (uint32_t)v->physical_tail) &&
             scope_vortex_rpc_cp_write(v, SCOPE_VX_CP_Q_TAIL_HI,
                                       (uint32_t)(v->physical_tail >> 32));
        deadline = g_get_monotonic_time() + SCOPE_VX_CP_TIMEOUT_US;
        while (ok && !qatomic_read(&v->worker_stop)) {
            uint32_t seq = 0;
            ok = scope_vortex_rpc_cp_read(v, SCOPE_VX_CP_Q_SEQNUM, &seq);
            if (!ok || (int32_t)(seq - target) >= 0) {
                v->physical_seq = seq;
                break;
            }
            if (g_get_monotonic_time() >= deadline) {
                ok = false;
                break;
            }
            g_usleep(SCOPE_VX_CP_POLL_US);
        }
    }
    if (ok) {
        /* The RX thread serves cached telemetry; only this worker owns RPC. */
        cycle_valid =
            scope_vortex_rpc_cp_read(v, SCOPE_VX_CP_CYCLE_LO, &cycle_lo) &&
            scope_vortex_rpc_cp_read(v, SCOPE_VX_CP_CYCLE_HI, &cycle_hi);
        for (i = 0; i < job->line_count; i++) {
            if (!scope_vortex_finish_transfer(v, &xfers[i])) {
                ok = false;
            }
            if (job->lines[i * SCOPE_VX_CP_CL_SIZE] == SCOPE_VX_OP_DCR_READ) {
                last_dcr_valid = scope_vortex_rpc_cp_read(
                    v, SCOPE_VX_CP_Q_LAST_DCR, &last_dcr_rsp);
            }
        }
    } else {
        for (i = 0; i < job->line_count; i++) {
            if (xfers[i].handle) {
                scope_vortex_rpc_free(v, xfers[i].handle);
            }
        }
    }

    qemu_mutex_lock(&v->manager->state_lock);
    if (cycle_valid) {
        v->caps[1] = cycle_lo;
        v->caps[2] = cycle_hi;
    }
    if (last_dcr_valid) {
        v->last_dcr_rsp = last_dcr_rsp;
    }
    if (!ok) {
        v->q_error = 1;
        v->failed = true;
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE VORTEX: backend=%u CP job failed at guest tail=0x%" PRIx64 "\n",
                      v->backend->id, job->guest_tail);
    }
    /* Advance on error as well so userspace observes failure instead of hanging. */
    v->retired_seq = job->target_seq;
    qemu_mutex_unlock(&v->manager->state_lock);
    if (cycle_valid) {
        scope_vortex_trace(v,
                           "job %s virtual_seq=%u physical_seq=%u cycles=%" PRIu64
                           " elapsed_us=%" PRId64 "\n",
                           ok ? "done" : "failed", job->target_seq,
                           v->physical_seq,
                           (uint64_t)cycle_lo | ((uint64_t)cycle_hi << 32),
                           g_get_monotonic_time() - started_us);
    } else {
        scope_vortex_trace(v,
                           "job %s virtual_seq=%u physical_seq=%u elapsed_us=%"
                           PRId64 "\n",
                           ok ? "done" : "failed", job->target_seq,
                           v->physical_seq,
                           g_get_monotonic_time() - started_us);
    }
    return ok;
}

static bool scope_vortex_submit_physical(ScopeVortexState *v,
                                          const uint8_t *lines,
                                          uint32_t line_count)
{
    uint64_t new_tail = v->physical_tail;
    uint32_t target;
    uint32_t i;
    uint32_t q_error = 0;
    int64_t deadline;
    int64_t started_us = g_get_monotonic_time();
    bool ok = true;

    if (!line_count) {
        return true;
    }
    for (i = 0; i < line_count; i++) {
        uint64_t ring_off = new_tail & (SCOPE_VX_CP_RING_SIZE - 1U);

        if (!scope_vortex_rpc_mem_write(v, v->physical_ring_handle, ring_off,
                                        lines + i * SCOPE_VX_CP_CL_SIZE,
                                        SCOPE_VX_CP_CL_SIZE)) {
            return false;
        }
        new_tail += SCOPE_VX_CP_CL_SIZE;
    }

    target = v->physical_seq + line_count;
    if (!scope_vortex_rpc_cp_write(v, SCOPE_VX_CP_Q_TAIL_LO,
                                   (uint32_t)new_tail) ||
        !scope_vortex_rpc_cp_write(v, SCOPE_VX_CP_Q_TAIL_HI,
                                   (uint32_t)(new_tail >> 32))) {
        return false;
    }
    v->physical_tail = new_tail;
    deadline = g_get_monotonic_time() + SCOPE_VX_CP_TIMEOUT_US;
    while (!qatomic_read(&v->worker_stop)) {
        uint32_t seq = 0;

        ok = scope_vortex_rpc_cp_read(v, SCOPE_VX_CP_Q_SEQNUM, &seq);
        if (!ok) {
            break;
        }
        if ((int32_t)(seq - target) >= 0) {
            v->physical_seq = seq;
            break;
        }
        if (g_get_monotonic_time() >= deadline) {
            ok = false;
            break;
        }
        g_usleep(SCOPE_VX_CP_POLL_US);
    }
    if (qatomic_read(&v->worker_stop)) {
        ok = false;
    }
    if (ok) {
        ok = scope_vortex_rpc_cp_read(v, SCOPE_VX_CP_Q_ERROR, &q_error) &&
             q_error == 0;
    }
    scope_vortex_trace(v,
                       "path=direct-p2p physical_batch=%u physical_seq=%u"
                       " q_error=%u command_latency_us=%" PRId64 "\n",
                       line_count, v->physical_seq, q_error,
                       g_get_monotonic_time() - started_us);
    return ok;
}

static void scope_vortex_retire_direct(ScopeVortexState *v,
                                       uint32_t virtual_seq)
{
    qemu_mutex_lock(&v->manager->state_lock);
    v->retired_seq = virtual_seq;
    qemu_mutex_unlock(&v->manager->state_lock);
}

static void scope_vortex_fail_direct(ScopeVortexState *v,
                                     ScopeVortexJob *job)
{
    scope_vortex_rpc_cp_write(v, SCOPE_VX_CP_CTRL, 0);
    scope_vortex_rpc_peer_unmap(v);
    scope_vortex_rpc_cp_write(v, SCOPE_VX_CP_Q_CONTROL, 2);
    scope_vortex_rpc_cp_write(v, SCOPE_VX_CP_CTRL, 2);

    qemu_mutex_lock(&v->manager->state_lock);
    v->q_error = 1;
    v->failed = true;
    /* A failed virtual command retires with Q_ERROR instead of hanging. */
    v->retired_seq = job->target_seq;
    qemu_mutex_unlock(&v->manager->state_lock);
    qemu_log_mask(LOG_GUEST_ERROR,
                  "SCOPE VORTEX: backend=%u direct-P2P job failed at "
                  "guest tail=0x%" PRIx64 "\n",
                  v->backend->id, job->guest_tail);
}

static bool scope_vortex_execute_job_direct(ScopeVortexState *v,
                                            ScopeVortexJob *job)
{
    uint32_t virtual_base = job->target_seq - job->line_count;
    uint32_t i = 0;
    uint32_t completed = 0;
    uint32_t cycle_lo = 0;
    uint32_t cycle_hi = 0;
    uint32_t last_dcr_rsp = 0;
    int64_t started_us = g_get_monotonic_time();
    bool cycle_valid = false;
    bool last_dcr_valid = false;
    bool ok = true;

    scope_vortex_trace(v,
                       "job start path=direct-p2p guest_tail=0x%" PRIx64
                       " lines=%u virtual_target=%u physical_seq=%u\n",
                       job->guest_tail, job->line_count, job->target_seq,
                       v->physical_seq);

    while (i < job->line_count && ok) {
        uint8_t *line = job->lines + i * SCOPE_VX_CP_CL_SIZE;
        uint8_t opcode = line[0];

        scope_vortex_trace(v,
                           "cmd=%u op=%s(0x%02x) arg0=0x%" PRIx64
                           " arg1=0x%" PRIx64 " arg2=0x%" PRIx64 "\n",
                           i, scope_vortex_opcode_name(opcode), opcode,
                           ldq_le_p(line + 4), ldq_le_p(line + 12),
                           ldq_le_p(line + 20));
        if (opcode == SCOPE_VX_OP_MEM_WRITE ||
            opcode == SCOPE_VX_OP_MEM_READ) {
            uint8_t patched[SCOPE_VX_CP_CL_SIZE];

            memcpy(patched, line, sizeof(patched));
            ok = scope_vortex_patch_direct_mem(v, patched) &&
                 scope_vortex_submit_physical(v, patched, 1);
            if (ok) {
                ++i;
                ++completed;
                scope_vortex_retire_direct(v, virtual_base + completed);
            }
            continue;
        }

        {
            uint32_t batch_start = i;

            while (i < job->line_count) {
                line = job->lines + i * SCOPE_VX_CP_CL_SIZE;
                opcode = line[0];
                if (opcode == SCOPE_VX_OP_MEM_WRITE ||
                    opcode == SCOPE_VX_OP_MEM_READ) {
                    break;
                }
                scope_vortex_trace(v,
                                   "cmd=%u op=%s(0x%02x) arg0=0x%" PRIx64
                                   " arg1=0x%" PRIx64 " arg2=0x%" PRIx64 "\n",
                                   i, scope_vortex_opcode_name(opcode), opcode,
                                   ldq_le_p(line + 4), ldq_le_p(line + 12),
                                   ldq_le_p(line + 20));
                if (!scope_vortex_validate_nonmem_line(v, line)) {
                    ok = false;
                    break;
                }
                ++i;
            }
            if (ok) {
                uint32_t batch_count = i - batch_start;
                uint32_t j;

                ok = scope_vortex_submit_physical(
                    v, job->lines + batch_start * SCOPE_VX_CP_CL_SIZE,
                    batch_count);
                if (ok) {
                    for (j = batch_start; j < i; ++j) {
                        if (job->lines[j * SCOPE_VX_CP_CL_SIZE] ==
                            SCOPE_VX_OP_DCR_READ) {
                            last_dcr_valid = scope_vortex_rpc_cp_read(
                                v, SCOPE_VX_CP_Q_LAST_DCR, &last_dcr_rsp);
                            if (!last_dcr_valid) {
                                ok = false;
                                break;
                            }
                        }
                    }
                }
                if (ok) {
                    completed += batch_count;
                    scope_vortex_retire_direct(v, virtual_base + completed);
                }
            }
        }
    }

    if (ok) {
        cycle_valid =
            scope_vortex_rpc_cp_read(v, SCOPE_VX_CP_CYCLE_LO, &cycle_lo) &&
            scope_vortex_rpc_cp_read(v, SCOPE_VX_CP_CYCLE_HI, &cycle_hi);
        qemu_mutex_lock(&v->manager->state_lock);
        if (cycle_valid) {
            v->caps[1] = cycle_lo;
            v->caps[2] = cycle_hi;
        }
        if (last_dcr_valid) {
            v->last_dcr_rsp = last_dcr_rsp;
        }
        qemu_mutex_unlock(&v->manager->state_lock);
    } else {
        scope_vortex_fail_direct(v, job);
    }

    scope_vortex_trace(v,
                       "job %s path=direct-p2p virtual_seq=%u physical_seq=%u"
                       " elapsed_us=%" PRId64 " payload_cpu_bytes=0\n",
                       ok ? "done" : "failed", job->target_seq,
                       v->physical_seq,
                       g_get_monotonic_time() - started_us);
    return ok;
}

static bool scope_vortex_execute_job(ScopeVortexState *v, ScopeVortexJob *job)
{
    if (v->direct_p2p) {
        return scope_vortex_execute_job_direct(v, job);
    }
    return scope_vortex_execute_job_mediated(v, job);
}

static void *scope_vortex_worker(void *opaque)
{
    ScopeVortexState *v = opaque;

    while (!qatomic_read(&v->worker_stop)) {
        ScopeVortexJob *job = g_async_queue_timeout_pop(v->jobs, 100000);
        if (!job) {
            continue;
        }
        if ((void *)job == (void *)v) {
            break;
        }
        qemu_mutex_lock(&v->manager->state_lock);
        if (v->failed) {
            /* Retire queued virtual work without touching a failed CP/socket. */
            v->retired_seq = job->target_seq;
            qemu_mutex_unlock(&v->manager->state_lock);
        } else {
            qemu_mutex_unlock(&v->manager->state_lock);
            scope_vortex_execute_job(v, job);
        }
        g_free(job);
    }
    return NULL;
}

static bool scope_vortex_try_submit_tail(ScopeVortexState *v)
{
    ScopeProxyState *s = v->manager;
    uint64_t delta;
    uint32_t count;
    ScopeVortexJob *job;
    uint32_t i;

    if (!v->pending_tail_valid || !(v->guest_q_control & 1U) ||
        v->guest_ring_log2 != SCOPE_VX_CP_RING_LOG2) {
        return false;
    }
    if (v->pending_tail < v->submitted_tail) {
        v->q_error = 2;
        v->pending_tail_valid = false;
        return true;
    }
    delta = v->pending_tail - v->submitted_tail;
    if (!delta || (delta & (SCOPE_VX_CP_CL_SIZE - 1U)) ||
        delta > SCOPE_VX_CP_RING_SIZE) {
        if (delta) {
            v->q_error = 3;
        }
        v->pending_tail_valid = false;
        return true;
    }
    count = delta / SCOPE_VX_CP_CL_SIZE;
    if (v->failed || g_get_monotonic_time() >= v->pending_tail_deadline_us) {
        v->q_error = v->failed ? 1 : 5;
        v->submitted_tail = v->pending_tail;
        v->submitted_seq += count;
        v->retired_seq = v->submitted_seq;
        v->pending_tail_valid = false;
        if (!v->failed) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "SCOPE VORTEX: backend=%u guest command ring visibility timeout\n",
                          v->backend->id);
        }
        return true;
    }
    job = g_malloc0(sizeof(*job) + delta);
    job->guest_tail = v->pending_tail;
    job->line_count = count;
    job->target_seq = v->submitted_seq + count;

    for (i = 0; i < count; i++) {
        uint8_t first[SCOPE_VX_CP_CL_SIZE];
        uint8_t second[SCOPE_VX_CP_CL_SIZE];
        uint64_t ring_off = (v->submitted_tail + i * SCOPE_VX_CP_CL_SIZE) &
                            (SCOPE_VX_CP_RING_SIZE - 1U);
        uint64_t guest_pa = v->guest_ring_base + ring_off;

        if (!scope_guest_mem_read(s, guest_pa, first, sizeof(first)) ||
            !scope_guest_mem_read(s, guest_pa, second, sizeof(second)) ||
            memcmp(first, second, sizeof(first))) {
            g_free(job);
            return false;
        }
        memcpy(job->lines + i * SCOPE_VX_CP_CL_SIZE, first, sizeof(first));
    }
    v->submitted_tail = v->pending_tail;
    v->submitted_seq = job->target_seq;
    v->pending_tail_valid = false;
    scope_vortex_trace(v,
                       "job queued guest_tail=0x%" PRIx64
                       " lines=%u virtual_target=%u\n",
                       job->guest_tail, job->line_count, job->target_seq);
    g_async_queue_push(v->jobs, job);
    return true;
}

static bool scope_vortex_bar_read(ScopeVortexState *v, uint32_t off,
                                  uint32_t *value)
{
    switch (off) {
    case SCOPE_VX_CP_CTRL: *value = v->cp_ctrl; return true;
    case SCOPE_VX_CP_DEV_CAPS: *value = v->caps[0]; return true;
    case SCOPE_VX_CP_CYCLE_LO: *value = v->caps[1]; return true;
    case SCOPE_VX_CP_CYCLE_HI: *value = v->caps[2]; return true;
    case SCOPE_VX_CP_GPU_CAPS_LO: *value = v->caps[3]; return true;
    case SCOPE_VX_CP_GPU_CAPS_HI: *value = v->caps[4]; return true;
    case SCOPE_VX_CP_ISA_CAPS_LO: *value = v->caps[5]; return true;
    case SCOPE_VX_CP_ISA_CAPS_HI: *value = v->caps[6]; return true;
    case SCOPE_VX_CP_Q_RING_LO: *value = v->guest_ring_base; return true;
    case SCOPE_VX_CP_Q_RING_HI: *value = v->guest_ring_base >> 32; return true;
    case SCOPE_VX_CP_Q_HEAD_LO: *value = v->guest_head_addr; return true;
    case SCOPE_VX_CP_Q_HEAD_HI: *value = v->guest_head_addr >> 32; return true;
    case SCOPE_VX_CP_Q_CMPL_LO: *value = v->guest_cmpl_addr; return true;
    case SCOPE_VX_CP_Q_CMPL_HI: *value = v->guest_cmpl_addr >> 32; return true;
    case SCOPE_VX_CP_Q_RING_LOG2: *value = v->guest_ring_log2; return true;
    case SCOPE_VX_CP_Q_CONTROL: *value = v->guest_q_control; return true;
    case SCOPE_VX_CP_Q_TAIL_LO: *value = v->guest_tail_lo; return true;
    case SCOPE_VX_CP_Q_TAIL_HI: *value = v->submitted_tail >> 32; return true;
    case SCOPE_VX_CP_Q_SEQNUM: *value = v->retired_seq; return true;
    case SCOPE_VX_CP_Q_ERROR: *value = v->q_error; return true;
    case SCOPE_VX_CP_Q_LAST_DCR: *value = v->last_dcr_rsp; return true;
    default: return false;
    }
}

static bool scope_vortex_bar_write(ScopeVortexState *v, uint32_t off,
                                   uint32_t value)
{
    switch (off) {
    case SCOPE_VX_CP_CTRL:
        v->cp_ctrl = value & 1U;
        return true;
    case SCOPE_VX_CP_Q_RING_LO:
        v->guest_ring_base = (v->guest_ring_base & ~UINT64_C(0xffffffff)) | value;
        return true;
    case SCOPE_VX_CP_Q_RING_HI:
        v->guest_ring_base = (v->guest_ring_base & UINT32_MAX) |
                             ((uint64_t)value << 32);
        return true;
    case SCOPE_VX_CP_Q_HEAD_LO:
        v->guest_head_addr = (v->guest_head_addr & ~UINT64_C(0xffffffff)) | value;
        return true;
    case SCOPE_VX_CP_Q_HEAD_HI:
        v->guest_head_addr = (v->guest_head_addr & UINT32_MAX) |
                             ((uint64_t)value << 32);
        return true;
    case SCOPE_VX_CP_Q_CMPL_LO:
        v->guest_cmpl_addr = (v->guest_cmpl_addr & ~UINT64_C(0xffffffff)) | value;
        return true;
    case SCOPE_VX_CP_Q_CMPL_HI:
        v->guest_cmpl_addr = (v->guest_cmpl_addr & UINT32_MAX) |
                             ((uint64_t)value << 32);
        return true;
    case SCOPE_VX_CP_Q_RING_LOG2:
        v->guest_ring_log2 = value & 0xffU;
        return true;
    case SCOPE_VX_CP_Q_CONTROL:
        v->guest_q_control = value & 1U;
        if (value & 2U) {
            v->submitted_tail = 0;
            v->pending_tail_valid = false;
            v->submitted_seq = 0;
            v->retired_seq = 0;
            v->q_error = v->failed ? 1 : 0;
        }
        return true;
    case SCOPE_VX_CP_Q_TAIL_LO:
        v->guest_tail_lo = value;
        return true;
    case SCOPE_VX_CP_Q_TAIL_HI:
        if (v->pending_tail_valid) {
            v->q_error = 4;
            return false;
        }
        v->pending_tail = ((uint64_t)value << 32) | v->guest_tail_lo;
        v->pending_tail_valid = true;
        v->pending_tail_deadline_us =
            g_get_monotonic_time() + SCOPE_VX_GUEST_VISIBLE_TIMEOUT_US;
        scope_vortex_try_submit_tail(v);
        return true;
    default:
        return false;
    }
}

static void scope_vortex_process_bar_packet(
    ScopeProxyState *s, const struct scope_dma32_packet *pkt)
{
    ScopeVortexState *v = s->active->vortex;
    uint8_t size = SCOPE_VSWITCH_PKT_SIZE(pkt->flags);
    uint8_t wstrb = SCOPE_VSWITCH_PKT_WSTRB(pkt->flags);
    uint8_t lane = pkt->bar_offset & 0x7U;
    uint8_t full_wstrb = 0x0fU << lane;
    uint64_t lane_data = ((uint64_t)pkt->guest_addr_lo << 32) | pkt->data;
    uint32_t value = scope_extract_dword32(lane_data, pkt->bar_offset);
    bool ok = false;

    if (!v || size != 4 || (pkt->bar_offset & 3) ||
        pkt->bar_offset >= SCOPE_VORTEX_BAR0_SIZE) {
        scope_write_bar_response(s, pkt->seq, 0x2U,
                                 pkt->type == SCOPE_PKT_TYPE_BAR_READ ?
                                 UINT64_MAX : 0,
                                 pkt->type == SCOPE_PKT_TYPE_BAR_READ, false);
        return;
    }
    if (pkt->type == SCOPE_PKT_TYPE_BAR_READ) {
        ok = scope_vortex_bar_read(v, pkt->bar_offset, &value);
        scope_write_bar_response(s, pkt->seq, ok ? 0 : 0x2U,
                                 ok ? scope_pack_dword32_for_offset(
                                          value, pkt->bar_offset) :
                                      UINT64_MAX,
                                 true, false);
    } else if (pkt->type == SCOPE_PKT_TYPE_BAR_WRITE) {
        ok = wstrb == full_wstrb &&
             scope_vortex_bar_write(v, pkt->bar_offset, value);
        scope_write_bar_response(s, pkt->seq, ok ? 0 : 0x2U,
                                 0, false, false);
    }
}

static bool scope_vortex_poll(ScopeProxyState *s, int64_t now_us)
{
    ScopeVortexState *v = s->active->vortex;

    return v && v->pending_tail_valid && scope_vortex_try_submit_tail(v);
}

static bool scope_vortex_backend_realize(ScopeProxyState *s, Error **errp)
{
    ScopeBackend *be = s->active;
    ScopeVortexState *v = g_new0(ScopeVortexState, 1);
    uint32_t i;

    be->vortex = v;
    v->manager = s;
    v->backend = be;
    v->socket_fd = -1;
    v->guest_ring_log2 = SCOPE_VX_CP_RING_LOG2;
    v->direct_p2p = be->vortex_direct_p2p;
    if (!scope_vortex_connect(v, be->bridge_socket, errp)) {
        return false;
    }
    if (v->direct_p2p) {
        if (s->guest_ddr_base != SCOPE_VX_P2P_GUEST_DDR_BASE ||
            s->guest_ddr_size != SCOPE_VX_P2P_GUEST_DDR_SIZE ||
            s->bypass_coherent_alias_base != SCOPE_VX_P2P_ALIAS_OFFSET ||
            s->fpga_bypass_bar_index != SCOPE_VX_P2P_BAR_INDEX) {
            error_setg(errp,
                       "Vortex direct-P2P requires guest DDR "
                       "[0x80000000,0x100000000), coherent alias 0x100000000, "
                       "and NM37 BAR2");
            return false;
        }
        if (!scope_vortex_rpc_peer_caps(v) ||
            !(v->peer_caps.flags & SCOPE_VORTEX_RPC_CAP_PEER_MAP) ||
            v->peer_caps.slot_size != SCOPE_VX_PEER_SLOT_SIZE ||
            v->peer_caps.control_size != SCOPE_VX_PEER_WINDOW_SIZE ||
            v->peer_caps.peer_size != SCOPE_VX_PEER_WINDOW_SIZE ||
            v->peer_caps.host_base >
                UINT64_MAX - v->peer_caps.control_size ||
            v->peer_caps.peer_base !=
                v->peer_caps.host_base + v->peer_caps.control_size ||
            !s->fpga_host_bdf || s->fpga_bypass_bar_index < 0) {
            error_setg(errp,
                       "Vortex direct-P2P peer capabilities or NM37 BAR "
                       "configuration are incompatible");
            return false;
        }
        scope_vortex_trace(v,
                           "path=direct-p2p control=[0x%" PRIx64
                           ",+0x%" PRIx64 "] peer=[0x%" PRIx64
                           ",+0x%" PRIx64 "] slot=0x%" PRIx64 "\n",
                           v->peer_caps.host_base, v->peer_caps.control_size,
                           v->peer_caps.peer_base, v->peer_caps.peer_size,
                           v->peer_caps.slot_size);
    }
    if (!scope_vortex_rpc_cp_write(v, SCOPE_VX_CP_CTRL, 2) ||
        !scope_vortex_rpc_cp_write(v, SCOPE_VX_CP_Q_CONTROL, 2) ||
        !scope_vortex_rpc_alloc(v, SCOPE_VX_CP_RING_SIZE,
                                &v->physical_ring_handle,
                                &v->physical_ring_addr) ||
        !scope_vortex_rpc_alloc(v, SCOPE_VX_CP_CL_SIZE,
                                &v->physical_head_handle,
                                &v->physical_head_addr) ||
        !scope_vortex_rpc_alloc(v, SCOPE_VX_CP_CL_SIZE,
                                &v->physical_cmpl_handle,
                                &v->physical_cmpl_addr)) {
        error_setg(errp, "failed to initialize physical Vortex CP buffers");
        return false;
    }
    for (i = 0; i < ARRAY_SIZE(v->caps); i++) {
        static const uint32_t offsets[] = {
            SCOPE_VX_CP_DEV_CAPS, SCOPE_VX_CP_CYCLE_LO,
            SCOPE_VX_CP_CYCLE_HI, SCOPE_VX_CP_GPU_CAPS_LO,
            SCOPE_VX_CP_GPU_CAPS_HI, SCOPE_VX_CP_ISA_CAPS_LO,
            SCOPE_VX_CP_ISA_CAPS_HI,
        };
        if (!scope_vortex_rpc_cp_read(v, offsets[i], &v->caps[i])) {
            error_setg(errp, "failed to read physical Vortex capabilities");
            return false;
        }
    }
    if ((v->caps[0] & 0xffU) < 1U ||
        ((v->caps[0] >> 8) & 0xffU) < SCOPE_VX_CP_RING_LOG2) {
        error_setg(errp,
                   "physical Vortex CP capabilities 0x%08x do not support "
                   "queue 0 with a %u-byte ring",
                   v->caps[0], SCOPE_VX_CP_RING_SIZE);
        return false;
    }
    if (!scope_vortex_rpc_cp_write(v, SCOPE_VX_CP_Q_RING_LO,
                                   v->physical_ring_addr) ||
        !scope_vortex_rpc_cp_write(v, SCOPE_VX_CP_Q_RING_HI,
                                   v->physical_ring_addr >> 32) ||
        !scope_vortex_rpc_cp_write(v, SCOPE_VX_CP_Q_HEAD_LO,
                                   v->physical_head_addr) ||
        !scope_vortex_rpc_cp_write(v, SCOPE_VX_CP_Q_HEAD_HI,
                                   v->physical_head_addr >> 32) ||
        !scope_vortex_rpc_cp_write(v, SCOPE_VX_CP_Q_CMPL_LO,
                                   v->physical_cmpl_addr) ||
        !scope_vortex_rpc_cp_write(v, SCOPE_VX_CP_Q_CMPL_HI,
                                   v->physical_cmpl_addr >> 32) ||
        !scope_vortex_rpc_cp_write(v, SCOPE_VX_CP_Q_RING_LOG2,
                                   SCOPE_VX_CP_RING_LOG2) ||
        !scope_vortex_rpc_cp_write(v, SCOPE_VX_CP_Q_CONTROL, 1) ||
        !scope_vortex_rpc_cp_write(v, SCOPE_VX_CP_CTRL, 1) ||
        !scope_vortex_rpc_cp_read(v, SCOPE_VX_CP_Q_SEQNUM,
                                  &v->physical_seq)) {
        error_setg(errp, "failed to program physical Vortex CP queue");
        return false;
    }

    /*
     * Q_CONTROL.reset is only a pulse in the current RTL; it does not reset
     * VX_cp_fetch.head_r or VX_cp_engine.seqnum_r.  A QEMU reconnect can
     * therefore observe a non-zero sequence baseline even after reprogramming
     * the physical ring.  The common runtime emits exactly one command per
     * 64-byte line, so retired seqnum and fetch head advance in lockstep.
     * Resume the monotonic tail at that head instead of incorrectly starting
     * again at zero (which would leave head > tail and park the fetcher).
     */
    v->physical_tail = (uint64_t)v->physical_seq * SCOPE_VX_CP_CL_SIZE;
    scope_vortex_trace(v,
                       "physical queue baseline seq=%u tail=0x%" PRIx64 "\n",
                       v->physical_seq, v->physical_tail);

    v->jobs = g_async_queue_new();
    qatomic_set(&v->worker_stop, 0);
    qemu_thread_create(&v->worker, "scope-vortex", scope_vortex_worker,
                       v, QEMU_THREAD_JOINABLE);
    v->worker_started = true;
    SCOPE_PRINTF("[SCOPE VORTEX] backend=%u bridge=%s path=%s caps=0x%08x\n",
                 be->id, be->bridge_socket,
                 v->direct_p2p ? "direct-p2p" : "mediated", v->caps[0]);
    return true;
}

static void scope_vortex_backend_cleanup(ScopeProxyState *s, ScopeBackend *be)
{
    ScopeVortexState *v = be->vortex;
    ScopeVortexJob *job;

    if (!v) {
        return;
    }
    if (v->worker_started) {
        qatomic_set(&v->worker_stop, 1);
        g_async_queue_push(v->jobs, v);
        qemu_thread_join(&v->worker);
    }
    if (v->jobs) {
        while ((job = g_async_queue_try_pop(v->jobs)) != NULL) {
            if ((void *)job != (void *)v) {
                g_free(job);
            }
        }
        g_async_queue_unref(v->jobs);
    }
    if (v->socket_fd >= 0) {
        if (v->direct_p2p) {
            scope_vortex_rpc_cp_write(v, SCOPE_VX_CP_CTRL, 0);
            scope_vortex_rpc_peer_unmap(v);
        }
        scope_vortex_rpc_free(v, v->physical_cmpl_handle);
        scope_vortex_rpc_free(v, v->physical_head_handle);
        scope_vortex_rpc_free(v, v->physical_ring_handle);
        close(v->socket_fd);
    scope_vortex_trace(v, "cleanup path=%s payload_cpu_bytes=%" PRIu64 "\n",
                       v->direct_p2p ? "direct-p2p" : "mediated",
                       v->payload_cpu_bytes);
    }
    g_free(v);
    be->vortex = NULL;
}

static const ScopeBackendOps scope_vortex_backend_ops = {
    .name = "vortex",
    .requires_real_pci = false,
    .realize = scope_vortex_backend_realize,
    .cleanup = scope_vortex_backend_cleanup,
    .process_bar_packet = scope_vortex_process_bar_packet,
    .poll = scope_vortex_poll,
};
