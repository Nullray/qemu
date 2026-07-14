#include "qemu/osdep.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pcie.h"
#include "hw/core/qdev-properties.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "qemu/main-loop.h"
#include "qemu/thread.h"
#include "qemu/atomic.h"
#include "block/nvme.h"
#include "hw/misc/scope_fpga_vswitch_abi.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <stdarg.h>

#define TYPE_SCOPE_PROXY "scope-fpga-vswitch-nvme"
OBJECT_DECLARE_SIMPLE_TYPE(ScopeProxyState, SCOPE_PROXY)

#define HOST_MBX_BASE           0x01000000U
#define SQE_MON_CFG_BASE        0x01001000U
#define HOST_ECAM_SHADOW_BASE   0x01010000U
#define HOST_ECAM_SHADOW_SIZE   0x00020000U

#define MBX_REG_ACK               0x10U
#define MBX_REG_GUEST_BAR0_LO     0x20U
#define MBX_REG_GUEST_BAR0_HI     0x24U
#define MBX_REG_GUEST_BAR0_SIZE   0x28U
#define MBX_REG_GUEST_BAR0_CTRL   0x2CU
#define MBX_REG_PROXY_CTRL        0x30U
#define MBX_REG_BAR_RESP_DATA_LO  0x34U
#define MBX_REG_BAR_RESP_SEQ      0x38U
#define MBX_REG_BAR_RESP_CTRL     0x3CU
#define MBX_REG_BAR_RESP_DATA_HI  0x4CU
#define MBX_REG_RP_INTX_CTRL      0x50U
#define MBX_REG_RP_INTX_STATUS    0x54U
#define MBX_REG_RP_INTX_COUNT     0x58U

#define MON_REG_ADMIN_SQ_BASE_LO  0x00U
#define MON_REG_ADMIN_SQ_BASE_HI  0x04U
#define MON_REG_ADMIN_SQ_BYTES    0x08U
#define MON_REG_ADMIN_SQ_CTRL     0x0CU
#define MON_REG_STATUS            0x3F0U
#define MON_BACKEND_STRIDE        0x20U

#define GUEST_BAR0_CTRL_VALID      (1U << 0)
#define GUEST_BAR0_CTRL_MEM_ENABLE (1U << 1)
#define GUEST_BAR0_CTRL_IS64       (1U << 2)
#define GUEST_BAR0_CTRL_BACKEND_SHIFT 8U
#define PROXY_CTRL_BAR_ROUTE_READY   (1U << 0)
#define PROXY_CTRL_ECAM_SHADOW_READY (1U << 1)
#define BAR_RESP_CTRL_TOGGLE_SHIFT 2
#define BAR_RESP_CTRL_DONE_SHIFT   3
#define MON_ADMIN_SQ_CTRL_VALID    (1U << 0)
#define MON_STATUS_OVERFLOW        (1U << 0)

#define IORESOURCE_MEM               0x00000200ULL
#define SCOPE_RP_BAR_APERTURE_SIZE   0x00100000U
#define SCOPE_GUEST_MMIO_BASE         0x50000000ULL
#define SCOPE_GUEST_MMIO_SIZE         0x01000000ULL
#define SCOPE_DMA32_RING_SIZE        0x10000U
#define SCOPE_MAX_NVME_QUEUES        256U
#define SCOPE_ADMIN_QID              0U
#define SCOPE_ADMIN_CID_SPACE        (UINT16_MAX + 1U)
#define SCOPE_ADMIN_SQ_MAX_TRACKED   4096U
#define SCOPE_DMA32_TYPE_MAX         SCOPE_PKT_TYPE_SQE_WRITE_DONE
#define SCOPE_NVME_DOORBELL_BASE     0x1000U
#define SCOPE_INTX_RETRY_INTERVAL_US 500000U
#define SCOPE_INTX_RETRY_LOW_US      20U
#define SCOPE_DEBUG_ADMIN_SQE_DONE_TIMEOUT_FALLBACK 1
#define SCOPE_ADMIN_SQE_DONE_FALLBACK_US 5000U
#define SCOPE_ADMIN_SQE_DONE_FALLBACK_RETRY_US 500000U
#define SCOPE_ADMIN_SQE_VISIBILITY_RETRY_US 100U
#define SCOPE_BAR_DONE_TIMEOUT_US       5000U
#define SCOPE_ADMIN_SQE_DONE_FULL_MASK UINT64_MAX
#define SCOPE_DEBUG_IO_PRP_TRACE 0
#define SCOPE_NVME_DEFAULT_CTRL_PAGE_SIZE 4096U
#define SCOPE_NVME_MIN_LBA_SHIFT 9U
#define SCOPE_NVME_MAX_LBA_SHIFT 30U
#define SCOPE_PRP_ENTRY_SIZE 8U
#define SCOPE_PRP_LIST_PAGE_LIMIT 1024U
#define SCOPE_REAL_NVME_DISABLE_TIMEOUT_US 5000000U
#define SCOPE_REAL_NVME_DISABLE_POLL_US 10000U
#define SCOPE_DEFAULT_XDMA_USER      "/dev/xdma0_user"
#define SCOPE_DEFAULT_XDMA_CTRL      "/dev/xdma0_control"
#define SCOPE_DEFAULT_XDMA_BYPASS    "/dev/xdma0_bypass"
#define SCOPE_DEFAULT_BYPASS_COHERENT_ALIAS_BASE 0x0000000100000000ULL
#define SCOPE_VSWITCH_MAX_NVME        13U
#define SCOPE_VSWITCH_ECAM_FUNC_COUNT 28U
#define SCOPE_VSWITCH_ECAM_FUNC_SIZE  0x1000U
#define SCOPE_VSWITCH_ROOT_BDF        0x0000U
#define SCOPE_VSWITCH_SW_UP_BDF       0x0100U
#define SCOPE_VSWITCH_DP_BDF(i)       ((uint16_t)(0x0208U + ((i) << 3)))
#define SCOPE_VSWITCH_NVME_BDF(i)     ((uint16_t)((3U + (i)) << 8))
#define SCOPE_VSWITCH_DP_SLOT(i)      (2U + 2U * (i))
#define SCOPE_VSWITCH_NVME_SLOT(i)    (3U + 2U * (i))
#define SCOPE_ROUTE_BASE              0x100U
#define SCOPE_ROUTE_STRIDE            0x20U
#define SCOPE_ROUTE_BAR_LO            0x00U
#define SCOPE_ROUTE_BAR_HI            0x04U
#define SCOPE_ROUTE_BAR_SIZE          0x08U
#define SCOPE_ROUTE_BDF               0x0cU
#define SCOPE_ROUTE_CTRL              0x10U
#define SCOPE_BACKEND_CONFIG_MAX      (64U * 1024U)

#ifndef SCOPE_PROXY_LOG_ENABLE
#define SCOPE_PROXY_LOG_ENABLE 1
#endif

#ifndef SCOPE_PROXY_LOG_IO
#define SCOPE_PROXY_LOG_IO 0
#endif

#ifndef SCOPE_PROXY_LOG_TIMESTAMP
#define SCOPE_PROXY_LOG_TIMESTAMP 0
#endif

#ifndef SCOPE_PROXY_LOG_FLUSH
#define SCOPE_PROXY_LOG_FLUSH 0
#endif

#if SCOPE_PROXY_LOG_ENABLE
static void scope_log_printf(const char *fmt, ...) G_GNUC_PRINTF(1, 2);
static void scope_log_continue_printf(const char *fmt, ...) G_GNUC_PRINTF(1, 2);

static void scope_log_printf(const char *fmt, ...)
{
    va_list ap;

#if SCOPE_PROXY_LOG_TIMESTAMP
    fprintf(stdout, "[SCOPE_TS_NS=%" PRId64 "] ",
            (int64_t)g_get_monotonic_time() * 1000);
#endif
    va_start(ap, fmt);
    vfprintf(stdout, fmt, ap);
    va_end(ap);
}

static void scope_log_continue_printf(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    vfprintf(stdout, fmt, ap);
    va_end(ap);
}
#endif

#if SCOPE_PROXY_LOG_ENABLE
#define SCOPE_PRINTF(...) scope_log_printf(__VA_ARGS__)
#define SCOPE_PRINTF_CONT(...) scope_log_continue_printf(__VA_ARGS__)
#else
#define SCOPE_PRINTF(...) do { if (0) fprintf(stdout, __VA_ARGS__); } while (0)
#define SCOPE_PRINTF_CONT(...) do { if (0) fprintf(stdout, __VA_ARGS__); } while (0)
#endif

#if SCOPE_PROXY_LOG_ENABLE && SCOPE_PROXY_LOG_IO
#define SCOPE_IO_PRINTF(...) scope_log_printf(__VA_ARGS__)
#define SCOPE_IO_PRINTF_CONT(...) scope_log_continue_printf(__VA_ARGS__)
#else
#define SCOPE_IO_PRINTF(...) do { if (0) fprintf(stdout, __VA_ARGS__); } while (0)
#define SCOPE_IO_PRINTF_CONT(...) do { if (0) fprintf(stdout, __VA_ARGS__); } while (0)
#endif

#if SCOPE_PROXY_LOG_ENABLE && SCOPE_PROXY_LOG_FLUSH
#define SCOPE_FFLUSH(stream) fflush(stream)
#else
#define SCOPE_FFLUSH(stream) do { } while (0)
#endif

#if SCOPE_PROXY_LOG_ENABLE && SCOPE_PROXY_LOG_IO && SCOPE_PROXY_LOG_FLUSH
#define SCOPE_IO_FFLUSH(stream) fflush(stream)
#else
#define SCOPE_IO_FFLUSH(stream) do { } while (0)
#endif

typedef struct ScopeVswitchConfigFn {
    uint16_t bdf;
    uint8_t config[SCOPE_VSWITCH_ECAM_FUNC_SIZE];
    uint8_t wmask[SCOPE_VSWITCH_ECAM_FUNC_SIZE];
    uint8_t w1cmask[SCOPE_VSWITCH_ECAM_FUNC_SIZE];
} ScopeVswitchConfigFn;

typedef struct ScopeSqState {
    bool valid;
    uint16_t qid;
    uint16_t depth;
    uint16_t linked_cqid;
    uint16_t last_guest_tail;
    uint64_t guest_base;
    uint64_t translated_base;
    bool seed_valid;
    uint64_t seed_guest_pa;
    NvmeCmd seed_cmd;
} ScopeSqState;

typedef struct ScopeCqState {
    bool valid;
    bool interrupt_enabled;
    uint16_t qid;
    uint16_t depth;
    uint16_t last_guest_head;
    uint16_t shadow_tail;
    bool phase;
    uint64_t guest_base;
    uint64_t translated_base;
} ScopeCqState;

typedef enum ScopeAdminTopoOpType {
    SCOPE_ADMIN_TOPO_OP_NONE = 0,
    SCOPE_ADMIN_TOPO_OP_CREATE_CQ,
    SCOPE_ADMIN_TOPO_OP_CREATE_SQ,
    SCOPE_ADMIN_TOPO_OP_DELETE_CQ,
    SCOPE_ADMIN_TOPO_OP_DELETE_SQ,
    SCOPE_ADMIN_TOPO_OP_IDENTIFY_NS,
} ScopeAdminTopoOpType;

typedef struct ScopePendingAdminOp {
    bool valid;
    ScopeAdminTopoOpType type;
    uint16_t qid;
    uint16_t cqid;
    uint16_t depth;
    bool interrupt_enabled;
    uint32_t nsid;
    uint64_t guest_base;
    uint64_t data_guest_base;
} ScopePendingAdminOp;

typedef struct ScopePendingDoorbell {
    bool valid;
    bool bar_done;
    uint32_t seq;
    uint32_t offset;
    uint64_t data;
    uint8_t wstrb;
    uint8_t size_bytes;
    uint16_t qid;
    uint16_t new_tail;
    int64_t pending_since_us;
    int64_t fallback_next_try_us;
    uint64_t fallback_try_count;
    bool slot_baseline_valid[SCOPE_ADMIN_SQ_MAX_TRACKED];
    uint32_t slot_baseline_seq[SCOPE_ADMIN_SQ_MAX_TRACKED];
} ScopePendingDoorbell;

typedef struct ScopeNvmeBackend {
    uint8_t id;
    uint16_t virtual_bdf;
    char *real_host_bdf;
    int real_bar_fd;
    void *real_bar0_map;
    size_t real_bar0_size;
    uint64_t real_bar0_flags;
    uint16_t original_pci_command;
    bool pci_command_saved;
    uint64_t nvme_cap;
    uint32_t nvme_vs;
    uint32_t doorbell_stride;
    uint32_t ctrl_page_size;
    GHashTable *ns_lba_shift_map;
    uint32_t guest_cc;
    uint32_t guest_aqa;
    uint32_t guest_int_mask;
    uint64_t guest_asq;
    uint64_t guest_acq;
    ScopeSqState sq[SCOPE_MAX_NVME_QUEUES];
    ScopeCqState cq[SCOPE_MAX_NVME_QUEUES];
    ScopePendingAdminOp *pending_admin_ops;
    bool admin_cid_outstanding[SCOPE_ADMIN_CID_SPACE];
    uint32_t admin_outstanding_count;
    ScopePendingDoorbell pending_sq_db;
    bool inferred_bar_done_valid;
    uint32_t inferred_bar_done_seq;
    uint32_t inferred_bar_done_offset;
    uint64_t inferred_bar_done_count;
    bool admin_sq_slot_done_valid[SCOPE_ADMIN_SQ_MAX_TRACKED];
    uint32_t admin_sq_slot_done_seq[SCOPE_ADMIN_SQ_MAX_TRACKED];
    uint64_t admin_sq_slot_done_mask[SCOPE_ADMIN_SQ_MAX_TRACKED];
    uint32_t admin_sq_slot_consumed_seq[SCOPE_ADMIN_SQ_MAX_TRACKED];
    bool admin_sq_slot_fallback_wait_done[SCOPE_ADMIN_SQ_MAX_TRACKED];
    bool intx_pending;
} ScopeNvmeBackend;

struct ScopeProxyState {
    PCIDevice parent_obj;

    int xdma_fd;
    int xdma_ctrl_fd;
    int xdma_bypass_fd;
    void *dma32_db_map;
    void *ecam_shadow_map;
    struct scope_xdma_dma32_doorbell dma32_db;

    QemuMutex xdma_lock;
    bool xdma_lock_inited;
    QemuMutex state_lock;
    bool state_lock_inited;

    QemuThread rx_thread;
    bool rx_thread_started;
    int rx_thread_stop;
    uint32_t bar_resp_toggle;

    char *legacy_real_host_bdf;
    char *backend_config;
    char *fpga_host_bdf;
    char *xdma_user_dev;
    char *xdma_ctrl_dev;
    char *xdma_event_dev;
    char *xdma_bypass_dev;

    uint64_t fpga_bypass_bar_base;
    uint64_t fpga_bypass_bar_size;
    uint64_t bypass_coherent_alias_base;
    bool guest_mem_raw_fallback;
    bool sqe_monitor_enable;
    int fpga_bypass_bar_index;
    uint32_t proxy_ctrl_shadow;
    ScopeVswitchConfigFn vcfg[SCOPE_VSWITCH_ECAM_FUNC_COUNT];
    ScopeNvmeBackend *backends;
    ScopeNvmeBackend *active;
    uint32_t backend_count;
    uint32_t dma32_ring_size;
    uint32_t bar_done_timeout_us;

    uint64_t guest_ddr_base;
    uint64_t guest_ddr_size;

    size_t host_page_size;

    bool virtual_intx_level;
    uint64_t virtual_intx_assert_count;
    uint64_t virtual_intx_deassert_count;
    uint64_t virtual_intx_retry_count;
    int64_t virtual_intx_last_retry_us;
    bool intx_retry_pulse;

    MemoryRegion dummy_bar0;
};

typedef struct ScopePciResource {
    uint64_t start;
    uint64_t end;
    uint64_t flags;
} ScopePciResource;

typedef enum ScopeSqeReadStatus {
    SCOPE_SQE_READ_OK = 0,
    SCOPE_SQE_READ_WAIT,
    SCOPE_SQE_READ_ERR,
} ScopeSqeReadStatus;

typedef struct ScopeJsonCursor {
    const char *buf;
    size_t len;
    size_t pos;
} ScopeJsonCursor;

static void scope_json_ws(ScopeJsonCursor *j)
{
    while (j->pos < j->len && g_ascii_isspace(j->buf[j->pos])) {
        j->pos++;
    }
}

static bool scope_json_ch(ScopeJsonCursor *j, char ch, Error **errp)
{
    scope_json_ws(j);
    if (j->pos >= j->len || j->buf[j->pos] != ch) {
        error_setg(errp, "backend config: expected '%c' at byte %zu", ch, j->pos);
        return false;
    }
    j->pos++;
    return true;
}

static char *scope_json_string(ScopeJsonCursor *j, Error **errp)
{
    GString *out;

    if (!scope_json_ch(j, '"', errp)) {
        return NULL;
    }
    out = g_string_new(NULL);
    while (j->pos < j->len) {
        unsigned char ch = j->buf[j->pos++];
        if (ch == '"') {
            return g_string_free(out, false);
        }
        if (ch < 0x20) {
            error_setg(errp, "backend config: control character in string");
            g_string_free(out, true);
            return NULL;
        }
        if (ch == '\\') {
            if (j->pos >= j->len) {
                error_setg(errp, "backend config: unterminated escape");
                g_string_free(out, true);
                return NULL;
            }
            ch = j->buf[j->pos++];
            switch (ch) {
            case '"': case '\\': case '/': break;
            case 'b': ch = '\b'; break;
            case 'f': ch = '\f'; break;
            case 'n': ch = '\n'; break;
            case 'r': ch = '\r'; break;
            case 't': ch = '\t'; break;
            default:
                error_setg(errp, "backend config: unsupported escape at byte %zu", j->pos - 1);
                g_string_free(out, true);
                return NULL;
            }
        }
        g_string_append_c(out, ch);
    }
    error_setg(errp, "backend config: unterminated string");
    g_string_free(out, true);
    return NULL;
}

static bool scope_valid_host_bdf(const char *s)
{
    unsigned int domain, bus, dev, fn;
    char tail;

    return s && sscanf(s, "%4x:%2x:%2x.%1x%c", &domain, &bus, &dev, &fn, &tail) == 4 &&
           domain <= 0xffff && bus <= 0xff && dev <= 0x1f && fn <= 7;
}

static bool scope_parse_backend_object(ScopeJsonCursor *j, ScopeNvmeBackend *be,
                                       Error **errp)
{
    bool have_bdf = false;

    if (!scope_json_ch(j, '{', errp)) {
        return false;
    }
    scope_json_ws(j);
    if (j->pos < j->len && j->buf[j->pos] == '}') {
        j->pos++;
    } else {
        while (true) {
            g_autofree char *key = scope_json_string(j, errp);
            g_autofree char *value = NULL;
            if (!key || !scope_json_ch(j, ':', errp)) {
                return false;
            }
            if (strcmp(key, "real-host-bdf")) {
                error_setg(errp, "backend config: unknown device key '%s'", key);
                return false;
            }
            if (have_bdf) {
                error_setg(errp, "backend config: duplicate real-host-bdf");
                return false;
            }
            value = scope_json_string(j, errp);
            if (!value || !scope_valid_host_bdf(value)) {
                if (!*errp) {
                    error_setg(errp, "backend config: invalid host BDF '%s'",
                               value ? value : "");
                }
                return false;
            }
            be->real_host_bdf = g_steal_pointer(&value);
            have_bdf = true;
            scope_json_ws(j);
            if (j->pos < j->len && j->buf[j->pos] == '}') {
                j->pos++;
                break;
            }
            if (!scope_json_ch(j, ',', errp)) {
                return false;
            }
        }
    }
    if (!have_bdf) {
        error_setg(errp, "backend config: device is missing real-host-bdf");
        return false;
    }
    return true;
}

static bool scope_parse_backend_config(ScopeProxyState *s, Error **errp)
{
    g_autofree char *contents = NULL;
    gsize len = 0;
    ScopeJsonCursor j;
    bool have_version = false, have_devices = false;
    unsigned int version = 0;
    GError *gerr = NULL;
    unsigned int init_i;

    s->backends = g_new0(ScopeNvmeBackend, SCOPE_VSWITCH_MAX_NVME);
    for (init_i = 0; init_i < SCOPE_VSWITCH_MAX_NVME; init_i++) {
        s->backends[init_i].real_bar_fd = -1;
    }
    if (!s->backend_config) {
        if (!s->legacy_real_host_bdf) {
            s->backend_count = 0;
            return true;
        }
        if (!scope_valid_host_bdf(s->legacy_real_host_bdf)) {
            error_setg(errp, "Invalid real-host-bdf '%s'", s->legacy_real_host_bdf);
            return false;
        }
        s->backends[0].real_host_bdf = g_strdup(s->legacy_real_host_bdf);
        s->backend_count = 1;
        return true;
    }
    if (s->legacy_real_host_bdf) {
        error_setg(errp, "backend-config and real-host-bdf are mutually exclusive");
        return false;
    }
    if (!g_file_get_contents(s->backend_config, &contents, &len, &gerr)) {
        error_setg(errp, "Cannot read backend config %s: %s", s->backend_config,
                   gerr->message);
        g_error_free(gerr);
        return false;
    }
    if (len > SCOPE_BACKEND_CONFIG_MAX) {
        error_setg(errp, "backend config exceeds 64 KiB");
        return false;
    }
    j = (ScopeJsonCursor) { .buf = contents, .len = len };
    if (!scope_json_ch(&j, '{', errp)) {
        return false;
    }
    while (true) {
        g_autofree char *key = NULL;
        scope_json_ws(&j);
        if (j.pos < j.len && j.buf[j.pos] == '}') {
            j.pos++;
            break;
        }
        key = scope_json_string(&j, errp);
        if (!key || !scope_json_ch(&j, ':', errp)) {
            return false;
        }
        if (!strcmp(key, "version")) {
            if (have_version) {
                error_setg(errp, "backend config: duplicate version");
                return false;
            }
            scope_json_ws(&j);
            if (j.pos >= j.len || !g_ascii_isdigit(j.buf[j.pos])) {
                error_setg(errp, "backend config: version must be an integer");
                return false;
            }
            while (j.pos < j.len && g_ascii_isdigit(j.buf[j.pos]))
                version = version * 10 + (j.buf[j.pos++] - '0');
            have_version = true;
        } else if (!strcmp(key, "devices")) {
            if (have_devices || !scope_json_ch(&j, '[', errp)) {
                if (!*errp) error_setg(errp, "backend config: duplicate devices");
                return false;
            }
            scope_json_ws(&j);
            while (j.pos < j.len && j.buf[j.pos] != ']') {
                uint32_t i;
                if (s->backend_count >= SCOPE_VSWITCH_MAX_NVME) {
                    error_setg(errp, "backend config contains more than 13 devices");
                    return false;
                }
                if (!scope_parse_backend_object(&j, &s->backends[s->backend_count], errp))
                    return false;
                for (i = 0; i < s->backend_count; i++) {
                    if (!strcmp(s->backends[i].real_host_bdf,
                                s->backends[s->backend_count].real_host_bdf)) {
                        error_setg(errp, "backend config: duplicate BDF %s",
                                   s->backends[i].real_host_bdf);
                        return false;
                    }
                }
                s->backend_count++;
                scope_json_ws(&j);
                if (j.pos < j.len && j.buf[j.pos] == ']') break;
                if (!scope_json_ch(&j, ',', errp)) return false;
                scope_json_ws(&j);
                if (j.pos < j.len && j.buf[j.pos] == ']') {
                    error_setg(errp, "backend config: trailing comma in devices");
                    return false;
                }
            }
            if (!scope_json_ch(&j, ']', errp)) return false;
            have_devices = true;
        } else {
            error_setg(errp, "backend config: unknown root key '%s'", key);
            return false;
        }
        scope_json_ws(&j);
        if (j.pos < j.len && j.buf[j.pos] == '}') continue;
        if (!scope_json_ch(&j, ',', errp)) return false;
        scope_json_ws(&j);
        if (j.pos < j.len && j.buf[j.pos] == '}') {
            error_setg(errp, "backend config: trailing comma in root object");
            return false;
        }
    }
    scope_json_ws(&j);
    if (j.pos != j.len || !have_version || !have_devices || version != 1) {
        error_setg(errp, "backend config requires exactly version 1 and devices array");
        return false;
    }
    return true;
}

static uint64_t dummy_bar_read(void *opaque, hwaddr addr, unsigned size)
{
    SCOPE_PRINTF("[SCOPE PROXY] Guest OS READ  BAR0: offset 0x%04" HWADDR_PRIx ", size %u bytes\n",
           addr, size);
    return 0;
}

static void dummy_bar_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    SCOPE_PRINTF("[SCOPE PROXY] Guest OS WRITE BAR0: offset 0x%04" HWADDR_PRIx
           ", value 0x%08" PRIx64 ", size %u bytes\n", addr, val, size);
}

static const MemoryRegionOps dummy_bar_ops = {
    .read = dummy_bar_read,
    .write = dummy_bar_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void scope_log_config_write(const char *path, uint32_t seq, uint32_t addr,
                                   int len, uint8_t wstrb, uint32_t data)
{
    SCOPE_PRINTF("[SCOPE PROXY][CFG][%s] seq=%u addr=0x%03x len=%d wstrb=0x%x data=0x%08x\n",
           path, seq, addr, len, wstrb, data);
    SCOPE_FFLUSH(stdout);
}

static void scope_log_nvme_cmd(const char *tag, const ScopeSqState *sq, uint16_t slot,
                               uint64_t guest_pa, const NvmeCmd *cmd)
{
    uint64_t mptr = cmd ? le64_to_cpu(cmd->mptr) : 0;
    uint64_t prp1 = cmd ? le64_to_cpu(cmd->dptr.prp1) : 0;
    uint64_t prp2 = cmd ? le64_to_cpu(cmd->dptr.prp2) : 0;

    SCOPE_PRINTF("[SCOPE PROXY][CMD][%s] qid=%u slot=%u guest_pa=0x%016" PRIx64
           " opcode=0x%02x cid=%u flags=0x%02x nsid=0x%08x mptr=0x%016" PRIx64
           " prp1=0x%016" PRIx64 " prp2=0x%016" PRIx64
           " cdw10=0x%08x cdw11=0x%08x\n",
           tag,
           sq ? sq->qid : 0U,
           slot,
           guest_pa,
           cmd ? cmd->opcode : 0U,
           cmd ? le16_to_cpu(cmd->cid) : 0U,
           cmd ? cmd->flags : 0U,
           cmd ? le32_to_cpu(cmd->nsid) : 0U,
           mptr,
           prp1,
           prp2,
           cmd ? le32_to_cpu(cmd->cdw10) : 0U,
           cmd ? le32_to_cpu(cmd->cdw11) : 0U);
    SCOPE_FFLUSH(stdout);
}

static void scope_log_nvme_cmd_dwords(const char *tag, const ScopeSqState *sq, uint16_t slot,
                                      uint64_t guest_pa, const NvmeCmd *cmd)
{
    uint32_t dw[sizeof(*cmd) / sizeof(uint32_t)];
    int i;

    memcpy(dw, cmd, sizeof(dw));

    for (i = 0; i < ARRAY_SIZE(dw); i += 4) {
        SCOPE_PRINTF("[SCOPE PROXY][CMD][%s][DW] qid=%u slot=%u guest_pa=0x%016" PRIx64
               " dw%02d=%08x dw%02d=%08x dw%02d=%08x dw%02d=%08x\n",
               tag,
               sq ? sq->qid : 0U,
               slot,
               guest_pa,
               i + 0, le32_to_cpu(dw[i + 0]),
               i + 1, le32_to_cpu(dw[i + 1]),
               i + 2, le32_to_cpu(dw[i + 2]),
               i + 3, le32_to_cpu(dw[i + 3]));
    }
    SCOPE_FFLUSH(stdout);
}

static void scope_log_nvme_cmd_hexdump(const char *tag, const ScopeSqState *sq, uint16_t slot,
                                       uint64_t guest_pa, const NvmeCmd *cmd)
{
    const uint8_t *bytes = (const uint8_t *)cmd;
    int i;

    for (i = 0; i < sizeof(*cmd); i += 16) {
        SCOPE_PRINTF("[SCOPE PROXY][CMD][%s][HEX] qid=%u slot=%u guest_pa=0x%016" PRIx64
               " +0x%02x: %02x %02x %02x %02x %02x %02x %02x %02x"
               " %02x %02x %02x %02x %02x %02x %02x %02x\n",
               tag,
               sq ? sq->qid : 0U,
               slot,
               guest_pa,
               i,
               bytes[i + 0], bytes[i + 1], bytes[i + 2], bytes[i + 3],
               bytes[i + 4], bytes[i + 5], bytes[i + 6], bytes[i + 7],
               bytes[i + 8], bytes[i + 9], bytes[i + 10], bytes[i + 11],
               bytes[i + 12], bytes[i + 13], bytes[i + 14], bytes[i + 15]);
    }
    SCOPE_FFLUSH(stdout);
}

static void scope_log_guest_translation(const char *tag, const ScopeSqState *sq, uint16_t slot,
                                        uint64_t guest_pa, size_t len,
                                        uint64_t bar_offset, uint64_t translated_pa)
{
    SCOPE_PRINTF("[SCOPE PROXY][CMD][%s][ADDR] qid=%u slot=%u guest_pa=0x%016" PRIx64
           " len=%zu bar_offset=0x%016" PRIx64 " host_bypass=0x%016" PRIx64 "\n",
           tag,
           sq ? sq->qid : 0U,
           slot,
           guest_pa,
           len,
           bar_offset,
           translated_pa);
    SCOPE_FFLUSH(stdout);
}

static void scope_log_nvme_cmd_head(const char *tag, const ScopeSqState *sq, uint16_t slot,
                                    uint64_t guest_pa, unsigned attempt, const NvmeCmd *cmd)
{
    uint32_t dw[4] = { 0 };

    memcpy(dw, cmd, sizeof(dw));

    SCOPE_PRINTF("[SCOPE PROXY][CMD][%s][HEAD] qid=%u slot=%u guest_pa=0x%016" PRIx64
           " attempt=%u dw00=%08x dw01=%08x dw02=%08x dw03=%08x\n",
           tag,
           sq ? sq->qid : 0U,
           slot,
           guest_pa,
           attempt,
           le32_to_cpu(dw[0]),
           le32_to_cpu(dw[1]),
           le32_to_cpu(dw[2]),
           le32_to_cpu(dw[3]));
    SCOPE_FFLUSH(stdout);
}

static bool scope_guest_mem_read(ScopeProxyState *s, uint64_t guest_pa, void *buf, size_t len);
static bool scope_guest_range_to_coherent_bar_offset(ScopeProxyState *s,
                                                     uint64_t guest_pa,
                                                     size_t len,
                                                     uint64_t *bar_offset);
static bool scope_handle_nvme_bar_write(ScopeProxyState *s, uint32_t offset, uint64_t data,
                                        uint8_t wstrb, uint8_t size_bytes);

static bool scope_nvme_cmd_is_zero(const NvmeCmd *cmd)
{
    static const uint8_t zero_cmd[sizeof(NvmeCmd)] = { 0 };

    return !memcmp(cmd, zero_cmd, sizeof(*cmd));
}

static bool scope_admin_cmd_has_no_dma_payload(const NvmeCmd *cmd)
{
    return le64_to_cpu(cmd->mptr) == 0 &&
           le64_to_cpu(cmd->dptr.prp1) == 0 &&
           le64_to_cpu(cmd->dptr.prp2) == 0;
}

static bool scope_admin_cmd_looks_plausible(const NvmeCmd *cmd)
{
    switch (cmd->opcode) {
    case NVME_ADM_CMD_IDENTIFY:
        return le64_to_cpu(cmd->dptr.prp1) != 0;
    case NVME_ADM_CMD_GET_LOG_PAGE:
        return le64_to_cpu(cmd->dptr.prp1) != 0;
    case NVME_ADM_CMD_ABORT:
        return true;
    case NVME_ADM_CMD_SET_FEATURES:
    case NVME_ADM_CMD_GET_FEATURES:
    case NVME_ADM_CMD_ASYNC_EV_REQ:
    case NVME_ADM_CMD_ACTIVATE_FW:
    case NVME_ADM_CMD_DOWNLOAD_FW:
    case NVME_ADM_CMD_NS_ATTACHMENT:
    case NVME_ADM_CMD_DIRECTIVE_SEND:
    case NVME_ADM_CMD_DIRECTIVE_RECV:
    case NVME_ADM_CMD_VIRT_MNGMT:
    case NVME_ADM_CMD_DBBUF_CONFIG:
    case NVME_ADM_CMD_FORMAT_NVM:
    case NVME_ADM_CMD_SECURITY_SEND:
    case NVME_ADM_CMD_SECURITY_RECV:
        return true;
    case NVME_ADM_CMD_CREATE_CQ: {
        const NvmeCreateCq *cq = (const NvmeCreateCq *)cmd;

        return cq->cqid && cq->cqid < SCOPE_MAX_NVME_QUEUES &&
               le64_to_cpu(cq->prp1) != 0;
    }
    case NVME_ADM_CMD_CREATE_SQ: {
        const NvmeCreateSq *sq = (const NvmeCreateSq *)cmd;

        return sq->sqid && sq->sqid < SCOPE_MAX_NVME_QUEUES &&
               sq->cqid && sq->cqid < SCOPE_MAX_NVME_QUEUES &&
               le64_to_cpu(sq->prp1) != 0;
    }
    case NVME_ADM_CMD_DELETE_CQ:
    case NVME_ADM_CMD_DELETE_SQ: {
        const NvmeDeleteQ *del = (const NvmeDeleteQ *)cmd;
        uint16_t qid = le16_to_cpu(del->qid);

        return qid && qid < SCOPE_MAX_NVME_QUEUES &&
               scope_admin_cmd_has_no_dma_payload(cmd);
    }
    default:
        return false;
    }
}

static uint64_t scope_admin_sqe_done_mask_for_packet(uint64_t guest_pa,
                                                     uint32_t bytes,
                                                     uint64_t slot_base)
{
    uint64_t write_start = guest_pa;
    uint64_t write_end = guest_pa + bytes;
    uint64_t slot_end = slot_base + sizeof(NvmeCmd);
    uint64_t cover_start;
    uint64_t cover_end;
    uint64_t cover_len;
    unsigned shift;

    if (!bytes || write_end <= write_start ||
        write_start >= slot_end || write_end <= slot_base) {
        return 0;
    }

    cover_start = write_start < slot_base ? slot_base : write_start;
    cover_end = write_end > slot_end ? slot_end : write_end;
    if (cover_end <= cover_start) {
        return 0;
    }

    shift = (unsigned)(cover_start - slot_base);
    cover_len = cover_end - cover_start;
    if (cover_len >= sizeof(NvmeCmd)) {
        return SCOPE_ADMIN_SQE_DONE_FULL_MASK;
    }

    return ((UINT64_C(1) << (unsigned)cover_len) - 1U) << shift;
}

static inline uint32_t scope_apply_wstrb32(uint32_t current, uint32_t data, uint8_t wstrb)
{
    uint32_t next = current;
    int i;

    for (i = 0; i < 4; i++) {
        if (wstrb & (1U << i)) {
            next &= ~(0xFFU << (i * 8));
            next |= ((data >> (i * 8)) & 0xFFU) << (i * 8);
        }
    }

    return next;
}

static inline uint64_t scope_apply_wstrb64(uint64_t current, uint64_t data, uint8_t wstrb)
{
    uint64_t next = current;
    int i;

    for (i = 0; i < 8; i++) {
        if (wstrb & (1U << i)) {
            next &= ~((uint64_t)0xFFU << (i * 8));
            next |= ((data >> (i * 8)) & 0xFFU) << (i * 8);
        }
    }

    return next;
}

static inline void scope_update_shadow_u64(uint64_t *shadow, bool high_dword,
                                           uint32_t data, uint8_t wstrb)
{
    uint8_t dword_wstrb = high_dword ? ((wstrb >> 4) & 0x0FU) : (wstrb & 0x0FU);
    uint32_t current = high_dword ? (uint32_t)(*shadow >> 32) : (uint32_t)(*shadow);
    uint32_t next = scope_apply_wstrb32(current, data, dword_wstrb);

    if (high_dword) {
        *shadow = (*shadow & 0x00000000ffffffffULL) | ((uint64_t)next << 32);
    } else {
        *shadow = (*shadow & 0xffffffff00000000ULL) | (uint64_t)next;
    }
}

static inline uint32_t scope_extract_dword32(uint64_t data, uint32_t offset)
{
    return (offset & 0x4U) ? (uint32_t)(data >> 32) : (uint32_t)data;
}

static inline uint8_t scope_extract_wstrb4(uint8_t wstrb, uint32_t offset)
{
    return (offset & 0x4U) ? ((wstrb >> 4) & 0x0FU) : (wstrb & 0x0FU);
}

static inline uint64_t scope_pack_dword32_for_offset(uint32_t value, uint32_t offset)
{
    return (offset & 0x4U) ? ((uint64_t)value << 32) : (uint64_t)value;
}

static inline uint8_t scope_pack_wstrb4_for_offset(uint32_t offset)
{
    return (offset & 0x4U) ? 0xF0U : 0x0FU;
}

static bool scope_parse_resource_line(const char *line, ScopePciResource *res)
{
    unsigned long long start = 0;
    unsigned long long end = 0;
    unsigned long long flags = 0;

    if (sscanf(line, "%llx %llx %llx", &start, &end, &flags) != 3) {
        return false;
    }

    res->start = start;
    res->end = end;
    res->flags = flags;
    return true;
}

static bool scope_read_pci_bar_resource(const char *bdf, int bar_index,
                                        ScopePciResource *res, Error **errp)
{
    g_autofree char *resource_path = NULL;
    FILE *fp = NULL;
    char line[256];
    int i;

    if (!bdf || !bdf[0]) {
        error_setg(errp, "Missing PCI BDF");
        return false;
    }
    if (!res || bar_index < 0 || bar_index > 5) {
        error_setg(errp, "Invalid BAR resource query");
        return false;
    }

    resource_path = g_strdup_printf("/sys/bus/pci/devices/%s/resource", bdf);
    fp = fopen(resource_path, "r");
    if (!fp) {
        error_setg_errno(errp, errno, "Failed to open %s", resource_path);
        return false;
    }

    for (i = 0; i <= bar_index; i++) {
        if (!fgets(line, sizeof(line), fp)) {
            fclose(fp);
            error_setg_errno(errp, errno, "Failed to read BAR%d line from %s",
                             bar_index, resource_path);
            return false;
        }
    }
    fclose(fp);

    if (!scope_parse_resource_line(line, res)) {
        error_setg(errp, "Failed to parse BAR%d resource line: %s", bar_index, line);
        return false;
    }

    return true;
}

static bool scope_xdma_write32_locked(ScopeProxyState *s, uint32_t offset, uint32_t value)
{
    return pwrite(s->xdma_fd, &value, sizeof(value), offset) == sizeof(value);
}

static bool scope_xdma_read32_locked(ScopeProxyState *s, uint32_t offset, uint32_t *value)
{
    return pread(s->xdma_fd, value, sizeof(*value), offset) == sizeof(*value);
}

static bool scope_read_rp_intx_regs(ScopeProxyState *s,
                                    uint32_t *status, uint32_t *count)
{
    bool ok;

    qemu_mutex_lock(&s->xdma_lock);
    ok = scope_xdma_read32_locked(s, HOST_MBX_BASE + MBX_REG_RP_INTX_STATUS,
                                  status) &&
         scope_xdma_read32_locked(s, HOST_MBX_BASE + MBX_REG_RP_INTX_COUNT,
                                  count);
    qemu_mutex_unlock(&s->xdma_lock);

    return ok;
}

static void scope_zero_dma32_ring(ScopeProxyState *s)
{
    if (!s->dma32_db_map || !s->dma32_db.size) {
        return;
    }

    memset(s->dma32_db_map, 0, s->dma32_db.size);
    smp_wmb();
}

static bool scope_write_bar_response(ScopeProxyState *s, uint32_t seq, uint32_t resp,
                                     uint64_t data, bool has_data, bool request_done)
{
    uint32_t ctrl;
    bool ok = true;

    qemu_mutex_lock(&s->xdma_lock);

    if (has_data) {
        ok = scope_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_BAR_RESP_DATA_LO,
                                       (uint32_t)data);
    }
    if (ok && has_data) {
        ok = scope_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_BAR_RESP_DATA_HI,
                                       (uint32_t)(data >> 32));
    }
    if (ok) {
        ok = scope_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_BAR_RESP_SEQ, seq);
    }
    if (ok) {
        s->bar_resp_toggle ^= 1U;
        ctrl = (resp & 0x3U) |
               (s->bar_resp_toggle << BAR_RESP_CTRL_TOGGLE_SHIFT) |
               ((request_done ? 1U : 0U) << BAR_RESP_CTRL_DONE_SHIFT);
        ok = scope_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_BAR_RESP_CTRL, ctrl);
    }

    qemu_mutex_unlock(&s->xdma_lock);
    return ok;
}

static bool scope_virtual_rp_set_intx(ScopeProxyState *s, bool level)
{
    bool old_level = s->virtual_intx_level;
    uint32_t rp_status = 0;
    uint32_t rp_count = 0;
    bool read_ok;
    bool ok;

    qemu_mutex_lock(&s->xdma_lock);
    ok = scope_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_RP_INTX_CTRL,
                                   level ? 1U : 0U);
    qemu_mutex_unlock(&s->xdma_lock);

    if (ok) {
        s->virtual_intx_level = level;
        if (!old_level && level) {
            s->virtual_intx_assert_count++;
            s->virtual_intx_last_retry_us = g_get_monotonic_time();
        } else if (old_level && !level) {
            s->virtual_intx_deassert_count++;
            s->virtual_intx_last_retry_us = 0;
        }
    }

    read_ok = scope_read_rp_intx_regs(s, &rp_status, &rp_count);
    SCOPE_PRINTF("[SCOPE PROXY][INTX][SET] old_level=%d new_level=%d "
           "write_ok=%d read_ok=%d assert_count=%" PRIu64
           " deassert_count=%" PRIu64 " rp_status=0x%08x rp_count=%u\n",
           old_level, level, ok, read_ok,
           s->virtual_intx_assert_count, s->virtual_intx_deassert_count,
           rp_status, rp_count);
    SCOPE_FFLUSH(stdout);

    return ok;
}

static void scope_virtual_rp_retry_intx_pulse(ScopeProxyState *s)
{
    int64_t now_us;

    if (!s->intx_retry_pulse || !s->virtual_intx_level) {
        return;
    }

    now_us = g_get_monotonic_time();
    if (s->virtual_intx_last_retry_us &&
        now_us - s->virtual_intx_last_retry_us <
        (int64_t)SCOPE_INTX_RETRY_INTERVAL_US) {
        return;
    }

    s->virtual_intx_retry_count++;
    s->virtual_intx_last_retry_us = now_us;
    SCOPE_PRINTF("[SCOPE PROXY][INTX][RETRY_PULSE] count=%" PRIu64
           " interval_us=%u low_us=%u\n",
           s->virtual_intx_retry_count, SCOPE_INTX_RETRY_INTERVAL_US,
           SCOPE_INTX_RETRY_LOW_US);
    SCOPE_FFLUSH(stdout);

    if (scope_virtual_rp_set_intx(s, false)) {
        if (SCOPE_INTX_RETRY_LOW_US) {
            g_usleep(SCOPE_INTX_RETRY_LOW_US);
        }
        scope_virtual_rp_set_intx(s, true);
    }
}

static bool scope_ack_cfg_packet(ScopeProxyState *s, uint32_t seq)
{
    bool ok;

    qemu_mutex_lock(&s->xdma_lock);
    ok = scope_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_ACK, seq);
    qemu_mutex_unlock(&s->xdma_lock);

    return ok;
}

static bool scope_sync_admin_window_to_fpga(ScopeProxyState *s, bool valid)
{
    uint64_t sq_base = 0;
    uint32_t sq_bytes = 0;
    uint32_t ctrl = 0;
    bool ok = true;
    uint32_t cfg_base = SQE_MON_CFG_BASE + s->active->id * MON_BACKEND_STRIDE;

    if (s->sqe_monitor_enable && valid &&
        s->active->sq[SCOPE_ADMIN_QID].valid &&
        s->active->sq[SCOPE_ADMIN_QID].depth) {
        sq_base = s->active->sq[SCOPE_ADMIN_QID].guest_base;
        sq_bytes = (uint32_t)s->active->sq[SCOPE_ADMIN_QID].depth * sizeof(NvmeCmd);
        ctrl |= MON_ADMIN_SQ_CTRL_VALID;
    }

    if (s->xdma_fd < 0) {
        return true;
    }

    qemu_mutex_lock(&s->xdma_lock);
    ok = scope_xdma_write32_locked(s, cfg_base + MON_REG_ADMIN_SQ_CTRL,
                                   0);
    ok = ok && scope_xdma_write32_locked(s, cfg_base + MON_REG_ADMIN_SQ_BASE_LO,
                                         (uint32_t)sq_base);
    ok = ok && scope_xdma_write32_locked(s, cfg_base + MON_REG_ADMIN_SQ_BASE_HI,
                                         (uint32_t)(sq_base >> 32));
    ok = ok && scope_xdma_write32_locked(s, cfg_base + MON_REG_ADMIN_SQ_BYTES,
                                         sq_bytes);
    ok = ok && scope_xdma_write32_locked(s, cfg_base + MON_REG_ADMIN_SQ_CTRL,
                                         ctrl);
    qemu_mutex_unlock(&s->xdma_lock);

    SCOPE_PRINTF("[SCOPE PROXY][SQE_MON][CFG] enabled=%d backend=%u virtual=%02x:00.0 "
           "valid=%d sq=0x%016" PRIx64
           " sq_bytes=%u ctrl=0x%08x result=%s\n",
           s->sqe_monitor_enable, s->active->id, 3U + s->active->id,
           (ctrl & MON_ADMIN_SQ_CTRL_VALID) != 0, sq_base, sq_bytes,
           ctrl, ok ? "OK" : "ERR");
    SCOPE_FFLUSH(stdout);
    return ok;
}

static int scope_vcfg_index_from_bdf(uint16_t bdf)
{
    unsigned int i;

    if (bdf == SCOPE_VSWITCH_ROOT_BDF) {
        return 0;
    }
    if (bdf == SCOPE_VSWITCH_SW_UP_BDF) {
        return 1;
    }
    for (i = 0; i < SCOPE_VSWITCH_MAX_NVME; i++) {
        if (bdf == SCOPE_VSWITCH_DP_BDF(i)) return SCOPE_VSWITCH_DP_SLOT(i);
        if (bdf == SCOPE_VSWITCH_NVME_BDF(i)) return SCOPE_VSWITCH_NVME_SLOT(i);
    }
    return -1;
}

static int scope_backend_index_from_bdf(const ScopeProxyState *s, uint16_t bdf)
{
    unsigned int i;
    for (i = 0; i < s->backend_count; i++)
        if (s->backends[i].virtual_bdf == bdf) return i;
    return -1;
}

static ScopeVswitchConfigFn *scope_vcfg_by_bdf(ScopeProxyState *s, uint16_t bdf)
{
    int index = scope_vcfg_index_from_bdf(bdf);

    if (index < 0 || (index >= 2 && (unsigned int)((index - 2) / 2) >= s->backend_count)) {
        return NULL;
    }
    return &s->vcfg[index];
}

static void scope_vcfg_allow_range(ScopeVswitchConfigFn *fn,
                                   uint32_t offset, uint32_t len)
{
    uint32_t i;

    for (i = 0; i < len && offset + i < sizeof(fn->wmask); i++) {
        fn->wmask[offset + i] = 0xff;
    }
}

static void scope_vcfg_w1c_range(ScopeVswitchConfigFn *fn,
                                 uint32_t offset, uint32_t len)
{
    uint32_t i;

    for (i = 0; i < len && offset + i < sizeof(fn->w1cmask); i++) {
        fn->w1cmask[offset + i] = 0xff;
    }
}

static void scope_vcfg_init_bridge(ScopeVswitchConfigFn *fn, uint16_t bdf,
                                   uint16_t device_id, uint8_t primary_bus,
                                   uint8_t secondary_bus, uint8_t subordinate_bus)
{
    memset(fn, 0, sizeof(*fn));
    fn->bdf = bdf;

    pci_config_set_vendor_id(fn->config, 0x1b36);
    pci_config_set_device_id(fn->config, device_id);
    pci_config_set_class(fn->config, PCI_CLASS_BRIDGE_PCI);
    fn->config[PCI_HEADER_TYPE] = PCI_HEADER_TYPE_BRIDGE;
    fn->config[PCI_PRIMARY_BUS] = primary_bus;
    fn->config[PCI_SECONDARY_BUS] = secondary_bus;
    fn->config[PCI_SUBORDINATE_BUS] = subordinate_bus;
    pci_set_word(fn->config + PCI_MEMORY_BASE, 0x5000);
    pci_set_word(fn->config + PCI_MEMORY_LIMIT, 0x50f0);
    fn->config[PCI_INTERRUPT_LINE] = 0xff;
    fn->config[PCI_INTERRUPT_PIN] = 0x01;

    scope_vcfg_allow_range(fn, PCI_COMMAND, 2);
    scope_vcfg_w1c_range(fn, PCI_STATUS, 2);
    scope_vcfg_allow_range(fn, PCI_PRIMARY_BUS, 4);
    scope_vcfg_allow_range(fn, PCI_MEMORY_BASE, 4);
    scope_vcfg_allow_range(fn, PCI_PREF_MEMORY_BASE, 12);
    scope_vcfg_allow_range(fn, PCI_INTERRUPT_LINE, 1);
    scope_vcfg_allow_range(fn, PCI_BRIDGE_CONTROL, 2);
    scope_vcfg_w1c_range(fn, PCI_SEC_STATUS, 2);
}

static void scope_vcfg_init_all(ScopeProxyState *s, PCIDevice *pci_dev)
{
    unsigned int i;
    uint8_t subordinate = MAX(2U, s->backend_count + 2U);

    memset(s->vcfg, 0xff, sizeof(s->vcfg));
    scope_vcfg_init_bridge(&s->vcfg[0], SCOPE_VSWITCH_ROOT_BDF,
                           0x1300, 0, 1, subordinate);
    scope_vcfg_init_bridge(&s->vcfg[1], SCOPE_VSWITCH_SW_UP_BDF,
                           0x1301, 1, 2, subordinate);
    for (i = 0; i < s->backend_count; i++) {
        ScopeVswitchConfigFn *ep = &s->vcfg[SCOPE_VSWITCH_NVME_SLOT(i)];
        scope_vcfg_init_bridge(&s->vcfg[SCOPE_VSWITCH_DP_SLOT(i)],
                               SCOPE_VSWITCH_DP_BDF(i), 0x1302,
                               2, 3 + i, 3 + i);
        memset(ep, 0, sizeof(*ep));
        ep->bdf = SCOPE_VSWITCH_NVME_BDF(i);
        memcpy(ep->config, pci_dev->config,
               MIN((size_t)pci_config_size(pci_dev), sizeof(ep->config)));
        memcpy(ep->wmask, pci_dev->wmask,
               MIN((size_t)pci_config_size(pci_dev), sizeof(ep->wmask)));
        memcpy(ep->w1cmask, pci_dev->w1cmask,
               MIN((size_t)pci_config_size(pci_dev), sizeof(ep->w1cmask)));
        pci_set_long(ep->wmask + PCI_BASE_ADDRESS_0,
                     (uint32_t)(~(s->backends[i].real_bar0_size - 1) &
                                PCI_BASE_ADDRESS_MEM_MASK));
        pci_set_long(ep->wmask + PCI_BASE_ADDRESS_0 + 4, UINT32_MAX);
    }
}

static void scope_vcfg_write_masked(ScopeVswitchConfigFn *fn,
                                    uint32_t offset, uint32_t value, int len)
{
    int i;

    for (i = 0; i < len && offset + i < sizeof(fn->config); i++) {
        uint8_t oldv = fn->config[offset + i];
        uint8_t newv = (value >> (i * 8)) & 0xff;
        uint8_t wmask = fn->wmask[offset + i];
        uint8_t w1c = fn->w1cmask[offset + i];
        uint8_t merged = (oldv & ~wmask) | (newv & wmask);

        merged &= ~(newv & w1c);
        fn->config[offset + i] = merged;
    }
}

static bool scope_sync_ecam_shadow_range_locked(ScopeProxyState *s, int index,
                                                uint32_t offset, uint32_t len)
{
    ScopeVswitchConfigFn *fn;
    uint32_t start;
    uint32_t end;
    uint32_t pos;

    if (index < 0 || index >= SCOPE_VSWITCH_ECAM_FUNC_COUNT) {
        return false;
    }

    fn = &s->vcfg[index];
    start = offset & ~3U;
    end = ROUND_UP(MIN(offset + len, (uint32_t)sizeof(fn->config)), 4);
    if (end > sizeof(fn->config)) {
        end = sizeof(fn->config);
    }

    for (pos = start; pos < end; pos += 4) {
        uint32_t word = ldl_le_p(fn->config + pos);
        uint32_t shadow_off = index * SCOPE_VSWITCH_ECAM_FUNC_SIZE + pos;
        if (s->ecam_shadow_map)
            *(volatile uint32_t *)((uint8_t *)s->ecam_shadow_map + shadow_off) = word;
        else if (!scope_xdma_write32_locked(s, HOST_ECAM_SHADOW_BASE + shadow_off, word))
            return false;
    }

    return true;
}

static bool scope_sync_ecam_shadow_range(ScopeProxyState *s, int index,
                                         uint32_t offset, uint32_t len,
                                         Error **errp)
{
    bool ok;

    qemu_mutex_lock(&s->xdma_lock);
    ok = scope_sync_ecam_shadow_range_locked(s, index, offset, len);
    qemu_mutex_unlock(&s->xdma_lock);

    if (!ok && errp) {
        error_setg(errp, "Failed to sync ECAM shadow slot %d range 0x%x..0x%x",
                   index, offset, offset + len);
    }
    return ok;
}

static bool scope_sync_ecam_shadow_all(ScopeProxyState *s, Error **errp)
{
    bool ok = true;
    int index;

    qemu_mutex_lock(&s->xdma_lock);
    for (index = 0; index < SCOPE_VSWITCH_ECAM_FUNC_COUNT; index++) {
        ok = scope_sync_ecam_shadow_range_locked(s, index, 0,
                                                 SCOPE_VSWITCH_ECAM_FUNC_SIZE);
        if (!ok) {
            break;
        }
    }
    qemu_mutex_unlock(&s->xdma_lock);

    if (!ok && errp) {
        error_setg(errp, "Failed to sync ECAM shadow BRAM slot %d", index);
    }
    return ok;
}

static bool scope_sync_ecam_shadow_fence(ScopeProxyState *s, int index,
                                         uint32_t offset)
{
    uint32_t value;
    uint32_t aligned = offset & ~3U;
    uint32_t host_off = HOST_ECAM_SHADOW_BASE +
                        index * SCOPE_VSWITCH_ECAM_FUNC_SIZE + aligned;
    bool ok;

    qemu_mutex_lock(&s->xdma_lock);
    smp_mb();
    if (s->ecam_shadow_map) {
        value = *(volatile uint32_t *)((uint8_t *)s->ecam_shadow_map +
                                      index * SCOPE_VSWITCH_ECAM_FUNC_SIZE + aligned);
        ok = true;
    } else {
        ok = scope_xdma_read32_locked(s, host_off, &value);
    }
    qemu_mutex_unlock(&s->xdma_lock);
    return ok;
}

static bool scope_set_proxy_ctrl_bits(ScopeProxyState *s, uint32_t bits,
                                      Error **errp)
{
    bool ok;

    s->proxy_ctrl_shadow |= bits;
    qemu_mutex_lock(&s->xdma_lock);
    ok = scope_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_PROXY_CTRL,
                                   s->proxy_ctrl_shadow);
    qemu_mutex_unlock(&s->xdma_lock);

    if (!ok && errp) {
        error_setg(errp, "Failed to update proxy control register");
    }
    return ok;
}

static bool scope_sync_guest_bar_shadow(ScopeProxyState *s,
                                        ScopeVswitchConfigFn *fn,
                                        ScopeNvmeBackend *be, Error **errp)
{
    uint32_t bar0_lo = ldl_le_p(fn->config + PCI_BASE_ADDRESS_0);
    uint32_t bar0_hi = ldl_le_p(fn->config + PCI_BASE_ADDRESS_0 + 4);
    uint16_t command = lduw_le_p(fn->config + PCI_COMMAND);
    bool is_mem = (bar0_lo & PCI_BASE_ADDRESS_SPACE) == PCI_BASE_ADDRESS_SPACE_MEMORY;
    bool is64 = is_mem &&
                ((bar0_lo & PCI_BASE_ADDRESS_MEM_TYPE_MASK) == PCI_BASE_ADDRESS_MEM_TYPE_64);
    uint64_t base = is_mem ?
        (((uint64_t)bar0_hi << 32) | (uint64_t)(bar0_lo & PCI_BASE_ADDRESS_MEM_MASK)) : 0;
    uint32_t ctrl = (uint32_t)be->id << GUEST_BAR0_CTRL_BACKEND_SHIFT;
    uint32_t route = HOST_MBX_BASE + SCOPE_ROUTE_BASE + be->id * SCOPE_ROUTE_STRIDE;
    bool ok;

    if (is_mem && base != 0 && base != UINT64_MAX) {
        ctrl |= GUEST_BAR0_CTRL_VALID;
    }
    if (command & PCI_COMMAND_MEMORY) {
        ctrl |= GUEST_BAR0_CTRL_MEM_ENABLE;
    }
    if (is64) {
        ctrl |= GUEST_BAR0_CTRL_IS64;
    }
    if ((ctrl & GUEST_BAR0_CTRL_VALID) &&
        (be->real_bar0_size == 0 ||
         base < SCOPE_GUEST_MMIO_BASE ||
         base > SCOPE_GUEST_MMIO_BASE + SCOPE_GUEST_MMIO_SIZE - be->real_bar0_size)) {
        SCOPE_PRINTF("[SCOPE VSWITCH][ROUTE][INVALID] backend=%u base=0x%016" PRIx64
                     " size=0x%zx outside PCI MEM window\n",
                     be->id, base, be->real_bar0_size);
        ctrl &= ~GUEST_BAR0_CTRL_VALID;
    }
    if (ctrl & GUEST_BAR0_CTRL_VALID) {
        unsigned int i;
        for (i = 0; i < s->backend_count; i++) {
            ScopeVswitchConfigFn *other;
            uint32_t other_lo, other_hi;
            uint64_t other_base;
            if (i == be->id) continue;
            other = &s->vcfg[SCOPE_VSWITCH_NVME_SLOT(i)];
            other_lo = ldl_le_p(other->config + PCI_BASE_ADDRESS_0);
            other_hi = ldl_le_p(other->config + PCI_BASE_ADDRESS_0 + 4);
            other_base = ((uint64_t)other_hi << 32) |
                         (other_lo & PCI_BASE_ADDRESS_MEM_MASK);
            if ((lduw_le_p(other->config + PCI_COMMAND) & PCI_COMMAND_MEMORY) &&
                other_base && base < other_base + s->backends[i].real_bar0_size &&
                other_base < base + be->real_bar0_size) {
                SCOPE_PRINTF("[SCOPE VSWITCH][ROUTE][COLLISION] backend=%u overlaps %u\n",
                             be->id, i);
                ctrl &= ~GUEST_BAR0_CTRL_VALID;
                break;
            }
        }
    }

    qemu_mutex_lock(&s->xdma_lock);
    ok = scope_xdma_write32_locked(s, route + SCOPE_ROUTE_CTRL, 0);
    ok = ok && scope_xdma_write32_locked(s, route + SCOPE_ROUTE_BAR_LO, (uint32_t)base);
    ok = ok && scope_xdma_write32_locked(s, route + SCOPE_ROUTE_BAR_HI,
                                         (uint32_t)(base >> 32));
    ok = ok && scope_xdma_write32_locked(s, route + SCOPE_ROUTE_BAR_SIZE,
                                         (uint32_t)be->real_bar0_size);
    ok = ok && scope_xdma_write32_locked(s, route + SCOPE_ROUTE_BDF, be->virtual_bdf);
    ok = ok && scope_xdma_write32_locked(s, route + SCOPE_ROUTE_CTRL, ctrl);
    qemu_mutex_unlock(&s->xdma_lock);

    if (!ok && errp) {
        error_setg(errp, "Failed to sync guest BAR0 shadow registers");
        return false;
    }
    if (!scope_set_proxy_ctrl_bits(s, PROXY_CTRL_BAR_ROUTE_READY, errp)) {
        return false;
    }
    if (ok) {
        SCOPE_PRINTF("[SCOPE PROXY][BAR][SHADOW] raw_lo=0x%08x raw_hi=0x%08x base=0x%016" PRIx64
               " size=0x%08zx ctrl=0x%08x cmd=0x%04x\n",
               bar0_lo, bar0_hi, base, be->real_bar0_size, ctrl, command);
        SCOPE_FFLUSH(stdout);
    }
    return ok;
}

static bool scope_real_bar_write(ScopeProxyState *s, uint32_t offset, uint64_t data,
                                 uint8_t wstrb, uint8_t size_bytes)
{
    volatile uint8_t *base;
    uint8_t lane = offset & 0x7U;
    uint8_t full_wstrb = ((1U << size_bytes) - 1U) << lane;
    uint8_t i;

    if (!s->active->real_bar0_map || !size_bytes || size_bytes > 8 ||
        (((offset & 0x7U) + size_bytes) > 8U) ||
        (size_t)offset + size_bytes > s->active->real_bar0_size) {
        return false;
    }

    base = (volatile uint8_t *)s->active->real_bar0_map + offset;

    /*
     * Real NVMe BAR registers are defined in their natural access width.
     * Do not do a 64-bit read/modify/write here: SQ/CQ doorbells share one
     * 64-bit lane, and a qword store to SQ0 would also touch CQ0.
     */
    if (size_bytes == 4 && (offset & 0x3U) == 0 &&
        (wstrb & full_wstrb) == full_wstrb) {
        *(volatile uint32_t *)base = scope_extract_dword32(data, offset);
        return true;
    }
    if (size_bytes == 8 && (offset & 0x7U) == 0 && wstrb == 0xFFU) {
        *(volatile uint64_t *)base = data;
        return true;
    }
    if (size_bytes == 2 && (offset & 0x1U) == 0 &&
        (wstrb & full_wstrb) == full_wstrb) {
        *(volatile uint16_t *)base = (uint16_t)(data >> (lane * 8));
        return true;
    }
    if (size_bytes == 1 && (wstrb & (1U << lane))) {
        *(volatile uint8_t *)base = (uint8_t)(data >> (lane * 8));
        return true;
    }

    for (i = 0; i < size_bytes; i++) {
        uint8_t byte_lane = lane + i;

        if (wstrb & (1U << byte_lane)) {
            *(volatile uint8_t *)(base + i) = (uint8_t)(data >> (byte_lane * 8));
        }
    }
    return true;
}

static bool scope_real_bar_read(ScopeProxyState *s, uint32_t offset, uint8_t size_bytes,
                                uint64_t *data)
{
    uint32_t aligned_offset;
    volatile uint64_t *reg;
    volatile uint8_t *base;

    if (!s->active->real_bar0_map || !data || !size_bytes || size_bytes > 8 ||
        (((offset & 0x7U) + size_bytes) > 8U)) {
        return false;
    }

    aligned_offset = offset & ~0x7U;
    if ((size_t)aligned_offset + sizeof(uint64_t) > s->active->real_bar0_size) {
        return false;
    }

    base = (volatile uint8_t *)s->active->real_bar0_map;
    reg = (volatile uint64_t *)(base + aligned_offset);
    *data = *reg;
    return true;
}

static bool scope_real_bar_write32(ScopeProxyState *s, uint32_t offset, uint32_t value)
{
    /*
     * BAR0 is accessed as 64-bit lanes, so a 32-bit write to offsets like
     * 0x14/0x1c must land in the upper dword of the containing qword.
     */
    return scope_real_bar_write(s, offset, scope_pack_dword32_for_offset(value, offset),
                                scope_pack_wstrb4_for_offset(offset), 4);
}

static bool scope_real_bar_read32(ScopeProxyState *s, uint32_t offset, uint32_t *value)
{
    uint64_t tmp = 0;
    bool ok = scope_real_bar_read(s, offset, 4, &tmp);

    if (ok) {
        *value = (uint32_t)(tmp >> ((offset & 0x4U) ? 32 : 0));
    }
    return ok;
}

static bool scope_real_nvme_disable(ScopeProxyState *s, const char *reason, Error **errp)
{
    int64_t deadline_us;
    uint32_t old_cc = 0;
    uint32_t csts = 0;

    if (!scope_real_bar_read32(s, NVME_REG_CC, &old_cc) ||
        !scope_real_bar_read32(s, NVME_REG_CSTS, &csts)) {
        if (errp) {
            error_setg(errp, "Failed to read real NVMe CC/CSTS before disable (%s)",
                       reason ? reason : "unknown");
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "SCOPE: Failed to read real NVMe CC/CSTS before disable (%s)\n",
                          reason ? reason : "unknown");
        }
        return false;
    }

    if (!scope_real_bar_write32(s, NVME_REG_CC, 0)) {
        if (errp) {
            error_setg(errp, "Failed to write real NVMe CC=0 during disable (%s)",
                       reason ? reason : "unknown");
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "SCOPE: Failed to write real NVMe CC=0 during disable (%s)\n",
                          reason ? reason : "unknown");
        }
        return false;
    }

    deadline_us = g_get_monotonic_time() + SCOPE_REAL_NVME_DISABLE_TIMEOUT_US;
    for (;;) {
        if (!scope_real_bar_read32(s, NVME_REG_CSTS, &csts)) {
            if (errp) {
                error_setg(errp, "Failed to read real NVMe CSTS while disabling (%s)",
                           reason ? reason : "unknown");
            } else {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "SCOPE: Failed to read real NVMe CSTS while disabling (%s)\n",
                              reason ? reason : "unknown");
            }
            return false;
        }
        if (!NVME_CSTS_RDY(csts)) {
            break;
        }
        if (g_get_monotonic_time() >= deadline_us) {
            if (errp) {
                error_setg(errp,
                           "Timed out disabling real NVMe (%s): old_cc=0x%08x "
                           "last_csts=0x%08x RDY=%u CFS=%u. Try unbind/FLR/rescan "
                           "for %s before starting QEMU",
                           reason ? reason : "unknown", old_cc, csts,
                           (unsigned)NVME_CSTS_RDY(csts),
                           (unsigned)NVME_CSTS_CFS(csts),
                           s->active->real_host_bdf ? s->active->real_host_bdf : "real NVMe");
            } else {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "SCOPE: Timed out disabling real NVMe (%s): old_cc=0x%08x "
                              "last_csts=0x%08x RDY=%u CFS=%u. Try unbind/FLR/rescan "
                              "for %s before starting QEMU\n",
                              reason ? reason : "unknown", old_cc, csts,
                              (unsigned)NVME_CSTS_RDY(csts),
                              (unsigned)NVME_CSTS_CFS(csts),
                              s->active->real_host_bdf ? s->active->real_host_bdf : "real NVMe");
            }
            return false;
        }
        g_usleep(SCOPE_REAL_NVME_DISABLE_POLL_US);
    }

    SCOPE_PRINTF("[SCOPE PROXY][REAL][DISABLE] reason=%s old_cc=0x%08x csts=0x%08x "
           "rdy=%u cfs=%u\n",
           reason ? reason : "unknown", old_cc, csts,
           (unsigned)NVME_CSTS_RDY(csts), (unsigned)NVME_CSTS_CFS(csts));
    SCOPE_FFLUSH(stdout);
    if (NVME_CSTS_CFS(csts)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE: real NVMe reports CFS after disable (%s), csts=0x%08x; "
                      "a PCIe FLR/rescan may be required if guest init still fails\n",
                      reason ? reason : "unknown", csts);
    }
    return true;
}

static bool scope_enable_real_pci_bus_master(ScopeProxyState *s, Error **errp)
{
    g_autofree char *config_path = NULL;
    uint8_t cmd_buf[2];
    uint16_t old_cmd;
    uint16_t new_cmd;
    uint16_t verify_cmd;
    int fd;

    if (!s->active->real_host_bdf || !s->active->real_host_bdf[0]) {
        error_setg(errp, "Property real-host-bdf is required");
        return false;
    }

    config_path = g_strdup_printf("/sys/bus/pci/devices/%s/config", s->active->real_host_bdf);
    fd = open(config_path, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        error_setg_errno(errp, errno, "Failed to open %s", config_path);
        return false;
    }

    if (pread(fd, cmd_buf, sizeof(cmd_buf), PCI_COMMAND) != sizeof(cmd_buf)) {
        error_setg_errno(errp, errno, "Failed to read PCI command from %s", config_path);
        close(fd);
        return false;
    }

    old_cmd = lduw_le_p(cmd_buf);
    if (!s->active->pci_command_saved) {
        s->active->original_pci_command = old_cmd;
        s->active->pci_command_saved = true;
    }
    new_cmd = old_cmd | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER;
    if (new_cmd != old_cmd) {
        stw_le_p(cmd_buf, new_cmd);
        if (pwrite(fd, cmd_buf, sizeof(cmd_buf), PCI_COMMAND) != sizeof(cmd_buf)) {
            error_setg_errno(errp, errno, "Failed to write PCI command to %s", config_path);
            close(fd);
            return false;
        }
    }

    if (pread(fd, cmd_buf, sizeof(cmd_buf), PCI_COMMAND) != sizeof(cmd_buf)) {
        error_setg_errno(errp, errno, "Failed to verify PCI command from %s", config_path);
        close(fd);
        return false;
    }
    close(fd);

    verify_cmd = lduw_le_p(cmd_buf);
    if ((verify_cmd & (PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER)) !=
        (PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER)) {
        error_setg(errp, "Real NVMe %s PCI command verify failed: 0x%04x",
                   s->active->real_host_bdf, verify_cmd);
        return false;
    }

    SCOPE_PRINTF("[SCOPE PROXY] Real NVMe PCI command %s: 0x%04x -> 0x%04x "
           "(verified 0x%04x, MEM+BUS_MASTER)\n",
           s->active->real_host_bdf, old_cmd, new_cmd, verify_cmd);
    SCOPE_FFLUSH(stdout);
    return true;
}

static void scope_restore_real_pci_command(ScopeNvmeBackend *be)
{
    g_autofree char *config_path = NULL;
    uint8_t cmd_buf[2];
    int fd;

    if (!be->pci_command_saved || !be->real_host_bdf) {
        return;
    }

    config_path = g_strdup_printf("/sys/bus/pci/devices/%s/config",
                                  be->real_host_bdf);
    fd = open(config_path, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE: cannot restore PCI command for %s: %s\n",
                      be->real_host_bdf, strerror(errno));
        return;
    }

    stw_le_p(cmd_buf, be->original_pci_command);
    if (pwrite(fd, cmd_buf, sizeof(cmd_buf), PCI_COMMAND) != sizeof(cmd_buf)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE: failed to restore PCI command for %s: %s\n",
                      be->real_host_bdf, strerror(errno));
    } else {
        SCOPE_PRINTF("[SCOPE PROXY][REAL][RESTORE] backend=%u real=%s "
               "pci_command=0x%04x controller_left_disabled=1\n",
               be->id, be->real_host_bdf, be->original_pci_command);
    }
    close(fd);
    be->pci_command_saved = false;
}

static void scope_reset_all_queue_state(ScopeProxyState *s)
{
    memset(s->active->sq, 0, sizeof(s->active->sq));
    memset(s->active->cq, 0, sizeof(s->active->cq));
    if (s->active->pending_admin_ops) {
        memset(s->active->pending_admin_ops, 0,
               sizeof(*s->active->pending_admin_ops) * SCOPE_ADMIN_CID_SPACE);
    }
    memset(s->active->admin_cid_outstanding, 0, sizeof(s->active->admin_cid_outstanding));
    s->active->admin_outstanding_count = 0;
    memset(&s->active->pending_sq_db, 0, sizeof(s->active->pending_sq_db));
    memset(s->active->admin_sq_slot_done_valid, 0, sizeof(s->active->admin_sq_slot_done_valid));
    memset(s->active->admin_sq_slot_done_seq, 0, sizeof(s->active->admin_sq_slot_done_seq));
    memset(s->active->admin_sq_slot_done_mask, 0, sizeof(s->active->admin_sq_slot_done_mask));
    memset(s->active->admin_sq_slot_consumed_seq, 0, sizeof(s->active->admin_sq_slot_consumed_seq));
    memset(s->active->admin_sq_slot_fallback_wait_done, 0,
           sizeof(s->active->admin_sq_slot_fallback_wait_done));
    s->active->intx_pending = false;
}

static bool scope_guest_range_to_bar_offset(ScopeProxyState *s, uint64_t guest_pa, size_t len,
                                            uint64_t *bar_offset)
{
    if (!len || !s->guest_ddr_size || !s->fpga_bypass_bar_size) {
        SCOPE_PRINTF("[SCOPE PROXY][DDR][RANGE][ERR] invalid translation request guest_pa=0x%016"
               PRIx64 " len=%zu guest_ddr_size=0x%016" PRIx64
               " bypass_bar_size=0x%016" PRIx64 "\n",
               guest_pa, len, s->guest_ddr_size, s->fpga_bypass_bar_size);
        SCOPE_FFLUSH(stdout);
        return false;
    }
    if (guest_pa < s->guest_ddr_base) {
        SCOPE_PRINTF("[SCOPE PROXY][DDR][RANGE][ERR] guest_pa=0x%016" PRIx64
               " below guest_ddr_base=0x%016" PRIx64 " len=%zu\n",
               guest_pa, s->guest_ddr_base, len);
        SCOPE_FFLUSH(stdout);
        return false;
    }

    if (guest_pa > s->guest_ddr_base + s->guest_ddr_size ||
        len > (s->guest_ddr_base + s->guest_ddr_size) - guest_pa) {
        SCOPE_PRINTF("[SCOPE PROXY][DDR][RANGE][ERR] guest_pa=0x%016" PRIx64
               " len=%zu exceeds guest DDR window [0x%016" PRIx64
               ", 0x%016" PRIx64 ")\n",
               guest_pa, len, s->guest_ddr_base,
               s->guest_ddr_base + s->guest_ddr_size);
        SCOPE_FFLUSH(stdout);
        return false;
    }
    if (guest_pa > s->fpga_bypass_bar_size || len > s->fpga_bypass_bar_size - guest_pa) {
        SCOPE_PRINTF("[SCOPE PROXY][DDR][RANGE][ERR] guest_pa=0x%016" PRIx64
               " len=%zu exceeds bypass BAR size=0x%016" PRIx64 "\n",
               guest_pa, len, s->fpga_bypass_bar_size);
        SCOPE_FFLUSH(stdout);
        return false;
    }

    *bar_offset = guest_pa;
    return true;
}

static bool scope_translate_guest_pa_for_host_bypass(ScopeProxyState *s, uint64_t guest_pa,
                                                     size_t len, uint64_t *translated_pa)
{
    uint64_t bar_offset;

    if (!scope_guest_range_to_coherent_bar_offset(s, guest_pa, len, &bar_offset)) {
        SCOPE_PRINTF("[SCOPE PROXY][DDR][HOST_XLATE][ERR] guest_pa=0x%016" PRIx64
               " len=%zu translation failed\n", guest_pa, len);
        SCOPE_FFLUSH(stdout);
        return false;
    }

    *translated_pa = s->fpga_bypass_bar_base + bar_offset;
    return true;
}

static bool scope_guest_range_to_coherent_bar_offset(ScopeProxyState *s,
                                                     uint64_t guest_pa,
                                                     size_t len,
                                                     uint64_t *bar_offset)
{
    uint64_t raw_offset;
    uint64_t alias_offset;

    if (!scope_guest_range_to_bar_offset(s, guest_pa, len, &raw_offset)) {
        SCOPE_PRINTF("[SCOPE PROXY][DDR][COHERENT_XLATE][ERR] guest_pa=0x%016" PRIx64
               " len=%zu raw translation failed\n", guest_pa, len);
        SCOPE_FFLUSH(stdout);
        return false;
    }

    if (!s->bypass_coherent_alias_base) {
        *bar_offset = raw_offset;
        return true;
    }

    if (s->bypass_coherent_alias_base > UINT64_MAX - raw_offset) {
        SCOPE_PRINTF("[SCOPE PROXY][DDR][COHERENT_XLATE][ERR] guest_pa=0x%016" PRIx64
               " raw_offset=0x%016" PRIx64 " alias_base=0x%016" PRIx64
               " overflows\n",
               guest_pa, raw_offset, s->bypass_coherent_alias_base);
        SCOPE_FFLUSH(stdout);
        return false;
    }

    alias_offset = s->bypass_coherent_alias_base + raw_offset;
    if (alias_offset > s->fpga_bypass_bar_size ||
        len > s->fpga_bypass_bar_size - alias_offset) {
        SCOPE_PRINTF("[SCOPE PROXY][DDR][COHERENT_XLATE][ERR] guest_pa=0x%016" PRIx64
               " len=%zu alias_offset=0x%016" PRIx64
               " exceeds bypass BAR size=0x%016" PRIx64 "\n",
               guest_pa, len, alias_offset, s->fpga_bypass_bar_size);
        SCOPE_FFLUSH(stdout);
        return false;
    }

    *bar_offset = alias_offset;
    return true;
}

static bool scope_translate_guest_pa_for_real_dma(ScopeProxyState *s, uint64_t guest_pa,
                                                  size_t len, uint64_t *translated_pa)
{
    uint64_t bar_offset;
    uint64_t alias_offset;

    /*
     * The real NVMe is controlled from the x86 host side, so its DMA reaches
     * role DDR through PCIe P2P into the FPGA XDMA EP bypass BAR.  Both
     * real-device DMA and QEMU queue-memory accesses use the coherent alias
     * window.  The FPGA alias bridge subtracts bypass_coherent_alias_base
     * before entering u_role/s_axi_dma, so the DUT still observes the
     * original guest physical address.
     */
    if (!scope_guest_range_to_bar_offset(s, guest_pa, len, &bar_offset)) {
        SCOPE_PRINTF("[SCOPE PROXY][DDR][DMA_XLATE][ERR] guest_pa=0x%016" PRIx64
               " len=%zu translation failed\n", guest_pa, len);
        SCOPE_FFLUSH(stdout);
        return false;
    }
    if (s->bypass_coherent_alias_base > UINT64_MAX - bar_offset) {
        SCOPE_PRINTF("[SCOPE PROXY][DDR][DMA_XLATE][ERR] guest_pa=0x%016" PRIx64
               " bar_offset=0x%016" PRIx64 " alias_base=0x%016" PRIx64
               " overflows\n",
               guest_pa, bar_offset, s->bypass_coherent_alias_base);
        SCOPE_FFLUSH(stdout);
        return false;
    }
    alias_offset = s->bypass_coherent_alias_base + bar_offset;
    if (alias_offset > s->fpga_bypass_bar_size ||
        len > s->fpga_bypass_bar_size - alias_offset) {
        SCOPE_PRINTF("[SCOPE PROXY][DDR][DMA_XLATE][ERR] guest_pa=0x%016" PRIx64
               " len=%zu alias_offset=0x%016" PRIx64
               " exceeds bypass BAR size=0x%016" PRIx64 "\n",
               guest_pa, len, alias_offset, s->fpga_bypass_bar_size);
        SCOPE_FFLUSH(stdout);
        return false;
    }
    *translated_pa = s->fpga_bypass_bar_base + alias_offset;
    return true;
}

static bool scope_guest_pa_in_ddr_window(ScopeProxyState *s, uint64_t guest_pa, size_t len)
{
    uint64_t guest_end;

    if (!len || !s->guest_ddr_size || !s->fpga_bypass_bar_size) {
        return false;
    }
    guest_end = s->guest_ddr_base + s->guest_ddr_size;
    if (guest_pa < s->guest_ddr_base || guest_pa > guest_end ||
        len > guest_end - guest_pa) {
        return false;
    }
    if (guest_pa > s->fpga_bypass_bar_size ||
        len > s->fpga_bypass_bar_size - guest_pa) {
        return false;
    }
    return true;
}

static bool scope_real_dma_to_guest_pa(ScopeProxyState *s, uint64_t real_dma,
                                       size_t len, uint64_t *guest_pa)
{
    uint64_t offset;
    uint64_t raw_guest_pa;

    if (!len || !s->fpga_bypass_bar_base || real_dma < s->fpga_bypass_bar_base) {
        return false;
    }

    offset = real_dma - s->fpga_bypass_bar_base;
    if (offset > s->fpga_bypass_bar_size || len > s->fpga_bypass_bar_size - offset) {
        return false;
    }

    if (s->bypass_coherent_alias_base &&
        offset >= s->bypass_coherent_alias_base) {
        raw_guest_pa = offset - s->bypass_coherent_alias_base;
        if (scope_guest_pa_in_ddr_window(s, raw_guest_pa, len)) {
            *guest_pa = raw_guest_pa;
            return true;
        }
    }

    if (scope_guest_pa_in_ddr_window(s, offset, len)) {
        *guest_pa = offset;
        return true;
    }

    return false;
}

static bool scope_translate_cmd_dma_addr_for_real(ScopeProxyState *s,
                                                  uint64_t addr, size_t len,
                                                  const char *field,
                                                  uint64_t *guest_pa,
                                                  uint64_t *translated_pa)
{
    uint64_t normalized_guest_pa = 0;

    if (scope_guest_pa_in_ddr_window(s, addr, len)) {
        if (!scope_translate_guest_pa_for_real_dma(s, addr, len, translated_pa)) {
            return false;
        }
        if (guest_pa) {
            *guest_pa = addr;
        }
        return true;
    }

    if (scope_real_dma_to_guest_pa(s, addr, len, &normalized_guest_pa)) {
        *translated_pa = addr;
        if (guest_pa) {
            *guest_pa = normalized_guest_pa;
        }
        SCOPE_PRINTF("[SCOPE PROXY][CMD][PATCH][IDEMPOTENT] field=%s "
               "addr=0x%016" PRIx64 " guest_pa=0x%016" PRIx64 "\n",
               field ? field : "addr", addr, normalized_guest_pa);
        SCOPE_FFLUSH(stdout);
        return true;
    }

    return scope_translate_guest_pa_for_real_dma(s, addr, len, translated_pa);
}

static bool scope_bypass_copy(ScopeProxyState *s, uint64_t bar_offset, void *buf, size_t len,
                              bool is_write)
{
    uint64_t map_base;
    uint64_t map_delta;
    uint64_t map_len_u64;
    size_t map_len;
    void *map;

    if (!buf || !len || s->xdma_bypass_fd < 0 || !s->host_page_size) {
        SCOPE_PRINTF("[SCOPE PROXY][DDR][MMAP][ERR] invalid bypass copy request bar_offset=0x%016"
               PRIx64 " len=%zu is_write=%d fd=%d host_page_size=%zu\n",
               bar_offset, len, is_write, s->xdma_bypass_fd, s->host_page_size);
        SCOPE_FFLUSH(stdout);
        return false;
    }
    if (bar_offset > s->fpga_bypass_bar_size || len > s->fpga_bypass_bar_size - bar_offset) {
        SCOPE_PRINTF("[SCOPE PROXY][DDR][MMAP][ERR] bar_offset=0x%016" PRIx64
               " len=%zu exceeds bypass BAR size=0x%016" PRIx64 "\n",
               bar_offset, len, s->fpga_bypass_bar_size);
        SCOPE_FFLUSH(stdout);
        return false;
    }

    map_base = bar_offset & ~((uint64_t)s->host_page_size - 1ULL);
    map_delta = bar_offset - map_base;
    map_len_u64 = map_delta + len;
    map_len_u64 = (map_len_u64 + s->host_page_size - 1ULL) &
                  ~((uint64_t)s->host_page_size - 1ULL);
    if (map_len_u64 > SIZE_MAX) {
        return false;
    }
    map_len = (size_t)map_len_u64;

    map = mmap(NULL, map_len, PROT_READ | PROT_WRITE, MAP_SHARED,
               s->xdma_bypass_fd, map_base);
    if (map == MAP_FAILED) {
        SCOPE_PRINTF("[SCOPE PROXY][DDR][MMAP][ERR] mmap failed bar_offset=0x%016" PRIx64
               " map_base=0x%016" PRIx64 " map_len=0x%zx errno=%d (%s)\n",
               bar_offset, map_base, map_len, errno, strerror(errno));
        SCOPE_FFLUSH(stdout);
        return false;
    }

    if (is_write) {
        memcpy((uint8_t *)map + map_delta, buf, len);
    } else {
        memcpy(buf, (uint8_t *)map + map_delta, len);
    }

    munmap(map, map_len);
    return true;
}

static bool scope_guest_mem_read(ScopeProxyState *s, uint64_t guest_pa, void *buf, size_t len)
{
    uint64_t bar_offset;
    bool coherent = s->bypass_coherent_alias_base != 0;

    if (!scope_guest_range_to_coherent_bar_offset(s, guest_pa, len, &bar_offset)) {
        SCOPE_PRINTF("[SCOPE PROXY][DDR][READ][ERR] guest_pa=0x%016" PRIx64
               " len=%zu range translation failed\n", guest_pa, len);
        SCOPE_FFLUSH(stdout);
        return false;
    }
    if (!scope_bypass_copy(s, bar_offset, buf, len, false)) {
        if (coherent && s->guest_mem_raw_fallback &&
            scope_guest_range_to_bar_offset(s, guest_pa, len, &bar_offset) &&
            scope_bypass_copy(s, bar_offset, buf, len, false)) {
            SCOPE_PRINTF("[SCOPE PROXY][DDR][READ][RAW_FALLBACK] guest_pa=0x%016" PRIx64
                   " raw_offset=0x%016" PRIx64 " len=%zu\n",
                   guest_pa, bar_offset, len);
            SCOPE_FFLUSH(stdout);
            return true;
        }
        SCOPE_PRINTF("[SCOPE PROXY][DDR][READ][ERR] guest_pa=0x%016" PRIx64
               " bar_offset=0x%016" PRIx64 " len=%zu copy failed\n",
               guest_pa, bar_offset, len);
        SCOPE_FFLUSH(stdout);
        return false;
    }
    return true;
}

static void scope_log_cqe_raw(const char *tag, uint16_t qid, uint16_t tail,
                              uint64_t guest_pa, const NvmeCqe *cqe)
{
    const uint8_t *bytes = (const uint8_t *)cqe;
    bool admin = qid == SCOPE_ADMIN_QID;

    if (admin) {
        SCOPE_PRINTF("[SCOPE PROXY][CQ][%s][DW] qid=%u tail=%u guest_pa=0x%016"
               PRIx64 " dw00=%08x dw01=%08x dw02=%08x dw03=%08x\n",
               tag, qid, tail, guest_pa,
               le32_to_cpu(cqe->result), le32_to_cpu(cqe->dw1),
               ((uint32_t)bytes[8]) | ((uint32_t)bytes[9] << 8) |
               ((uint32_t)bytes[10] << 16) | ((uint32_t)bytes[11] << 24),
               ((uint32_t)bytes[12]) | ((uint32_t)bytes[13] << 8) |
               ((uint32_t)bytes[14] << 16) | ((uint32_t)bytes[15] << 24));
        SCOPE_PRINTF("[SCOPE PROXY][CQ][%s][HEX] qid=%u tail=%u guest_pa=0x%016"
               PRIx64 " bytes:", tag, qid, tail, guest_pa);
    } else {
        SCOPE_IO_PRINTF("[SCOPE PROXY][CQ][%s][DW] qid=%u tail=%u guest_pa=0x%016"
               PRIx64 " dw00=%08x dw01=%08x dw02=%08x dw03=%08x\n",
               tag, qid, tail, guest_pa,
               le32_to_cpu(cqe->result), le32_to_cpu(cqe->dw1),
               ((uint32_t)bytes[8]) | ((uint32_t)bytes[9] << 8) |
               ((uint32_t)bytes[10] << 16) | ((uint32_t)bytes[11] << 24),
               ((uint32_t)bytes[12]) | ((uint32_t)bytes[13] << 8) |
               ((uint32_t)bytes[14] << 16) | ((uint32_t)bytes[15] << 24));
        SCOPE_IO_PRINTF("[SCOPE PROXY][CQ][%s][HEX] qid=%u tail=%u guest_pa=0x%016"
               PRIx64 " bytes:", tag, qid, tail, guest_pa);
    }
    for (unsigned i = 0; i < sizeof(*cqe); i++) {
        if (admin) {
            SCOPE_PRINTF_CONT(" %02x", bytes[i]);
        } else {
            SCOPE_IO_PRINTF_CONT(" %02x", bytes[i]);
        }
    }
    if (admin) {
        SCOPE_PRINTF_CONT("\n");
        SCOPE_FFLUSH(stdout);
    } else {
        SCOPE_IO_PRINTF_CONT("\n");
        SCOPE_IO_FFLUSH(stdout);
    }
}

static bool scope_guest_mem_write(ScopeProxyState *s, uint64_t guest_pa,
                                  const void *buf, size_t len)
{
    uint64_t bar_offset;
    bool coherent = s->bypass_coherent_alias_base != 0;

    if (!scope_guest_range_to_coherent_bar_offset(s, guest_pa, len, &bar_offset)) {
        SCOPE_PRINTF("[SCOPE PROXY][DDR][WRITE][ERR] guest_pa=0x%016" PRIx64
               " len=%zu range translation failed\n", guest_pa, len);
        SCOPE_FFLUSH(stdout);
        return false;
    }
    if (!scope_bypass_copy(s, bar_offset, (void *)buf, len, true)) {
        if (coherent && s->guest_mem_raw_fallback &&
            scope_guest_range_to_bar_offset(s, guest_pa, len, &bar_offset) &&
            scope_bypass_copy(s, bar_offset, (void *)buf, len, true)) {
            SCOPE_PRINTF("[SCOPE PROXY][DDR][WRITE][RAW_FALLBACK] guest_pa=0x%016" PRIx64
                   " raw_offset=0x%016" PRIx64 " len=%zu\n",
                   guest_pa, bar_offset, len);
            SCOPE_FFLUSH(stdout);
            return true;
        }
        SCOPE_PRINTF("[SCOPE PROXY][DDR][WRITE][ERR] guest_pa=0x%016" PRIx64
               " bar_offset=0x%016" PRIx64 " len=%zu copy failed\n",
               guest_pa, bar_offset, len);
        SCOPE_FFLUSH(stdout);
        return false;
    }
    return true;
}

static void scope_capture_admin_seed(ScopeProxyState *s)
{
    ScopeSqState *sq = &s->active->sq[SCOPE_ADMIN_QID];
    uint64_t seed_pa;
    NvmeCmd seed = { 0 };

    sq->seed_valid = false;
    sq->seed_guest_pa = 0;
    memset(&sq->seed_cmd, 0, sizeof(sq->seed_cmd));

    if (!sq->valid || !sq->guest_base || !sq->depth) {
        return;
    }

    seed_pa = sq->guest_base;
    if (!scope_guest_mem_read(s, seed_pa, &seed, sizeof(seed))) {
        SCOPE_PRINTF("[SCOPE PROXY][CMD][SEED][ERR] qid=%u slot=0 guest_pa=0x%016" PRIx64
               " capture failed\n",
               sq->qid, seed_pa);
        SCOPE_FFLUSH(stdout);
        return;
    }

    sq->seed_valid = true;
    sq->seed_guest_pa = seed_pa;
    sq->seed_cmd = seed;

    SCOPE_PRINTF("[SCOPE PROXY][CMD][SEED] qid=%u slot=0 guest_pa=0x%016" PRIx64
           " zero=%d plausible=%d\n",
           sq->qid, seed_pa, scope_nvme_cmd_is_zero(&seed),
           scope_admin_cmd_looks_plausible(&seed));
    SCOPE_FFLUSH(stdout);
}

static ScopeSqeReadStatus scope_read_admin_sqe_stable(ScopeProxyState *s,
                                                       const ScopeSqState *sq,
                                                       uint16_t slot,
                                                       uint64_t guest_pa,
                                                       NvmeCmd *cmd,
                                                       bool allow_stable_seed)
{
    NvmeCmd first = { 0 };
    NvmeCmd second = { 0 };
    bool matches_seed;
    bool plausible;

    /*
     * Do not sleep in the shared RX worker.  Two immediate coherent-alias
     * reads reject a torn update; a caller that observes WAIT schedules a
     * later retry and lets packets for other backends continue to drain.
     */
    if (!scope_guest_mem_read(s, guest_pa, &first, sizeof(first)) ||
        !scope_guest_mem_read(s, guest_pa, &second, sizeof(second))) {
        return SCOPE_SQE_READ_ERR;
    }

    scope_log_nvme_cmd_head("RAW", sq, slot, guest_pa, 1U, &first);
    if (memcmp(&first, &second, sizeof(first)) != 0) {
        SCOPE_PRINTF("[SCOPE PROXY][CMD][RAW][UNSTABLE] backend=%u qid=%u "
               "slot=%u guest_pa=0x%016" PRIx64 "\n",
               s->active->id, sq ? sq->qid : 0U, slot, guest_pa);
        SCOPE_FFLUSH(stdout);
        return SCOPE_SQE_READ_WAIT;
    }

    *cmd = second;
    matches_seed = sq && sq->seed_valid && sq->seed_guest_pa == guest_pa &&
                   memcmp(cmd, &sq->seed_cmd, sizeof(*cmd)) == 0;
    plausible = !scope_nvme_cmd_is_zero(cmd) &&
                scope_admin_cmd_looks_plausible(cmd) &&
                (!matches_seed || allow_stable_seed);
    if (plausible) {
        return SCOPE_SQE_READ_OK;
    }

    SCOPE_PRINTF("[SCOPE PROXY][CMD][RAW][WAIT_VISIBLE] backend=%u qid=%u "
           "slot=%u guest_pa=0x%016" PRIx64 " zero=%d plausible=%d "
           "matches_seed=%d\n",
           s->active->id, sq ? sq->qid : 0U, slot, guest_pa,
           scope_nvme_cmd_is_zero(cmd), scope_admin_cmd_looks_plausible(cmd),
           matches_seed);
    SCOPE_FFLUSH(stdout);
    return (scope_nvme_cmd_is_zero(cmd) || matches_seed) ?
           SCOPE_SQE_READ_WAIT : SCOPE_SQE_READ_ERR;
}

static bool scope_refresh_admin_queue_state(ScopeProxyState *s)
{
    uint64_t translated_asq = 0;
    uint64_t translated_acq = 0;
    uint16_t sq_depth = NVME_AQA_ASQS(s->active->guest_aqa) + 1;
    uint16_t cq_depth = NVME_AQA_ACQS(s->active->guest_aqa) + 1;

    memset(&s->active->sq[SCOPE_ADMIN_QID], 0, sizeof(s->active->sq[SCOPE_ADMIN_QID]));
    memset(&s->active->cq[SCOPE_ADMIN_QID], 0, sizeof(s->active->cq[SCOPE_ADMIN_QID]));

    if (!s->active->guest_asq || !s->active->guest_acq || !sq_depth || !cq_depth) {
        return scope_sync_admin_window_to_fpga(s, false);
    }
    if (!scope_translate_guest_pa_for_real_dma(s, s->active->guest_asq, 1, &translated_asq) ||
        !scope_translate_guest_pa_for_real_dma(s, s->active->guest_acq, 1, &translated_acq)) {
        return false;
    }

    /*
     * Do not clear guest Admin CQ memory here.  ACQ/AQA/ASQ writes only tell
     * the controller where queues live; the guest driver owns queue memory
     * initialization.  Clearing through XDMA bypass can race with, or even
     * overwrite, real NVMe CQE DMA writes.
     */

    s->active->sq[SCOPE_ADMIN_QID].valid = true;
    s->active->sq[SCOPE_ADMIN_QID].qid = SCOPE_ADMIN_QID;
    s->active->sq[SCOPE_ADMIN_QID].depth = sq_depth;
    s->active->sq[SCOPE_ADMIN_QID].linked_cqid = SCOPE_ADMIN_QID;
    s->active->sq[SCOPE_ADMIN_QID].guest_base = s->active->guest_asq;
    s->active->sq[SCOPE_ADMIN_QID].translated_base = translated_asq;

    s->active->cq[SCOPE_ADMIN_QID].valid = true;
    s->active->cq[SCOPE_ADMIN_QID].interrupt_enabled = true;
    s->active->cq[SCOPE_ADMIN_QID].qid = SCOPE_ADMIN_QID;
    s->active->cq[SCOPE_ADMIN_QID].depth = cq_depth;
    s->active->cq[SCOPE_ADMIN_QID].shadow_tail = 0;
    s->active->cq[SCOPE_ADMIN_QID].phase = true;
    s->active->cq[SCOPE_ADMIN_QID].guest_base = s->active->guest_acq;
    s->active->cq[SCOPE_ADMIN_QID].translated_base = translated_acq;

    scope_capture_admin_seed(s);

    return scope_sync_admin_window_to_fpga(s, true);
}

static bool scope_sync_admin_regs_to_real(ScopeProxyState *s)
{
    uint64_t translated_asq = 0;
    uint64_t translated_acq = 0;
    uint64_t host_bypass = 0;

    if (!scope_real_bar_write32(s, NVME_REG_AQA, s->active->guest_aqa)) {
        return false;
    }

    if (s->active->guest_asq) {
        if (!scope_translate_guest_pa_for_real_dma(s, s->active->guest_asq, 1, &translated_asq)) {
            return false;
        }
        if (scope_translate_guest_pa_for_host_bypass(s, s->active->guest_asq, 1, &host_bypass)) {
            SCOPE_PRINTF("[SCOPE PROXY][ADMIN][XLATE] ASQ guest=0x%016" PRIx64
                   " real_dma=0x%016" PRIx64 " host_bypass=0x%016" PRIx64 "\n",
                   s->active->guest_asq, translated_asq, host_bypass);
            SCOPE_FFLUSH(stdout);
        }
        if (!scope_real_bar_write32(s, NVME_REG_ASQ, (uint32_t)translated_asq) ||
            !scope_real_bar_write32(s, NVME_REG_ASQ + 4,
                                    (uint32_t)(translated_asq >> 32))) {
            return false;
        }
    }

    if (s->active->guest_acq) {
        if (!scope_translate_guest_pa_for_real_dma(s, s->active->guest_acq, 1, &translated_acq)) {
            return false;
        }
        if (scope_translate_guest_pa_for_host_bypass(s, s->active->guest_acq, 1, &host_bypass)) {
            SCOPE_PRINTF("[SCOPE PROXY][ADMIN][XLATE] ACQ guest=0x%016" PRIx64
                   " real_dma=0x%016" PRIx64 " host_bypass=0x%016" PRIx64 "\n",
                   s->active->guest_acq, translated_acq, host_bypass);
            SCOPE_FFLUSH(stdout);
        }
        if (!scope_real_bar_write32(s, NVME_REG_ACQ, (uint32_t)translated_acq) ||
            !scope_real_bar_write32(s, NVME_REG_ACQ + 4,
                                    (uint32_t)(translated_acq >> 32))) {
            return false;
        }
    }

    return scope_refresh_admin_queue_state(s);
}

static bool scope_register_cq(ScopeProxyState *s, uint16_t qid, uint64_t guest_base,
                              uint16_t depth, bool interrupt_enabled)
{
    uint64_t translated = 0;

    if (!qid || qid >= SCOPE_MAX_NVME_QUEUES || !depth) {
        return false;
    }
    if (!scope_translate_guest_pa_for_real_dma(s, guest_base, 1, &translated)) {
        return false;
    }

    memset(&s->active->cq[qid], 0, sizeof(s->active->cq[qid]));
    s->active->cq[qid].valid = true;
    s->active->cq[qid].interrupt_enabled = interrupt_enabled;
    s->active->cq[qid].qid = qid;
    s->active->cq[qid].depth = depth;
    s->active->cq[qid].shadow_tail = 0;
    s->active->cq[qid].phase = true;
    s->active->cq[qid].guest_base = guest_base;
    s->active->cq[qid].translated_base = translated;
    return true;
}

static bool scope_register_sq(ScopeProxyState *s, uint16_t qid, uint16_t cqid,
                              uint64_t guest_base, uint16_t depth)
{
    uint64_t translated = 0;

    if (!qid || qid >= SCOPE_MAX_NVME_QUEUES || !depth) {
        return false;
    }
    if (!scope_translate_guest_pa_for_real_dma(s, guest_base, 1, &translated)) {
        return false;
    }

    memset(&s->active->sq[qid], 0, sizeof(s->active->sq[qid]));
    s->active->sq[qid].valid = true;
    s->active->sq[qid].qid = qid;
    s->active->sq[qid].depth = depth;
    s->active->sq[qid].linked_cqid = cqid;
    s->active->sq[qid].guest_base = guest_base;
    s->active->sq[qid].translated_base = translated;
    return true;
}

static uint64_t scope_min_u64(uint64_t a, uint64_t b)
{
    return a < b ? a : b;
}

static bool scope_should_log_prp_success(const char *tag)
{
#if SCOPE_DEBUG_IO_PRP_TRACE
    return true;
#else
    return !tag || strcmp(tag, "io") != 0;
#endif
}

static uint64_t scope_div_round_up_u64(uint64_t n, uint64_t d)
{
    return d ? ((n + d - 1) / d) : 0;
}

static uint8_t scope_get_ns_lba_shift(ScopeProxyState *s, uint32_t nsid)
{
    gpointer value = NULL;

    if (s->active->ns_lba_shift_map &&
        g_hash_table_lookup_extended(s->active->ns_lba_shift_map,
                                     GUINT_TO_POINTER(nsid), NULL, &value)) {
        return (uint8_t)GPOINTER_TO_UINT(value);
    }

    SCOPE_PRINTF("[SCOPE PROXY][NS][LBA_SHIFT][MISS] nsid=%u using fallback_shift=%u "
           "(Identify Namespace has not completed yet)\n",
           nsid, SCOPE_NVME_MIN_LBA_SHIFT);
    SCOPE_FFLUSH(stdout);
    return SCOPE_NVME_MIN_LBA_SHIFT;
}

static uint64_t scope_admin_cmd_data_len(const NvmeCmd *cmd)
{
    switch (cmd->opcode) {
    case NVME_ADM_CMD_IDENTIFY:
        return NVME_IDENTIFY_DATA_SIZE;
    default:
        return 0;
    }
}

static uint64_t scope_io_cmd_data_len(ScopeProxyState *s, const NvmeCmd *cmd)
{
    switch (cmd->opcode) {
    case NVME_CMD_READ:
    case NVME_CMD_WRITE:
    case NVME_CMD_COMPARE: {
        const NvmeRwCmd *rw = (const NvmeRwCmd *)cmd;
        uint32_t nsid = le32_to_cpu(rw->nsid);
        uint64_t nlb = (uint64_t)le16_to_cpu(rw->nlb) + 1U;
        uint8_t lba_shift = scope_get_ns_lba_shift(s, nsid);

        if (lba_shift > 63 || nlb > (UINT64_MAX >> lba_shift)) {
            SCOPE_PRINTF("[SCOPE PROXY][CMD][LEN][ERR] opcode=0x%02x cid=%u nsid=%u "
                   "nlb=%" PRIu64 " lba_shift=%u overflow\n",
                   cmd->opcode, le16_to_cpu(cmd->cid), nsid, nlb, lba_shift);
            SCOPE_FFLUSH(stdout);
            return 0;
        }
        return nlb << lba_shift;
    }
    case NVME_CMD_DSM: {
        const NvmeDsmCmd *dsm = (const NvmeDsmCmd *)cmd;
        uint64_t ranges = (uint64_t)le32_to_cpu(dsm->nr) + 1U;

        if (ranges > UINT64_MAX / sizeof(NvmeDsmRange)) {
            SCOPE_PRINTF("[SCOPE PROXY][CMD][LEN][ERR] opcode=0x%02x cid=%u "
                   "DSM range count overflow nr=%u\n",
                   cmd->opcode, le16_to_cpu(cmd->cid), le32_to_cpu(dsm->nr));
            SCOPE_FFLUSH(stdout);
            return 0;
        }
        return ranges * sizeof(NvmeDsmRange);
    }
    default:
        return 0;
    }
}

static void scope_record_identify_ns_lba_shift(ScopeProxyState *s,
                                               uint32_t nsid,
                                               uint64_t data_guest_base)
{
    NvmeIdNs idns;
    uint8_t flbas_idx;
    uint8_t lba_shift;

    if (!nsid || nsid == NVME_NSID_BROADCAST) {
        SCOPE_PRINTF("[SCOPE PROXY][NS][LBA_SHIFT][SKIP] nsid=%u data_guest=0x%016"
               PRIx64 " is not a concrete namespace\n",
               nsid, data_guest_base);
        SCOPE_FFLUSH(stdout);
        return;
    }

    if (!scope_guest_mem_read(s, data_guest_base, &idns, sizeof(idns))) {
        SCOPE_PRINTF("[SCOPE PROXY][NS][LBA_SHIFT][ERR] nsid=%u data_guest=0x%016"
               PRIx64 " failed to read Identify Namespace data\n",
               nsid, data_guest_base);
        SCOPE_FFLUSH(stdout);
        return;
    }

    flbas_idx = NVME_ID_NS_FLBAS_INDEX(idns.flbas);
    if (flbas_idx >= NVME_MAX_NLBAF || flbas_idx > idns.nlbaf) {
        SCOPE_PRINTF("[SCOPE PROXY][NS][LBA_SHIFT][ERR] nsid=%u flbas=0x%02x "
               "flbas_idx=%u nlbaf=%u invalid\n",
               nsid, idns.flbas, flbas_idx, idns.nlbaf);
        SCOPE_FFLUSH(stdout);
        return;
    }

    lba_shift = idns.lbaf[flbas_idx].ds;
    if (lba_shift < SCOPE_NVME_MIN_LBA_SHIFT ||
        lba_shift > SCOPE_NVME_MAX_LBA_SHIFT) {
        SCOPE_PRINTF("[SCOPE PROXY][NS][LBA_SHIFT][ERR] nsid=%u flbas_idx=%u "
               "ds=%u outside supported range [%u,%u]\n",
               nsid, flbas_idx, lba_shift,
               SCOPE_NVME_MIN_LBA_SHIFT, SCOPE_NVME_MAX_LBA_SHIFT);
        SCOPE_FFLUSH(stdout);
        return;
    }

    if (!s->active->ns_lba_shift_map) {
        s->active->ns_lba_shift_map = g_hash_table_new(g_direct_hash, g_direct_equal);
    }
    g_hash_table_insert(s->active->ns_lba_shift_map, GUINT_TO_POINTER(nsid),
                        GUINT_TO_POINTER((guint)lba_shift));
    SCOPE_PRINTF("[SCOPE PROXY][NS][LBA_SHIFT] nsid=%u flbas=0x%02x idx=%u "
           "nlbaf=%u ds=%u lba_size=%" PRIu64 " nsze=%" PRIu64
           " ncap=%" PRIu64 "\n",
           nsid, idns.flbas, flbas_idx, idns.nlbaf, lba_shift,
           (uint64_t)1 << lba_shift,
           le64_to_cpu(idns.nsze), le64_to_cpu(idns.ncap));
    SCOPE_FFLUSH(stdout);
}

static void scope_update_ctrl_page_size(ScopeProxyState *s)
{
    uint32_t mps = NVME_CC_MPS(s->active->guest_cc);
    uint32_t page_size = SCOPE_NVME_DEFAULT_CTRL_PAGE_SIZE;

    if (mps <= 20) {
        page_size = 1U << (12U + mps);
    } else {
        SCOPE_PRINTF("[SCOPE PROXY][CTRL][MPS][WARN] cc=0x%08x mps=%u invalid, "
               "using page_size=%u\n",
               s->active->guest_cc, mps, page_size);
        SCOPE_FFLUSH(stdout);
    }

    if (s->active->ctrl_page_size != page_size) {
        SCOPE_PRINTF("[SCOPE PROXY][CTRL][MPS] cc=0x%08x mps=%u page_size=%u\n",
               s->active->guest_cc, mps, page_size);
        SCOPE_FFLUSH(stdout);
    }
    s->active->ctrl_page_size = page_size;
}

static bool scope_patch_prp_list(ScopeProxyState *s,
                                 uint64_t list_guest_pa,
                                 uint64_t bytes_remaining,
                                 uint32_t page_size,
                                 const char *tag,
                                 uint8_t opcode,
                                 uint16_t cid)
{
    uint64_t *entries;
    uint64_t patched_entries = 0;
    uint64_t list_pages = 0;
    uint64_t first_list_guest_pa = list_guest_pa;
    uint32_t entries_per_page = page_size / SCOPE_PRP_ENTRY_SIZE;

    if (page_size < SCOPE_PRP_ENTRY_SIZE || entries_per_page < 2) {
        SCOPE_PRINTF("[SCOPE PROXY][CMD][PRP_LIST][ERR] tag=%s opcode=0x%02x cid=%u "
               "invalid page_size=%u\n",
               tag ? tag : "cmd", opcode, cid, page_size);
        SCOPE_FFLUSH(stdout);
        return false;
    }

    entries = g_malloc(page_size);
    while (bytes_remaining) {
        uint64_t data_pages_left =
            scope_div_round_up_u64(bytes_remaining, page_size);
        uint64_t data_slots =
            (data_pages_left > entries_per_page) ?
            (entries_per_page - 1U) : data_pages_left;
        uint64_t current_list_guest_pa = list_guest_pa;
        bool has_next_list = data_pages_left > entries_per_page;

        if (++list_pages > SCOPE_PRP_LIST_PAGE_LIMIT) {
            SCOPE_PRINTF("[SCOPE PROXY][CMD][PRP_LIST][ERR] tag=%s opcode=0x%02x cid=%u "
                   "too many PRP list pages first=0x%016" PRIx64
                   " current=0x%016" PRIx64 "\n",
                   tag ? tag : "cmd", opcode, cid,
                   first_list_guest_pa, current_list_guest_pa);
            SCOPE_FFLUSH(stdout);
            g_free(entries);
            return false;
        }

        if (!scope_guest_mem_read(s, current_list_guest_pa, entries, page_size)) {
            SCOPE_PRINTF("[SCOPE PROXY][CMD][PRP_LIST][ERR] tag=%s opcode=0x%02x cid=%u "
                   "read list guest=0x%016" PRIx64 " failed\n",
                   tag ? tag : "cmd", opcode, cid, current_list_guest_pa);
            SCOPE_FFLUSH(stdout);
            g_free(entries);
            return false;
        }

        if (!data_slots) {
            SCOPE_PRINTF("[SCOPE PROXY][CMD][PRP_LIST][ERR] tag=%s opcode=0x%02x cid=%u "
                   "empty PRP list guest=0x%016" PRIx64
                   " bytes_remaining=%" PRIu64 "\n",
                   tag ? tag : "cmd", opcode, cid, current_list_guest_pa,
                   bytes_remaining);
            SCOPE_FFLUSH(stdout);
            g_free(entries);
            return false;
        }

        for (uint64_t i = 0; i < data_slots; i++) {
            uint64_t entry_addr = le64_to_cpu(entries[i]);
            uint64_t entry_guest_pa = 0;
            uint64_t translated = 0;
            uint64_t entry_len = scope_min_u64(bytes_remaining, page_size);

            if (!entry_addr) {
                SCOPE_PRINTF("[SCOPE PROXY][CMD][PRP_LIST][ERR] tag=%s opcode=0x%02x cid=%u "
                       "zero data entry list=0x%016" PRIx64 " index=%" PRIu64 "\n",
                       tag ? tag : "cmd", opcode, cid, current_list_guest_pa, i);
                SCOPE_FFLUSH(stdout);
                g_free(entries);
                return false;
            }

            if (!scope_translate_cmd_dma_addr_for_real(s, entry_addr, (size_t)entry_len,
                                                       "prp_list.data",
                                                       &entry_guest_pa, &translated)) {
                SCOPE_PRINTF("[SCOPE PROXY][CMD][PRP_LIST][ERR] tag=%s opcode=0x%02x cid=%u "
                       "data entry translation failed list=0x%016" PRIx64
                       " index=%" PRIu64 " addr=0x%016" PRIx64
                       " len=%" PRIu64 "\n",
                       tag ? tag : "cmd", opcode, cid, current_list_guest_pa,
                       i, entry_addr, entry_len);
                SCOPE_FFLUSH(stdout);
                g_free(entries);
                return false;
            }

            entries[i] = cpu_to_le64(translated);
            bytes_remaining -= entry_len;
            patched_entries++;
        }

        if (has_next_list) {
            uint64_t next_addr = le64_to_cpu(entries[entries_per_page - 1U]);
            uint64_t next_guest_pa = 0;
            uint64_t next_translated = 0;

            if (!next_addr) {
                SCOPE_PRINTF("[SCOPE PROXY][CMD][PRP_LIST][ERR] tag=%s opcode=0x%02x cid=%u "
                       "missing next-list pointer list=0x%016" PRIx64 "\n",
                       tag ? tag : "cmd", opcode, cid, current_list_guest_pa);
                SCOPE_FFLUSH(stdout);
                g_free(entries);
                return false;
            }
            if (!scope_translate_cmd_dma_addr_for_real(s, next_addr, page_size,
                                                       "prp_list.next",
                                                       &next_guest_pa,
                                                       &next_translated)) {
                SCOPE_PRINTF("[SCOPE PROXY][CMD][PRP_LIST][ERR] tag=%s opcode=0x%02x cid=%u "
                       "next-list translation failed list=0x%016" PRIx64
                       " next=0x%016" PRIx64 "\n",
                       tag ? tag : "cmd", opcode, cid, current_list_guest_pa,
                       next_addr);
                SCOPE_FFLUSH(stdout);
                g_free(entries);
                return false;
            }
            entries[entries_per_page - 1U] = cpu_to_le64(next_translated);
            list_guest_pa = next_guest_pa;
        }

        if (!scope_guest_mem_write(s, current_list_guest_pa, entries, page_size)) {
            SCOPE_PRINTF("[SCOPE PROXY][CMD][PRP_LIST][ERR] tag=%s opcode=0x%02x cid=%u "
                   "write patched list guest=0x%016" PRIx64 " failed\n",
                   tag ? tag : "cmd", opcode, cid, current_list_guest_pa);
            SCOPE_FFLUSH(stdout);
            g_free(entries);
            return false;
        }
    }

    if (scope_should_log_prp_success(tag)) {
        SCOPE_PRINTF("[SCOPE PROXY][CMD][PRP_LIST] tag=%s opcode=0x%02x cid=%u "
               "first_list_guest=0x%016" PRIx64 " list_pages=%" PRIu64
               " data_entries=%" PRIu64 " page_size=%u\n",
               tag ? tag : "cmd", opcode, cid, first_list_guest_pa, list_pages,
               patched_entries, page_size);
        SCOPE_FFLUSH(stdout);
    }
    g_free(entries);
    return true;
}

static bool scope_patch_simple_command_prps(ScopeProxyState *s, NvmeCmd *cmd,
                                            const char *tag,
                                            uint64_t *first_data_guest_pa)
{
    uint64_t translated = 0;
    uint64_t guest_pa = 0;

    if (first_data_guest_pa) {
        *first_data_guest_pa = 0;
    }

    if (cmd->dptr.prp1) {
        if (!scope_translate_cmd_dma_addr_for_real(s, le64_to_cpu(cmd->dptr.prp1), 1,
                                                   "prp1", &guest_pa, &translated)) {
            SCOPE_PRINTF("[SCOPE PROXY][CMD][PATCH][ERR] tag=%s opcode=0x%02x cid=%u "
                   "prp1=0x%016" PRIx64 " translation failed\n",
                   tag ? tag : "cmd", cmd->opcode, le16_to_cpu(cmd->cid),
                   le64_to_cpu(cmd->dptr.prp1));
            SCOPE_FFLUSH(stdout);
            return false;
        }
        cmd->dptr.prp1 = cpu_to_le64(translated);
        if (first_data_guest_pa) {
            *first_data_guest_pa = guest_pa;
        }
    }
    if (cmd->dptr.prp2) {
        if (!scope_translate_cmd_dma_addr_for_real(s, le64_to_cpu(cmd->dptr.prp2), 1,
                                                   "prp2", NULL, &translated)) {
            SCOPE_PRINTF("[SCOPE PROXY][CMD][PATCH][ERR] tag=%s opcode=0x%02x cid=%u "
                   "prp2=0x%016" PRIx64 " translation failed\n",
                   tag ? tag : "cmd", cmd->opcode, le16_to_cpu(cmd->cid),
                   le64_to_cpu(cmd->dptr.prp2));
            SCOPE_FFLUSH(stdout);
            return false;
        }
        cmd->dptr.prp2 = cpu_to_le64(translated);
    }

    return true;
}

static bool scope_patch_prps_for_real_dma(ScopeProxyState *s, NvmeCmd *cmd,
                                          uint64_t data_len,
                                          const char *tag,
                                          uint64_t *first_data_guest_pa)
{
    uint32_t page_size = s->active->ctrl_page_size ?
                         s->active->ctrl_page_size : SCOPE_NVME_DEFAULT_CTRL_PAGE_SIZE;
    uint64_t page_mask = (uint64_t)page_size - 1ULL;
    uint64_t prp1_addr = le64_to_cpu(cmd->dptr.prp1);
    uint64_t prp2_addr = le64_to_cpu(cmd->dptr.prp2);
    uint64_t prp1_guest_pa = 0;
    uint64_t translated = 0;
    uint64_t first_len;
    uint64_t remaining;

    if (first_data_guest_pa) {
        *first_data_guest_pa = 0;
    }

    if (!data_len) {
        return scope_patch_simple_command_prps(s, cmd, tag, first_data_guest_pa);
    }

    if (!prp1_addr) {
        SCOPE_PRINTF("[SCOPE PROXY][CMD][PRP][ERR] tag=%s opcode=0x%02x cid=%u "
               "data_len=%" PRIu64 " but PRP1 is zero\n",
               tag ? tag : "cmd", cmd->opcode, le16_to_cpu(cmd->cid), data_len);
        SCOPE_FFLUSH(stdout);
        return false;
    }

    if (page_size < SCOPE_NVME_DEFAULT_CTRL_PAGE_SIZE ||
        (page_size & (page_size - 1U)) != 0) {
        SCOPE_PRINTF("[SCOPE PROXY][CMD][PRP][ERR] tag=%s opcode=0x%02x cid=%u "
               "invalid controller page_size=%u\n",
               tag ? tag : "cmd", cmd->opcode, le16_to_cpu(cmd->cid), page_size);
        SCOPE_FFLUSH(stdout);
        return false;
    }

    first_len = scope_min_u64(data_len, page_size - (prp1_addr & page_mask));
    if (!scope_translate_cmd_dma_addr_for_real(s, prp1_addr, (size_t)first_len,
                                               "prp1", &prp1_guest_pa, &translated)) {
        SCOPE_PRINTF("[SCOPE PROXY][CMD][PRP][ERR] tag=%s opcode=0x%02x cid=%u "
               "PRP1 translation failed prp1=0x%016" PRIx64
               " len=%" PRIu64 "\n",
               tag ? tag : "cmd", cmd->opcode, le16_to_cpu(cmd->cid),
               prp1_addr, first_len);
        SCOPE_FFLUSH(stdout);
        return false;
    }
    cmd->dptr.prp1 = cpu_to_le64(translated);
    if (first_data_guest_pa) {
        *first_data_guest_pa = prp1_guest_pa;
    }

    remaining = data_len - first_len;
    if (!remaining) {
        if (scope_should_log_prp_success(tag)) {
            SCOPE_PRINTF("[SCOPE PROXY][CMD][PRP] tag=%s opcode=0x%02x cid=%u "
                   "data_len=%" PRIu64 " page_size=%u mode=prp1\n",
                   tag ? tag : "cmd", cmd->opcode, le16_to_cpu(cmd->cid),
                   data_len, page_size);
            SCOPE_FFLUSH(stdout);
        }
        return true;
    }

    if (!prp2_addr) {
        SCOPE_PRINTF("[SCOPE PROXY][CMD][PRP][ERR] tag=%s opcode=0x%02x cid=%u "
               "data_len=%" PRIu64 " remaining=%" PRIu64 " but PRP2 is zero\n",
               tag ? tag : "cmd", cmd->opcode, le16_to_cpu(cmd->cid),
               data_len, remaining);
        SCOPE_FFLUSH(stdout);
        return false;
    }

    if (remaining <= page_size) {
        uint64_t prp2_guest_pa = 0;

        if (!scope_translate_cmd_dma_addr_for_real(s, prp2_addr, (size_t)remaining,
                                                   "prp2", &prp2_guest_pa,
                                                   &translated)) {
            SCOPE_PRINTF("[SCOPE PROXY][CMD][PRP][ERR] tag=%s opcode=0x%02x cid=%u "
                   "PRP2 data translation failed prp2=0x%016" PRIx64
                   " len=%" PRIu64 "\n",
                   tag ? tag : "cmd", cmd->opcode, le16_to_cpu(cmd->cid),
                   prp2_addr, remaining);
            SCOPE_FFLUSH(stdout);
            return false;
        }
        cmd->dptr.prp2 = cpu_to_le64(translated);
        if (scope_should_log_prp_success(tag)) {
            SCOPE_PRINTF("[SCOPE PROXY][CMD][PRP] tag=%s opcode=0x%02x cid=%u "
                   "data_len=%" PRIu64 " page_size=%u mode=prp1_prp2 "
                   "prp2_guest=0x%016" PRIx64 "\n",
                   tag ? tag : "cmd", cmd->opcode, le16_to_cpu(cmd->cid),
                   data_len, page_size, prp2_guest_pa);
            SCOPE_FFLUSH(stdout);
        }
        return true;
    }

    {
        uint64_t list_guest_pa = 0;

        if (!scope_translate_cmd_dma_addr_for_real(s, prp2_addr, page_size,
                                                   "prp2.list",
                                                   &list_guest_pa, &translated)) {
            SCOPE_PRINTF("[SCOPE PROXY][CMD][PRP][ERR] tag=%s opcode=0x%02x cid=%u "
                   "PRP2 list pointer translation failed prp2=0x%016" PRIx64
                   " page_size=%u\n",
                   tag ? tag : "cmd", cmd->opcode, le16_to_cpu(cmd->cid),
                   prp2_addr, page_size);
            SCOPE_FFLUSH(stdout);
            return false;
        }
        cmd->dptr.prp2 = cpu_to_le64(translated);
        if (scope_should_log_prp_success(tag)) {
            SCOPE_PRINTF("[SCOPE PROXY][CMD][PRP] tag=%s opcode=0x%02x cid=%u "
                   "data_len=%" PRIu64 " page_size=%u mode=prp_list "
                   "list_guest=0x%016" PRIx64 "\n",
                   tag ? tag : "cmd", cmd->opcode, le16_to_cpu(cmd->cid),
                   data_len, page_size, list_guest_pa);
            SCOPE_FFLUSH(stdout);
        }
        return scope_patch_prp_list(s, list_guest_pa, remaining, page_size,
                                    tag, cmd->opcode, le16_to_cpu(cmd->cid));
    }
}

static bool scope_patch_common_command_buffers(ScopeProxyState *s, NvmeCmd *cmd,
                                               uint64_t data_len,
                                               const char *tag,
                                               uint64_t *first_data_guest_pa)
{
    uint64_t translated = 0;
    uint8_t psdt = NVME_CMD_FLAGS_PSDT(cmd->flags);

    if (first_data_guest_pa) {
        *first_data_guest_pa = 0;
    }

    if (cmd->mptr) {
        if (!scope_translate_cmd_dma_addr_for_real(s, le64_to_cpu(cmd->mptr), 1,
                                                   "mptr", NULL, &translated)) {
            SCOPE_PRINTF("[SCOPE PROXY][CMD][PATCH][ERR] tag=%s opcode=0x%02x cid=%u "
                   "mptr=0x%016" PRIx64 " translation failed\n",
                   tag ? tag : "cmd", cmd->opcode, le16_to_cpu(cmd->cid),
                   le64_to_cpu(cmd->mptr));
            SCOPE_FFLUSH(stdout);
            return false;
        }
        cmd->mptr = cpu_to_le64(translated);
    }

    if (psdt != NVME_PSDT_PRP) {
        if (cmd->dptr.prp1 || cmd->dptr.prp2) {
            SCOPE_PRINTF("[SCOPE PROXY][CMD][PATCH][ERR] tag=%s opcode=0x%02x cid=%u "
                   "unsupported PSDT=%u with prp1=0x%016" PRIx64
                   " prp2=0x%016" PRIx64 "\n",
                   tag ? tag : "cmd", cmd->opcode, le16_to_cpu(cmd->cid), psdt,
                   le64_to_cpu(cmd->dptr.prp1), le64_to_cpu(cmd->dptr.prp2));
            SCOPE_FFLUSH(stdout);
        }
        return (cmd->dptr.prp1 == 0 && cmd->dptr.prp2 == 0);
    }

    return scope_patch_prps_for_real_dma(s, cmd, data_len, tag, first_data_guest_pa);
}

static void scope_stage_pending_admin_op(ScopeProxyState *s, uint16_t cid,
                                         const ScopePendingAdminOp *op)
{
    if (!s->active->pending_admin_ops || !op || !op->valid) {
        return;
    }

    s->active->pending_admin_ops[cid] = *op;
}

static bool scope_admin_cqe_success(const NvmeCqe *cqe)
{
    return (le16_to_cpu(cqe->status) >> 1) == NVME_SUCCESS;
}

static bool scope_admin_cqe_expected(ScopeProxyState *s, const ScopeCqState *cq,
                                     const NvmeCqe *cqe)
{
    uint16_t cid = le16_to_cpu(cqe->cid);
    uint16_t sq_id = le16_to_cpu(cqe->sq_id);
    uint16_t sq_head = le16_to_cpu(cqe->sq_head);
    const ScopeSqState *admin_sq = &s->active->sq[SCOPE_ADMIN_QID];

    if (sq_id != SCOPE_ADMIN_QID) {
        SCOPE_PRINTF("[SCOPE PROXY][CQ][STALE] qid=%u tail=%u cid=%u "
               "unexpected_sq_id=%u status=0x%04x\n",
               cq->qid, cq->shadow_tail, cid, sq_id, le16_to_cpu(cqe->status));
        SCOPE_FFLUSH(stdout);
        return false;
    }

    if (admin_sq->valid && admin_sq->depth && sq_head >= admin_sq->depth) {
        SCOPE_PRINTF("[SCOPE PROXY][CQ][STALE] qid=%u tail=%u cid=%u "
               "sq_head=%u depth=%u status=0x%04x\n",
               cq->qid, cq->shadow_tail, cid, sq_head, admin_sq->depth,
               le16_to_cpu(cqe->status));
        SCOPE_FFLUSH(stdout);
        return false;
    }

    if (!s->active->admin_cid_outstanding[cid]) {
        SCOPE_PRINTF("[SCOPE PROXY][CQ][STALE] qid=%u tail=%u cid=%u "
               "sq_head=%u no_outstanding_cmd status=0x%04x\n",
               cq->qid, cq->shadow_tail, cid, sq_head, le16_to_cpu(cqe->status));
        SCOPE_FFLUSH(stdout);
        return false;
    }

    return true;
}

static void scope_commit_pending_admin_op(ScopeProxyState *s, const NvmeCqe *cqe)
{
    ScopePendingAdminOp *op;
    uint16_t cid;
    bool ok = true;

    if (!s->active->pending_admin_ops) {
        return;
    }

    cid = le16_to_cpu(cqe->cid);
    op = &s->active->pending_admin_ops[cid];
    if (!op->valid) {
        return;
    }

    if (scope_admin_cqe_success(cqe)) {
        switch (op->type) {
        case SCOPE_ADMIN_TOPO_OP_CREATE_CQ:
            ok = scope_register_cq(s, op->qid, op->guest_base, op->depth,
                                   op->interrupt_enabled);
            break;
        case SCOPE_ADMIN_TOPO_OP_CREATE_SQ:
            ok = scope_register_sq(s, op->qid, op->cqid, op->guest_base, op->depth);
            break;
        case SCOPE_ADMIN_TOPO_OP_DELETE_CQ:
            if (op->qid < SCOPE_MAX_NVME_QUEUES) {
                memset(&s->active->cq[op->qid], 0, sizeof(s->active->cq[op->qid]));
            } else {
                ok = false;
            }
            break;
        case SCOPE_ADMIN_TOPO_OP_DELETE_SQ:
            if (op->qid < SCOPE_MAX_NVME_QUEUES) {
                memset(&s->active->sq[op->qid], 0, sizeof(s->active->sq[op->qid]));
            } else {
                ok = false;
            }
            break;
        case SCOPE_ADMIN_TOPO_OP_IDENTIFY_NS:
            scope_record_identify_ns_lba_shift(s, op->nsid, op->data_guest_base);
            break;
        default:
            break;
        }
    }

    if (!ok) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE: Failed to apply admin topology update for cid=%u type=%u\n",
                      cid, op->type);
    }

    memset(op, 0, sizeof(*op));
}

static bool scope_patch_admin_cmd(ScopeProxyState *s, NvmeCmd *cmd,
                                  ScopePendingAdminOp *pending_op)
{
    memset(pending_op, 0, sizeof(*pending_op));

    switch (cmd->opcode) {
    case NVME_ADM_CMD_CREATE_CQ: {
        NvmeCreateCq *cq = (NvmeCreateCq *)cmd;
        uint64_t guest_base = 0;
        uint64_t translated = 0;

        if (!scope_translate_cmd_dma_addr_for_real(s, cq->prp1, 1,
                                                   "create_cq.prp1",
                                                   &guest_base, &translated)) {
            SCOPE_PRINTF("[SCOPE PROXY][CMD][ADMIN][ERR] CREATE_CQ cid=%u cqid=%u qsize=%u "
                   "guest_base=0x%016" PRIx64 " translation failed\n",
                   le16_to_cpu(cq->cid), le16_to_cpu(cq->cqid), le16_to_cpu(cq->qsize) + 1U,
                   le64_to_cpu(cq->prp1));
            SCOPE_FFLUSH(stdout);
            return false;
        }
        cq->prp1 = translated;
        pending_op->valid = true;
        pending_op->type = SCOPE_ADMIN_TOPO_OP_CREATE_CQ;
        pending_op->qid = cq->cqid;
        pending_op->depth = cq->qsize + 1;
        pending_op->interrupt_enabled = NVME_CQ_FLAGS_IEN(cq->cq_flags);
        pending_op->guest_base = guest_base;
        return true;
    }
    case NVME_ADM_CMD_CREATE_SQ: {
        NvmeCreateSq *sq = (NvmeCreateSq *)cmd;
        uint64_t guest_base = 0;
        uint64_t translated = 0;

        if (!scope_translate_cmd_dma_addr_for_real(s, sq->prp1, 1,
                                                   "create_sq.prp1",
                                                   &guest_base, &translated)) {
            SCOPE_PRINTF("[SCOPE PROXY][CMD][ADMIN][ERR] CREATE_SQ cid=%u sqid=%u cqid=%u "
                   "qsize=%u guest_base=0x%016" PRIx64 " translation failed\n",
                   le16_to_cpu(sq->cid), le16_to_cpu(sq->sqid), le16_to_cpu(sq->cqid),
                   le16_to_cpu(sq->qsize) + 1U, le64_to_cpu(sq->prp1));
            SCOPE_FFLUSH(stdout);
            return false;
        }
        sq->prp1 = translated;
        pending_op->valid = true;
        pending_op->type = SCOPE_ADMIN_TOPO_OP_CREATE_SQ;
        pending_op->qid = sq->sqid;
        pending_op->cqid = sq->cqid;
        pending_op->depth = sq->qsize + 1;
        pending_op->guest_base = guest_base;
        return true;
    }
    case NVME_ADM_CMD_DELETE_CQ: {
        NvmeDeleteQ *del = (NvmeDeleteQ *)cmd;

        if (!del->qid || del->qid >= SCOPE_MAX_NVME_QUEUES) {
            SCOPE_PRINTF("[SCOPE PROXY][CMD][ADMIN][ERR] DELETE_CQ cid=%u invalid qid=%u\n",
                   le16_to_cpu(del->cid), le16_to_cpu(del->qid));
            SCOPE_FFLUSH(stdout);
            return false;
        }
        pending_op->valid = true;
        pending_op->type = SCOPE_ADMIN_TOPO_OP_DELETE_CQ;
        pending_op->qid = del->qid;
        return true;
    }
    case NVME_ADM_CMD_DELETE_SQ: {
        NvmeDeleteQ *del = (NvmeDeleteQ *)cmd;

        if (!del->qid || del->qid >= SCOPE_MAX_NVME_QUEUES) {
            SCOPE_PRINTF("[SCOPE PROXY][CMD][ADMIN][ERR] DELETE_SQ cid=%u invalid qid=%u\n",
                   le16_to_cpu(del->cid), le16_to_cpu(del->qid));
            SCOPE_FFLUSH(stdout);
            return false;
        }
        pending_op->valid = true;
        pending_op->type = SCOPE_ADMIN_TOPO_OP_DELETE_SQ;
        pending_op->qid = del->qid;
        return true;
    }
    case NVME_ADM_CMD_IDENTIFY: {
        NvmeIdentify *id = (NvmeIdentify *)cmd;
        uint32_t nsid = le32_to_cpu(id->nsid);
        uint8_t cns = id->cns;
        uint64_t data_guest_base = 0;

        if (!scope_patch_common_command_buffers(s, cmd,
                                                scope_admin_cmd_data_len(cmd),
                                                "admin_identify",
                                                &data_guest_base)) {
            return false;
        }

        if ((cns == NVME_ID_CNS_NS || cns == NVME_ID_CNS_NS_PRESENT) &&
            nsid && nsid != NVME_NSID_BROADCAST) {
            pending_op->valid = true;
            pending_op->type = SCOPE_ADMIN_TOPO_OP_IDENTIFY_NS;
            pending_op->nsid = nsid;
            pending_op->data_guest_base = data_guest_base;
            SCOPE_PRINTF("[SCOPE PROXY][NS][IDENTIFY_TRACK] cid=%u nsid=%u cns=0x%02x "
                   "data_guest=0x%016" PRIx64 "\n",
                   le16_to_cpu(cmd->cid), nsid, cns, data_guest_base);
            SCOPE_FFLUSH(stdout);
        }
        return true;
    }
    default:
        return scope_patch_common_command_buffers(s, cmd,
                                                  scope_admin_cmd_data_len(cmd),
                                                  "admin",
                                                  NULL);
    }
}

static bool scope_patch_io_cmd(ScopeProxyState *s, const ScopeSqState *sq,
                               NvmeCmd *cmd)
{
    uint64_t data_len = scope_io_cmd_data_len(s, cmd);

    if (data_len && SCOPE_DEBUG_IO_PRP_TRACE) {
        SCOPE_PRINTF("[SCOPE PROXY][CMD][IO][LEN] qid=%u opcode=0x%02x cid=%u "
               "nsid=%u data_len=%" PRIu64 "\n",
               sq ? sq->qid : 0U, cmd->opcode, le16_to_cpu(cmd->cid),
               le32_to_cpu(cmd->nsid), data_len);
        SCOPE_FFLUSH(stdout);
    }
    return scope_patch_common_command_buffers(s, cmd, data_len, "io", NULL);
}

static ScopeSqeReadStatus scope_process_new_sq_entries(ScopeProxyState *s,
                                                       ScopeSqState *sq,
                                                       uint16_t new_tail,
                                                       bool allow_stable_seed)
{
    uint16_t cursor;

    if (!sq->valid || !sq->depth || new_tail >= sq->depth) {
        SCOPE_PRINTF("[SCOPE PROXY][SQ][ERR] invalid SQ state qid=%u valid=%d depth=%u "
               "last_tail=%u new_tail=%u\n",
               sq ? sq->qid : 0U, sq ? sq->valid : 0, sq ? sq->depth : 0U,
               sq ? sq->last_guest_tail : 0U, new_tail);
        SCOPE_FFLUSH(stdout);
        return SCOPE_SQE_READ_ERR;
    }

    cursor = sq->last_guest_tail;
    while (cursor != new_tail) {
        uint64_t cmd_guest_pa = sq->guest_base + (uint64_t)cursor * sizeof(NvmeCmd);
        NvmeCmd cmd;
        NvmeCmd cmd_visible;
        ScopePendingAdminOp pending_admin_op = { 0 };
        uint64_t cmd_bar_offset = 0;
        uint64_t cmd_translated_pa = 0;
        bool have_translation = false;
        bool ok;

        if (sq->qid == SCOPE_ADMIN_QID) {
            ScopeSqeReadStatus read_status =
                scope_read_admin_sqe_stable(s, sq, cursor, cmd_guest_pa, &cmd,
                                            allow_stable_seed);

            if (read_status == SCOPE_SQE_READ_WAIT) {
                SCOPE_PRINTF("[SCOPE PROXY][SQ][WAIT_VISIBLE] qid=%u slot=%u "
                       "guest_pa=0x%016" PRIx64 " depth=%u last_tail=%u new_tail=%u\n",
                       sq->qid, cursor, cmd_guest_pa, sq->depth,
                       sq->last_guest_tail, new_tail);
                SCOPE_FFLUSH(stdout);
                return SCOPE_SQE_READ_WAIT;
            }
            if (read_status != SCOPE_SQE_READ_OK) {
                SCOPE_PRINTF("[SCOPE PROXY][SQ][ERR] failed to read stable admin SQE qid=%u slot=%u "
                       "guest_pa=0x%016" PRIx64 " depth=%u last_tail=%u new_tail=%u\n",
                       sq->qid, cursor, cmd_guest_pa, sq->depth,
                       sq->last_guest_tail, new_tail);
                SCOPE_FFLUSH(stdout);
                return SCOPE_SQE_READ_ERR;
            }
        } else if (!scope_guest_mem_read(s, cmd_guest_pa, &cmd, sizeof(cmd))) {
            SCOPE_PRINTF("[SCOPE PROXY][SQ][ERR] failed to read SQE qid=%u slot=%u "
                   "guest_pa=0x%016" PRIx64 " depth=%u last_tail=%u new_tail=%u\n",
                   sq->qid, cursor, cmd_guest_pa, sq->depth, sq->last_guest_tail, new_tail);
            SCOPE_FFLUSH(stdout);
            return SCOPE_SQE_READ_ERR;
        }

        if (sq->qid == SCOPE_ADMIN_QID) {
            have_translation =
                scope_guest_range_to_bar_offset(s, cmd_guest_pa, sizeof(cmd), &cmd_bar_offset) &&
                scope_translate_guest_pa_for_host_bypass(s, cmd_guest_pa, sizeof(cmd),
                                                         &cmd_translated_pa);
            if (have_translation) {
                scope_log_guest_translation("RAW", sq, cursor, cmd_guest_pa,
                                            sizeof(cmd), cmd_bar_offset, cmd_translated_pa);
            }
            scope_log_nvme_cmd("RAW", sq, cursor, cmd_guest_pa, &cmd);
            scope_log_nvme_cmd_dwords("RAW", sq, cursor, cmd_guest_pa, &cmd);
            scope_log_nvme_cmd_hexdump("RAW", sq, cursor, cmd_guest_pa, &cmd);
        }

        ok = (sq->qid == SCOPE_ADMIN_QID) ?
            scope_patch_admin_cmd(s, &cmd, &pending_admin_op) :
            scope_patch_io_cmd(s, sq, &cmd);
        if (!ok) {
            scope_log_nvme_cmd("PATCH_ERR", sq, cursor, cmd_guest_pa, &cmd);
            scope_log_nvme_cmd_dwords("PATCH_ERR", sq, cursor, cmd_guest_pa, &cmd);
            scope_log_nvme_cmd_hexdump("PATCH_ERR", sq, cursor, cmd_guest_pa, &cmd);
            return SCOPE_SQE_READ_ERR;
        }
        if (sq->qid == SCOPE_ADMIN_QID) {
            scope_log_nvme_cmd("PATCHED", sq, cursor, cmd_guest_pa, &cmd);
            scope_log_nvme_cmd_dwords("PATCHED", sq, cursor, cmd_guest_pa, &cmd);
        }

        if (!scope_guest_mem_write(s, cmd_guest_pa, &cmd, sizeof(cmd))) {
            scope_log_nvme_cmd("WRITEBACK_ERR", sq, cursor, cmd_guest_pa, &cmd);
            return SCOPE_SQE_READ_ERR;
        }
        if (sq->qid == SCOPE_ADMIN_QID &&
            scope_guest_mem_read(s, cmd_guest_pa, &cmd_visible, sizeof(cmd_visible))) {
            if (memcmp(&cmd, &cmd_visible, sizeof(cmd)) != 0) {
                SCOPE_PRINTF("[SCOPE PROXY][CMD][VISIBLE_AFTER_WRITEBACK][ERR] qid=%u slot=%u "
                       "guest_pa=0x%016" PRIx64 "\n",
                       sq->qid, cursor, cmd_guest_pa);
                SCOPE_FFLUSH(stdout);
                scope_log_nvme_cmd("VISIBLE_AFTER_WRITEBACK", sq, cursor, cmd_guest_pa,
                                   &cmd_visible);
                scope_log_nvme_cmd_dwords("VISIBLE_AFTER_WRITEBACK", sq, cursor, cmd_guest_pa,
                                          &cmd_visible);
                scope_log_nvme_cmd_hexdump("VISIBLE_AFTER_WRITEBACK", sq, cursor, cmd_guest_pa,
                                           &cmd_visible);
            } else if (have_translation) {
                SCOPE_PRINTF("[SCOPE PROXY][CMD][VISIBLE_AFTER_WRITEBACK][OK] qid=%u slot=%u "
                       "guest_pa=0x%016" PRIx64 " host_bypass=0x%016" PRIx64 "\n",
                       sq->qid, cursor, cmd_guest_pa, cmd_translated_pa);
                SCOPE_FFLUSH(stdout);
            }
        }
        if (sq->qid == SCOPE_ADMIN_QID && pending_admin_op.valid) {
            scope_stage_pending_admin_op(s, le16_to_cpu(cmd.cid), &pending_admin_op);
        }
        if (sq->qid == SCOPE_ADMIN_QID) {
            uint16_t cid = le16_to_cpu(cmd.cid);

            if (!s->active->admin_cid_outstanding[cid]) {
                s->active->admin_cid_outstanding[cid] = true;
                s->active->admin_outstanding_count++;
            }
            SCOPE_PRINTF("[SCOPE PROXY][CMD][ADMIN][TRACK] backend=%u qid=%u "
                   "slot=%u cid=%u outstanding=%u\n",
                   s->active->id, sq->qid, cursor, cid,
                   s->active->admin_outstanding_count);
            SCOPE_FFLUSH(stdout);
        }

        cursor = (cursor + 1U) % sq->depth;
    }

    sq->last_guest_tail = new_tail;
    return SCOPE_SQE_READ_OK;
}

static bool scope_is_doorbell_offset(ScopeProxyState *s, uint32_t aligned_offset,
                                     bool *is_sq, uint16_t *qid)
{
    uint32_t rel;
    uint32_t index;

    if (!s->active->doorbell_stride || aligned_offset < SCOPE_NVME_DOORBELL_BASE) {
        return false;
    }

    rel = aligned_offset - SCOPE_NVME_DOORBELL_BASE;
    if (rel % s->active->doorbell_stride) {
        return false;
    }

    index = rel / s->active->doorbell_stride;
    *qid = index >> 1;
    *is_sq = ((index & 1U) == 0);
    return true;
}

static bool scope_read_guest_cqe_stable(ScopeProxyState *s, uint64_t guest_pa,
                                        NvmeCqe *cqe)
{
    NvmeCqe a;
    NvmeCqe b;

    if (!scope_guest_mem_read(s, guest_pa, &a, sizeof(a))) {
        return false;
    }
    smp_rmb();
    if (!scope_guest_mem_read(s, guest_pa, &b, sizeof(b))) {
        return false;
    }
    if (memcmp(&a, &b, sizeof(a)) != 0) {
        return false;
    }

    *cqe = a;
    return true;
}

static bool scope_publish_guest_cqe_for_cpu(ScopeProxyState *s, const ScopeCqState *cq,
                                            uint64_t guest_pa, const NvmeCqe *cqe)
{
    NvmeCqe visible;

    if (!scope_guest_mem_write(s, guest_pa, cqe, sizeof(*cqe))) {
        SCOPE_PRINTF("[SCOPE PROXY][CQ][PUBLISH][ERR] qid=%u tail=%u guest_pa=0x%016"
               PRIx64 " write failed\n",
               cq->qid, cq->shadow_tail, guest_pa);
        SCOPE_FFLUSH(stdout);
        return false;
    }

    smp_wmb();
    if (!scope_read_guest_cqe_stable(s, guest_pa, &visible)) {
        SCOPE_PRINTF("[SCOPE PROXY][CQ][PUBLISH][ERR] qid=%u tail=%u guest_pa=0x%016"
               PRIx64 " readback failed\n",
               cq->qid, cq->shadow_tail, guest_pa);
        SCOPE_FFLUSH(stdout);
        return false;
    }

    if (memcmp(&visible, cqe, sizeof(*cqe)) != 0) {
        SCOPE_PRINTF("[SCOPE PROXY][CQ][PUBLISH][ERR] qid=%u tail=%u guest_pa=0x%016"
               PRIx64 " readback mismatch\n",
               cq->qid, cq->shadow_tail, guest_pa);
        SCOPE_FFLUSH(stdout);
        scope_log_cqe_raw("PUBLISH_EXPECT", cq->qid, cq->shadow_tail,
                          guest_pa, cqe);
        scope_log_cqe_raw("PUBLISH_VISIBLE", cq->qid, cq->shadow_tail,
                          guest_pa, &visible);
        return false;
    }

    if (cq->qid == SCOPE_ADMIN_QID) {
        SCOPE_PRINTF("[SCOPE PROXY][CQ][PUBLISH] qid=%u tail=%u guest_pa=0x%016"
               PRIx64 " result=OK\n",
               cq->qid, cq->shadow_tail, guest_pa);
        SCOPE_FFLUSH(stdout);
    } else {
        SCOPE_IO_PRINTF("[SCOPE PROXY][CQ][PUBLISH] qid=%u tail=%u guest_pa=0x%016"
               PRIx64 " result=OK\n",
               cq->qid, cq->shadow_tail, guest_pa);
        SCOPE_IO_FFLUSH(stdout);
    }
    return true;
}

static bool scope_cqe_phase_matches(const NvmeCqe *cqe, bool expected_phase)
{
    return !!(le16_to_cpu(cqe->status) & 0x1U) == expected_phase;
}

static bool scope_admin_has_outstanding_cmd(const ScopeProxyState *s)
{
    return s->active->admin_outstanding_count != 0;
}

static bool scope_refresh_cq_shadow_tail(ScopeProxyState *s, ScopeCqState *cq,
                                         bool *advanced)
{
    bool moved = false;

    if (!cq->valid || !cq->depth) {
        if (advanced) {
            *advanced = false;
        }
        return true;
    }

    /*
     * Do not interpret stale Admin CQ contents before QEMU has forwarded an
     * Admin command.  The guest owns ACQ memory and it can contain old phase-1
     * data across boots/tests; scanning it early produced fake CQ_STALE entries
     * and could perturb the virtual INTx state before the first SQ doorbell.
     */
    if (cq->qid == SCOPE_ADMIN_QID && !scope_admin_has_outstanding_cmd(s)) {
        if (advanced) {
            *advanced = false;
        }
        return true;
    }

    for (;;) {
        NvmeCqe cqe;
        uint64_t guest_pa;

        guest_pa = cq->guest_base + (uint64_t)cq->shadow_tail * sizeof(cqe);
        if (!scope_read_guest_cqe_stable(s, guest_pa, &cqe)) {
            break;
        }
        if (!scope_cqe_phase_matches(&cqe, cq->phase)) {
            break;
        }
        if (cq->qid == SCOPE_ADMIN_QID &&
            !scope_admin_cqe_expected(s, cq, &cqe)) {
            break;
        }
        if (cq->qid == SCOPE_ADMIN_QID) {
            SCOPE_PRINTF("[SCOPE PROXY][CQ][SEEN] backend=%u qid=%u tail=%u "
                   "phase=%u guest_pa=0x%016"
                   PRIx64 " result=0x%08x dw1=0x%08x sq_head=%u sq_id=%u cid=%u "
                   "status=0x%04x\n",
                   s->active->id, cq->qid, cq->shadow_tail, cq->phase, guest_pa,
                   le32_to_cpu(cqe.result), le32_to_cpu(cqe.dw1),
                   le16_to_cpu(cqe.sq_head), le16_to_cpu(cqe.sq_id),
                   le16_to_cpu(cqe.cid), le16_to_cpu(cqe.status));
            SCOPE_FFLUSH(stdout);
        } else {
            SCOPE_IO_PRINTF("[SCOPE PROXY][CQ][SEEN] qid=%u tail=%u phase=%u guest_pa=0x%016"
                   PRIx64 " result=0x%08x dw1=0x%08x sq_head=%u sq_id=%u cid=%u "
                   "status=0x%04x\n",
                   cq->qid, cq->shadow_tail, cq->phase, guest_pa,
                   le32_to_cpu(cqe.result), le32_to_cpu(cqe.dw1),
                   le16_to_cpu(cqe.sq_head), le16_to_cpu(cqe.sq_id),
                   le16_to_cpu(cqe.cid), le16_to_cpu(cqe.status));
            SCOPE_IO_FFLUSH(stdout);
        }
        scope_log_cqe_raw("SEEN", cq->qid, cq->shadow_tail, guest_pa, &cqe);
        if (!scope_publish_guest_cqe_for_cpu(s, cq, guest_pa, &cqe)) {
            break;
        }
        if (cq->qid == SCOPE_ADMIN_QID) {
            uint16_t cid = le16_to_cpu(cqe.cid);

            if (s->active->admin_cid_outstanding[cid]) {
                s->active->admin_cid_outstanding[cid] = false;
                if (s->active->admin_outstanding_count) {
                    s->active->admin_outstanding_count--;
                }
            }
            scope_commit_pending_admin_op(s, &cqe);
        }

        moved = true;
        cq->shadow_tail++;
        if (cq->shadow_tail == cq->depth) {
            cq->shadow_tail = 0;
            cq->phase = !cq->phase;
        }
    }

    if (advanced) {
        *advanced = moved;
    }
    return true;
}

static bool scope_cq_has_pending(const ScopeCqState *cq)
{
    return cq->valid && cq->interrupt_enabled && (cq->shadow_tail != cq->last_guest_head);
}

static void scope_update_virtual_intx(ScopeProxyState *s)
{
    bool pending = false;
    bool new_completion = false;
    bool old_level;
    unsigned int b, i;
    ScopeNvmeBackend *saved = s->active;

    for (b = 0; b < s->backend_count; b++) {
        bool backend_pending = false;
        s->active = &s->backends[b];
        if (NVME_CC_EN(s->active->guest_cc)) {
            for (i = 0; i < SCOPE_MAX_NVME_QUEUES; i++) {
                bool advanced = false;
                if (!scope_refresh_cq_shadow_tail(s, &s->active->cq[i], &advanced))
                    continue;
                new_completion |= advanced;
                backend_pending |= scope_cq_has_pending(&s->active->cq[i]);
            }
            if (s->active->guest_int_mask & 0x1U)
                backend_pending = false;
        }
        s->active->intx_pending = backend_pending;
        pending |= backend_pending;
    }
    s->active = saved;

    old_level = s->virtual_intx_level;
    if (pending && (!old_level || new_completion)) {
        if (scope_virtual_rp_set_intx(s, true)) {
            if (!old_level) {
                SCOPE_PRINTF("[SCOPE PROXY][INTX] assert pending completion(s) detected\n");
            } else {
                SCOPE_PRINTF("[SCOPE PROXY][INTX] refresh asserted level for new completion\n");
            }
            SCOPE_FFLUSH(stdout);
        }
    } else if (!pending && s->virtual_intx_level) {
        if (scope_virtual_rp_set_intx(s, false)) {
            SCOPE_PRINTF("[SCOPE PROXY][INTX] deassert no pending completion\n");
            SCOPE_FFLUSH(stdout);
        }
    } else if (pending && s->virtual_intx_level) {
        scope_virtual_rp_retry_intx_pulse(s);
    }
}

static const char *scope_nvme_reg_name(ScopeProxyState *s, uint32_t aligned_offset,
                                       char *buf, size_t buf_len)
{
    bool is_sq = false;
    uint16_t qid = 0;

    switch (aligned_offset) {
    case NVME_REG_CAP:
        return "CAP_LO";
    case NVME_REG_CAP + 4:
        return "CAP_HI";
    case NVME_REG_VS:
        return "VS";
    case NVME_REG_INTMS:
        return "INTMS";
    case NVME_REG_INTMC:
        return "INTMC";
    case NVME_REG_CC:
        return "CC";
    case NVME_REG_CSTS:
        return "CSTS";
    case NVME_REG_AQA:
        return "AQA";
    case NVME_REG_ASQ:
        return "ASQ_LO";
    case NVME_REG_ASQ + 4:
        return "ASQ_HI";
    case NVME_REG_ACQ:
        return "ACQ_LO";
    case NVME_REG_ACQ + 4:
        return "ACQ_HI";
    default:
        if (scope_is_doorbell_offset(s, aligned_offset, &is_sq, &qid)) {
            snprintf(buf, buf_len, "%s%u_DOORBELL",
                     is_sq ? "SQ" : "CQ", qid);
            return buf;
        }
        snprintf(buf, buf_len, "off_0x%04x", aligned_offset);
        return buf;
    }
}

static void scope_log_bar_write(ScopeProxyState *s, uint32_t seq, uint32_t offset,
                                uint32_t flags, uint8_t size_bytes, uint8_t wstrb, uint64_t data,
                                bool ok)
{
    char reg_buf[64];
    const char *reg_name = scope_nvme_reg_name(s, offset & ~0x3U, reg_buf, sizeof(reg_buf));

    if (ok) {
        SCOPE_IO_PRINTF("[SCOPE PROXY][BAR][WR] seq=%u off=0x%04x reg=%s flags=0x%08x size=%u wstrb=0x%02x data=0x%016" PRIx64 " result=OK\n",
               seq, offset, reg_name, flags, size_bytes, wstrb, data);
        SCOPE_IO_FFLUSH(stdout);
    } else {
        SCOPE_PRINTF("[SCOPE PROXY][BAR][WR] seq=%u off=0x%04x reg=%s flags=0x%08x size=%u wstrb=0x%02x data=0x%016" PRIx64 " result=ERR\n",
               seq, offset, reg_name, flags, size_bytes, wstrb, data);
        SCOPE_FFLUSH(stdout);
    }
}

static void scope_log_bar_read(ScopeProxyState *s, uint32_t seq, uint32_t offset,
                               uint32_t flags, uint8_t size_bytes, uint64_t data, uint32_t resp)
{
    char reg_buf[64];
    const char *reg_name = scope_nvme_reg_name(s, offset & ~0x3U, reg_buf, sizeof(reg_buf));

    if (resp == 0) {
        SCOPE_IO_PRINTF("[SCOPE PROXY][BAR][RD] seq=%u off=0x%04x reg=%s flags=0x%08x size=%u resp=0x%x data=0x%016" PRIx64 "\n",
               seq, offset, reg_name, flags, size_bytes, resp, data);
        SCOPE_IO_FFLUSH(stdout);
    } else {
        SCOPE_PRINTF("[SCOPE PROXY][BAR][RD] seq=%u off=0x%04x reg=%s flags=0x%08x size=%u resp=0x%x data=0x%016" PRIx64 "\n",
               seq, offset, reg_name, flags, size_bytes, resp, data);
        SCOPE_FFLUSH(stdout);
    }
}

static bool scope_is_early_sq_doorbell_write(ScopeProxyState *s, uint32_t offset,
                                             uint8_t wstrb, uint8_t size_bytes,
                                             uint16_t *qid)
{
    bool is_sq = false;
    uint16_t db_qid = 0;
    uint32_t aligned_offset = offset & ~0x3U;

    if (!scope_is_doorbell_offset(s, aligned_offset, &is_sq, &db_qid) ||
        !is_sq ||
        db_qid != SCOPE_ADMIN_QID ||
        size_bytes != 4 ||
        scope_extract_wstrb4(wstrb, offset) != 0x0FU) {
        return false;
    }

    if (qid) {
        *qid = db_qid;
    }
    return true;
}

static void scope_defer_sq_doorbell_write(ScopeProxyState *s, uint32_t seq, uint32_t offset,
                                          uint64_t data, uint8_t wstrb, uint8_t size_bytes,
                                          uint16_t qid)
{
    uint32_t dword_data = scope_extract_dword32(data, offset);
    ScopeSqState *sq = (qid < SCOPE_MAX_NVME_QUEUES) ? &s->active->sq[qid] : NULL;
    uint16_t new_tail = dword_data & 0xFFFFU;
    int64_t now_us = g_get_monotonic_time();
    uint16_t cursor;

    if (s->active->pending_sq_db.valid) {
        SCOPE_PRINTF("[SCOPE PROXY][DB][DEFER][ERR] replacing pending SQ doorbell "
               "old_seq=%u old_off=0x%04x new_seq=%u new_off=0x%04x\n",
               s->active->pending_sq_db.seq, s->active->pending_sq_db.offset, seq, offset);
        SCOPE_FFLUSH(stdout);
    }

    s->active->pending_sq_db.valid = true;
    s->active->pending_sq_db.bar_done = false;
    s->active->pending_sq_db.seq = seq;
    s->active->pending_sq_db.offset = offset;
    s->active->pending_sq_db.data = data;
    s->active->pending_sq_db.wstrb = wstrb;
    s->active->pending_sq_db.size_bytes = size_bytes;
    s->active->pending_sq_db.qid = qid;
    s->active->pending_sq_db.new_tail = new_tail;
    s->active->pending_sq_db.pending_since_us = now_us;
    s->active->pending_sq_db.fallback_next_try_us =
        now_us + SCOPE_ADMIN_SQE_DONE_FALLBACK_US;
    s->active->pending_sq_db.fallback_try_count = 0;
    memset(s->active->pending_sq_db.slot_baseline_valid, 0,
           sizeof(s->active->pending_sq_db.slot_baseline_valid));
    memset(s->active->pending_sq_db.slot_baseline_seq, 0,
           sizeof(s->active->pending_sq_db.slot_baseline_seq));

    if (s->sqe_monitor_enable && sq && sq->valid && sq->depth &&
        new_tail < sq->depth) {
        cursor = sq->last_guest_tail;
        while (cursor != new_tail) {
            if (cursor < SCOPE_ADMIN_SQ_MAX_TRACKED) {
                s->active->pending_sq_db.slot_baseline_valid[cursor] =
                    s->active->admin_sq_slot_done_valid[cursor];
                s->active->pending_sq_db.slot_baseline_seq[cursor] =
                    s->active->admin_sq_slot_done_seq[cursor];
            }
            cursor = (cursor + 1U) % sq->depth;
        }
    }

    SCOPE_PRINTF("[SCOPE PROXY][DB][DEFER] qid=%u seq=%u off=0x%04x size=%u "
           "wstrb=0x%02x data=0x%016" PRIx64 " new_tail=%u\n",
           qid, seq, offset, size_bytes, wstrb, data,
           s->active->pending_sq_db.new_tail);
    SCOPE_FFLUSH(stdout);
}

static bool scope_admin_sq_slots_ready(ScopeProxyState *s, const ScopeSqState *sq,
                                       const ScopePendingDoorbell *pending)
{
    uint16_t new_tail = pending->new_tail;
    uint16_t cursor;

    if (!sq->valid || !sq->depth || new_tail >= sq->depth) {
        return false;
    }

    cursor = sq->last_guest_tail;
    while (cursor != new_tail) {
        uint32_t done_seq = 0;
        uint32_t consumed_seq = 0;
        uint32_t baseline_seq = 0;
        uint64_t done_mask = 0;
        bool done_valid = false;
        bool baseline_valid = false;

        if (cursor < SCOPE_ADMIN_SQ_MAX_TRACKED) {
            done_valid = s->active->admin_sq_slot_done_valid[cursor];
            done_seq = s->active->admin_sq_slot_done_seq[cursor];
            done_mask = s->active->admin_sq_slot_done_mask[cursor];
            consumed_seq = s->active->admin_sq_slot_consumed_seq[cursor];
            baseline_valid = pending->slot_baseline_valid[cursor];
            baseline_seq = pending->slot_baseline_seq[cursor];
        }

        /*
         * SQE_WRITE_DONE may legitimately arrive before the SQ doorbell packet.
         * In that case the pending doorbell snapshots that done_seq as the
         * baseline; accepting done_seq == baseline_seq is correct as long as it
         * has not already been consumed by a previous doorbell.  Requiring a
         * strictly newer seq here caused intermittent hangs when slot writes
         * completed before their doorbells reached QEMU.
         */
        if (cursor >= SCOPE_ADMIN_SQ_MAX_TRACKED ||
            !done_valid ||
            done_mask != SCOPE_ADMIN_SQE_DONE_FULL_MASK ||
            done_seq <= consumed_seq ||
            (baseline_valid && done_seq < baseline_seq)) {
            SCOPE_PRINTF("[SCOPE PROXY][DB][WAIT_SQE] qid=%u slot=%u new_tail=%u "
                   "done_valid=%d done_seq=%u consumed_seq=%u "
                   "baseline_valid=%d baseline_seq=%u done_mask=0x%016" PRIx64 "\n",
                   sq->qid, cursor, new_tail, done_valid, done_seq,
                   consumed_seq, baseline_valid, baseline_seq, done_mask);
            SCOPE_FFLUSH(stdout);
            return false;
        }
        cursor = (cursor + 1U) % sq->depth;
    }

    return true;
}

static void scope_mark_admin_sq_slots_consumed(ScopeProxyState *s, uint16_t old_tail,
                                               uint16_t new_tail, uint16_t depth,
                                               bool fallback_without_done)
{
    uint16_t cursor = old_tail;

    while (cursor != new_tail) {
        if (cursor < SCOPE_ADMIN_SQ_MAX_TRACKED) {
            if (s->active->admin_sq_slot_done_valid[cursor]) {
                s->active->admin_sq_slot_consumed_seq[cursor] =
                    s->active->admin_sq_slot_done_seq[cursor];
                s->active->admin_sq_slot_fallback_wait_done[cursor] = false;
                s->active->admin_sq_slot_done_valid[cursor] = false;
                s->active->admin_sq_slot_done_mask[cursor] = 0;
            } else if (fallback_without_done) {
                s->active->admin_sq_slot_fallback_wait_done[cursor] = true;
                s->active->admin_sq_slot_done_mask[cursor] = 0;
            }
        }
        cursor = (cursor + 1U) % depth;
    }
}

static void scope_try_process_pending_sq_doorbell(ScopeProxyState *s)
{
    ScopePendingDoorbell pending;
    ScopeSqState *sq;
    uint16_t old_tail;
    uint32_t flags;
    bool fallback_without_done = false;
    bool coherent_without_monitor;
    bool slots_ready;
    bool ok;
    int64_t now_us;

    if (!s->active->pending_sq_db.valid) {
        return;
    }
    if (!s->active->pending_sq_db.bar_done) {
        SCOPE_PRINTF("[SCOPE PROXY][DB][WAIT_BAR_DONE] qid=%u seq=%u off=0x%04x\n",
               s->active->pending_sq_db.qid, s->active->pending_sq_db.seq, s->active->pending_sq_db.offset);
        SCOPE_FFLUSH(stdout);
        return;
    }
    if (s->active->pending_sq_db.qid != SCOPE_ADMIN_QID) {
        SCOPE_PRINTF("[SCOPE PROXY][DB][ERR] unexpected deferred non-admin SQ doorbell qid=%u\n",
               s->active->pending_sq_db.qid);
        SCOPE_FFLUSH(stdout);
        return;
    }

    now_us = g_get_monotonic_time();
    sq = &s->active->sq[s->active->pending_sq_db.qid];
    coherent_without_monitor = !s->sqe_monitor_enable;
    slots_ready = coherent_without_monitor ||
                  scope_admin_sq_slots_ready(s, sq,
                                             &s->active->pending_sq_db);
    if (!slots_ready) {
#if SCOPE_DEBUG_ADMIN_SQE_DONE_TIMEOUT_FALLBACK
        if (now_us < s->active->pending_sq_db.fallback_next_try_us) {
            return;
        }
        s->active->pending_sq_db.fallback_try_count++;
        s->active->pending_sq_db.fallback_next_try_us =
            now_us + SCOPE_ADMIN_SQE_DONE_FALLBACK_RETRY_US;
        fallback_without_done = true;

        SCOPE_PRINTF("[SCOPE PROXY][DB][FALLBACK_TRY] qid=%u seq=%u off=0x%04x "
               "new_tail=%u try=%" PRIu64 " elapsed_us=%" PRId64
               " wait_done_timeout_us=%u retry_us=%u\n",
               s->active->pending_sq_db.qid, s->active->pending_sq_db.seq,
               s->active->pending_sq_db.offset, s->active->pending_sq_db.new_tail,
               s->active->pending_sq_db.fallback_try_count,
               now_us - s->active->pending_sq_db.pending_since_us,
               SCOPE_ADMIN_SQE_DONE_FALLBACK_US,
               SCOPE_ADMIN_SQE_DONE_FALLBACK_RETRY_US);
        SCOPE_FFLUSH(stdout);
#else
        return;
#endif
    }

    pending = s->active->pending_sq_db;
    old_tail = sq->last_guest_tail;

    SCOPE_PRINTF("[SCOPE PROXY][DB][%s] qid=%u seq=%u off=0x%04x new_tail=%u\n",
           coherent_without_monitor ? "COHERENT_READY" :
           fallback_without_done ? "FALLBACK_READY" : "READY",
           pending.qid, pending.seq, pending.offset, pending.new_tail);
    SCOPE_FFLUSH(stdout);

    ok = false;
    if (pending.qid == SCOPE_ADMIN_QID) {
        ScopeSqeReadStatus process_status =
            scope_process_new_sq_entries(s, sq, pending.new_tail,
                                          false);

        if (process_status == SCOPE_SQE_READ_WAIT) {
            s->active->pending_sq_db.fallback_next_try_us =
                now_us + SCOPE_ADMIN_SQE_VISIBILITY_RETRY_US;
            SCOPE_PRINTF("[SCOPE PROXY][DB][%s] qid=%u seq=%u "
                   "off=0x%04x new_tail=%u retry_us=%u\n",
                   coherent_without_monitor ? "COHERENT_WAIT_VISIBLE" :
                   fallback_without_done ? "FALLBACK_WAIT_VISIBLE" :
                                           "WAIT_SQE_VISIBLE",
                   pending.qid, pending.seq, pending.offset, pending.new_tail,
                   SCOPE_ADMIN_SQE_VISIBILITY_RETRY_US);
            SCOPE_FFLUSH(stdout);
            return;
        }
        if (process_status != SCOPE_SQE_READ_OK) {
            SCOPE_PRINTF("[SCOPE PROXY][DB][SQ][%s] qid=%u last_tail=%u new_tail=%u "
                   "depth=%u guest_base=0x%016" PRIx64 " status=%d\n",
                   coherent_without_monitor ? "COHERENT_NOT_READY" :
                   fallback_without_done ? "FALLBACK_NOT_READY" : "ERR",
                   pending.qid, sq->last_guest_tail, pending.new_tail,
                   sq->depth, sq->guest_base, process_status);
            SCOPE_FFLUSH(stdout);
            if (coherent_without_monitor || fallback_without_done) {
                s->active->pending_sq_db.fallback_next_try_us =
                    now_us + (coherent_without_monitor ?
                              SCOPE_ADMIN_SQE_VISIBILITY_RETRY_US :
                              SCOPE_ADMIN_SQE_DONE_FALLBACK_RETRY_US);
                return;
            }
        } else {
            uint32_t aligned_offset = pending.offset & ~0x3U;

            ok = scope_real_bar_write(s, aligned_offset, pending.data,
                                      pending.wstrb, pending.size_bytes);
            if (!ok) {
                SCOPE_PRINTF("[SCOPE PROXY][DB][ERR] failed to forward deferred doorbell "
                       "off=0x%04x aligned=0x%04x data=0x%016" PRIx64
                       " wstrb=0x%02x\n",
                       pending.offset, aligned_offset, pending.data, pending.wstrb);
                SCOPE_FFLUSH(stdout);
            }
        }
    }
    flags = ((uint32_t)pending.size_bytes << 8) | pending.wstrb;
    scope_log_bar_write(s, pending.seq, pending.offset, flags, pending.size_bytes,
                        pending.wstrb, pending.data, ok);
    if (ok) {
        scope_mark_admin_sq_slots_consumed(s, old_tail, pending.new_tail,
                                           sq->depth, fallback_without_done);
        if (fallback_without_done) {
            SCOPE_PRINTF("[SCOPE PROXY][DB][FALLBACK_FORWARDED] qid=%u seq=%u "
                   "off=0x%04x new_tail=%u try=%" PRIu64 "\n",
                   pending.qid, pending.seq, pending.offset, pending.new_tail,
                   pending.fallback_try_count);
            SCOPE_FFLUSH(stdout);
        } else if (coherent_without_monitor) {
            SCOPE_PRINTF("[SCOPE PROXY][DB][COHERENT_FORWARDED] qid=%u seq=%u "
                   "off=0x%04x new_tail=%u\n",
                   pending.qid, pending.seq, pending.offset, pending.new_tail);
            SCOPE_FFLUSH(stdout);
        }
        s->active->pending_sq_db.valid = false;
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE: Deferred SQ doorbell processing failed after SQE done\n");
    }
}

static void scope_process_deferred_sq_doorbell(ScopeProxyState *s,
                                               const struct scope_dma32_packet *pkt)
{
    if (s->active->inferred_bar_done_valid &&
        s->active->inferred_bar_done_seq == pkt->seq &&
        s->active->inferred_bar_done_offset == pkt->bar_offset) {
        SCOPE_PRINTF("[SCOPE PROXY][DB][BAR_DONE][LATE_IGNORED] backend=%u "
                     "seq=%u off=0x%04x\n",
                     s->active->id, pkt->seq, pkt->bar_offset);
        SCOPE_FFLUSH(stdout);
        s->active->inferred_bar_done_valid = false;
        return;
    }

    if (!s->active->pending_sq_db.valid) {
        SCOPE_PRINTF("[SCOPE PROXY][DB][BAR_DONE][ERR] seq=%u off=0x%04x without pending doorbell\n",
               pkt->seq, pkt->bar_offset);
        SCOPE_FFLUSH(stdout);
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE: BAR write done packet without pending SQ doorbell\n");
        return;
    }
    if (s->active->pending_sq_db.seq != pkt->seq) {
        SCOPE_PRINTF("[SCOPE PROXY][DB][BAR_DONE][ERR] seq mismatch pending_seq=%u done_seq=%u "
               "pending_off=0x%04x done_off=0x%04x\n",
               s->active->pending_sq_db.seq, pkt->seq, s->active->pending_sq_db.offset, pkt->bar_offset);
        SCOPE_FFLUSH(stdout);
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE: BAR write done packet sequence mismatch\n");
        return;
    }

    s->active->pending_sq_db.bar_done = true;
    SCOPE_PRINTF("[SCOPE PROXY][DB][BAR_DONE] seq=%u off=0x%04x size=%u wstrb=0x%02x\n",
           s->active->pending_sq_db.seq, s->active->pending_sq_db.offset,
           s->active->pending_sq_db.size_bytes, s->active->pending_sq_db.wstrb);
    SCOPE_FFLUSH(stdout);
    scope_try_process_pending_sq_doorbell(s);
}

static bool scope_handle_doorbell_write(ScopeProxyState *s, uint32_t offset, uint64_t data,
                                        uint8_t wstrb, uint8_t size_bytes)
{
    uint32_t aligned_offset = offset & ~0x3U;
    uint32_t dword_data = scope_extract_dword32(data, offset);
    uint8_t dword_wstrb = scope_extract_wstrb4(wstrb, offset);
    bool is_sq = false;
    uint16_t qid = 0;

    if (size_bytes != 4 || dword_wstrb != 0x0FU) {
        SCOPE_PRINTF("[SCOPE PROXY][DB][ERR] invalid doorbell write off=0x%04x size=%u "
               "wstrb=0x%02x data=0x%016" PRIx64 "\n",
               offset, size_bytes, dword_wstrb, data);
        SCOPE_FFLUSH(stdout);
        return false;
    }
    if (!scope_is_doorbell_offset(s, aligned_offset, &is_sq, &qid) ||
        qid >= SCOPE_MAX_NVME_QUEUES) {
        SCOPE_PRINTF("[SCOPE PROXY][DB][ERR] unknown doorbell off=0x%04x aligned=0x%04x "
               "data=0x%016" PRIx64 "\n",
               offset, aligned_offset, data);
        SCOPE_FFLUSH(stdout);
        return false;
    }

    if (is_sq) {
        ScopeSqState *sq = &s->active->sq[qid];
        uint16_t new_tail = dword_data & 0xFFFFU;

        ScopeSqeReadStatus process_status =
            scope_process_new_sq_entries(s, sq, new_tail, false);

        if (process_status != SCOPE_SQE_READ_OK) {
            SCOPE_PRINTF("[SCOPE PROXY][DB][SQ][ERR] qid=%u last_tail=%u new_tail=%u "
                   "depth=%u guest_base=0x%016" PRIx64 " status=%d\n",
                   qid, sq->last_guest_tail, new_tail, sq->depth, sq->guest_base,
                   process_status);
            SCOPE_FFLUSH(stdout);
            return false;
        }
    } else {
        ScopeCqState *cq = &s->active->cq[qid];

        if (cq->valid) {
            cq->last_guest_head = dword_data & 0xFFFFU;
        }
        scope_update_virtual_intx(s);
    }

    if (!scope_real_bar_write(s, aligned_offset, data, wstrb, size_bytes)) {
        SCOPE_PRINTF("[SCOPE PROXY][DB][ERR] failed to forward real doorbell off=0x%04x "
               "aligned=0x%04x data=0x%016" PRIx64 " wstrb=0x%02x\n",
               offset, aligned_offset, data, wstrb);
        SCOPE_FFLUSH(stdout);
        return false;
    }

    return true;
}

static bool scope_handle_nvme_bar_write(ScopeProxyState *s, uint32_t offset, uint64_t data,
                                        uint8_t wstrb, uint8_t size_bytes)
{
    uint32_t aligned_offset = offset & ~0x3U;
    uint32_t qword_offset = offset & ~0x7U;
    uint32_t dword_data = scope_extract_dword32(data, offset);
    uint8_t dword_wstrb = scope_extract_wstrb4(wstrb, offset);
    bool is_sq = false;
    uint16_t qid = 0;

    if (!size_bytes || size_bytes > 8 || (((offset & 0x7U) + size_bytes) > 8U)) {
        return false;
    }

    if (scope_is_doorbell_offset(s, aligned_offset, &is_sq, &qid)) {
        return scope_handle_doorbell_write(s, offset, data, wstrb, size_bytes);
    }

    if (qword_offset == NVME_REG_ASQ) {
        if (size_bytes == 8 && (offset & 0x7U) == 0) {
            s->active->guest_asq = scope_apply_wstrb64(s->active->guest_asq, data, wstrb);
        } else {
            scope_update_shadow_u64(&s->active->guest_asq, aligned_offset == (NVME_REG_ASQ + 4),
                                    dword_data, wstrb);
        }
        return scope_sync_admin_regs_to_real(s);
    }

    if (qword_offset == NVME_REG_ACQ) {
        if (size_bytes == 8 && (offset & 0x7U) == 0) {
            s->active->guest_acq = scope_apply_wstrb64(s->active->guest_acq, data, wstrb);
        } else {
            scope_update_shadow_u64(&s->active->guest_acq, aligned_offset == (NVME_REG_ACQ + 4),
                                    dword_data, wstrb);
        }
        return scope_sync_admin_regs_to_real(s);
    }

    switch (aligned_offset) {
    case NVME_REG_CC: {
        uint32_t old_cc = s->active->guest_cc;

        s->active->guest_cc = scope_apply_wstrb32(s->active->guest_cc, dword_data, dword_wstrb);
        scope_update_ctrl_page_size(s);
        if (!scope_sync_admin_regs_to_real(s)) {
            return false;
        }
        if (NVME_CC_EN(s->active->guest_cc)) {
            if (!scope_real_bar_write32(s, NVME_REG_CC, s->active->guest_cc)) {
                return false;
            }
        } else if (!scope_real_nvme_disable(s, "guest CC.EN=0", NULL)) {
            return false;
        }

        if (!NVME_CC_EN(s->active->guest_cc)) {
            scope_reset_all_queue_state(s);
            if (!scope_sync_admin_window_to_fpga(s, false)) {
                return false;
            }
            scope_update_virtual_intx(s);
        } else if (!NVME_CC_EN(old_cc)) {
            scope_reset_all_queue_state(s);
            if (!scope_refresh_admin_queue_state(s)) {
                return false;
            }
        }
        return true;
    }
    case NVME_REG_AQA:
        s->active->guest_aqa = scope_apply_wstrb32(s->active->guest_aqa, dword_data, dword_wstrb);
        return scope_sync_admin_regs_to_real(s);

    case NVME_REG_INTMS:
        s->active->guest_int_mask |= scope_apply_wstrb32(0, dword_data, dword_wstrb);
        scope_update_virtual_intx(s);
        return scope_real_bar_write(s, aligned_offset, data, wstrb, size_bytes);
    case NVME_REG_INTMC:
        s->active->guest_int_mask &= ~scope_apply_wstrb32(0, dword_data, dword_wstrb);
        scope_update_virtual_intx(s);
        return scope_real_bar_write(s, aligned_offset, data, wstrb, size_bytes);
    default:
        return scope_real_bar_write(s, aligned_offset, data, wstrb, size_bytes);
    }
}

static bool scope_handle_nvme_bar_read(ScopeProxyState *s, uint32_t offset, uint8_t size_bytes,
                                       uint64_t *data)
{
    uint32_t aligned_offset = offset & ~0x3U;
    uint32_t qword_offset = offset & ~0x7U;

    if (!data || !size_bytes || size_bytes > 8 || (((offset & 0x7U) + size_bytes) > 8U)) {
        return false;
    }

    if (qword_offset == NVME_REG_CAP) {
        *data = s->active->nvme_cap;
        return true;
    }
    if (qword_offset == NVME_REG_ASQ) {
        *data = s->active->guest_asq;
        return true;
    }
    if (qword_offset == NVME_REG_ACQ) {
        *data = s->active->guest_acq;
        return true;
    }

    switch (aligned_offset) {
    case NVME_REG_VS:
        *data = scope_pack_dword32_for_offset(s->active->nvme_vs, offset);
        return true;
    case NVME_REG_CC:
        *data = scope_pack_dword32_for_offset(s->active->guest_cc, offset);
        return true;
    case NVME_REG_AQA:
        *data = scope_pack_dword32_for_offset(s->active->guest_aqa, offset);
        return true;
    case NVME_REG_INTMS:
    case NVME_REG_INTMC:
        *data = scope_pack_dword32_for_offset(s->active->guest_int_mask, offset);
        return true;
    case NVME_REG_CSTS:
        return scope_real_bar_read(s, aligned_offset, size_bytes, data);
    default:
        return scope_real_bar_read(s, aligned_offset, size_bytes, data);
    }
}

static bool scope_read_stable_packet(const void *slot_base,
                                     struct scope_dma32_packet *pkt)
{
    struct scope_dma32_packet a;
    struct scope_dma32_packet b;

    memcpy(&a, slot_base, sizeof(a));
    smp_rmb();
    memcpy(&b, slot_base, sizeof(b));

    if (memcmp(&a, &b, sizeof(a)) != 0) {
        return false;
    }

    *pkt = a;
    return true;
}

static bool scope_find_bar_done_in_ring(ScopeProxyState *s,
                                        const ScopeNvmeBackend *backend,
                                        const ScopePendingDoorbell *pending,
                                        size_t *slot_out)
{
    const size_t slot_size = sizeof(struct scope_dma32_packet);
    const size_t slot_count = s->dma32_db.size / slot_size;
    const uint8_t *ring_base = s->dma32_db_map;
    size_t i;

    if (!ring_base || !slot_count) {
        return false;
    }

    for (i = 0; i < slot_count; i++) {
        struct scope_dma32_packet pkt;

        if (!scope_read_stable_packet(ring_base + i * slot_size, &pkt)) {
            continue;
        }
        if (pkt.magic != XDMA_DMA32_PKT_MAGIC ||
            pkt.len != sizeof(pkt) ||
            pkt.type != SCOPE_PKT_TYPE_BAR_WRITE_DONE ||
            pkt.seq != pending->seq ||
            pkt.bar_offset != pending->offset ||
            SCOPE_VSWITCH_PKT_BDF(pkt.flags) != backend->virtual_bdf) {
            continue;
        }

        if (slot_out) {
            *slot_out = i;
        }
        return true;
    }

    return false;
}

static bool scope_recover_missing_bar_done(ScopeProxyState *s, int64_t now_us)
{
    ScopeNvmeBackend *backend = s->active;
    ScopePendingDoorbell *pending = &backend->pending_sq_db;
    int64_t elapsed_us;
    size_t ring_slot = 0;
    bool found;

    if (!s->bar_done_timeout_us || !pending->valid || pending->bar_done) {
        return false;
    }

    elapsed_us = now_us - pending->pending_since_us;
    if (elapsed_us < s->bar_done_timeout_us) {
        return false;
    }

    found = scope_find_bar_done_in_ring(s, backend, pending, &ring_slot);
    SCOPE_PRINTF("[SCOPE PROXY][DB][BAR_DONE_TIMEOUT] backend=%u virtual=%02x:00.0 "
                 "seq=%u off=0x%04x elapsed_us=%" PRId64
                 " timeout_us=%u\n",
                 backend->id, 3U + backend->id, pending->seq,
                 pending->offset, elapsed_us, s->bar_done_timeout_us);
    SCOPE_PRINTF("[SCOPE PROXY][DMA32][SEARCH] type=%u backend=%u seq=%u "
                 "off=0x%04x found=%d",
                 SCOPE_PKT_TYPE_BAR_WRITE_DONE, backend->id, pending->seq,
                 pending->offset, found);
    if (found) {
        SCOPE_PRINTF_CONT(" slot=%zu", ring_slot);
    }
    SCOPE_PRINTF_CONT("\n");
    SCOPE_FFLUSH(stdout);

    /*
     * QEMU already returned the early BAR response before this timer starts.
     * At this point the coherent alias is the authority for SQE visibility;
     * infer BAR completion so a lost one-shot DONE notification cannot stall
     * an entire backend.  The normal SQE stable-read checks still gate the
     * real NVMe doorbell write.
     */
    pending->bar_done = true;
    backend->inferred_bar_done_valid = true;
    backend->inferred_bar_done_seq = pending->seq;
    backend->inferred_bar_done_offset = pending->offset;
    backend->inferred_bar_done_count++;

    SCOPE_PRINTF("[SCOPE PROXY][DB][BAR_DONE_INFERRED] backend=%u seq=%u "
                 "off=0x%04x count=%" PRIu64 " ring_found=%d\n",
                 backend->id, pending->seq, pending->offset,
                 backend->inferred_bar_done_count, found);
    SCOPE_FFLUSH(stdout);
    scope_try_process_pending_sq_doorbell(s);
    return true;
}

static void scope_process_sqe_write_done_packet(ScopeProxyState *s,
                                                const struct scope_dma32_packet *pkt)
{
    uint16_t slot = SCOPE_VSWITCH_SQE_SLOT(pkt->flags);
    uint16_t qid = SCOPE_VSWITCH_SQE_QID(pkt->flags);
    uint8_t bresp = SCOPE_VSWITCH_SQE_BRESP(pkt->flags);
    uint8_t backend_id = SCOPE_VSWITCH_SQE_BACKEND(pkt->flags);
    bool overflow = SCOPE_VSWITCH_SQE_OVERFLOW(pkt->flags);
    uint32_t bytes = pkt->data;
    uint64_t guest_pa = ((uint64_t)pkt->guest_addr_lo << 32) | pkt->bar_offset;
    uint64_t expected_pa = 0;
    uint64_t done_mask = 0;
    uint64_t new_mask = 0;

    if (!s->sqe_monitor_enable) {
        return;
    }

    if (backend_id >= s->backend_count) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE: SQE DONE references inactive backend %u\n", backend_id);
        return;
    }
    s->active = &s->backends[backend_id];

    if (qid == SCOPE_ADMIN_QID) {
        SCOPE_PRINTF("[SCOPE PROXY][SQE][DONE] backend=%u virtual=%02x:00.0 "
               "qid=%u slot=%u seq=%u guest_pa=0x%016"
               PRIx64 " bytes=%u bresp=0x%x overflow=%d\n",
               backend_id, 3U + backend_id, qid, slot, pkt->seq, guest_pa,
               bytes, bresp, overflow);
        SCOPE_FFLUSH(stdout);
    } else {
        SCOPE_IO_PRINTF("[SCOPE PROXY][SQE][DONE] qid=%u slot=%u seq=%u guest_pa=0x%016"
               PRIx64 " bytes=%u bresp=0x%x overflow=%d\n",
               qid, slot, pkt->seq, guest_pa, bytes, bresp, overflow);
        SCOPE_IO_FFLUSH(stdout);
    }

    if (overflow) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE: SQE write-done monitor overflow was observed\n");
    }
    if (bresp != 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE: SQE write completed with AXI bresp=0x%x\n", bresp);
        return;
    }
    if (qid != SCOPE_ADMIN_QID || slot >= SCOPE_ADMIN_SQ_MAX_TRACKED) {
        return;
    }

    if (s->active->sq[SCOPE_ADMIN_QID].valid) {
        expected_pa = s->active->sq[SCOPE_ADMIN_QID].guest_base +
                      (uint64_t)slot * sizeof(NvmeCmd);
        if (guest_pa != expected_pa) {
            SCOPE_PRINTF("[SCOPE PROXY][SQE][DONE][WARN] qid=%u slot=%u guest_pa=0x%016"
                   PRIx64 " expected=0x%016" PRIx64 "\n",
                   qid, slot, guest_pa, expected_pa);
            SCOPE_FFLUSH(stdout);
        }
    }

    done_mask = scope_admin_sqe_done_mask_for_packet(guest_pa, bytes, expected_pa);
    new_mask = s->active->admin_sq_slot_done_mask[slot] | done_mask;
    s->active->admin_sq_slot_done_mask[slot] = new_mask;

    SCOPE_PRINTF("[SCOPE PROXY][SQE][DONE][COVER] backend=%u qid=%u slot=%u seq=%u "
           "packet_mask=0x%016" PRIx64 " slot_mask=0x%016" PRIx64 "\n",
           backend_id, qid, slot, pkt->seq, done_mask, new_mask);
    SCOPE_FFLUSH(stdout);

    if (new_mask != SCOPE_ADMIN_SQE_DONE_FULL_MASK) {
        SCOPE_PRINTF("[SCOPE PROXY][SQE][DONE][PARTIAL] backend=%u qid=%u "
               "slot=%u seq=%u "
               "bytes=%u guest_pa=0x%016" PRIx64 " expected=0x%016" PRIx64 "\n",
               backend_id, qid, slot, pkt->seq, bytes, guest_pa, expected_pa);
        SCOPE_FFLUSH(stdout);
        scope_try_process_pending_sq_doorbell(s);
        return;
    }

    s->active->admin_sq_slot_done_valid[slot] = true;
    s->active->admin_sq_slot_done_seq[slot] = pkt->seq;
    if (s->active->admin_sq_slot_fallback_wait_done[slot]) {
        s->active->admin_sq_slot_consumed_seq[slot] = pkt->seq;
        s->active->admin_sq_slot_fallback_wait_done[slot] = false;
        s->active->admin_sq_slot_done_valid[slot] = false;
        s->active->admin_sq_slot_done_mask[slot] = 0;
        SCOPE_PRINTF("[SCOPE PROXY][SQE][DONE][FALLBACK_CONSUMED] qid=%u slot=%u "
               "seq=%u\n",
               qid, slot, pkt->seq);
        SCOPE_FFLUSH(stdout);
    }
    scope_try_process_pending_sq_doorbell(s);
}

static void scope_process_bar_packet(ScopeProxyState *s, const struct scope_dma32_packet *pkt)
{
    uint32_t resp = 0x2U;
    uint64_t data = 0;
    bool ok = false;
    /*
     * vSwitch RTL packs BAR packet metadata as:
     *   flags[15:0]  = requester BDF
     *   flags[18:16] = BAR index
     *   flags[23:20] = size bytes
     *   flags[31:24] = write strobe
     * BAR packets reuse guest_addr_lo as the upper 32 bits of the 64-bit lane.
     */
    uint16_t bdf = SCOPE_VSWITCH_PKT_BDF(pkt->flags);
    uint8_t bar = SCOPE_VSWITCH_PKT_BAR(pkt->flags);
    uint8_t size_bytes = SCOPE_VSWITCH_PKT_SIZE(pkt->flags);
    uint8_t wstrb = SCOPE_VSWITCH_PKT_WSTRB(pkt->flags);
    uint64_t lane_data = ((uint64_t)pkt->guest_addr_lo << 32) | pkt->data;
    int backend_id = scope_backend_index_from_bdf(s, bdf);

    if (backend_id < 0 || bar != 0) {
        if (pkt->type == SCOPE_PKT_TYPE_BAR_READ)
            scope_write_bar_response(s, pkt->seq, resp, UINT64_MAX, true, false);
        else
            scope_write_bar_response(s, pkt->seq, resp, 0, false, false);
        return;
    }
    s->active = &s->backends[backend_id];

    switch (pkt->type) {
    case SCOPE_PKT_TYPE_BAR_WRITE: {
        uint16_t qid = 0;
        bool early_resp = scope_is_early_sq_doorbell_write(s, pkt->bar_offset,
                                                           wstrb, size_bytes, &qid);

        if (bar != 0) {
            SCOPE_PRINTF("[SCOPE VSWITCH][BAR][WR][DROP] seq=%u bdf=%02x:%02x.%u "
                         "bar=%u off=0x%04x size=%u wstrb=0x%02x\n",
                         pkt->seq, bdf >> 8, (bdf >> 3) & 0x1f, bdf & 0x7,
                         bar, pkt->bar_offset, size_bytes, wstrb);
            SCOPE_FFLUSH(stdout);
            if (!scope_write_bar_response(s, pkt->seq, resp, 0, false, false)) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "SCOPE: Failed to write dropped BAR write response\n");
            }
            break;
        }

        if (early_resp &&
            !scope_write_bar_response(s, pkt->seq, 0, 0, false, true)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "SCOPE: Failed to write early BAR doorbell response\n");
        }

        if (early_resp) {
            scope_defer_sq_doorbell_write(s, pkt->seq, pkt->bar_offset, lane_data,
                                          wstrb, size_bytes, qid);
            scope_log_bar_write(s, pkt->seq, pkt->bar_offset, pkt->flags, size_bytes,
                                wstrb, lane_data, true);
            break;
        }

        ok = scope_handle_nvme_bar_write(s, pkt->bar_offset, lane_data, wstrb, size_bytes);
        resp = ok ? 0x0U : 0x2U;
        scope_log_bar_write(s, pkt->seq, pkt->bar_offset, pkt->flags, size_bytes, wstrb,
                            lane_data, ok);
        if (!scope_write_bar_response(s, pkt->seq, resp, 0, false, false)) {
            qemu_log_mask(LOG_GUEST_ERROR, "SCOPE: Failed to write BAR write response\n");
        }
        break;
    }
    case SCOPE_PKT_TYPE_BAR_WRITE_DONE:
        scope_process_deferred_sq_doorbell(s, pkt);
        break;
    case SCOPE_PKT_TYPE_BAR_READ:
        if (bar != 0) {
            SCOPE_PRINTF("[SCOPE VSWITCH][BAR][RD][DROP] seq=%u bdf=%02x:%02x.%u "
                         "bar=%u off=0x%04x size=%u\n",
                         pkt->seq, bdf >> 8, (bdf >> 3) & 0x1f, bdf & 0x7,
                         bar, pkt->bar_offset, size_bytes);
            SCOPE_FFLUSH(stdout);
            if (!scope_write_bar_response(s, pkt->seq, resp, UINT64_MAX, true, false)) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "SCOPE: Failed to write dropped BAR read response\n");
            }
            break;
        }
        ok = scope_handle_nvme_bar_read(s, pkt->bar_offset, size_bytes, &data);
        resp = ok ? 0x0U : 0x2U;
        scope_log_bar_read(s, pkt->seq, pkt->bar_offset, pkt->flags, size_bytes, data, resp);
        if (!scope_write_bar_response(s, pkt->seq, resp, data, true, false)) {
            qemu_log_mask(LOG_GUEST_ERROR, "SCOPE: Failed to write BAR read response\n");
        }
        break;
    default:
        break;
    }
}

static void scope_process_cfg_packet(ScopeProxyState *s, const struct scope_dma32_packet *pkt)
{
    PCIDevice *pci_dev = PCI_DEVICE(s);
    uint16_t bdf = SCOPE_VSWITCH_PKT_BDF(pkt->flags);
    uint32_t wstrb = SCOPE_VSWITCH_PKT_WSTRB(pkt->flags) & 0x0FU;
    uint32_t addr = pkt->bar_offset;
    uint32_t mask = wstrb;
    uint32_t val;
    uint32_t actual_addr;
    uint32_t config_limit = pci_config_size(pci_dev);
    ScopeVswitchConfigFn *fn = scope_vcfg_by_bdf(s, bdf);
    int shadow_index = scope_vcfg_index_from_bdf(bdf);
    int offset = 0;
    int len = 0;
    Error *local_err = NULL;

    while ((mask & 1U) == 0U && offset < 4) {
        mask >>= 1;
        offset++;
    }
    while (((mask >> len) & 1U) && len < 4) {
        len++;
    }

    if (!len) {
        if (!scope_ack_cfg_packet(s, pkt->seq)) {
            qemu_log_mask(LOG_GUEST_ERROR, "SCOPE: Failed to ACK zero-length CFG packet\n");
        }
        return;
    }

    val = (pkt->data >> (offset * 8)) &
          (0xffffffffU >> ((4 - len) * 8));
    actual_addr = addr + (uint32_t)offset;

    SCOPE_PRINTF("[SCOPE VSWITCH][CFG][WR] seq=%u bdf=%02x:%02x.%u "
                 "off=0x%03x len=%d wstrb=0x%x val=0x%08x\n",
                 pkt->seq, bdf >> 8, (bdf >> 3) & 0x1f, bdf & 0x7,
                 actual_addr, len, wstrb, val);
    SCOPE_FFLUSH(stdout);

    if (!fn) {
        if (!scope_ack_cfg_packet(s, pkt->seq)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "SCOPE: Failed to ACK unsupported CFG packet seq=%u\n",
                          pkt->seq);
        }
        return;
    }

    scope_log_config_write("C2H", pkt->seq, actual_addr, len, wstrb, val);

    {
        int backend_id = scope_backend_index_from_bdf(s, bdf);
        if (backend_id >= 0) {
        bool sync_ok = true;
        ScopeNvmeBackend *be = &s->backends[backend_id];

        if (actual_addr + (uint32_t)len <= config_limit) {
            scope_vcfg_write_masked(fn, actual_addr, val, len);

            sync_ok = scope_sync_ecam_shadow_range(s, shadow_index,
                                                   actual_addr, len,
                                                   &local_err) &&
                      scope_sync_guest_bar_shadow(s, fn, be, &local_err) &&
                      scope_sync_ecam_shadow_fence(s, shadow_index, actual_addr);

            if (!sync_ok && local_err) {
                qemu_log_mask(LOG_GUEST_ERROR, "SCOPE: %s\n",
                              error_get_pretty(local_err));
                error_free(local_err);
                local_err = NULL;
            }
        }
        } else if (actual_addr + (uint32_t)len <= sizeof(fn->config)) {
        bool sync_ok;

        scope_vcfg_write_masked(fn, actual_addr, val, len);
        sync_ok = scope_sync_ecam_shadow_range(s, shadow_index, actual_addr, len,
                                               &local_err) &&
                  scope_sync_ecam_shadow_fence(s, shadow_index, actual_addr);
        if (!sync_ok && local_err) {
            qemu_log_mask(LOG_GUEST_ERROR, "SCOPE: %s\n",
                          error_get_pretty(local_err));
            error_free(local_err);
            local_err = NULL;
        }
        }
    }

    if (!scope_ack_cfg_packet(s, pkt->seq)) {
        qemu_log_mask(LOG_GUEST_ERROR, "SCOPE: Failed to ACK CFG packet seq=%u\n",
                      pkt->seq);
    }
}

static bool scope_poll_dma32_ring(ScopeProxyState *s,
                                  uint32_t *last_seq_by_slot,
                                  uint32_t *last_type_by_slot,
                                  size_t slot_count, size_t *cursor)
{
    const size_t slot_size = sizeof(struct scope_dma32_packet);
    uint8_t *ring_base = s->dma32_db_map;
    bool progressed = false;
    size_t n;

    for (n = 0; n < MIN(slot_count, (size_t)128); n++) {
        size_t i = (*cursor + n) % slot_count;
        struct scope_dma32_packet pkt;

        if (!scope_read_stable_packet(ring_base + i * slot_size, &pkt)) {
            continue;
        }
        if (pkt.magic != XDMA_DMA32_PKT_MAGIC ||
            pkt.len != sizeof(pkt) ||
            pkt.seq == 0 ||
            pkt.type == 0 ||
            pkt.type > SCOPE_DMA32_TYPE_MAX) {
            continue;
        }
        if (pkt.seq == last_seq_by_slot[i] && pkt.type == last_type_by_slot[i]) {
            continue;
        }

        last_seq_by_slot[i] = pkt.seq;
        last_type_by_slot[i] = pkt.type;
        progressed = true;

        qemu_mutex_lock(&s->state_lock);
        if (pkt.type == SCOPE_PKT_TYPE_CFG_WRITE) {
            scope_process_cfg_packet(s, &pkt);
        } else if (pkt.type == SCOPE_PKT_TYPE_SQE_WRITE_DONE) {
            scope_process_sqe_write_done_packet(s, &pkt);
        } else if (pkt.type == SCOPE_PKT_TYPE_BAR_WRITE ||
                   pkt.type == SCOPE_PKT_TYPE_BAR_READ ||
                   pkt.type == SCOPE_PKT_TYPE_BAR_WRITE_DONE) {
            scope_process_bar_packet(s, &pkt);
        }
        qemu_mutex_unlock(&s->state_lock);
    }
    *cursor = (*cursor + MIN(slot_count, (size_t)128)) % slot_count;

    return progressed;
}

static void *scope_proxy_rx_thread(void *opaque)
{
    ScopeProxyState *s = SCOPE_PROXY(opaque);
    const size_t slot_count = s->dma32_db.size / sizeof(struct scope_dma32_packet);
    uint32_t *last_seq_by_slot = g_new0(uint32_t, slot_count);
    uint32_t *last_type_by_slot = g_new0(uint32_t, slot_count);
    int64_t last_pending_log_us = 0;
    size_t ring_cursor = 0;

    while (!qatomic_read(&s->rx_thread_stop)) {
        bool progressed = false;
        unsigned int b;

        progressed = scope_poll_dma32_ring(s, last_seq_by_slot,
                                           last_type_by_slot, slot_count,
                                           &ring_cursor) || progressed;

        qemu_mutex_lock(&s->state_lock);
        scope_update_virtual_intx(s);
        for (b = 0; b < s->backend_count; b++) {
            s->active = &s->backends[b];
            if (s->active->pending_sq_db.valid) {
                int64_t now_us = g_get_monotonic_time();

                if (now_us - last_pending_log_us >= 1000000) {
                    SCOPE_PRINTF("[SCOPE PROXY][DB][PENDING] backend=%u qid=%u seq=%u bar_done=%d "
                           "off=0x%04x new_tail=%u\n",
                           b,
                           s->active->pending_sq_db.qid, s->active->pending_sq_db.seq,
                           s->active->pending_sq_db.bar_done,
                           s->active->pending_sq_db.offset,
                           s->active->pending_sq_db.new_tail);
                    SCOPE_FFLUSH(stdout);
                    last_pending_log_us = now_us;
                }
                if (!s->active->pending_sq_db.bar_done) {
                    progressed = scope_recover_missing_bar_done(s, now_us) ||
                                 progressed;
                }
                if (s->active->pending_sq_db.valid &&
                    s->active->pending_sq_db.bar_done &&
                    now_us >= s->active->pending_sq_db.fallback_next_try_us) {
                    scope_try_process_pending_sq_doorbell(s);
                    progressed = true;
                }
            }
        }
        qemu_mutex_unlock(&s->state_lock);

        if (!progressed) {
            g_usleep(20);
        }
    }

    g_free(last_type_by_slot);
    g_free(last_seq_by_slot);
    return NULL;
}

static bool scope_alloc_dma32_ring(ScopeProxyState *s, Error **errp)
{
    off_t mmap_offset;

    if (s->dma32_ring_size < 0x1000 || s->dma32_ring_size > 0x400000 ||
        (s->dma32_ring_size & 0xfff)) {
        error_setg(errp, "dma32-ring-size must be page aligned and in [4KiB, 4MiB]");
        return false;
    }
    s->dma32_db.size = s->dma32_ring_size;
    if (ioctl(s->xdma_ctrl_fd, XDMA_IOC_DMA32_DB_ALLOC, &s->dma32_db) < 0) {
        error_setg_errno(errp, errno, "XDMA_IOC_DMA32_DB_ALLOC failed");
        return false;
    }

    mmap_offset = (off_t)XDMA_DMA32_DB_MMAP_PGOFF * (off_t)s->host_page_size;
    s->dma32_db_map = mmap(NULL, s->dma32_db.size, PROT_READ | PROT_WRITE,
                           MAP_SHARED, s->xdma_ctrl_fd, mmap_offset);
    if (s->dma32_db_map == MAP_FAILED) {
        s->dma32_db_map = NULL;
        ioctl(s->xdma_ctrl_fd, XDMA_IOC_DMA32_DB_FREE);
        memset(&s->dma32_db, 0, sizeof(s->dma32_db));
        error_setg_errno(errp, errno, "Failed to mmap DMA32 ring buffer");
        return false;
    }

    /* The previous run may have left stale packets in the coherent buffer. */
    scope_zero_dma32_ring(s);
    return true;
}

static bool scope_parse_real_bar0(ScopeProxyState *s, Error **errp)
{
    ScopePciResource res;
    g_autofree char *resource0_path = NULL;

    if (!s->active->real_host_bdf || !s->active->real_host_bdf[0]) {
        error_setg(errp, "Property real-host-bdf is required");
        return false;
    }

    if (!scope_read_pci_bar_resource(s->active->real_host_bdf, 0, &res, errp)) {
        return false;
    }
    if (!(res.flags & IORESOURCE_MEM)) {
        error_setg(errp, "Real BAR0 of %s is not a memory BAR", s->active->real_host_bdf);
        return false;
    }
    if (res.end < res.start) {
        error_setg(errp, "Real BAR0 of %s has invalid range", s->active->real_host_bdf);
        return false;
    }

    s->active->real_bar0_size = (size_t)(res.end - res.start + 1);
    s->active->real_bar0_flags = res.flags;

    if (!s->active->real_bar0_size) {
        error_setg(errp, "Real BAR0 of %s has zero size", s->active->real_host_bdf);
        return false;
    }
    if (s->active->real_bar0_size & (s->active->real_bar0_size - 1)) {
        error_setg(errp, "Real BAR0 size 0x%zx of %s is not a power of two",
                   s->active->real_bar0_size, s->active->real_host_bdf);
        return false;
    }
    if (s->active->real_bar0_size > SCOPE_RP_BAR_APERTURE_SIZE) {
        error_setg(errp, "Real BAR0 size 0x%zx exceeds RP BAR aperture 0x%x",
                   s->active->real_bar0_size, SCOPE_RP_BAR_APERTURE_SIZE);
        return false;
    }

    resource0_path = g_strdup_printf("/sys/bus/pci/devices/%s/resource0", s->active->real_host_bdf);
    s->active->real_bar_fd = open(resource0_path, O_RDWR | O_SYNC);
    if (s->active->real_bar_fd < 0) {
        error_setg_errno(errp, errno, "Failed to open %s", resource0_path);
        return false;
    }

    s->active->real_bar0_map = mmap(NULL, s->active->real_bar0_size, PROT_READ | PROT_WRITE,
                            MAP_SHARED, s->active->real_bar_fd, 0);
    if (s->active->real_bar0_map == MAP_FAILED) {
        s->active->real_bar0_map = NULL;
        error_setg_errno(errp, errno, "Failed to mmap %s", resource0_path);
        return false;
    }

    return true;
}

static bool scope_parse_fpga_bypass_bar(ScopeProxyState *s, Error **errp)
{
    g_autofree char *resource_path = NULL;
    FILE *fp = NULL;
    char line[256];
    int bar_index = -1;
    int i;
    ScopePciResource chosen = { 0 };

    if (!s->fpga_host_bdf || !s->fpga_host_bdf[0]) {
        error_setg(errp, "Property fpga-host-bdf is required");
        return false;
    }

    resource_path = g_strdup_printf("/sys/bus/pci/devices/%s/resource", s->fpga_host_bdf);
    fp = fopen(resource_path, "r");
    if (!fp) {
        error_setg_errno(errp, errno, "Failed to open %s", resource_path);
        return false;
    }

    for (i = 0; i < 6; i++) {
        ScopePciResource res;

        if (!fgets(line, sizeof(line), fp)) {
            fclose(fp);
            error_setg_errno(errp, errno, "Failed to read BAR%d line from %s",
                             i, resource_path);
            return false;
        }
        if (!scope_parse_resource_line(line, &res)) {
            continue;
        }
        if (!(res.flags & IORESOURCE_MEM) || res.end < res.start) {
            continue;
        }
        if ((res.end - res.start + 1) == 0) {
            continue;
        }

        bar_index = i;
        chosen = res;
    }
    fclose(fp);

    if (bar_index < 0) {
        error_setg(errp, "Failed to find FPGA bypass BAR for %s", s->fpga_host_bdf);
        return false;
    }

    s->fpga_bypass_bar_index = bar_index;
    s->fpga_bypass_bar_base = chosen.start;
    s->fpga_bypass_bar_size = chosen.end - chosen.start + 1;

    if (!s->fpga_bypass_bar_base || !s->fpga_bypass_bar_size) {
        error_setg(errp, "FPGA bypass BAR for %s is not assigned", s->fpga_host_bdf);
        return false;
    }
    if (s->guest_ddr_size) {
        uint64_t guest_end;
        uint64_t required_end;

        if (s->guest_ddr_base > UINT64_MAX - s->guest_ddr_size) {
            error_setg(errp, "Guest DDR window overflows: base=0x%" PRIx64
                       " size=0x%" PRIx64,
                       s->guest_ddr_base, s->guest_ddr_size);
            return false;
        }
        guest_end = s->guest_ddr_base + s->guest_ddr_size;
        required_end = guest_end;
        if (s->bypass_coherent_alias_base) {
            if (s->bypass_coherent_alias_base > UINT64_MAX - guest_end) {
                error_setg(errp, "Bypass coherent alias window overflows: "
                           "alias_base=0x%" PRIx64 " guest_end=0x%" PRIx64,
                           s->bypass_coherent_alias_base, guest_end);
                return false;
            }
            required_end = s->bypass_coherent_alias_base + guest_end;
        }
        if (s->fpga_bypass_bar_size < required_end) {
            error_setg(errp, "FPGA bypass BAR size 0x%" PRIx64
                       " is smaller than required span 0x%" PRIx64
                       " (guest_end=0x%" PRIx64 ", alias_base=0x%" PRIx64 ")",
                       s->fpga_bypass_bar_size, required_end, guest_end,
                       s->bypass_coherent_alias_base);
            return false;
        }
    }

    return true;
}

static bool scope_init_nvme_capability_cache(ScopeProxyState *s, Error **errp)
{
    uint32_t cap_lo = 0;
    uint32_t cap_hi = 0;

    if (!scope_real_bar_read32(s, NVME_REG_CAP, &cap_lo) ||
        !scope_real_bar_read32(s, NVME_REG_CAP + 4, &cap_hi) ||
        !scope_real_bar_read32(s, NVME_REG_VS, &s->active->nvme_vs)) {
        error_setg(errp, "Failed to read real NVMe capability registers");
        return false;
    }

    s->active->nvme_cap = ((uint64_t)cap_hi << 32) | cap_lo;
    s->active->doorbell_stride = 4U << NVME_CAP_DSTRD(s->active->nvme_cap);
    if (!s->active->doorbell_stride) {
        error_setg(errp, "Invalid NVMe CAP.DSTRD");
        return false;
    }

    return true;
}

static void scope_proxy_cleanup(ScopeProxyState *s)
{
    unsigned int i;

    if (s->rx_thread_started) {
        qatomic_set(&s->rx_thread_stop, 1);
        qemu_thread_join(&s->rx_thread);
        s->rx_thread_started = false;
    }

    if (s->dma32_db_map) {
        munmap(s->dma32_db_map, s->dma32_db.size);
        s->dma32_db_map = NULL;
    }
    if (s->ecam_shadow_map) {
        munmap(s->ecam_shadow_map, HOST_ECAM_SHADOW_SIZE);
        s->ecam_shadow_map = NULL;
    }
    if (s->xdma_ctrl_fd >= 0 && s->dma32_db.size) {
        ioctl(s->xdma_ctrl_fd, XDMA_IOC_DMA32_DB_FREE);
        memset(&s->dma32_db, 0, sizeof(s->dma32_db));
    }

    if (s->backends) {
        for (i = 0; i < SCOPE_VSWITCH_MAX_NVME; i++) {
            ScopeNvmeBackend *be = &s->backends[i];
            scope_restore_real_pci_command(be);
            if (be->real_bar0_map) munmap(be->real_bar0_map, be->real_bar0_size);
            if (be->real_bar_fd >= 0) close(be->real_bar_fd);
            g_free(be->pending_admin_ops);
            if (be->ns_lba_shift_map) g_hash_table_destroy(be->ns_lba_shift_map);
            g_free(be->real_host_bdf);
        }
    }
    if (s->xdma_bypass_fd >= 0) {
        close(s->xdma_bypass_fd);
        s->xdma_bypass_fd = -1;
    }
    if (s->xdma_fd >= 0) {
        close(s->xdma_fd);
        s->xdma_fd = -1;
    }
    if (s->xdma_ctrl_fd >= 0) {
        close(s->xdma_ctrl_fd);
        s->xdma_ctrl_fd = -1;
    }
    if (s->xdma_lock_inited) {
        qemu_mutex_destroy(&s->xdma_lock);
        s->xdma_lock_inited = false;
    }
    if (s->state_lock_inited) {
        qemu_mutex_destroy(&s->state_lock);
        s->state_lock_inited = false;
    }
    g_free(s->backends);
    s->backends = NULL;
    s->active = NULL;
    s->backend_count = 0;
}

static void scope_proxy_realize(PCIDevice *pci_dev, Error **errp)
{
    ScopeProxyState *s = SCOPE_PROXY(pci_dev);
    uint32_t dummy_ack = 1;
    const char *xdma_user_dev = s->xdma_user_dev ? s->xdma_user_dev : SCOPE_DEFAULT_XDMA_USER;
    const char *xdma_ctrl_dev = s->xdma_ctrl_dev ? s->xdma_ctrl_dev : SCOPE_DEFAULT_XDMA_CTRL;
    const char *xdma_bypass_dev = s->xdma_bypass_dev ? s->xdma_bypass_dev : SCOPE_DEFAULT_XDMA_BYPASS;
    unsigned int i;
    long host_page_size;
    size_t template_bar_size = 0x4000;

    SCOPE_PRINTF("\n[SCOPE VSWITCH] Initializing virtual-switch NVMe proxy device...\n");

    if (!s->guest_ddr_size) {
        error_setg(errp, "Property guest-ddr-size is required");
        return;
    }

    if (!scope_parse_backend_config(s, errp)) goto fail;
    for (i = 0; i < s->backend_count; i++) {
        ScopeNvmeBackend *be = &s->backends[i];
        be->id = i;
        be->virtual_bdf = SCOPE_VSWITCH_NVME_BDF(i);
        be->real_bar_fd = -1;
        be->ctrl_page_size = SCOPE_NVME_DEFAULT_CTRL_PAGE_SIZE;
        be->ns_lba_shift_map = g_hash_table_new(g_direct_hash, g_direct_equal);
        be->pending_admin_ops = g_new0(ScopePendingAdminOp, SCOPE_ADMIN_CID_SPACE);
    }
    s->active = s->backend_count ? &s->backends[0] : NULL;

    host_page_size = sysconf(_SC_PAGESIZE);
    s->host_page_size = host_page_size > 0 ? host_page_size : 4096;

    qemu_mutex_init(&s->xdma_lock);
    s->xdma_lock_inited = true;
    qemu_mutex_init(&s->state_lock);
    s->state_lock_inited = true;
    if (!scope_parse_fpga_bypass_bar(s, errp)) {
        goto fail;
    }
    for (i = 0; i < s->backend_count; i++) {
        s->active = &s->backends[i];
        if (!scope_enable_real_pci_bus_master(s, errp) ||
            !scope_parse_real_bar0(s, errp) ||
            !scope_init_nvme_capability_cache(s, errp) ||
            !scope_real_nvme_disable(s, "qemu startup", errp)) goto fail;
        template_bar_size = MAX(template_bar_size, s->active->real_bar0_size);
    }

    s->xdma_fd = open(xdma_user_dev, O_RDWR | O_SYNC);
    if (s->xdma_fd < 0) {
        error_setg_errno(errp, errno, "Failed to open %s", xdma_user_dev);
        goto fail;
    }
    s->ecam_shadow_map = mmap(NULL, HOST_ECAM_SHADOW_SIZE, PROT_READ | PROT_WRITE,
                              MAP_SHARED, s->xdma_fd, HOST_ECAM_SHADOW_BASE);
    if (s->ecam_shadow_map == MAP_FAILED) {
        s->ecam_shadow_map = NULL;
        error_setg_errno(errp, errno, "Failed to mmap ECAM shadow window");
        goto fail;
    }

    s->xdma_ctrl_fd = open(xdma_ctrl_dev, O_RDWR);
    if (s->xdma_ctrl_fd < 0) {
        error_setg_errno(errp, errno, "Failed to open %s", xdma_ctrl_dev);
        goto fail;
    }

    s->xdma_bypass_fd = open(xdma_bypass_dev, O_RDWR | O_SYNC);
    if (s->xdma_bypass_fd < 0) {
        error_setg_errno(errp, errno, "Failed to open %s", xdma_bypass_dev);
        goto fail;
    }

    if (!scope_alloc_dma32_ring(s, errp)) {
        goto fail;
    }

    pci_config_set_vendor_id(pci_dev->config, 0x8086);
    pci_config_set_device_id(pci_dev->config, 0x4802);
    pci_config_set_class(pci_dev->config, PCI_CLASS_STORAGE_EXPRESS);
    pci_dev->config[PCI_CLASS_PROG] = 0x02;
    pci_dev->config[PCI_INTERRUPT_PIN] = 0x01;
    pci_dev->config[PCI_INTERRUPT_LINE] = 0xff;

    if (pcie_endpoint_cap_init(pci_dev, 0x70) < 0) {
        error_setg(errp, "Failed to initialize PCIe endpoint capability");
        goto fail;
    }

    memory_region_init_io(&s->dummy_bar0, OBJECT(s), &dummy_bar_ops, s,
                          "scope-bar0", template_bar_size);
    pci_register_bar(pci_dev, 0,
                     PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_64,
                     &s->dummy_bar0);

    s->proxy_ctrl_shadow = 0;
    scope_vcfg_init_all(s, pci_dev);
    if (!scope_sync_ecam_shadow_all(s, errp)) {
        goto fail;
    }
    if (!scope_sync_ecam_shadow_fence(s, SCOPE_VSWITCH_ECAM_FUNC_COUNT - 1,
                                      SCOPE_VSWITCH_ECAM_FUNC_SIZE - 4)) {
        error_setg(errp, "Failed to read back ECAM shadow BRAM after init");
        goto fail;
    }
    if (!scope_set_proxy_ctrl_bits(s, PROXY_CTRL_ECAM_SHADOW_READY, errp)) goto fail;
    qemu_mutex_lock(&s->xdma_lock);
    if (!scope_xdma_write32_locked(s, SQE_MON_CFG_BASE + MON_REG_STATUS,
                                   MON_STATUS_OVERFLOW)) {
        qemu_mutex_unlock(&s->xdma_lock);
        error_setg(errp, "Failed to clear SQE monitor status during startup");
        goto fail;
    }
    qemu_mutex_unlock(&s->xdma_lock);
    for (i = 0; i < SCOPE_VSWITCH_MAX_NVME; i++) {
        uint32_t route = HOST_MBX_BASE + SCOPE_ROUTE_BASE + i * SCOPE_ROUTE_STRIDE;
        qemu_mutex_lock(&s->xdma_lock);
        scope_xdma_write32_locked(s, route + SCOPE_ROUTE_CTRL, 0);
        scope_xdma_write32_locked(s,
                                  SQE_MON_CFG_BASE + i * MON_BACKEND_STRIDE +
                                  MON_REG_ADMIN_SQ_CTRL,
                                  0);
        qemu_mutex_unlock(&s->xdma_lock);
    }
    for (i = 0; i < s->backend_count; i++) {
        s->active = &s->backends[i];
        if (!scope_sync_guest_bar_shadow(s, &s->vcfg[SCOPE_VSWITCH_NVME_SLOT(i)],
                                         s->active, errp) ||
            !scope_sync_admin_window_to_fpga(s, false)) goto fail;
    }

    qemu_mutex_lock(&s->xdma_lock);
    if (!scope_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_ACK, dummy_ack)) {
        qemu_mutex_unlock(&s->xdma_lock);
        error_setg(errp, "Failed to reset FPGA Mailbox ACK");
        goto fail;
    }
    qemu_mutex_unlock(&s->xdma_lock);

    if (!scope_virtual_rp_set_intx(s, false)) {
        error_setg(errp, "Failed to reset proxy INTx state");
        goto fail;
    }

    qatomic_set(&s->rx_thread_stop, 0);
    qemu_thread_create(&s->rx_thread, "scope-proxy-rx", scope_proxy_rx_thread,
                       s, QEMU_THREAD_JOINABLE);
    s->rx_thread_started = true;

    SCOPE_PRINTF("[SCOPE PROXY] Active NVMe backends=%u\n", s->backend_count);
    for (i = 0; i < s->backend_count; i++)
        SCOPE_PRINTF("[SCOPE PROXY] backend=%u virtual=%02x:00.0 real=%s BAR0=0x%zx\n",
                     i, 3 + i, s->backends[i].real_host_bdf,
                     s->backends[i].real_bar0_size);
    SCOPE_PRINTF("[SCOPE PROXY] FPGA bypass BAR%d @ 0x%016" PRIx64 ", size=0x%016" PRIx64 "\n",
           s->fpga_bypass_bar_index, s->fpga_bypass_bar_base, s->fpga_bypass_bar_size);
    SCOPE_PRINTF("[SCOPE PROXY] Real DMA coherent alias offset=0x%016" PRIx64 "\n",
           s->bypass_coherent_alias_base);
    SCOPE_PRINTF("[SCOPE PROXY] QEMU guest memory access alias offset=0x%016" PRIx64
           ", raw_fallback=%d\n",
           s->bypass_coherent_alias_base, s->guest_mem_raw_fallback);
    SCOPE_PRINTF("[SCOPE PROXY] SQE visibility mode=%s\n",
           s->sqe_monitor_enable ? "write-done-monitor" : "coherent-alias-read");
    SCOPE_PRINTF("[SCOPE PROXY] BAR_DONE timeout recovery=%s timeout_us=%u\n",
           s->bar_done_timeout_us ? "enabled" : "disabled",
           s->bar_done_timeout_us);
    SCOPE_PRINTF("[SCOPE PROXY] Guest DDR base=0x%016" PRIx64 ", size=0x%016" PRIx64 "\n",
           s->guest_ddr_base, s->guest_ddr_size);
    SCOPE_PRINTF("[SCOPE PROXY] INTx-only virtual device, MSI/MSI-X disabled.\n");
    if (s->xdma_event_dev) {
        SCOPE_PRINTF("[SCOPE PROXY] xdma-event-dev=%s is accepted for command-line "
               "compatibility but ignored; CFG writes use DMA32 C2H packets.\n",
               s->xdma_event_dev);
    }
    SCOPE_PRINTF("[SCOPE PROXY] DMA32 CFG/BAR RX thread started.\n\n");
    return;

fail:
    scope_proxy_cleanup(s);
}

static void scope_proxy_exit(PCIDevice *pci_dev)
{
    ScopeProxyState *s = SCOPE_PROXY(pci_dev);

    scope_proxy_cleanup(s);
    SCOPE_PRINTF("[SCOPE PROXY] Device exited and cleaned up.\n");
}

static void scope_proxy_instance_init(Object *obj)
{
    ScopeProxyState *s = SCOPE_PROXY(obj);

    s->xdma_fd = -1;
    s->xdma_ctrl_fd = -1;
    s->xdma_bypass_fd = -1;
    s->backends = NULL;
    s->active = NULL;
    s->backend_count = 0;
    s->dma32_ring_size = SCOPE_DMA32_RING_SIZE;
    s->bar_done_timeout_us = SCOPE_BAR_DONE_TIMEOUT_US;
    s->dma32_db_map = NULL;
    s->ecam_shadow_map = NULL;
    memset(&s->dma32_db, 0, sizeof(s->dma32_db));
    s->xdma_lock_inited = false;
    s->state_lock_inited = false;
    s->rx_thread_started = false;
    s->rx_thread_stop = 0;
    s->bar_resp_toggle = 0;
    s->fpga_bypass_bar_base = 0;
    s->fpga_bypass_bar_size = 0;
    s->bypass_coherent_alias_base = SCOPE_DEFAULT_BYPASS_COHERENT_ALIAS_BASE;
    s->guest_mem_raw_fallback = false;
    s->sqe_monitor_enable = false;
    s->fpga_bypass_bar_index = -1;
    s->guest_ddr_base = 0;
    s->guest_ddr_size = 0;
    s->host_page_size = 0;
    s->virtual_intx_level = false;
    s->virtual_intx_assert_count = 0;
    s->virtual_intx_deassert_count = 0;
    s->virtual_intx_retry_count = 0;
    s->virtual_intx_last_retry_us = 0;
    s->intx_retry_pulse = false;
}

static const Property scope_proxy_properties[] = {
    DEFINE_PROP_STRING("real-host-bdf", ScopeProxyState, legacy_real_host_bdf),
    DEFINE_PROP_STRING("backend-config", ScopeProxyState, backend_config),
    DEFINE_PROP_UINT32("dma32-ring-size", ScopeProxyState, dma32_ring_size,
                       SCOPE_DMA32_RING_SIZE),
    DEFINE_PROP_UINT32("bar-done-timeout-us", ScopeProxyState,
                       bar_done_timeout_us, SCOPE_BAR_DONE_TIMEOUT_US),
    DEFINE_PROP_STRING("fpga-host-bdf", ScopeProxyState, fpga_host_bdf),
    DEFINE_PROP_STRING("xdma-user-dev", ScopeProxyState, xdma_user_dev),
    DEFINE_PROP_STRING("xdma-ctrl-dev", ScopeProxyState, xdma_ctrl_dev),
    DEFINE_PROP_STRING("xdma-event-dev", ScopeProxyState, xdma_event_dev),
    DEFINE_PROP_STRING("xdma-bypass-dev", ScopeProxyState, xdma_bypass_dev),
    DEFINE_PROP_UINT64("guest-ddr-base", ScopeProxyState, guest_ddr_base, 0),
    DEFINE_PROP_UINT64("guest-ddr-size", ScopeProxyState, guest_ddr_size, 0),
    DEFINE_PROP_UINT64("bypass-coherent-alias-base", ScopeProxyState,
                       bypass_coherent_alias_base,
                       SCOPE_DEFAULT_BYPASS_COHERENT_ALIAS_BASE),
    DEFINE_PROP_BOOL("guest-mem-raw-fallback", ScopeProxyState,
                     guest_mem_raw_fallback, false),
    DEFINE_PROP_BOOL("sqe-monitor-enable", ScopeProxyState,
                     sqe_monitor_enable, false),
    DEFINE_PROP_BOOL("intx-retry-pulse", ScopeProxyState,
                     intx_retry_pulse, false),
};

static void scope_proxy_class_init(ObjectClass *class, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize = scope_proxy_realize;
    k->exit = scope_proxy_exit;

    device_class_set_props(dc, scope_proxy_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo scope_proxy_info = {
    .name          = TYPE_SCOPE_PROXY,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(ScopeProxyState),
    .instance_init = scope_proxy_instance_init,
    .class_init    = scope_proxy_class_init,
    .interfaces    = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};

static void scope_proxy_register_types(void)
{
    type_register_static(&scope_proxy_info);
}
type_init(scope_proxy_register_types)
