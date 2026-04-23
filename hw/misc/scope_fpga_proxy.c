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
#include "hw/misc/scope_fpga_proxy_abi.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>

#define TYPE_SCOPE_PROXY "scope-fpga-proxy"
OBJECT_DECLARE_SIMPLE_TYPE(ScopeProxyState, SCOPE_PROXY)

#define HOST_MBX_BASE    0x01000000U
#define HOST_VCONF_BASE  0x01010000U

#define MBX_REG_STATUS            0x00U
#define MBX_REG_AWADDR            0x04U
#define MBX_REG_WDATA             0x08U
#define MBX_REG_WSTRB             0x0CU
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

#define GUEST_BAR0_CTRL_VALID      (1U << 0)
#define GUEST_BAR0_CTRL_MEM_ENABLE (1U << 1)
#define GUEST_BAR0_CTRL_IS64       (1U << 2)
#define PROXY_CTRL_REAL_BAR_READY  (1U << 0)
#define BAR_RESP_CTRL_TOGGLE_SHIFT 2

#define IORESOURCE_MEM               0x00000200ULL
#define SCOPE_RP_BAR_APERTURE_SIZE   0x00100000U
#define SCOPE_DMA32_RING_SIZE        0x1000U
#define SCOPE_MAX_NVME_QUEUES        256U
#define SCOPE_ADMIN_QID              0U
#define SCOPE_ADMIN_CID_SPACE        (UINT16_MAX + 1U)
#define SCOPE_NVME_DOORBELL_BASE     0x1000U
#define SCOPE_ADMIN_SQE_INITIAL_DELAY_US 2000000U
#define SCOPE_ADMIN_SQE_RETRY_MAX    10U
#define SCOPE_ADMIN_SQE_RETRY_DELAY_US 1000000U
#define SCOPE_DEFAULT_XDMA_USER      "/dev/xdma0_user"
#define SCOPE_DEFAULT_XDMA_CTRL      "/dev/xdma0_control"
#define SCOPE_DEFAULT_XDMA_EVENT     "/dev/xdma0_events_0"
#define SCOPE_DEFAULT_XDMA_BYPASS    "/dev/xdma0_bypass"

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
} ScopeAdminTopoOpType;

typedef struct ScopePendingAdminOp {
    bool valid;
    ScopeAdminTopoOpType type;
    uint16_t qid;
    uint16_t cqid;
    uint16_t depth;
    bool interrupt_enabled;
    uint64_t guest_base;
} ScopePendingAdminOp;

struct ScopeProxyState {
    PCIDevice parent_obj;

    int xdma_fd;
    int xdma_ctrl_fd;
    int event_fd;
    int xdma_bypass_fd;
    int real_bar_fd;

    void *real_bar0_map;
    size_t real_bar0_size;
    uint64_t real_bar0_flags;

    void *dma32_db_map;
    struct scope_xdma_dma32_doorbell dma32_db;

    QemuMutex xdma_lock;
    bool xdma_lock_inited;

    QemuThread rx_thread;
    bool rx_thread_started;
    int rx_thread_stop;
    uint32_t bar_resp_toggle;

    char *real_host_bdf;
    char *fpga_host_bdf;
    char *xdma_user_dev;
    char *xdma_ctrl_dev;
    char *xdma_event_dev;
    char *xdma_bypass_dev;

    uint64_t fpga_bypass_bar_base;
    uint64_t fpga_bypass_bar_size;
    int fpga_bypass_bar_index;

    uint64_t guest_ddr_base;
    uint64_t guest_ddr_size;

    size_t host_page_size;

    uint64_t nvme_cap;
    uint32_t nvme_vs;
    uint32_t doorbell_stride;

    uint32_t guest_cc;
    uint32_t guest_aqa;
    uint32_t guest_int_mask;
    uint64_t guest_asq;
    uint64_t guest_acq;
    bool virtual_intx_level;

    ScopeSqState sq[SCOPE_MAX_NVME_QUEUES];
    ScopeCqState cq[SCOPE_MAX_NVME_QUEUES];
    ScopePendingAdminOp *pending_admin_ops;

    MemoryRegion dummy_bar0;
};

typedef struct ScopePciResource {
    uint64_t start;
    uint64_t end;
    uint64_t flags;
} ScopePciResource;

static uint64_t dummy_bar_read(void *opaque, hwaddr addr, unsigned size)
{
    printf("[SCOPE PROXY] Guest OS READ  BAR0: offset 0x%04" HWADDR_PRIx ", size %u bytes\n",
           addr, size);
    return 0;
}

static void dummy_bar_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    printf("[SCOPE PROXY] Guest OS WRITE BAR0: offset 0x%04" HWADDR_PRIx
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
    printf("[SCOPE PROXY][CFG][%s] seq=%u addr=0x%03x len=%d wstrb=0x%x data=0x%08x\n",
           path, seq, addr, len, wstrb, data);
    fflush(stdout);
}

static void scope_log_nvme_cmd(const char *tag, const ScopeSqState *sq, uint16_t slot,
                               uint64_t guest_pa, const NvmeCmd *cmd)
{
    uint64_t mptr = cmd ? le64_to_cpu(cmd->mptr) : 0;
    uint64_t prp1 = cmd ? le64_to_cpu(cmd->dptr.prp1) : 0;
    uint64_t prp2 = cmd ? le64_to_cpu(cmd->dptr.prp2) : 0;

    printf("[SCOPE PROXY][CMD][%s] qid=%u slot=%u guest_pa=0x%016" PRIx64
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
    fflush(stdout);
}

static void scope_log_nvme_cmd_dwords(const char *tag, const ScopeSqState *sq, uint16_t slot,
                                      uint64_t guest_pa, const NvmeCmd *cmd)
{
    uint32_t dw[sizeof(*cmd) / sizeof(uint32_t)];
    int i;

    memcpy(dw, cmd, sizeof(dw));

    for (i = 0; i < ARRAY_SIZE(dw); i += 4) {
        printf("[SCOPE PROXY][CMD][%s][DW] qid=%u slot=%u guest_pa=0x%016" PRIx64
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
    fflush(stdout);
}

static void scope_log_nvme_cmd_hexdump(const char *tag, const ScopeSqState *sq, uint16_t slot,
                                       uint64_t guest_pa, const NvmeCmd *cmd)
{
    const uint8_t *bytes = (const uint8_t *)cmd;
    int i;

    for (i = 0; i < sizeof(*cmd); i += 16) {
        printf("[SCOPE PROXY][CMD][%s][HEX] qid=%u slot=%u guest_pa=0x%016" PRIx64
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
    fflush(stdout);
}

static void scope_log_guest_translation(const char *tag, const ScopeSqState *sq, uint16_t slot,
                                        uint64_t guest_pa, size_t len,
                                        uint64_t bar_offset, uint64_t translated_pa)
{
    printf("[SCOPE PROXY][CMD][%s][ADDR] qid=%u slot=%u guest_pa=0x%016" PRIx64
           " len=%zu bar_offset=0x%016" PRIx64 " translated=0x%016" PRIx64 "\n",
           tag,
           sq ? sq->qid : 0U,
           slot,
           guest_pa,
           len,
           bar_offset,
           translated_pa);
    fflush(stdout);
}

static void scope_log_nvme_cmd_head(const char *tag, const ScopeSqState *sq, uint16_t slot,
                                    uint64_t guest_pa, unsigned attempt, const NvmeCmd *cmd)
{
    uint32_t dw[4] = { 0 };

    memcpy(dw, cmd, sizeof(dw));

    printf("[SCOPE PROXY][CMD][%s][HEAD] qid=%u slot=%u guest_pa=0x%016" PRIx64
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
    fflush(stdout);
}

static bool scope_guest_mem_read(ScopeProxyState *s, uint64_t guest_pa, void *buf, size_t len);

static bool scope_nvme_cmd_is_zero(const NvmeCmd *cmd)
{
    static const uint8_t zero_cmd[sizeof(NvmeCmd)] = { 0 };

    return !memcmp(cmd, zero_cmd, sizeof(*cmd));
}

static bool scope_admin_cmd_looks_plausible(const NvmeCmd *cmd)
{
    switch (cmd->opcode) {
    case NVME_ADM_CMD_IDENTIFY:
    case NVME_ADM_CMD_GET_LOG_PAGE:
    case NVME_ADM_CMD_ABORT:
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

        return del->qid && del->qid < SCOPE_MAX_NVME_QUEUES;
    }
    default:
        return false;
    }
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

static void scope_zero_dma32_ring(ScopeProxyState *s)
{
    if (!s->dma32_db_map || !s->dma32_db.size) {
        return;
    }

    memset(s->dma32_db_map, 0, s->dma32_db.size);
    smp_wmb();
}

static bool scope_write_bar_response(ScopeProxyState *s, uint32_t seq, uint32_t resp,
                                     uint64_t data, bool has_data)
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
        ctrl = (resp & 0x3U) | (s->bar_resp_toggle << BAR_RESP_CTRL_TOGGLE_SHIFT);
        ok = scope_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_BAR_RESP_CTRL, ctrl);
    }

    qemu_mutex_unlock(&s->xdma_lock);
    return ok;
}

static bool scope_virtual_rp_set_intx(ScopeProxyState *s, bool level)
{
    bool ok;

    qemu_mutex_lock(&s->xdma_lock);
    ok = scope_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_RP_INTX_CTRL,
                                   level ? 1U : 0U);
    qemu_mutex_unlock(&s->xdma_lock);

    if (ok) {
        s->virtual_intx_level = level;
    }
    return ok;
}

static bool scope_ack_cfg_packet(ScopeProxyState *s, uint32_t seq)
{
    bool ok;

    qemu_mutex_lock(&s->xdma_lock);
    ok = scope_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_ACK, seq);
    qemu_mutex_unlock(&s->xdma_lock);

    return ok;
}

static bool scope_sync_vconf_to_fpga(ScopeProxyState *s, PCIDevice *pci_dev, Error **errp)
{
    uint32_t *config_ptr = (uint32_t *)pci_dev->config;
    bool ok = true;
    int i;

    qemu_mutex_lock(&s->xdma_lock);
    for (i = 0; i < 1024; i++) {
        if (pwrite(s->xdma_fd, &config_ptr[i], 4, HOST_VCONF_BASE + i * 4) != 4) {
            ok = false;
            break;
        }
    }
    qemu_mutex_unlock(&s->xdma_lock);

    if (!ok && errp) {
        error_setg(errp, "Failed to sync VCONF BRAM at word %d", i);
    }
    return ok;
}

static bool scope_sync_guest_bar_shadow(ScopeProxyState *s, PCIDevice *pci_dev, Error **errp)
{
    uint32_t bar0_lo = ldl_le_p(pci_dev->config + PCI_BASE_ADDRESS_0);
    uint32_t bar0_hi = ldl_le_p(pci_dev->config + PCI_BASE_ADDRESS_0 + 4);
    uint16_t command = lduw_le_p(pci_dev->config + PCI_COMMAND);
    bool is_mem = (bar0_lo & PCI_BASE_ADDRESS_SPACE) == PCI_BASE_ADDRESS_SPACE_MEMORY;
    bool is64 = is_mem &&
                ((bar0_lo & PCI_BASE_ADDRESS_MEM_TYPE_MASK) == PCI_BASE_ADDRESS_MEM_TYPE_64);
    uint64_t base = is_mem ?
        (((uint64_t)bar0_hi << 32) | (uint64_t)(bar0_lo & PCI_BASE_ADDRESS_MEM_MASK)) : 0;
    uint32_t ctrl = 0;
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

    qemu_mutex_lock(&s->xdma_lock);
    ok = scope_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_GUEST_BAR0_LO, (uint32_t)base);
    ok = ok && scope_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_GUEST_BAR0_HI,
                                         (uint32_t)(base >> 32));
    ok = ok && scope_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_GUEST_BAR0_SIZE,
                                         (uint32_t)s->real_bar0_size);
    ok = ok && scope_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_GUEST_BAR0_CTRL, ctrl);
    ok = ok && scope_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_PROXY_CTRL,
                                         PROXY_CTRL_REAL_BAR_READY);
    qemu_mutex_unlock(&s->xdma_lock);

    if (!ok && errp) {
        error_setg(errp, "Failed to sync guest BAR0 shadow registers");
    }
    if (ok) {
        printf("[SCOPE PROXY][BAR][SHADOW] raw_lo=0x%08x raw_hi=0x%08x base=0x%016" PRIx64
               " size=0x%08zx ctrl=0x%08x cmd=0x%04x\n",
               bar0_lo, bar0_hi, base, s->real_bar0_size, ctrl, command);
        fflush(stdout);
    }
    return ok;
}

static bool scope_real_bar_write(ScopeProxyState *s, uint32_t offset, uint64_t data,
                                 uint8_t wstrb, uint8_t size_bytes)
{
    uint32_t aligned_offset;
    volatile uint64_t *reg;
    volatile uint8_t *base;
    uint64_t current;
    uint64_t next;

    if (!s->real_bar0_map || !size_bytes || size_bytes > 8 ||
        (((offset & 0x7U) + size_bytes) > 8U)) {
        return false;
    }

    aligned_offset = offset & ~0x7U;
    if ((size_t)aligned_offset + sizeof(uint64_t) > s->real_bar0_size) {
        return false;
    }

    base = (volatile uint8_t *)s->real_bar0_map;
    reg = (volatile uint64_t *)(base + aligned_offset);
    current = *reg;
    next = scope_apply_wstrb64(current, data, wstrb);
    *reg = next;
    return true;
}

static bool scope_real_bar_read(ScopeProxyState *s, uint32_t offset, uint8_t size_bytes,
                                uint64_t *data)
{
    uint32_t aligned_offset;
    volatile uint64_t *reg;
    volatile uint8_t *base;

    if (!s->real_bar0_map || !data || !size_bytes || size_bytes > 8 ||
        (((offset & 0x7U) + size_bytes) > 8U)) {
        return false;
    }

    aligned_offset = offset & ~0x7U;
    if ((size_t)aligned_offset + sizeof(uint64_t) > s->real_bar0_size) {
        return false;
    }

    base = (volatile uint8_t *)s->real_bar0_map;
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

static void scope_reset_all_queue_state(ScopeProxyState *s)
{
    memset(s->sq, 0, sizeof(s->sq));
    memset(s->cq, 0, sizeof(s->cq));
    if (s->pending_admin_ops) {
        memset(s->pending_admin_ops, 0,
               sizeof(*s->pending_admin_ops) * SCOPE_ADMIN_CID_SPACE);
    }
}

static bool scope_guest_range_to_bar_offset(ScopeProxyState *s, uint64_t guest_pa, size_t len,
                                            uint64_t *bar_offset)
{
    if (!len || !s->guest_ddr_size || !s->fpga_bypass_bar_size) {
        printf("[SCOPE PROXY][DDR][RANGE][ERR] invalid translation request guest_pa=0x%016"
               PRIx64 " len=%zu guest_ddr_size=0x%016" PRIx64
               " bypass_bar_size=0x%016" PRIx64 "\n",
               guest_pa, len, s->guest_ddr_size, s->fpga_bypass_bar_size);
        fflush(stdout);
        return false;
    }
    if (guest_pa < s->guest_ddr_base) {
        printf("[SCOPE PROXY][DDR][RANGE][ERR] guest_pa=0x%016" PRIx64
               " below guest_ddr_base=0x%016" PRIx64 " len=%zu\n",
               guest_pa, s->guest_ddr_base, len);
        fflush(stdout);
        return false;
    }

    if (guest_pa > s->guest_ddr_base + s->guest_ddr_size ||
        len > (s->guest_ddr_base + s->guest_ddr_size) - guest_pa) {
        printf("[SCOPE PROXY][DDR][RANGE][ERR] guest_pa=0x%016" PRIx64
               " len=%zu exceeds guest DDR window [0x%016" PRIx64
               ", 0x%016" PRIx64 ")\n",
               guest_pa, len, s->guest_ddr_base,
               s->guest_ddr_base + s->guest_ddr_size);
        fflush(stdout);
        return false;
    }
    if (guest_pa > s->fpga_bypass_bar_size || len > s->fpga_bypass_bar_size - guest_pa) {
        printf("[SCOPE PROXY][DDR][RANGE][ERR] guest_pa=0x%016" PRIx64
               " len=%zu exceeds bypass BAR size=0x%016" PRIx64 "\n",
               guest_pa, len, s->fpga_bypass_bar_size);
        fflush(stdout);
        return false;
    }

    *bar_offset = guest_pa;
    return true;
}

static bool scope_translate_guest_pa(ScopeProxyState *s, uint64_t guest_pa, size_t len,
                                     uint64_t *translated_pa)
{
    uint64_t bar_offset;

    if (!scope_guest_range_to_bar_offset(s, guest_pa, len, &bar_offset)) {
        printf("[SCOPE PROXY][DDR][XLATE][ERR] guest_pa=0x%016" PRIx64
               " len=%zu translation failed\n", guest_pa, len);
        fflush(stdout);
        return false;
    }

    *translated_pa = s->fpga_bypass_bar_base + bar_offset;
    return true;
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
        printf("[SCOPE PROXY][DDR][MMAP][ERR] invalid bypass copy request bar_offset=0x%016"
               PRIx64 " len=%zu is_write=%d fd=%d host_page_size=%zu\n",
               bar_offset, len, is_write, s->xdma_bypass_fd, s->host_page_size);
        fflush(stdout);
        return false;
    }
    if (bar_offset > s->fpga_bypass_bar_size || len > s->fpga_bypass_bar_size - bar_offset) {
        printf("[SCOPE PROXY][DDR][MMAP][ERR] bar_offset=0x%016" PRIx64
               " len=%zu exceeds bypass BAR size=0x%016" PRIx64 "\n",
               bar_offset, len, s->fpga_bypass_bar_size);
        fflush(stdout);
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
        printf("[SCOPE PROXY][DDR][MMAP][ERR] mmap failed bar_offset=0x%016" PRIx64
               " map_base=0x%016" PRIx64 " map_len=0x%zx errno=%d (%s)\n",
               bar_offset, map_base, map_len, errno, strerror(errno));
        fflush(stdout);
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

    if (!scope_guest_range_to_bar_offset(s, guest_pa, len, &bar_offset)) {
        printf("[SCOPE PROXY][DDR][READ][ERR] guest_pa=0x%016" PRIx64
               " len=%zu range translation failed\n", guest_pa, len);
        fflush(stdout);
        return false;
    }
    if (!scope_bypass_copy(s, bar_offset, buf, len, false)) {
        printf("[SCOPE PROXY][DDR][READ][ERR] guest_pa=0x%016" PRIx64
               " bar_offset=0x%016" PRIx64 " len=%zu copy failed\n",
               guest_pa, bar_offset, len);
        fflush(stdout);
        return false;
    }
    return true;
}

static bool scope_guest_mem_write(ScopeProxyState *s, uint64_t guest_pa,
                                  const void *buf, size_t len)
{
    uint64_t bar_offset;

    if (!scope_guest_range_to_bar_offset(s, guest_pa, len, &bar_offset)) {
        printf("[SCOPE PROXY][DDR][WRITE][ERR] guest_pa=0x%016" PRIx64
               " len=%zu range translation failed\n", guest_pa, len);
        fflush(stdout);
        return false;
    }
    if (!scope_bypass_copy(s, bar_offset, (void *)buf, len, true)) {
        printf("[SCOPE PROXY][DDR][WRITE][ERR] guest_pa=0x%016" PRIx64
               " bar_offset=0x%016" PRIx64 " len=%zu copy failed\n",
               guest_pa, bar_offset, len);
        fflush(stdout);
        return false;
    }
    return true;
}

static void scope_capture_admin_seed(ScopeProxyState *s)
{
    ScopeSqState *sq = &s->sq[SCOPE_ADMIN_QID];
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
        printf("[SCOPE PROXY][CMD][SEED][ERR] qid=%u slot=0 guest_pa=0x%016" PRIx64
               " capture failed\n",
               sq->qid, seed_pa);
        fflush(stdout);
        return;
    }

    sq->seed_valid = true;
    sq->seed_guest_pa = seed_pa;
    sq->seed_cmd = seed;

    printf("[SCOPE PROXY][CMD][SEED] qid=%u slot=0 guest_pa=0x%016" PRIx64
           " zero=%d plausible=%d\n",
           sq->qid, seed_pa, scope_nvme_cmd_is_zero(&seed),
           scope_admin_cmd_looks_plausible(&seed));
    fflush(stdout);
}

static bool scope_read_admin_sqe_with_retry(ScopeProxyState *s, const ScopeSqState *sq,
                                            uint16_t slot, uint64_t guest_pa, NvmeCmd *cmd)
{
    NvmeCmd sample = { 0 };
    bool have_sample = false;
    bool sample_plausible = false;
    unsigned stable_plausible_reads = 0;
    unsigned attempt;

    if (SCOPE_ADMIN_SQE_INITIAL_DELAY_US) {
        printf("[SCOPE PROXY][CMD][RAW][RETRY_DELAY] qid=%u slot=%u "
               "guest_pa=0x%016" PRIx64 " initial_wait_us=%u\n",
               sq ? sq->qid : 0U, slot, guest_pa, SCOPE_ADMIN_SQE_INITIAL_DELAY_US);
        fflush(stdout);
        g_usleep(SCOPE_ADMIN_SQE_INITIAL_DELAY_US);
    }

    for (attempt = 0; attempt < SCOPE_ADMIN_SQE_RETRY_MAX; ++attempt) {
        if (attempt) {
            g_usleep(SCOPE_ADMIN_SQE_RETRY_DELAY_US);
        }

        if (!scope_guest_mem_read(s, guest_pa, &sample, sizeof(sample))) {
            return false;
        }

        scope_log_nvme_cmd_head("RAW", sq, slot, guest_pa, attempt + 1U, &sample);

        sample_plausible =
            !scope_nvme_cmd_is_zero(&sample) &&
            scope_admin_cmd_looks_plausible(&sample);

        if (!have_sample || memcmp(cmd, &sample, sizeof(sample)) != 0) {
            if (have_sample) {
                printf("[SCOPE PROXY][CMD][RAW][RETRY_UPDATE] qid=%u slot=%u "
                       "guest_pa=0x%016" PRIx64 " attempt=%u\n",
                       sq ? sq->qid : 0U, slot, guest_pa, attempt + 1U);
                fflush(stdout);
                scope_log_nvme_cmd("RAW_RETRY", sq, slot, guest_pa, &sample);
                scope_log_nvme_cmd_dwords("RAW_RETRY", sq, slot, guest_pa, &sample);
                scope_log_nvme_cmd_hexdump("RAW_RETRY", sq, slot, guest_pa, &sample);
            }

            *cmd = sample;
            have_sample = true;
            stable_plausible_reads = sample_plausible ? 1U : 0U;
            continue;
        }

        if (sample_plausible) {
            stable_plausible_reads++;
            if (stable_plausible_reads >= 2U) {
                if (attempt) {
                    printf("[SCOPE PROXY][CMD][RAW][RETRY_STABLE] qid=%u slot=%u "
                           "guest_pa=0x%016" PRIx64 " attempt=%u\n",
                           sq ? sq->qid : 0U, slot, guest_pa, attempt + 1U);
                    fflush(stdout);
                }
                return true;
            }
        }
    }

    printf("[SCOPE PROXY][CMD][RAW][RETRY_EXHAUSTED] qid=%u slot=%u "
           "guest_pa=0x%016" PRIx64 " zero=%d plausible=%d matches_seed=%d attempts=%u "
           "waited_us=%u\n",
           sq ? sq->qid : 0U, slot, guest_pa, scope_nvme_cmd_is_zero(cmd),
           scope_admin_cmd_looks_plausible(cmd),
           sq && sq->seed_valid && sq->seed_guest_pa == guest_pa &&
           memcmp(cmd, &sq->seed_cmd, sizeof(*cmd)) == 0,
           SCOPE_ADMIN_SQE_RETRY_MAX,
           SCOPE_ADMIN_SQE_INITIAL_DELAY_US +
           (SCOPE_ADMIN_SQE_RETRY_MAX > 0U ?
            (SCOPE_ADMIN_SQE_RETRY_MAX - 1U) * SCOPE_ADMIN_SQE_RETRY_DELAY_US : 0U));
    fflush(stdout);
    return have_sample &&
           !scope_nvme_cmd_is_zero(cmd) &&
           scope_admin_cmd_looks_plausible(cmd);
}

static bool scope_refresh_admin_queue_state(ScopeProxyState *s)
{
    uint64_t translated_asq = 0;
    uint64_t translated_acq = 0;
    uint16_t sq_depth = NVME_AQA_ASQS(s->guest_aqa) + 1;
    uint16_t cq_depth = NVME_AQA_ACQS(s->guest_aqa) + 1;

    memset(&s->sq[SCOPE_ADMIN_QID], 0, sizeof(s->sq[SCOPE_ADMIN_QID]));
    memset(&s->cq[SCOPE_ADMIN_QID], 0, sizeof(s->cq[SCOPE_ADMIN_QID]));

    if (!s->guest_asq || !s->guest_acq || !sq_depth || !cq_depth) {
        return true;
    }
    if (!scope_translate_guest_pa(s, s->guest_asq, 1, &translated_asq) ||
        !scope_translate_guest_pa(s, s->guest_acq, 1, &translated_acq)) {
        return false;
    }

    s->sq[SCOPE_ADMIN_QID].valid = true;
    s->sq[SCOPE_ADMIN_QID].qid = SCOPE_ADMIN_QID;
    s->sq[SCOPE_ADMIN_QID].depth = sq_depth;
    s->sq[SCOPE_ADMIN_QID].linked_cqid = SCOPE_ADMIN_QID;
    s->sq[SCOPE_ADMIN_QID].guest_base = s->guest_asq;
    s->sq[SCOPE_ADMIN_QID].translated_base = translated_asq;

    s->cq[SCOPE_ADMIN_QID].valid = true;
    s->cq[SCOPE_ADMIN_QID].interrupt_enabled = true;
    s->cq[SCOPE_ADMIN_QID].qid = SCOPE_ADMIN_QID;
    s->cq[SCOPE_ADMIN_QID].depth = cq_depth;
    s->cq[SCOPE_ADMIN_QID].shadow_tail = 0;
    s->cq[SCOPE_ADMIN_QID].phase = true;
    s->cq[SCOPE_ADMIN_QID].guest_base = s->guest_acq;
    s->cq[SCOPE_ADMIN_QID].translated_base = translated_acq;

    scope_capture_admin_seed(s);

    return true;
}

static bool scope_sync_admin_regs_to_real(ScopeProxyState *s)
{
    uint64_t translated_asq = 0;
    uint64_t translated_acq = 0;

    if (!scope_real_bar_write32(s, NVME_REG_AQA, s->guest_aqa)) {
        return false;
    }

    if (s->guest_asq) {
        if (!scope_translate_guest_pa(s, s->guest_asq, 1, &translated_asq)) {
            return false;
        }
        if (!scope_real_bar_write32(s, NVME_REG_ASQ, (uint32_t)translated_asq) ||
            !scope_real_bar_write32(s, NVME_REG_ASQ + 4,
                                    (uint32_t)(translated_asq >> 32))) {
            return false;
        }
    }

    if (s->guest_acq) {
        if (!scope_translate_guest_pa(s, s->guest_acq, 1, &translated_acq)) {
            return false;
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
    if (!scope_translate_guest_pa(s, guest_base, 1, &translated)) {
        return false;
    }

    memset(&s->cq[qid], 0, sizeof(s->cq[qid]));
    s->cq[qid].valid = true;
    s->cq[qid].interrupt_enabled = interrupt_enabled;
    s->cq[qid].qid = qid;
    s->cq[qid].depth = depth;
    s->cq[qid].shadow_tail = 0;
    s->cq[qid].phase = true;
    s->cq[qid].guest_base = guest_base;
    s->cq[qid].translated_base = translated;
    return true;
}

static bool scope_register_sq(ScopeProxyState *s, uint16_t qid, uint16_t cqid,
                              uint64_t guest_base, uint16_t depth)
{
    uint64_t translated = 0;

    if (!qid || qid >= SCOPE_MAX_NVME_QUEUES || !depth) {
        return false;
    }
    if (!scope_translate_guest_pa(s, guest_base, 1, &translated)) {
        return false;
    }

    memset(&s->sq[qid], 0, sizeof(s->sq[qid]));
    s->sq[qid].valid = true;
    s->sq[qid].qid = qid;
    s->sq[qid].depth = depth;
    s->sq[qid].linked_cqid = cqid;
    s->sq[qid].guest_base = guest_base;
    s->sq[qid].translated_base = translated;
    return true;
}

static bool scope_patch_common_command_buffers(ScopeProxyState *s, NvmeCmd *cmd)
{
    uint64_t translated = 0;
    uint8_t psdt = NVME_CMD_FLAGS_PSDT(cmd->flags);

    if (cmd->mptr) {
        if (!scope_translate_guest_pa(s, cmd->mptr, 1, &translated)) {
            printf("[SCOPE PROXY][CMD][PATCH][ERR] opcode=0x%02x cid=%u mptr=0x%016"
                   PRIx64 " translation failed\n",
                   cmd->opcode, le16_to_cpu(cmd->cid), le64_to_cpu(cmd->mptr));
            fflush(stdout);
            return false;
        }
        cmd->mptr = translated;
    }

    if (psdt != NVME_PSDT_PRP) {
        if (cmd->dptr.prp1 || cmd->dptr.prp2) {
            printf("[SCOPE PROXY][CMD][PATCH][ERR] opcode=0x%02x cid=%u unsupported "
                   "PSDT=%u with prp1=0x%016" PRIx64 " prp2=0x%016" PRIx64 "\n",
                   cmd->opcode, le16_to_cpu(cmd->cid), psdt,
                   le64_to_cpu(cmd->dptr.prp1), le64_to_cpu(cmd->dptr.prp2));
            fflush(stdout);
        }
        return (cmd->dptr.prp1 == 0 && cmd->dptr.prp2 == 0);
    }

    if (cmd->dptr.prp1) {
        if (!scope_translate_guest_pa(s, cmd->dptr.prp1, 1, &translated)) {
            printf("[SCOPE PROXY][CMD][PATCH][ERR] opcode=0x%02x cid=%u prp1=0x%016"
                   PRIx64 " translation failed\n",
                   cmd->opcode, le16_to_cpu(cmd->cid), le64_to_cpu(cmd->dptr.prp1));
            fflush(stdout);
            return false;
        }
        cmd->dptr.prp1 = translated;
    }
    if (cmd->dptr.prp2) {
        if (!scope_translate_guest_pa(s, cmd->dptr.prp2, 1, &translated)) {
            printf("[SCOPE PROXY][CMD][PATCH][ERR] opcode=0x%02x cid=%u prp2=0x%016"
                   PRIx64 " translation failed\n",
                   cmd->opcode, le16_to_cpu(cmd->cid), le64_to_cpu(cmd->dptr.prp2));
            fflush(stdout);
            return false;
        }
        cmd->dptr.prp2 = translated;
    }

    return true;
}

static void scope_stage_pending_admin_op(ScopeProxyState *s, uint16_t cid,
                                         const ScopePendingAdminOp *op)
{
    if (!s->pending_admin_ops || !op || !op->valid) {
        return;
    }

    s->pending_admin_ops[cid] = *op;
}

static bool scope_admin_cqe_success(const NvmeCqe *cqe)
{
    return (le16_to_cpu(cqe->status) >> 1) == NVME_SUCCESS;
}

static void scope_commit_pending_admin_op(ScopeProxyState *s, const NvmeCqe *cqe)
{
    ScopePendingAdminOp *op;
    uint16_t cid;
    bool ok = true;

    if (!s->pending_admin_ops) {
        return;
    }

    cid = le16_to_cpu(cqe->cid);
    op = &s->pending_admin_ops[cid];
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
                memset(&s->cq[op->qid], 0, sizeof(s->cq[op->qid]));
            } else {
                ok = false;
            }
            break;
        case SCOPE_ADMIN_TOPO_OP_DELETE_SQ:
            if (op->qid < SCOPE_MAX_NVME_QUEUES) {
                memset(&s->sq[op->qid], 0, sizeof(s->sq[op->qid]));
            } else {
                ok = false;
            }
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
        uint64_t guest_base = cq->prp1;
        uint64_t translated = 0;

        if (!scope_translate_guest_pa(s, guest_base, 1, &translated)) {
            printf("[SCOPE PROXY][CMD][ADMIN][ERR] CREATE_CQ cid=%u cqid=%u qsize=%u "
                   "guest_base=0x%016" PRIx64 " translation failed\n",
                   le16_to_cpu(cq->cid), le16_to_cpu(cq->cqid), le16_to_cpu(cq->qsize) + 1U,
                   le64_to_cpu(cq->prp1));
            fflush(stdout);
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
        uint64_t guest_base = sq->prp1;
        uint64_t translated = 0;

        if (!scope_translate_guest_pa(s, guest_base, 1, &translated)) {
            printf("[SCOPE PROXY][CMD][ADMIN][ERR] CREATE_SQ cid=%u sqid=%u cqid=%u "
                   "qsize=%u guest_base=0x%016" PRIx64 " translation failed\n",
                   le16_to_cpu(sq->cid), le16_to_cpu(sq->sqid), le16_to_cpu(sq->cqid),
                   le16_to_cpu(sq->qsize) + 1U, le64_to_cpu(sq->prp1));
            fflush(stdout);
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
            printf("[SCOPE PROXY][CMD][ADMIN][ERR] DELETE_CQ cid=%u invalid qid=%u\n",
                   le16_to_cpu(del->cid), le16_to_cpu(del->qid));
            fflush(stdout);
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
            printf("[SCOPE PROXY][CMD][ADMIN][ERR] DELETE_SQ cid=%u invalid qid=%u\n",
                   le16_to_cpu(del->cid), le16_to_cpu(del->qid));
            fflush(stdout);
            return false;
        }
        pending_op->valid = true;
        pending_op->type = SCOPE_ADMIN_TOPO_OP_DELETE_SQ;
        pending_op->qid = del->qid;
        return true;
    }
    default:
        return scope_patch_common_command_buffers(s, cmd);
    }
}

static bool scope_patch_io_cmd(ScopeProxyState *s, NvmeCmd *cmd)
{
    return scope_patch_common_command_buffers(s, cmd);
}

static bool scope_process_new_sq_entries(ScopeProxyState *s, ScopeSqState *sq,
                                         uint16_t new_tail)
{
    uint16_t cursor;

    if (!sq->valid || !sq->depth || new_tail >= sq->depth) {
        printf("[SCOPE PROXY][SQ][ERR] invalid SQ state qid=%u valid=%d depth=%u "
               "last_tail=%u new_tail=%u\n",
               sq ? sq->qid : 0U, sq ? sq->valid : 0, sq ? sq->depth : 0U,
               sq ? sq->last_guest_tail : 0U, new_tail);
        fflush(stdout);
        return false;
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
            if (!scope_read_admin_sqe_with_retry(s, sq, cursor, cmd_guest_pa, &cmd)) {
                printf("[SCOPE PROXY][SQ][ERR] failed to read stable admin SQE qid=%u slot=%u "
                       "guest_pa=0x%016" PRIx64 " depth=%u last_tail=%u new_tail=%u\n",
                       sq->qid, cursor, cmd_guest_pa, sq->depth, sq->last_guest_tail, new_tail);
                fflush(stdout);
                return false;
            }
        } else if (!scope_guest_mem_read(s, cmd_guest_pa, &cmd, sizeof(cmd))) {
            printf("[SCOPE PROXY][SQ][ERR] failed to read SQE qid=%u slot=%u "
                   "guest_pa=0x%016" PRIx64 " depth=%u last_tail=%u new_tail=%u\n",
                   sq->qid, cursor, cmd_guest_pa, sq->depth, sq->last_guest_tail, new_tail);
            fflush(stdout);
            return false;
        }

        if (sq->qid == SCOPE_ADMIN_QID) {
            have_translation =
                scope_guest_range_to_bar_offset(s, cmd_guest_pa, sizeof(cmd), &cmd_bar_offset);
            if (have_translation) {
                cmd_translated_pa = s->fpga_bypass_bar_base + cmd_bar_offset;
                scope_log_guest_translation("RAW", sq, cursor, cmd_guest_pa,
                                            sizeof(cmd), cmd_bar_offset, cmd_translated_pa);
            }
            scope_log_nvme_cmd("RAW", sq, cursor, cmd_guest_pa, &cmd);
            scope_log_nvme_cmd_dwords("RAW", sq, cursor, cmd_guest_pa, &cmd);
            scope_log_nvme_cmd_hexdump("RAW", sq, cursor, cmd_guest_pa, &cmd);
        }

        ok = (sq->qid == SCOPE_ADMIN_QID) ?
            scope_patch_admin_cmd(s, &cmd, &pending_admin_op) :
            scope_patch_io_cmd(s, &cmd);
        if (!ok) {
            scope_log_nvme_cmd("PATCH_ERR", sq, cursor, cmd_guest_pa, &cmd);
            scope_log_nvme_cmd_dwords("PATCH_ERR", sq, cursor, cmd_guest_pa, &cmd);
            scope_log_nvme_cmd_hexdump("PATCH_ERR", sq, cursor, cmd_guest_pa, &cmd);
            return false;
        }

        if (!scope_guest_mem_write(s, cmd_guest_pa, &cmd, sizeof(cmd))) {
            scope_log_nvme_cmd("WRITEBACK_ERR", sq, cursor, cmd_guest_pa, &cmd);
            return false;
        }
        if (sq->qid == SCOPE_ADMIN_QID &&
            scope_guest_mem_read(s, cmd_guest_pa, &cmd_visible, sizeof(cmd_visible))) {
            if (memcmp(&cmd, &cmd_visible, sizeof(cmd)) != 0) {
                printf("[SCOPE PROXY][CMD][VISIBLE_AFTER_WRITEBACK][ERR] qid=%u slot=%u "
                       "guest_pa=0x%016" PRIx64 "\n",
                       sq->qid, cursor, cmd_guest_pa);
                fflush(stdout);
                scope_log_nvme_cmd("VISIBLE_AFTER_WRITEBACK", sq, cursor, cmd_guest_pa,
                                   &cmd_visible);
                scope_log_nvme_cmd_dwords("VISIBLE_AFTER_WRITEBACK", sq, cursor, cmd_guest_pa,
                                          &cmd_visible);
                scope_log_nvme_cmd_hexdump("VISIBLE_AFTER_WRITEBACK", sq, cursor, cmd_guest_pa,
                                           &cmd_visible);
            } else if (have_translation) {
                printf("[SCOPE PROXY][CMD][VISIBLE_AFTER_WRITEBACK][OK] qid=%u slot=%u "
                       "guest_pa=0x%016" PRIx64 " translated=0x%016" PRIx64 "\n",
                       sq->qid, cursor, cmd_guest_pa, cmd_translated_pa);
                fflush(stdout);
            }
        }
        if (sq->qid == SCOPE_ADMIN_QID && pending_admin_op.valid) {
            scope_stage_pending_admin_op(s, le16_to_cpu(cmd.cid), &pending_admin_op);
        }

        cursor = (cursor + 1U) % sq->depth;
    }

    sq->last_guest_tail = new_tail;
    return true;
}

static bool scope_is_doorbell_offset(ScopeProxyState *s, uint32_t aligned_offset,
                                     bool *is_sq, uint16_t *qid)
{
    uint32_t rel;
    uint32_t index;

    if (!s->doorbell_stride || aligned_offset < SCOPE_NVME_DOORBELL_BASE) {
        return false;
    }

    rel = aligned_offset - SCOPE_NVME_DOORBELL_BASE;
    if (rel % s->doorbell_stride) {
        return false;
    }

    index = rel / s->doorbell_stride;
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

static bool scope_cqe_phase_matches(const NvmeCqe *cqe, bool expected_phase)
{
    return !!(le16_to_cpu(cqe->status) & 0x1U) == expected_phase;
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
        if (cq->qid == SCOPE_ADMIN_QID) {
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
    unsigned i;

    for (i = 0; i < SCOPE_MAX_NVME_QUEUES; i++) {
        bool advanced = false;

        if (!scope_refresh_cq_shadow_tail(s, &s->cq[i], &advanced)) {
            continue;
        }
        new_completion |= advanced;
        pending |= scope_cq_has_pending(&s->cq[i]);
    }

    if (s->guest_int_mask & 0x1U) {
        pending = false;
    }

    if (pending && !s->virtual_intx_level) {
        if (scope_virtual_rp_set_intx(s, true)) {
            printf("[SCOPE PROXY][INTX] assert pending completion(s) detected\n");
            fflush(stdout);
        }
    } else if (!pending && s->virtual_intx_level) {
        if (scope_virtual_rp_set_intx(s, false)) {
            printf("[SCOPE PROXY][INTX] deassert no pending completion\n");
            fflush(stdout);
        }
    } else if (new_completion && pending) {
        printf("[SCOPE PROXY][INTX] completion observed while level already asserted\n");
        fflush(stdout);
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

    printf("[SCOPE PROXY][BAR][WR] seq=%u off=0x%04x reg=%s flags=0x%08x size=%u wstrb=0x%02x data=0x%016" PRIx64 " result=%s\n",
           seq, offset, reg_name, flags, size_bytes, wstrb, data, ok ? "OK" : "ERR");
    fflush(stdout);
}

static void scope_log_bar_read(ScopeProxyState *s, uint32_t seq, uint32_t offset,
                               uint32_t flags, uint8_t size_bytes, uint64_t data, uint32_t resp)
{
    char reg_buf[64];
    const char *reg_name = scope_nvme_reg_name(s, offset & ~0x3U, reg_buf, sizeof(reg_buf));

    printf("[SCOPE PROXY][BAR][RD] seq=%u off=0x%04x reg=%s flags=0x%08x size=%u resp=0x%x data=0x%016" PRIx64 "\n",
           seq, offset, reg_name, flags, size_bytes, resp, data);
    fflush(stdout);
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
        printf("[SCOPE PROXY][DB][ERR] invalid doorbell write off=0x%04x size=%u "
               "wstrb=0x%02x data=0x%016" PRIx64 "\n",
               offset, size_bytes, dword_wstrb, data);
        fflush(stdout);
        return false;
    }
    if (!scope_is_doorbell_offset(s, aligned_offset, &is_sq, &qid) ||
        qid >= SCOPE_MAX_NVME_QUEUES) {
        printf("[SCOPE PROXY][DB][ERR] unknown doorbell off=0x%04x aligned=0x%04x "
               "data=0x%016" PRIx64 "\n",
               offset, aligned_offset, data);
        fflush(stdout);
        return false;
    }

    if (is_sq) {
        ScopeSqState *sq = &s->sq[qid];
        uint16_t new_tail = dword_data & 0xFFFFU;

        if (!scope_process_new_sq_entries(s, sq, new_tail)) {
            printf("[SCOPE PROXY][DB][SQ][ERR] qid=%u last_tail=%u new_tail=%u "
                   "depth=%u guest_base=0x%016" PRIx64 "\n",
                   qid, sq->last_guest_tail, new_tail, sq->depth, sq->guest_base);
            fflush(stdout);
            return false;
        }
    } else {
        ScopeCqState *cq = &s->cq[qid];

        if (cq->valid) {
            cq->last_guest_head = dword_data & 0xFFFFU;
        }
        scope_update_virtual_intx(s);
    }

    if (!scope_real_bar_write(s, aligned_offset, data, wstrb, size_bytes)) {
        printf("[SCOPE PROXY][DB][ERR] failed to forward real doorbell off=0x%04x "
               "aligned=0x%04x data=0x%016" PRIx64 " wstrb=0x%02x\n",
               offset, aligned_offset, data, wstrb);
        fflush(stdout);
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
            s->guest_asq = scope_apply_wstrb64(s->guest_asq, data, wstrb);
        } else {
            scope_update_shadow_u64(&s->guest_asq, aligned_offset == (NVME_REG_ASQ + 4),
                                    dword_data, wstrb);
        }
        return scope_sync_admin_regs_to_real(s);
    }

    if (qword_offset == NVME_REG_ACQ) {
        if (size_bytes == 8 && (offset & 0x7U) == 0) {
            s->guest_acq = scope_apply_wstrb64(s->guest_acq, data, wstrb);
        } else {
            scope_update_shadow_u64(&s->guest_acq, aligned_offset == (NVME_REG_ACQ + 4),
                                    dword_data, wstrb);
        }
        return scope_sync_admin_regs_to_real(s);
    }

    switch (aligned_offset) {
    case NVME_REG_CC: {
        uint32_t old_cc = s->guest_cc;

        s->guest_cc = scope_apply_wstrb32(s->guest_cc, dword_data, dword_wstrb);
        if (!scope_sync_admin_regs_to_real(s) ||
            !scope_real_bar_write32(s, NVME_REG_CC, s->guest_cc)) {
            return false;
        }

        if (NVME_CC_EN(old_cc) && !NVME_CC_EN(s->guest_cc)) {
            if (s->virtual_intx_level) {
                scope_virtual_rp_set_intx(s, false);
            }
            scope_reset_all_queue_state(s);
        } else if (!NVME_CC_EN(old_cc) && NVME_CC_EN(s->guest_cc)) {
            scope_reset_all_queue_state(s);
            if (!scope_refresh_admin_queue_state(s)) {
                return false;
            }
        }
        return true;
    }
    case NVME_REG_AQA:
        s->guest_aqa = scope_apply_wstrb32(s->guest_aqa, dword_data, dword_wstrb);
        return scope_sync_admin_regs_to_real(s);
    
    case NVME_REG_INTMS:
        s->guest_int_mask |= scope_apply_wstrb32(0, dword_data, dword_wstrb);
        scope_update_virtual_intx(s);
        return scope_real_bar_write(s, aligned_offset, data, wstrb, size_bytes);
    case NVME_REG_INTMC:
        s->guest_int_mask &= ~scope_apply_wstrb32(0, dword_data, dword_wstrb);
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
        *data = s->nvme_cap;
        return true;
    }
    if (qword_offset == NVME_REG_ASQ) {
        *data = s->guest_asq;
        return true;
    }
    if (qword_offset == NVME_REG_ACQ) {
        *data = s->guest_acq;
        return true;
    }

    switch (aligned_offset) {
    case NVME_REG_VS:
        *data = scope_pack_dword32_for_offset(s->nvme_vs, offset);
        return true;
    case NVME_REG_CC:
        *data = scope_pack_dword32_for_offset(s->guest_cc, offset);
        return true;
    case NVME_REG_AQA:
        *data = scope_pack_dword32_for_offset(s->guest_aqa, offset);
        return true;
    case NVME_REG_INTMS:
    case NVME_REG_INTMC:
        *data = scope_pack_dword32_for_offset(s->guest_int_mask, offset);
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

static void scope_process_bar_packet(ScopeProxyState *s, const struct scope_dma32_packet *pkt)
{
    uint32_t resp = 0x2U;
    uint64_t data = 0;
    bool ok = false;
    /*
     * RTL packs BAR packet metadata as:
     *   flags[7:0]   = wstrb
     *   flags[15:8]  = size_bytes
     * BAR packets reuse guest_addr_lo as the upper 32 bits of the 64-bit lane.
     */
    uint8_t size_bytes = (pkt->flags >> 8) & 0xFFU;
    uint8_t wstrb = pkt->flags & 0xFFU;
    uint64_t lane_data = ((uint64_t)pkt->guest_addr_lo << 32) | pkt->data;

    switch (pkt->type) {
    case SCOPE_PKT_TYPE_BAR_WRITE:
        ok = scope_handle_nvme_bar_write(s, pkt->bar_offset, lane_data, wstrb, size_bytes);
        resp = ok ? 0x0U : 0x2U;
        scope_log_bar_write(s, pkt->seq, pkt->bar_offset, pkt->flags, size_bytes, wstrb,
                            lane_data, ok);
        if (!scope_write_bar_response(s, pkt->seq, resp, 0, false)) {
            qemu_log_mask(LOG_GUEST_ERROR, "SCOPE: Failed to write BAR write response\n");
        }
        break;
    case SCOPE_PKT_TYPE_BAR_READ:
        ok = scope_handle_nvme_bar_read(s, pkt->bar_offset, size_bytes, &data);
        resp = ok ? 0x0U : 0x2U;
        scope_log_bar_read(s, pkt->seq, pkt->bar_offset, pkt->flags, size_bytes, data, resp);
        if (!scope_write_bar_response(s, pkt->seq, resp, data, true)) {
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
    uint32_t mailbox_status = 0;
    uint32_t mailbox_awaddr = 0;
    uint32_t mailbox_wdata = 0;
    uint32_t mailbox_wstrb = 0;
    uint32_t meta = pkt->guest_addr_lo;
    uint32_t wstrb = meta & 0x0FU;
    uint32_t addr = pkt->bar_offset;
    uint32_t mask = wstrb;
    uint32_t val;
    uint32_t actual_addr;
    uint32_t config_limit = pci_config_size(pci_dev);
    int offset = 0;
    int len = 0;
    bool mailbox_match = false;
    Error *local_err = NULL;

    /*
     * CFG_WRITE packets are emitted together with the mailbox state machine in
     * the RTL. Prefer handling them here, but only if the current mailbox
     * payload still matches this packet so we do not double-consume a request
     * already processed by the IRQ/mailbox path.
     */
    qemu_mutex_lock(&s->xdma_lock);
    mailbox_match =
        scope_xdma_read32_locked(s, HOST_MBX_BASE + MBX_REG_STATUS, &mailbox_status) &&
        scope_xdma_read32_locked(s, HOST_MBX_BASE + MBX_REG_AWADDR, &mailbox_awaddr) &&
        scope_xdma_read32_locked(s, HOST_MBX_BASE + MBX_REG_WDATA, &mailbox_wdata) &&
        scope_xdma_read32_locked(s, HOST_MBX_BASE + MBX_REG_WSTRB, &mailbox_wstrb);
    qemu_mutex_unlock(&s->xdma_lock);

    mailbox_match = mailbox_match &&
                    mailbox_status == 1U &&
                    mailbox_awaddr == addr &&
                    mailbox_wdata == pkt->data &&
                    (mailbox_wstrb & 0x0FU) == wstrb;

    if (!mailbox_match) {
        printf("[SCOPE PROXY][CFG][C2H] seq=%u ignored due to mailbox mismatch "
               "(mbx_status=0x%x mbx_awaddr=0x%03x mbx_wdata=0x%08x mbx_wstrb=0x%x "
               "pkt_addr=0x%03x pkt_wdata=0x%08x pkt_wstrb=0x%x)\n",
               pkt->seq, mailbox_status, mailbox_awaddr, mailbox_wdata,
               mailbox_wstrb & 0x0FU, addr, pkt->data, wstrb);
        fflush(stdout);
        return;
    }

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

    scope_log_config_write("C2H", pkt->seq, actual_addr, len, wstrb, val);

    if (actual_addr + (uint32_t)len <= config_limit) {
        bql_lock();
        pci_default_write_config(pci_dev, actual_addr, val, len);

        if (!scope_sync_vconf_to_fpga(s, pci_dev, &local_err) ||
            !scope_sync_guest_bar_shadow(s, pci_dev, &local_err)) {
            if (local_err) {
                qemu_log_mask(LOG_GUEST_ERROR, "SCOPE: %s\n",
                              error_get_pretty(local_err));
                error_free(local_err);
                local_err = NULL;
            }
        }
        bql_unlock();
    }

    if (!scope_ack_cfg_packet(s, pkt->seq)) {
        qemu_log_mask(LOG_GUEST_ERROR, "SCOPE: Failed to ACK CFG packet seq=%u\n",
                      pkt->seq);
    }
}

static bool scope_poll_dma32_ring(ScopeProxyState *s, uint32_t last_seq_by_type[4])
{
    const size_t slot_size = sizeof(struct scope_dma32_packet);
    const size_t slot_count = s->dma32_db.size / slot_size;
    uint8_t *ring_base = s->dma32_db_map;
    bool progressed = false;
    size_t i;

    for (i = 0; i < slot_count; i++) {
        struct scope_dma32_packet pkt;

        if (!scope_read_stable_packet(ring_base + i * slot_size, &pkt)) {
            continue;
        }
        if (pkt.magic != XDMA_DMA32_PKT_MAGIC ||
            pkt.len != sizeof(pkt) ||
            pkt.seq == 0 ||
            pkt.type > SCOPE_PKT_TYPE_BAR_READ) {
            continue;
        }
        if (pkt.seq <= last_seq_by_type[pkt.type]) {
            continue;
        }

        last_seq_by_type[pkt.type] = pkt.seq;
        progressed = true;

        if (pkt.type == SCOPE_PKT_TYPE_CFG_WRITE) {
            scope_process_cfg_packet(s, &pkt);
        } else if (pkt.type == SCOPE_PKT_TYPE_BAR_WRITE ||
                   pkt.type == SCOPE_PKT_TYPE_BAR_READ) {
            scope_process_bar_packet(s, &pkt);
        }
    }

    return progressed;
}

static void *scope_proxy_rx_thread(void *opaque)
{
    ScopeProxyState *s = SCOPE_PROXY(opaque);
    uint32_t last_seq_by_type[4] = { 0 };

    while (!qatomic_read(&s->rx_thread_stop)) {
        bool progressed = scope_poll_dma32_ring(s, last_seq_by_type);

        scope_update_virtual_intx(s);

        if (!progressed) {
            g_usleep(50);
        }
    }

    return NULL;
}

static bool scope_alloc_dma32_ring(ScopeProxyState *s, Error **errp)
{
    off_t mmap_offset;

    s->dma32_db.size = SCOPE_DMA32_RING_SIZE;
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

    if (!s->real_host_bdf || !s->real_host_bdf[0]) {
        error_setg(errp, "Property real-host-bdf is required");
        return false;
    }

    if (!scope_read_pci_bar_resource(s->real_host_bdf, 0, &res, errp)) {
        return false;
    }
    if (!(res.flags & IORESOURCE_MEM)) {
        error_setg(errp, "Real BAR0 of %s is not a memory BAR", s->real_host_bdf);
        return false;
    }
    if (res.end < res.start) {
        error_setg(errp, "Real BAR0 of %s has invalid range", s->real_host_bdf);
        return false;
    }

    s->real_bar0_size = (size_t)(res.end - res.start + 1);
    s->real_bar0_flags = res.flags;

    if (!s->real_bar0_size) {
        error_setg(errp, "Real BAR0 of %s has zero size", s->real_host_bdf);
        return false;
    }
    if (s->real_bar0_size > SCOPE_RP_BAR_APERTURE_SIZE) {
        error_setg(errp, "Real BAR0 size 0x%zx exceeds RP BAR aperture 0x%x",
                   s->real_bar0_size, SCOPE_RP_BAR_APERTURE_SIZE);
        return false;
    }

    resource0_path = g_strdup_printf("/sys/bus/pci/devices/%s/resource0", s->real_host_bdf);
    s->real_bar_fd = open(resource0_path, O_RDWR | O_SYNC);
    if (s->real_bar_fd < 0) {
        error_setg_errno(errp, errno, "Failed to open %s", resource0_path);
        return false;
    }

    s->real_bar0_map = mmap(NULL, s->real_bar0_size, PROT_READ | PROT_WRITE,
                            MAP_SHARED, s->real_bar_fd, 0);
    if (s->real_bar0_map == MAP_FAILED) {
        s->real_bar0_map = NULL;
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
    if (s->guest_ddr_size && s->fpga_bypass_bar_size < s->guest_ddr_size) {
        error_setg(errp, "FPGA bypass BAR size 0x%" PRIx64
                   " is smaller than guest-ddr-size 0x%" PRIx64,
                   s->fpga_bypass_bar_size, s->guest_ddr_size);
        return false;
    }

    return true;
}

static bool scope_init_nvme_capability_cache(ScopeProxyState *s, Error **errp)
{
    uint32_t cap_lo = 0;
    uint32_t cap_hi = 0;

    if (!scope_real_bar_read32(s, NVME_REG_CAP, &cap_lo) ||
        !scope_real_bar_read32(s, NVME_REG_CAP + 4, &cap_hi) ||
        !scope_real_bar_read32(s, NVME_REG_VS, &s->nvme_vs)) {
        error_setg(errp, "Failed to read real NVMe capability registers");
        return false;
    }

    s->nvme_cap = ((uint64_t)cap_hi << 32) | cap_lo;
    s->doorbell_stride = 4U << NVME_CAP_DSTRD(s->nvme_cap);
    if (!s->doorbell_stride) {
        error_setg(errp, "Invalid NVMe CAP.DSTRD");
        return false;
    }

    return true;
}

static void scope_hardware_interrupt_cb(void *opaque)
{
    ScopeProxyState *s = SCOPE_PROXY(opaque);
    PCIDevice *pci_dev = PCI_DEVICE(s);
    uint32_t events_count = 0;
    uint32_t dummy_ack = 1;
    uint32_t status = 0;
    uint32_t awaddr = 0;
    uint32_t wdata = 0;
    uint32_t wstrb = 0;
    uint32_t val = 0;
    uint32_t config_limit = pci_config_size(pci_dev);
    int len = 4;
    Error *local_err = NULL;

    if (read(s->event_fd, &events_count, sizeof(events_count)) <= 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "SCOPE: Failed to read eventfd\n");
        return;
    }

    qemu_mutex_lock(&s->xdma_lock);
    if (!scope_xdma_read32_locked(s, HOST_MBX_BASE + MBX_REG_STATUS, &status) ||
        !scope_xdma_read32_locked(s, HOST_MBX_BASE + MBX_REG_AWADDR, &awaddr) ||
        !scope_xdma_read32_locked(s, HOST_MBX_BASE + MBX_REG_WDATA, &wdata) ||
        !scope_xdma_read32_locked(s, HOST_MBX_BASE + MBX_REG_WSTRB, &wstrb)) {
        qemu_mutex_unlock(&s->xdma_lock);
        qemu_log_mask(LOG_GUEST_ERROR, "SCOPE: Failed to read mailbox payload\n");
        goto send_ack;
    }
    qemu_mutex_unlock(&s->xdma_lock);

    if (status != 1) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE: IRQ fired but mailbox status=%u, force ACK to avoid deadlock\n",
                      status);
        goto send_ack;
    }

    val = wdata;
    if (wstrb == 0x1U) {
        len = 1;
        val = wdata & 0xFFU;
    } else if (wstrb == 0x2U) {
        len = 1;
        val = (wdata >> 8) & 0xFFU;
    } else if (wstrb == 0x4U) {
        len = 1;
        val = (wdata >> 16) & 0xFFU;
    } else if (wstrb == 0x8U) {
        len = 1;
        val = (wdata >> 24) & 0xFFU;
    } else if (wstrb == 0x3U) {
        len = 2;
        val = wdata & 0xFFFFU;
    } else if (wstrb == 0xCU) {
        len = 2;
        val = (wdata >> 16) & 0xFFFFU;
    }

    if (awaddr + len <= config_limit) {
        scope_log_config_write("IRQ", 0, awaddr, len, wstrb, val);
        pci_default_write_config(pci_dev, awaddr, val, len);

        if (!scope_sync_vconf_to_fpga(s, pci_dev, &local_err) ||
            !scope_sync_guest_bar_shadow(s, pci_dev, &local_err)) {
            if (local_err) {
                qemu_log_mask(LOG_GUEST_ERROR, "SCOPE: %s\n", error_get_pretty(local_err));
                error_free(local_err);
            }
        }
    }

send_ack:
    qemu_mutex_lock(&s->xdma_lock);
    if (!scope_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_ACK, dummy_ack)) {
        qemu_log_mask(LOG_GUEST_ERROR, "SCOPE: Failed to write ACK\n");
    }
    qemu_mutex_unlock(&s->xdma_lock);
}

static void scope_proxy_cleanup(ScopeProxyState *s)
{
    if (s->event_fd >= 0) {
        qemu_set_fd_handler(s->event_fd, NULL, NULL, NULL);
    }

    if (s->rx_thread_started) {
        qatomic_set(&s->rx_thread_stop, 1);
        qemu_thread_join(&s->rx_thread);
        s->rx_thread_started = false;
    }

    if (s->dma32_db_map) {
        munmap(s->dma32_db_map, s->dma32_db.size);
        s->dma32_db_map = NULL;
    }
    if (s->xdma_ctrl_fd >= 0 && s->dma32_db.size) {
        ioctl(s->xdma_ctrl_fd, XDMA_IOC_DMA32_DB_FREE);
        memset(&s->dma32_db, 0, sizeof(s->dma32_db));
    }

    if (s->real_bar0_map) {
        munmap(s->real_bar0_map, s->real_bar0_size);
        s->real_bar0_map = NULL;
    }
    if (s->real_bar_fd >= 0) {
        close(s->real_bar_fd);
        s->real_bar_fd = -1;
    }
    if (s->xdma_bypass_fd >= 0) {
        close(s->xdma_bypass_fd);
        s->xdma_bypass_fd = -1;
    }
    if (s->event_fd >= 0) {
        close(s->event_fd);
        s->event_fd = -1;
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
    g_free(s->pending_admin_ops);
    s->pending_admin_ops = NULL;
}

static void scope_proxy_realize(PCIDevice *pci_dev, Error **errp)
{
    ScopeProxyState *s = SCOPE_PROXY(pci_dev);
    uint32_t dummy_ack = 1;
    const char *xdma_user_dev = s->xdma_user_dev ? s->xdma_user_dev : SCOPE_DEFAULT_XDMA_USER;
    const char *xdma_ctrl_dev = s->xdma_ctrl_dev ? s->xdma_ctrl_dev : SCOPE_DEFAULT_XDMA_CTRL;
    const char *xdma_event_dev = s->xdma_event_dev ? s->xdma_event_dev : SCOPE_DEFAULT_XDMA_EVENT;
    const char *xdma_bypass_dev = s->xdma_bypass_dev ? s->xdma_bypass_dev : SCOPE_DEFAULT_XDMA_BYPASS;

    printf("\n[SCOPE PROXY] Initializing flat-DDR NVMe proxy sandbox device...\n");

    if (!s->guest_ddr_size) {
        error_setg(errp, "Property guest-ddr-size is required");
        return;
    }

    s->host_page_size = sysconf(_SC_PAGESIZE);
    if (!s->host_page_size) {
        s->host_page_size = 4096;
    }

    qemu_mutex_init(&s->xdma_lock);
    s->xdma_lock_inited = true;
    s->pending_admin_ops = g_new0(ScopePendingAdminOp, SCOPE_ADMIN_CID_SPACE);

    if (!scope_parse_real_bar0(s, errp) ||
        !scope_parse_fpga_bypass_bar(s, errp) ||
        !scope_init_nvme_capability_cache(s, errp)) {
        goto fail;
    }

    s->xdma_fd = open(xdma_user_dev, O_RDWR | O_SYNC);
    if (s->xdma_fd < 0) {
        error_setg_errno(errp, errno, "Failed to open %s", xdma_user_dev);
        goto fail;
    }

    s->xdma_ctrl_fd = open(xdma_ctrl_dev, O_RDWR);
    if (s->xdma_ctrl_fd < 0) {
        error_setg_errno(errp, errno, "Failed to open %s", xdma_ctrl_dev);
        goto fail;
    }

    s->event_fd = open(xdma_event_dev, O_RDONLY);
    if (s->event_fd < 0) {
        error_setg_errno(errp, errno, "Failed to open %s", xdma_event_dev);
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
    pci_config_set_device_id(pci_dev->config, 0x0a54);
    pci_config_set_class(pci_dev->config, PCI_CLASS_STORAGE_EXPRESS);
    pci_dev->config[PCI_CLASS_PROG] = 0x02;
    pci_dev->config[PCI_INTERRUPT_PIN] = 0x01;
    pci_dev->config[PCI_INTERRUPT_LINE] = 0xff;

    if (pcie_endpoint_cap_init(pci_dev, 0x70) < 0) {
        error_setg(errp, "Failed to initialize PCIe endpoint capability");
        goto fail;
    }

    memory_region_init_io(&s->dummy_bar0, OBJECT(s), &dummy_bar_ops, s,
                          "scope-bar0", s->real_bar0_size);
    pci_register_bar(pci_dev, 0,
                     PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_64,
                     &s->dummy_bar0);

    if (!scope_sync_vconf_to_fpga(s, pci_dev, errp) ||
        !scope_sync_guest_bar_shadow(s, pci_dev, errp)) {
        goto fail;
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

    qemu_set_fd_handler(s->event_fd, scope_hardware_interrupt_cb, NULL, s);

    printf("[SCOPE PROXY] Real NVMe BAR0 %s mapped, size=0x%zx\n",
           s->real_host_bdf, s->real_bar0_size);
    printf("[SCOPE PROXY] FPGA bypass BAR%d @ 0x%016" PRIx64 ", size=0x%016" PRIx64 "\n",
           s->fpga_bypass_bar_index, s->fpga_bypass_bar_base, s->fpga_bypass_bar_size);
    printf("[SCOPE PROXY] Guest DDR base=0x%016" PRIx64 ", size=0x%016" PRIx64 "\n",
           s->guest_ddr_base, s->guest_ddr_size);
    printf("[SCOPE PROXY] NVMe CAP=0x%016" PRIx64 ", VS=0x%08x, DSTRD=%" PRIu64
           ", doorbell_stride=%u\n",
           s->nvme_cap, s->nvme_vs, NVME_CAP_DSTRD(s->nvme_cap), s->doorbell_stride);
    printf("[SCOPE PROXY] INTx-only virtual device, MSI/MSI-X disabled.\n");
    printf("[SCOPE PROXY] Config IRQ path armed, DMA32 BAR RX thread started.\n\n");
    return;

fail:
    scope_proxy_cleanup(s);
}

static void scope_proxy_exit(PCIDevice *pci_dev)
{
    ScopeProxyState *s = SCOPE_PROXY(pci_dev);

    scope_proxy_cleanup(s);
    printf("[SCOPE PROXY] Device exited and cleaned up.\n");
}

static void scope_proxy_instance_init(Object *obj)
{
    ScopeProxyState *s = SCOPE_PROXY(obj);

    s->xdma_fd = -1;
    s->xdma_ctrl_fd = -1;
    s->event_fd = -1;
    s->xdma_bypass_fd = -1;
    s->real_bar_fd = -1;
    s->real_bar0_map = NULL;
    s->real_bar0_size = 0;
    s->real_bar0_flags = 0;
    s->dma32_db_map = NULL;
    memset(&s->dma32_db, 0, sizeof(s->dma32_db));
    s->xdma_lock_inited = false;
    s->rx_thread_started = false;
    s->rx_thread_stop = 0;
    s->bar_resp_toggle = 0;
    s->fpga_bypass_bar_base = 0;
    s->fpga_bypass_bar_size = 0;
    s->fpga_bypass_bar_index = -1;
    s->guest_ddr_base = 0;
    s->guest_ddr_size = 0;
    s->host_page_size = 0;
    s->nvme_cap = 0;
    s->nvme_vs = 0;
    s->doorbell_stride = 0;
    s->guest_cc = 0;
    s->guest_aqa = 0;
    s->guest_int_mask = 0;
    s->guest_asq = 0;
    s->guest_acq = 0;
    s->virtual_intx_level = false;
    s->pending_admin_ops = NULL;
    scope_reset_all_queue_state(s);
}

static const Property scope_proxy_properties[] = {
    DEFINE_PROP_STRING("real-host-bdf", ScopeProxyState, real_host_bdf),
    DEFINE_PROP_STRING("fpga-host-bdf", ScopeProxyState, fpga_host_bdf),
    DEFINE_PROP_STRING("xdma-user-dev", ScopeProxyState, xdma_user_dev),
    DEFINE_PROP_STRING("xdma-ctrl-dev", ScopeProxyState, xdma_ctrl_dev),
    DEFINE_PROP_STRING("xdma-event-dev", ScopeProxyState, xdma_event_dev),
    DEFINE_PROP_STRING("xdma-bypass-dev", ScopeProxyState, xdma_bypass_dev),
    DEFINE_PROP_UINT64("guest-ddr-base", ScopeProxyState, guest_ddr_base, 0),
    DEFINE_PROP_UINT64("guest-ddr-size", ScopeProxyState, guest_ddr_size, 0),
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
