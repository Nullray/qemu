#include "qemu/osdep.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pcie.h"
#include "hw/core/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/thread.h"
#include "qemu/atomic.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/misc/scope_fpga_proxy_abi.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#define TYPE_SCOPE_MMIO_LAT_TEST "scope-fpga-mmio-lat-test"
OBJECT_DECLARE_SIMPLE_TYPE(ScopeMmioLatTestState, SCOPE_MMIO_LAT_TEST)

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

#define GUEST_BAR0_CTRL_VALID      (1U << 0)
#define GUEST_BAR0_CTRL_MEM_ENABLE (1U << 1)
#define GUEST_BAR0_CTRL_IS64       (1U << 2)
#define PROXY_CTRL_REAL_BAR_READY  (1U << 0)
#define BAR_RESP_CTRL_TOGGLE_SHIFT 2

#define IORESOURCE_MEM             0x00000200ULL
#define SCOPE_DMA32_RING_SIZE      0x1000U
#define SCOPE_DMA32_TYPE_MAX       SCOPE_PKT_TYPE_SQE_WRITE_DONE
#define SCOPE_DEFAULT_XDMA_USER    "/dev/xdma0_user"
#define SCOPE_DEFAULT_XDMA_CTRL    "/dev/xdma0_control"
#define SCOPE_DEFAULT_XDMA_EVENT   "/dev/xdma0_events_0"

typedef struct ScopePciResource {
    uint64_t start;
    uint64_t end;
    uint64_t flags;
} ScopePciResource;

struct ScopeMmioLatTestState {
    PCIDevice parent_obj;

    char *real_host_bdf;
    char *xdma_user_dev;
    char *xdma_ctrl_dev;
    char *xdma_event_dev;

    int xdma_fd;
    int xdma_ctrl_fd;
    int event_fd;
    int real_bar_fd;
    void *real_bar0_map;
    size_t real_bar0_size;

    long host_page_size;
    struct scope_xdma_dma32_doorbell dma32_db;
    void *dma32_db_map;

    QemuMutex xdma_lock;
    bool xdma_lock_inited;
    QemuThread rx_thread;
    bool rx_thread_started;
    int rx_thread_stop;
    uint32_t bar_resp_toggle;
    bool first_bar_write_seen;

    MemoryRegion dummy_bar0;
};

static bool scope_lat_parse_resource_line(const char *line, ScopePciResource *res)
{
    unsigned long long start;
    unsigned long long end;
    unsigned long long flags;

    if (sscanf(line, "%llx %llx %llx", &start, &end, &flags) != 3) {
        return false;
    }

    res->start = start;
    res->end = end;
    res->flags = flags;
    return true;
}

static bool scope_lat_read_pci_bar_resource(const char *bdf, int bar_index,
                                            ScopePciResource *res, Error **errp)
{
    g_autofree char *path = NULL;
    FILE *fp;
    char line[256];
    int i;

    if (!bdf || !*bdf) {
        error_setg(errp, "Property real-host-bdf is required");
        return false;
    }

    path = g_strdup_printf("/sys/bus/pci/devices/%s/resource", bdf);
    fp = fopen(path, "r");
    if (!fp) {
        error_setg_errno(errp, errno, "Failed to open %s", path);
        return false;
    }

    for (i = 0; i <= bar_index; i++) {
        if (!fgets(line, sizeof(line), fp)) {
            fclose(fp);
            error_setg(errp, "Failed to read BAR%d resource from %s",
                       bar_index, path);
            return false;
        }
    }
    fclose(fp);

    if (!scope_lat_parse_resource_line(line, res)) {
        error_setg(errp, "Failed to parse BAR%d resource line for %s",
                   bar_index, bdf);
        return false;
    }
    if (!(res->flags & IORESOURCE_MEM) || res->end < res->start) {
        error_setg(errp, "BAR%d for %s is not a valid MMIO resource",
                   bar_index, bdf);
        return false;
    }
    return true;
}

static bool scope_lat_xdma_write32_locked(ScopeMmioLatTestState *s,
                                          uint32_t offset, uint32_t value)
{
    return pwrite(s->xdma_fd, &value, sizeof(value), offset) == sizeof(value);
}

static bool scope_lat_xdma_read32_locked(ScopeMmioLatTestState *s,
                                         uint32_t offset, uint32_t *value)
{
    return pread(s->xdma_fd, value, sizeof(*value), offset) == sizeof(*value);
}

static void scope_lat_zero_dma32_ring(ScopeMmioLatTestState *s)
{
    if (!s->dma32_db_map || !s->dma32_db.size) {
        return;
    }

    memset(s->dma32_db_map, 0, s->dma32_db.size);
    smp_wmb();
}

static uint64_t scope_lat_now_ns(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
}

static bool scope_lat_sync_vconf_to_fpga(ScopeMmioLatTestState *s,
                                         PCIDevice *pci_dev, Error **errp)
{
    uint32_t *config_ptr = (uint32_t *)pci_dev->config;
    uint32_t i;

    qemu_mutex_lock(&s->xdma_lock);
    for (i = 0; i < 1024; i++) {
        if (!scope_lat_xdma_write32_locked(s, HOST_VCONF_BASE + i * 4,
                                           config_ptr[i])) {
            qemu_mutex_unlock(&s->xdma_lock);
            error_setg_errno(errp, errno,
                             "Failed to sync VCONF dword %u to FPGA", i);
            return false;
        }
    }
    qemu_mutex_unlock(&s->xdma_lock);
    return true;
}

static bool scope_lat_sync_guest_bar_shadow(ScopeMmioLatTestState *s,
                                            PCIDevice *pci_dev, Error **errp)
{
    uint32_t bar0_lo = ldl_le_p(pci_dev->config + PCI_BASE_ADDRESS_0);
    uint32_t bar0_hi = ldl_le_p(pci_dev->config + PCI_BASE_ADDRESS_0 + 4);
    uint16_t command = pci_get_word(pci_dev->config + PCI_COMMAND);
    bool is_mem = (bar0_lo & PCI_BASE_ADDRESS_SPACE) == PCI_BASE_ADDRESS_SPACE_MEMORY;
    bool is64 = is_mem &&
                ((bar0_lo & PCI_BASE_ADDRESS_MEM_TYPE_MASK) ==
                 PCI_BASE_ADDRESS_MEM_TYPE_64);
    uint64_t base = is64 ?
        (((uint64_t)bar0_hi << 32) | (uint64_t)(bar0_lo & PCI_BASE_ADDRESS_MEM_MASK)) : 0;
    uint32_t ctrl = GUEST_BAR0_CTRL_VALID |
                    (is64 ? GUEST_BAR0_CTRL_IS64 : 0) |
                    ((command & PCI_COMMAND_MEMORY) ? GUEST_BAR0_CTRL_MEM_ENABLE : 0);
    bool ok;

    qemu_mutex_lock(&s->xdma_lock);
    ok = scope_lat_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_GUEST_BAR0_LO,
                                       (uint32_t)base);
    ok = ok && scope_lat_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_GUEST_BAR0_HI,
                                             (uint32_t)(base >> 32));
    ok = ok && scope_lat_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_GUEST_BAR0_SIZE,
                                             (uint32_t)s->real_bar0_size);
    ok = ok && scope_lat_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_GUEST_BAR0_CTRL,
                                             ctrl);
    ok = ok && scope_lat_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_PROXY_CTRL,
                                             PROXY_CTRL_REAL_BAR_READY);
    qemu_mutex_unlock(&s->xdma_lock);

    if (!ok) {
        error_setg_errno(errp, errno, "Failed to sync guest BAR0 shadow to FPGA");
        return false;
    }

    printf("[SCOPE MMIO LAT][CFG] BAR0 base=0x%016" PRIx64
           " size=0x%zx ctrl=0x%x command=0x%x\n",
           base, s->real_bar0_size, ctrl, command);
    fflush(stdout);
    return true;
}

static bool scope_lat_ack_cfg(ScopeMmioLatTestState *s, uint32_t seq)
{
    bool ok;

    qemu_mutex_lock(&s->xdma_lock);
    ok = scope_lat_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_ACK, seq);
    qemu_mutex_unlock(&s->xdma_lock);
    return ok;
}

static bool scope_lat_write_bar_response(ScopeMmioLatTestState *s, uint32_t seq,
                                         uint32_t resp, uint64_t data,
                                         bool has_data)
{
    uint32_t ctrl;
    bool ok = true;

    qemu_mutex_lock(&s->xdma_lock);
    if (has_data) {
        ok = scope_lat_xdma_write32_locked(s,
                                           HOST_MBX_BASE + MBX_REG_BAR_RESP_DATA_LO,
                                           (uint32_t)data);
    }
    if (ok && has_data) {
        ok = scope_lat_xdma_write32_locked(s,
                                           HOST_MBX_BASE + MBX_REG_BAR_RESP_DATA_HI,
                                           (uint32_t)(data >> 32));
    }
    if (ok) {
        ok = scope_lat_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_BAR_RESP_SEQ,
                                           seq);
    }
    if (ok) {
        s->bar_resp_toggle ^= 1U;
        ctrl = (resp & 0x3U) | (s->bar_resp_toggle << BAR_RESP_CTRL_TOGGLE_SHIFT);
        ok = scope_lat_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_BAR_RESP_CTRL,
                                           ctrl);
    }
    qemu_mutex_unlock(&s->xdma_lock);
    return ok;
}

static inline uint32_t scope_lat_extract_dword32(uint64_t data, uint32_t offset)
{
    return (uint32_t)(data >> ((offset & 0x4U) ? 32 : 0));
}

static bool scope_lat_real_bar_write(ScopeMmioLatTestState *s, uint32_t offset,
                                     uint64_t data, uint8_t wstrb,
                                     uint8_t size_bytes)
{
    volatile uint8_t *base;
    uint8_t lane = offset & 0x7U;
    uint8_t full_wstrb;
    uint8_t i;

    if (!s->real_bar0_map || !size_bytes || size_bytes > 8 ||
        (((offset & 0x7U) + size_bytes) > 8U) ||
        (size_t)offset + size_bytes > s->real_bar0_size) {
        return false;
    }

    full_wstrb = ((1U << size_bytes) - 1U) << lane;
    base = (volatile uint8_t *)s->real_bar0_map + offset;

    if (size_bytes == 4 && (offset & 0x3U) == 0 &&
        (wstrb & full_wstrb) == full_wstrb) {
        *(volatile uint32_t *)base = scope_lat_extract_dword32(data, offset);
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
            *(volatile uint8_t *)(base + i) =
                (uint8_t)(data >> (byte_lane * 8));
        }
    }
    return true;
}

static bool scope_lat_real_bar_read(ScopeMmioLatTestState *s, uint32_t offset,
                                    uint8_t size_bytes, uint64_t *data)
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

static bool scope_lat_read_stable_packet(const void *slot_base,
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

static void scope_lat_log_cfg_write(const char *path, uint32_t seq,
                                    uint32_t addr, int len,
                                    uint32_t wstrb, uint32_t val)
{
    printf("[SCOPE MMIO LAT][CFG][%s] seq=%u off=0x%03x len=%d "
           "wstrb=0x%x val=0x%08x\n",
           path, seq, addr, len, wstrb, val);
    fflush(stdout);
}

static void scope_lat_apply_cfg_write(ScopeMmioLatTestState *s, const char *path,
                                      uint32_t seq, uint32_t addr,
                                      uint32_t data, uint32_t wstrb)
{
    PCIDevice *pci_dev = PCI_DEVICE(s);
    uint32_t mask = wstrb;
    uint32_t config_limit = pci_config_size(pci_dev);
    int offset = 0;
    int len = 0;
    uint32_t val;
    uint32_t actual_addr;
    Error *local_err = NULL;

    while ((mask & 1U) == 0U && offset < 4) {
        mask >>= 1;
        offset++;
    }
    while (((mask >> len) & 1U) && len < 4) {
        len++;
    }

    if (!len) {
        scope_lat_ack_cfg(s, seq);
        return;
    }

    val = (data >> (offset * 8)) & (0xffffffffU >> ((4 - len) * 8));
    actual_addr = addr + (uint32_t)offset;
    scope_lat_log_cfg_write(path, seq, actual_addr, len, wstrb, val);

    if (actual_addr + (uint32_t)len <= config_limit) {
        bql_lock();
        pci_default_write_config(pci_dev, actual_addr, val, len);
        if (!scope_lat_sync_vconf_to_fpga(s, pci_dev, &local_err) ||
            !scope_lat_sync_guest_bar_shadow(s, pci_dev, &local_err)) {
            if (local_err) {
                qemu_log_mask(LOG_GUEST_ERROR, "SCOPE MMIO LAT: %s\n",
                              error_get_pretty(local_err));
                error_free(local_err);
            }
        }
        bql_unlock();
    }

    if (!scope_lat_ack_cfg(s, seq)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE MMIO LAT: Failed to ACK CFG packet seq=%u\n", seq);
    }
}

static void scope_lat_process_cfg_packet(ScopeMmioLatTestState *s,
                                         const struct scope_dma32_packet *pkt)
{
    uint32_t mailbox_status = 0;
    uint32_t mailbox_awaddr = 0;
    uint32_t mailbox_wdata = 0;
    uint32_t mailbox_wstrb = 0;
    uint32_t wstrb = pkt->guest_addr_lo & 0x0FU;
    bool mailbox_match;

    qemu_mutex_lock(&s->xdma_lock);
    mailbox_match =
        scope_lat_xdma_read32_locked(s, HOST_MBX_BASE + MBX_REG_STATUS,
                                     &mailbox_status) &&
        scope_lat_xdma_read32_locked(s, HOST_MBX_BASE + MBX_REG_AWADDR,
                                     &mailbox_awaddr) &&
        scope_lat_xdma_read32_locked(s, HOST_MBX_BASE + MBX_REG_WDATA,
                                     &mailbox_wdata) &&
        scope_lat_xdma_read32_locked(s, HOST_MBX_BASE + MBX_REG_WSTRB,
                                     &mailbox_wstrb);
    qemu_mutex_unlock(&s->xdma_lock);

    mailbox_match = mailbox_match &&
                    mailbox_status == 1U &&
                    mailbox_awaddr == pkt->bar_offset &&
                    mailbox_wdata == pkt->data &&
                    (mailbox_wstrb & 0x0FU) == wstrb;

    if (!mailbox_match) {
        printf("[SCOPE MMIO LAT][CFG][C2H] seq=%u ignored "
               "(mbx_status=0x%x mbx_awaddr=0x%03x mbx_wdata=0x%08x "
               "mbx_wstrb=0x%x pkt_addr=0x%03x pkt_wdata=0x%08x pkt_wstrb=0x%x)\n",
               pkt->seq, mailbox_status, mailbox_awaddr, mailbox_wdata,
               mailbox_wstrb & 0x0FU, pkt->bar_offset, pkt->data, wstrb);
        fflush(stdout);
        return;
    }

    scope_lat_apply_cfg_write(s, "C2H", pkt->seq, pkt->bar_offset,
                              pkt->data, wstrb);
}

static void scope_lat_process_bar_packet(ScopeMmioLatTestState *s,
                                         const struct scope_dma32_packet *pkt)
{
    uint8_t size_bytes = (pkt->flags >> 8) & 0xFFU;
    uint8_t wstrb = pkt->flags & 0xFFU;
    uint64_t lane_data = ((uint64_t)pkt->guest_addr_lo << 32) | pkt->data;
    uint64_t data = 0;
    uint64_t rx_ns;
    uint64_t real_write_done_ns;
    uint64_t resp_done_ns;
    uint32_t resp = 0x2U;
    bool ok = false;

    switch (pkt->type) {
    case SCOPE_PKT_TYPE_BAR_WRITE:
        rx_ns = scope_lat_now_ns();
        ok = scope_lat_real_bar_write(s, pkt->bar_offset, lane_data, wstrb,
                                      size_bytes);
        real_write_done_ns = scope_lat_now_ns();
        resp = ok ? 0x0U : 0x2U;
        if (!scope_lat_write_bar_response(s, pkt->seq, resp, 0, false)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "SCOPE MMIO LAT: Failed to write BAR write response\n");
        }
        resp_done_ns = scope_lat_now_ns();
        printf("[SCOPE MMIO LAT][BAR_WRITE]%s seq=%u off=0x%04x size=%u "
               "wstrb=0x%02x data=0x%016" PRIx64 " ok=%d "
               "rx_ns=%" PRIu64 " real_write_done_ns=%" PRIu64
               " resp_done_ns=%" PRIu64 " qemu_fast_ns=%" PRIu64 "\n",
               s->first_bar_write_seen ? "" : "[FIRST]",
               pkt->seq, pkt->bar_offset, size_bytes, wstrb, lane_data, ok,
               rx_ns, real_write_done_ns, resp_done_ns, resp_done_ns - rx_ns);
        s->first_bar_write_seen = true;
        fflush(stdout);
        break;
    case SCOPE_PKT_TYPE_BAR_READ:
        rx_ns = scope_lat_now_ns();
        ok = scope_lat_real_bar_read(s, pkt->bar_offset, size_bytes, &data);
        resp = ok ? 0x0U : 0x2U;
        if (!scope_lat_write_bar_response(s, pkt->seq, resp, data, true)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "SCOPE MMIO LAT: Failed to write BAR read response\n");
        }
        resp_done_ns = scope_lat_now_ns();
        printf("[SCOPE MMIO LAT][BAR_READ] seq=%u off=0x%04x size=%u "
               "resp=0x%x data=0x%016" PRIx64 " qemu_ns=%" PRIu64 "\n",
               pkt->seq, pkt->bar_offset, size_bytes, resp, data,
               resp_done_ns - rx_ns);
        fflush(stdout);
        break;
    default:
        break;
    }
}

static bool scope_lat_poll_dma32_ring(ScopeMmioLatTestState *s,
                                      uint32_t *last_seq_by_slot,
                                      uint32_t *last_type_by_slot,
                                      size_t slot_count)
{
    const size_t slot_size = sizeof(struct scope_dma32_packet);
    const uint8_t *ring_base = (const uint8_t *)s->dma32_db_map;
    bool progressed = false;
    size_t i;

    for (i = 0; i < slot_count; i++) {
        struct scope_dma32_packet pkt;

        if (!scope_lat_read_stable_packet(ring_base + i * slot_size, &pkt)) {
            continue;
        }
        if (pkt.magic != XDMA_DMA32_PKT_MAGIC ||
            pkt.type == 0 || pkt.type > SCOPE_DMA32_TYPE_MAX) {
            continue;
        }
        if (last_seq_by_slot[i] == pkt.seq &&
            last_type_by_slot[i] == pkt.type) {
            continue;
        }

        last_seq_by_slot[i] = pkt.seq;
        last_type_by_slot[i] = pkt.type;
        progressed = true;

        if (pkt.type == SCOPE_PKT_TYPE_CFG_WRITE) {
            scope_lat_process_cfg_packet(s, &pkt);
        } else if (pkt.type == SCOPE_PKT_TYPE_BAR_WRITE ||
                   pkt.type == SCOPE_PKT_TYPE_BAR_READ) {
            scope_lat_process_bar_packet(s, &pkt);
        }
    }

    return progressed;
}

static void *scope_lat_rx_thread(void *opaque)
{
    ScopeMmioLatTestState *s = opaque;
    const size_t slot_count = s->dma32_db.size / sizeof(struct scope_dma32_packet);
    uint32_t *last_seq_by_slot = g_new0(uint32_t, slot_count);
    uint32_t *last_type_by_slot = g_new0(uint32_t, slot_count);

    while (!qatomic_read(&s->rx_thread_stop)) {
        bool progressed = scope_lat_poll_dma32_ring(s, last_seq_by_slot,
                                                    last_type_by_slot,
                                                    slot_count);
        if (!progressed) {
            g_usleep(50);
        }
    }

    g_free(last_seq_by_slot);
    g_free(last_type_by_slot);
    return NULL;
}

static bool scope_lat_alloc_dma32_ring(ScopeMmioLatTestState *s, Error **errp)
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
        error_setg_errno(errp, errno, "Failed to mmap DMA32 ring");
        return false;
    }

    scope_lat_zero_dma32_ring(s);
    printf("[SCOPE MMIO LAT] DMA32 ring size=0x%x host_dma=0x%016" PRIx64 "\n",
           s->dma32_db.size, s->dma32_db.dma_addr);
    fflush(stdout);
    return true;
}

static bool scope_lat_parse_real_bar0(ScopeMmioLatTestState *s, Error **errp)
{
    ScopePciResource res;
    g_autofree char *resource0_path = NULL;

    if (!scope_lat_read_pci_bar_resource(s->real_host_bdf, 0, &res, errp)) {
        return false;
    }

    s->real_bar0_size = (size_t)(res.end - res.start + 1);
    if (!s->real_bar0_size) {
        error_setg(errp, "Real BAR0 size is zero");
        return false;
    }

    resource0_path = g_strdup_printf("/sys/bus/pci/devices/%s/resource0",
                                     s->real_host_bdf);
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

    printf("[SCOPE MMIO LAT] Real BAR0 %s mapped, size=0x%zx\n",
           s->real_host_bdf, s->real_bar0_size);
    fflush(stdout);
    return true;
}

static void scope_lat_hardware_interrupt_cb(void *opaque)
{
    ScopeMmioLatTestState *s = SCOPE_MMIO_LAT_TEST(opaque);
    uint32_t events_count = 0;
    uint32_t status = 0;
    uint32_t awaddr = 0;
    uint32_t wdata = 0;
    uint32_t wstrb = 0;
    bool ok;

    if (read(s->event_fd, &events_count, sizeof(events_count)) <= 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE MMIO LAT: Failed to read XDMA event fd\n");
        return;
    }

    qemu_mutex_lock(&s->xdma_lock);
    ok = scope_lat_xdma_read32_locked(s, HOST_MBX_BASE + MBX_REG_STATUS, &status) &&
         scope_lat_xdma_read32_locked(s, HOST_MBX_BASE + MBX_REG_AWADDR, &awaddr) &&
         scope_lat_xdma_read32_locked(s, HOST_MBX_BASE + MBX_REG_WDATA, &wdata) &&
         scope_lat_xdma_read32_locked(s, HOST_MBX_BASE + MBX_REG_WSTRB, &wstrb);
    qemu_mutex_unlock(&s->xdma_lock);

    if (!ok || status != 1U) {
        scope_lat_ack_cfg(s, 1);
        return;
    }

    scope_lat_apply_cfg_write(s, "IRQ", 1, awaddr, wdata, wstrb & 0x0FU);
}

static uint64_t scope_lat_dummy_bar_read(void *opaque, hwaddr addr,
                                         unsigned size)
{
    printf("[SCOPE MMIO LAT][DUMMY_BAR_READ] off=0x%" HWADDR_PRIx
           " size=%u\n", addr, size);
    fflush(stdout);
    return 0;
}

static void scope_lat_dummy_bar_write(void *opaque, hwaddr addr, uint64_t val,
                                      unsigned size)
{
    printf("[SCOPE MMIO LAT][DUMMY_BAR_WRITE] off=0x%" HWADDR_PRIx
           " size=%u val=0x%016" PRIx64 "\n", addr, size, val);
    fflush(stdout);
}

static const MemoryRegionOps scope_lat_dummy_bar_ops = {
    .read = scope_lat_dummy_bar_read,
    .write = scope_lat_dummy_bar_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void scope_lat_cleanup(ScopeMmioLatTestState *s)
{
    if (s->rx_thread_started) {
        qatomic_set(&s->rx_thread_stop, 1);
        qemu_thread_join(&s->rx_thread);
        s->rx_thread_started = false;
    }

    if (s->event_fd >= 0) {
        qemu_set_fd_handler(s->event_fd, NULL, NULL, NULL);
        close(s->event_fd);
        s->event_fd = -1;
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
}

static void scope_lat_realize(PCIDevice *pci_dev, Error **errp)
{
    ScopeMmioLatTestState *s = SCOPE_MMIO_LAT_TEST(pci_dev);
    const char *xdma_user_dev = s->xdma_user_dev ?
        s->xdma_user_dev : SCOPE_DEFAULT_XDMA_USER;
    const char *xdma_ctrl_dev = s->xdma_ctrl_dev ?
        s->xdma_ctrl_dev : SCOPE_DEFAULT_XDMA_CTRL;
    const char *xdma_event_dev = s->xdma_event_dev ?
        s->xdma_event_dev : SCOPE_DEFAULT_XDMA_EVENT;
    uint32_t dummy_ack = 1;

    printf("\n[SCOPE MMIO LAT] Initializing fast-path MMIO latency test device...\n");

    s->host_page_size = sysconf(_SC_PAGESIZE);
    if (!s->host_page_size) {
        s->host_page_size = 4096;
    }

    qemu_mutex_init(&s->xdma_lock);
    s->xdma_lock_inited = true;

    if (!scope_lat_parse_real_bar0(s, errp)) {
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

    if (!scope_lat_alloc_dma32_ring(s, errp)) {
        goto fail;
    }

    pci_config_set_vendor_id(pci_dev->config, 0x1b36);
    pci_config_set_device_id(pci_dev->config, 0x110e);
    pci_config_set_class(pci_dev->config, PCI_CLASS_OTHERS);
    pci_dev->config[PCI_INTERRUPT_PIN] = 0x00;

    if (pcie_endpoint_cap_init(pci_dev, 0x70) < 0) {
        error_setg(errp, "Failed to initialize PCIe endpoint capability");
        goto fail;
    }

    memory_region_init_io(&s->dummy_bar0, OBJECT(s), &scope_lat_dummy_bar_ops, s,
                          "scope-mmio-lat-bar0", s->real_bar0_size);
    pci_register_bar(pci_dev, 0,
                     PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_64,
                     &s->dummy_bar0);

    if (!scope_lat_sync_vconf_to_fpga(s, pci_dev, errp) ||
        !scope_lat_sync_guest_bar_shadow(s, pci_dev, errp)) {
        goto fail;
    }

    qemu_mutex_lock(&s->xdma_lock);
    if (!scope_lat_xdma_write32_locked(s, HOST_MBX_BASE + MBX_REG_ACK, dummy_ack)) {
        qemu_mutex_unlock(&s->xdma_lock);
        error_setg(errp, "Failed to reset FPGA mailbox ACK");
        goto fail;
    }
    qemu_mutex_unlock(&s->xdma_lock);

    qatomic_set(&s->rx_thread_stop, 0);
    qemu_thread_create(&s->rx_thread, "scope-mmio-lat-rx", scope_lat_rx_thread,
                       s, QEMU_THREAD_JOINABLE);
    s->rx_thread_started = true;

    qemu_set_fd_handler(s->event_fd, scope_lat_hardware_interrupt_cb, NULL, s);

    printf("[SCOPE MMIO LAT] Vendor-specific PCI test device ready.\n\n");
    fflush(stdout);
    return;

fail:
    scope_lat_cleanup(s);
}

static void scope_lat_exit(PCIDevice *pci_dev)
{
    ScopeMmioLatTestState *s = SCOPE_MMIO_LAT_TEST(pci_dev);

    scope_lat_cleanup(s);
    printf("[SCOPE MMIO LAT] Device exited and cleaned up.\n");
    fflush(stdout);
}

static void scope_lat_instance_init(Object *obj)
{
    ScopeMmioLatTestState *s = SCOPE_MMIO_LAT_TEST(obj);

    s->xdma_fd = -1;
    s->xdma_ctrl_fd = -1;
    s->event_fd = -1;
    s->real_bar_fd = -1;
    s->real_bar0_map = NULL;
    s->real_bar0_size = 0;
    s->host_page_size = 0;
    s->dma32_db_map = NULL;
    memset(&s->dma32_db, 0, sizeof(s->dma32_db));
    s->xdma_lock_inited = false;
    s->rx_thread_started = false;
    s->rx_thread_stop = 0;
    s->bar_resp_toggle = 0;
    s->first_bar_write_seen = false;
}

static const Property scope_lat_properties[] = {
    DEFINE_PROP_STRING("real-host-bdf", ScopeMmioLatTestState, real_host_bdf),
    DEFINE_PROP_STRING("xdma-user-dev", ScopeMmioLatTestState, xdma_user_dev),
    DEFINE_PROP_STRING("xdma-ctrl-dev", ScopeMmioLatTestState, xdma_ctrl_dev),
    DEFINE_PROP_STRING("xdma-event-dev", ScopeMmioLatTestState, xdma_event_dev),
};

static void scope_lat_class_init(ObjectClass *class, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize = scope_lat_realize;
    k->exit = scope_lat_exit;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    device_class_set_props(dc, scope_lat_properties);
}

static const TypeInfo scope_lat_info = {
    .name = TYPE_SCOPE_MMIO_LAT_TEST,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(ScopeMmioLatTestState),
    .instance_init = scope_lat_instance_init,
    .class_init = scope_lat_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};

static void scope_lat_register_types(void)
{
    type_register_static(&scope_lat_info);
}

type_init(scope_lat_register_types)
