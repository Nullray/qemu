#include "qemu/osdep.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pcie.h"
#include "hw/pci/msix.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h> // 引入标准输入输出以支持 printf
#include <linux/ioctl.h>

#define TYPE_SCOPE_PROXY "scope-fpga-proxy"
OBJECT_DECLARE_SIMPLE_TYPE(ScopeProxyState, SCOPE_PROXY)

#define XDMA_IOC_MAGIC  'x'
struct xdma_dma32_doorbell {
    unsigned int size;
    unsigned int reserved;
    unsigned long long dma_addr;
};
#define XDMA_IOC_DMA32_DB_ALLOC _IOWR(XDMA_IOC_MAGIC, 0x20, struct xdma_dma32_doorbell)
#define XDMA_IOC_DMA32_DB_FREE  _IO(XDMA_IOC_MAGIC, 0x21)

#define XDMA_DMA32_DB_MMAP_PGOFF 0x100000U

/* Single-slot mailbox-over-C2H mode: one in-flight write request at a time. */
#define SCOPE_RING_SLOT_BYTES 32U

// -----------------------------------------------------------------------------
// 硬件地址映射
// -----------------------------------------------------------------------------
#define HOST_MBX_BASE    0x01000000  
#define HOST_VCONF_BASE  0x01010000  

#define MBX_REG_ACK      0x10

#define XDMA_DMA32_PKT_MAGIC 0x5844504bU

struct ScopeProxyState {
    PCIDevice parent_obj;
    
    int xdma_fd;
    int xdma_ctrl_fd;
    uint8_t *cyclic_ring_buf;
    size_t ring_size;
    size_t read_offset;
    size_t scan_offset;

    QemuThread rx_thread;
    bool rx_thread_running;
    bool rx_thread_started;
    MemoryRegion dummy_bar0;
};

static void scope_ring_zero_region(ScopeProxyState *s, size_t start, size_t len)
{
    size_t first;

    if (!len || !s->ring_size) {
        return;
    }

    first = len;
    if (first > (s->ring_size - start)) {
        first = s->ring_size - start;
    }

    memset(s->cyclic_ring_buf + start, 0, first);
    if (len > first) {
        memset(s->cyclic_ring_buf, 0, len - first);
    }
}

static void scope_dump_packet_payload(const uint8_t *payload, size_t len)
{
    size_t dump_len = len < 64 ? len : 64;

    printf("[SCOPE PROXY] Payload (%zu bytes):", len);
    for (size_t i = 0; i < dump_len; ++i) {
        printf(" %02x", payload[i]);
    }
    if (len > dump_len) {
        printf(" ...");
    }
    printf("\n");
}

static const char *scope_channel_name(uint32_t channel)
{
    switch (channel) {
    case 1:
        return "aw";
    case 2:
        return "ar";
    default:
        return "unknown";
    }
}

static uint32_t scope_load_u32(const uint8_t *payload, size_t word_index)
{
    size_t base = word_index * 4;
    uint32_t v = 0;

    v |= ((uint32_t)payload[base + 0]) << 0;
    v |= ((uint32_t)payload[base + 1]) << 8;
    v |= ((uint32_t)payload[base + 2]) << 16;
    v |= ((uint32_t)payload[base + 3]) << 24;
    return v;
}

/*
 * Some C2H restart paths may resume writing from a non-zero position in the
 * cyclic buffer. Scan the ring in small windows to resync without assuming the
 * producer always starts at slot 0.
 */
static ssize_t scope_ring_find_packet_offset(ScopeProxyState *s)
{
    const size_t step = SCOPE_RING_SLOT_BYTES;
    const size_t scan_slots_per_poll = 256; /* 8KB per poll */
    size_t i;
    size_t off;

    if (!s->ring_size || step == 0 || s->ring_size < step) {
        return -1;
    }

    if (s->scan_offset >= s->ring_size) {
        s->scan_offset = 0;
    }

    off = s->scan_offset;
    for (i = 0; i < scan_slots_per_poll; i++) {
        uint8_t *p = s->cyclic_ring_buf + off;
        uint32_t magic = scope_load_u32(p, 0);
        uint32_t type = scope_load_u32(p, 1);

        if (magic == XDMA_DMA32_PKT_MAGIC && type == 1) {
            s->scan_offset = off;
            return (ssize_t)off;
        }

        off += step;
        if (off >= s->ring_size) {
            off = 0;
        }
    }

    s->scan_offset = off;
    return -1;
}

static void scope_ack_packet(ScopeProxyState *s, uint32_t seq)
{
    if (pwrite(s->xdma_fd, &seq, 4, HOST_MBX_BASE + MBX_REG_ACK) != 4) {
        printf("[SCOPE PROXY] ACK write failed for seq=%u: %s\n", seq, strerror(errno));
        return;
    }

    printf("[SCOPE PROXY] ACK sent: seq=%u\n", seq);
}

static void scope_handle_notify_packet(ScopeProxyState *s,
                                       const uint8_t *payload,
                                       size_t len)
{
    uint32_t magic, type, channel, pkt_len, seq;
    uint32_t addr, wdata, meta;
    uint32_t awprot, wstrb;

    if (len < 32) {
        return;
    }

    magic = scope_load_u32(payload, 0);
    type = scope_load_u32(payload, 1);
    channel = scope_load_u32(payload, 2);
    pkt_len = scope_load_u32(payload, 3);
    seq = scope_load_u32(payload, 4);
    addr = scope_load_u32(payload, 5);
    wdata = scope_load_u32(payload, 6);
    meta = scope_load_u32(payload, 7);
    wstrb = meta & 0x0f;
    awprot = (meta >> 4) & 0x7;

    if (magic != XDMA_DMA32_PKT_MAGIC || type != 1) {
        return;
    }

    printf("[SCOPE PROXY] notify parsed: seq=%u ch=%u(%s) len=%u addr=0x%08x wdata=0x%08x wstrb=0x%x awprot=0x%x\n",
           seq, channel, scope_channel_name(channel), pkt_len,
           addr, wdata, wstrb, awprot);

    // 解析 wstrb 知道真正写了哪些字节
    int offset = 0;
    uint32_t mask = wstrb;
    while ((mask & 1) == 0 && offset < 4) {
        mask >>= 1;
        offset++;
    }
    int write_len = 0;
    while ((mask >> write_len) & 1 && write_len < 4) {
        write_len++;
    }

    if (write_len > 0) {
        uint32_t val = (wdata >> (offset * 8)) & (0xFFFFFFFF >> ((4 - write_len) * 8));
        uint32_t actual_addr = addr + offset;

        PCIDevice *pci_dev = PCI_DEVICE(s);
        PCIDeviceClass *pc = PCI_DEVICE_GET_CLASS(pci_dev);
        
        // 1. 让 QEMU 模拟执行这段配置空间写入（会触发 BAR 映射等虚拟设备的副作用）
        pc->config_write(pci_dev, actual_addr, val, write_len);

        // 2. 将 QEMU 内存中最新的 4 字节状态同步回 FPGA BRAM
        // 这样香山下一次读取配置空间才能读到最新值！
        uint32_t sync_addr = addr & ~3U;
        uint32_t new_dword = *(uint32_t *)(pci_dev->config + sync_addr);
        if (pwrite(s->xdma_fd, &new_dword, 4, HOST_VCONF_BASE + sync_addr) != 4) {
            printf("[SCOPE PROXY WARNING] Failed to sync config back to FPGA BRAM at 0x%x\n", sync_addr);
        } else {
            printf("[SCOPE PROXY] Synced config space addr 0x%03x to FPGA BRAM: 0x%08x\n", sync_addr, new_dword);
        }
    }

    scope_ack_packet(s, seq);
}

static void *scope_c2h_rx_thread(void *opaque)
{
    ScopeProxyState *s = SCOPE_PROXY(opaque);
    uint64_t drop_count = 0;
    uint64_t poll_loops = 0; // 新增心跳计数器

    printf("[SCOPE PROXY] rx_thread started. Polling cyclic ring mapped at %p...\n", s->cyclic_ring_buf);
    fflush(stdout);

    s->read_offset = 0;
    while (s->rx_thread_running) {
        poll_loops++;
        if (poll_loops % 50000 == 0) { // 约每休眠 50000*100us = 5秒 输出一次心跳
            uint32_t fw_status = 0, fw_awaddr = 0, fw_wdata = 0;
            // 通过 AXI Master 旁路直接读取 FPGA 的 Mailbox 寄存器，探查 FPGA 硬件状态
            if (pread(s->xdma_fd, &fw_status, 4, HOST_MBX_BASE + 0) != 4) fw_status = 0xFFFFFFFF;
            if (pread(s->xdma_fd, &fw_awaddr, 4, HOST_MBX_BASE + 4) != 4) fw_awaddr = 0xFFFFFFFF;
            if (pread(s->xdma_fd, &fw_wdata, 4, HOST_MBX_BASE + 8) != 4) fw_wdata = 0xFFFFFFFF;

            uint32_t c2h0_ctrl = 0, c2h1_ctrl = 0;
            if (pread(s->xdma_ctrl_fd, &c2h0_ctrl, 4, 0x1004) != 4) c2h0_ctrl = 0xFFFFFFFF;
            if (pread(s->xdma_ctrl_fd, &c2h1_ctrl, 4, 0x1104) != 4) c2h1_ctrl = 0xFFFFFFFF;

            printf("[SCOPE PROXY DEBUG] rx_thread heartbeat: still polling, current read_offset=0x%zx\n", s->read_offset);
            printf("[SCOPE PROXY DEBUG] Head of ring memory dump (first 128 bytes):\n");
            for (int i = 0; i < 128; i++) {
                printf("%02x ", s->cyclic_ring_buf[i]);
                if ((i + 1) % 16 == 0) printf("\n");
            }
            printf("[SCOPE PROXY DEBUG] Current read pointer memory check: magic=0x%08x, type=0x%08x\n", 
                   scope_load_u32(s->cyclic_ring_buf + s->read_offset, 0), 
                   scope_load_u32(s->cyclic_ring_buf + s->read_offset, 1));
            printf("[SCOPE PROXY HW-PROBE] FPGA Mailbox check: status=0x%08x (1=Wait ACK), intercepted_awaddr=0x%08x, wdata=0x%08x\n",
                   fw_status, fw_awaddr, fw_wdata);
            printf("[SCOPE PROXY HW-PROBE] XDMA C2H Engines Ctrl Regs: C2H_0=0x%08x, C2H_1=0x%08x\n", 
                   c2h0_ctrl, c2h1_ctrl);
            fflush(stdout);
        }

        uint8_t *ptr = s->cyclic_ring_buf;
        size_t pkt_off = 0;
        uint32_t magic = scope_load_u32(ptr, 0);
        uint32_t type  = scope_load_u32(ptr, 1);

        if (!(magic == XDMA_DMA32_PKT_MAGIC && type == 1)) {
            ssize_t found = scope_ring_find_packet_offset(s);
            if (found >= 0) {
                pkt_off = (size_t)found;
                ptr = s->cyclic_ring_buf + pkt_off;
                magic = scope_load_u32(ptr, 0);
                type = scope_load_u32(ptr, 1);
                if (pkt_off != 0) {
                    printf("[SCOPE PROXY INFO] Resynced packet at ring offset 0x%zx (slot0 empty).\n", pkt_off);
                    fflush(stdout);
                }
            }
        }

        if (magic == XDMA_DMA32_PKT_MAGIC && type == 1) {
            uint32_t pkt_len = scope_load_u32(ptr, 3);
            size_t consume_len = SCOPE_RING_SLOT_BYTES;
            
            // 包长合法性检查，如果过大会越界或死循环
            if (pkt_len == 0 || pkt_len > 4096) {
                printf("[SCOPE PROXY WARNING] Invalid pkt_len=%u at single slot. Forcing len=32.\n", pkt_len);
                pkt_len = 32;
            }

            bql_lock();
            printf("\n[SCOPE PROXY] >>> PACKET RECEIVED VIA C2H >>> slot=0x%zx, len=%u bytes\n", pkt_off, pkt_len);
            scope_dump_packet_payload(ptr, pkt_len);
            scope_handle_notify_packet(s, ptr, pkt_len);
            printf("[SCOPE PROXY] <<< PACKET CONSUMED <<<\n\n");
            fflush(stdout);
            bql_unlock();

            // 单 in-flight：消费后清理命中槽位并继续轮询。
            scope_ring_zero_region(s, pkt_off, consume_len);
            s->read_offset = pkt_off;
            s->scan_offset = pkt_off;
            
        } else if (magic != 0) {
            if (magic != XDMA_DMA32_PKT_MAGIC) {
                drop_count++;
                if (drop_count % 10000 == 0) {
                    printf("[SCOPE PROXY DEBUG] rx_thread: skipped %lu garbage bytes. current magic=0x%08x\n", drop_count, magic);
                    fflush(stdout);
                }
                memset(ptr, 0, 4);
                s->read_offset = pkt_off;
            } else {
                // 魔数正确，但 type 不是 1？
                printf("[SCOPE PROXY WARNING] Found MAGIC but unknown type=%u at single slot! Data might be incomplete or corrupted. Clearing 4 bytes.\n", type);
                fflush(stdout);
                memset(ptr, 0, 4);
                s->read_offset = pkt_off;
            }
        } else {
            // 本位置暂无有效封包魔数刷入，软件休眠让出 CPU
            usleep(100);
        }
    }
    printf("[SCOPE PROXY] rx_thread exiting.\n");
    fflush(stdout);
    return NULL;
}

// -----------------------------------------------------------------------------
// 增加一组 Dummy 的读写回调，加入打印以观察 Linux 是否在操作物理内存
// -----------------------------------------------------------------------------
static uint64_t dummy_bar_read(void *opaque, hwaddr addr, unsigned size) {
    printf("[SCOPE PROXY] Guest OS READ  BAR0: offset 0x%04lX, size %u bytes\n", addr, size);
    fflush(stdout);
    return 0;
}
static void dummy_bar_write(void *opaque, hwaddr addr, uint64_t val, unsigned size) {
    printf("[SCOPE PROXY] Guest OS WRITE BAR0: offset 0x%04lX, value 0x%08lX, size %u bytes\n", addr, val, size);
    fflush(stdout);
}
static const MemoryRegionOps dummy_bar_ops = {
    .read = dummy_bar_read,
    .write = dummy_bar_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

// -----------------------------------------------------------------------------
// PCI 配置空间钩子：监控 Linux 的发现与绑定行为
// -----------------------------------------------------------------------------
static uint32_t scope_proxy_config_read(PCIDevice *pci_dev, uint32_t addr, int len)
{
    uint32_t val = pci_default_read_config(pci_dev, addr, len);
    if (addr < 64) { // 指打印标头的常用读取，防止刷屏
        printf("[SCOPE PROXY] PCI Config Read : addr=0x%04x, len=%d, val=0x%08x\n", addr, len, val);
        fflush(stdout);
    }
    return val;
}

static void scope_proxy_config_write(PCIDevice *pci_dev, uint32_t addr, uint32_t val, int len)
{
    printf("[SCOPE PROXY] PCI Config Write: addr=0x%04x, len=%d, val=0x%08x\n", addr, len, val);
    fflush(stdout);
    pci_default_write_config(pci_dev, addr, val, len);
}

// -----------------------------------------------------------------------------
// 设备初始化阶段
// -----------------------------------------------------------------------------
static void scope_proxy_realize(PCIDevice *pci_dev, Error **errp)
{
    ScopeProxyState *s = SCOPE_PROXY(pci_dev);

    s->xdma_fd = -1;
    s->xdma_ctrl_fd = -1;
    s->rx_thread_running = false;
    s->rx_thread_started = false;

    printf("\n[SCOPE PROXY] Initializing Virtual NVMe Device...\n");

    s->xdma_fd = open("/dev/xdma0_user", O_RDWR | O_SYNC);
    if (s->xdma_fd < 0) {
        error_setg(errp, "Failed to open /dev/xdma0_user. Please check XDMA driver.");
        return;
    }
    printf("[SCOPE PROXY] XDMA User Device opened successfully (fd=%d).\n", s->xdma_fd);
    fflush(stdout);

    s->xdma_ctrl_fd = open("/dev/xdma0_control", O_RDWR);
    if (s->xdma_ctrl_fd < 0) {
        error_setg(errp, "Failed to open /dev/xdma0_control. Please check XDMA driver.");
        close(s->xdma_fd);
        s->xdma_fd = -1;
        return;
    }
    printf("[SCOPE PROXY] XDMA Ctrl Device opened successfully (fd=%d).\n", s->xdma_ctrl_fd);
    fflush(stdout);

    struct xdma_dma32_doorbell db_req = { 0 };
    db_req.size = 4 * 1024 * 1024; // 设定环形缓冲区为 4MB
    printf("[SCOPE PROXY DEBUG] Requesting Cyclic DMA DB ALLOC for %u bytes (4MB)...\n", db_req.size);
    fflush(stdout);
    
    int ioctl_ret = ioctl(s->xdma_ctrl_fd, XDMA_IOC_DMA32_DB_ALLOC, &db_req);
    printf("[SCOPE PROXY DEBUG] IOCTL XDMA_IOC_DMA32_DB_ALLOC returned: %d\n", ioctl_ret);
    fflush(stdout);

    if (ioctl_ret < 0) {
        error_setg(errp, "Failed to start Cyclic DMA via IOCTL: %s", strerror(errno));
        close(s->xdma_ctrl_fd);
        close(s->xdma_fd);
        return;
    }
    s->ring_size = db_req.size;
    printf("[SCOPE PROXY DEBUG] DMA ring_size confirmed: %zu bytes, physical dma_addr: 0x%llx\n", 
           s->ring_size, db_req.dma_addr);
    fflush(stdout);

    // 内存映射设备内的连续物理地址空间（页偏移对应 cdev_ctrl 中 mmap 所支持的位置）
    printf("[SCOPE PROXY DEBUG] Attempting mmap of cyclic ring. fd=%d, size=%zu, offset=0x%llx\n", 
           s->xdma_ctrl_fd, s->ring_size, (unsigned long long)((off_t)XDMA_DMA32_DB_MMAP_PGOFF << 12));
    fflush(stdout);

    s->cyclic_ring_buf = mmap(NULL, s->ring_size, PROT_READ | PROT_WRITE, 
                              MAP_SHARED, s->xdma_ctrl_fd, (off_t)XDMA_DMA32_DB_MMAP_PGOFF << 12);
    if (s->cyclic_ring_buf == MAP_FAILED) {
        error_setg(errp, "Failed to mmap cyclic DMA ring buffer: %s", strerror(errno));
        ioctl(s->xdma_ctrl_fd, XDMA_IOC_DMA32_DB_FREE);
        close(s->xdma_ctrl_fd);
        close(s->xdma_fd);
        return;
    }
    printf("[SCOPE PROXY] 4MB Cyclic DMA ring mapped at %p.\n", s->cyclic_ring_buf);
    
    volatile uint8_t *vbuf = (volatile uint8_t *)s->cyclic_ring_buf;
    
    // 调试：打印映射内存前128个字节，检查是否可读且清零
    printf("[SCOPE PROXY DEBUG] First 128 bytes of mapped region:\n");
    for (int i = 0; i < 128; i++) {
        printf("%02x ", vbuf[i]);
        if ((i + 1) % 16 == 0) printf("\n");
    }
    printf("\n");
    
    // 清除上一次运行可能遗留的旧数据（物理内存可能没被内核完全擦除）
    for (size_t i = 0; i < s->ring_size; i++) {
        vbuf[i] = 0;
    }
    asm volatile ("mfence" ::: "memory"); // 强制刷入内存
    
    printf("[SCOPE PROXY DEBUG] Memory wiped to 0 to prevent stale packets.\n");
    printf("[SCOPE PROXY DEBUG] First 128 bytes of mapped region:\n");
    for (int i = 0; i < 128; i++) {
        printf("%02x ", vbuf[i]);
        if ((i + 1) % 16 == 0) printf("\n");
    }
    printf("\n");
    fflush(stdout);

    pci_config_set_vendor_id(pci_dev->config, 0x10EE);
    pci_config_set_device_id(pci_dev->config, 0x903F);
    //pci_dev->config[PCI_CLASS_PROG] = 0x02;
    //pci_config_set_class(pci_dev->config, PCI_CLASS_STORAGE_EXPRESS);
    pci_config_set_class(pci_dev->config, PCI_CLASS_NOT_DEFINED);
    pcie_endpoint_cap_init(pci_dev, 0x70);

    // 【修改点】：将 BAR0 缩小为 16KB，防止香山 MMIO 内存池爆仓！
    memory_region_init_io(&s->dummy_bar0, OBJECT(s), &dummy_bar_ops, s, "scope-bar0", 16 * 1024);
    pci_register_bar(pci_dev, 0, 
                     PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_64, 
                     &s->dummy_bar0);

    printf("[SCOPE PROXY] BAR0 registered.\n");
    fflush(stdout);

    msix_init_exclusive_bar(pci_dev, 64, 2, errp);
    
    printf("[SCOPE PROXY] MSI-X initialized. Injecting config...\n");
    fflush(stdout);

    uint32_t *config_ptr = (uint32_t *)pci_dev->config;
    for (int i = 0; i < 1024; i++) {
        if (pwrite(s->xdma_fd, &config_ptr[i], 4, HOST_VCONF_BASE + i * 4) != 4) {
            error_setg(errp, "Failed to sync initial config at offset 0x%x", i * 4);
            return;
        }
    }
    printf("[SCOPE PROXY] Initial Type 0 Header (4KB) injected into FPGA BRAM.\n");
    fflush(stdout);

    uint32_t dummy_ack = 1;
    if (pwrite(s->xdma_fd, &dummy_ack, 4, HOST_MBX_BASE + MBX_REG_ACK) != 4) {
        error_setg(errp, "Failed to reset FPGA Mailbox ACK");
        return;
    }

    s->rx_thread_running = true;
    qemu_thread_create(&s->rx_thread, "scope_rx", scope_c2h_rx_thread, s, QEMU_THREAD_JOINABLE);
    s->rx_thread_started = true;

    printf("[SCOPE PROXY] C2H blocking rx thread armed.\n");
    printf("[SCOPE PROXY] Initialization Complete. Ready for 香山 Linux enumeration!\n\n");
    fflush(stdout);
}

// -----------------------------------------------------------------------------
// 设备清理阶段
// -----------------------------------------------------------------------------
static void scope_proxy_exit(PCIDevice *pci_dev)
{
    ScopeProxyState *s = SCOPE_PROXY(pci_dev);
    
    s->rx_thread_running = false;
    if (s->rx_thread_started) {
        /* Wait for the polling thread to observe rx_thread_running=false
         * before unmapping the shared ring buffer.
         */
        qemu_thread_join(&s->rx_thread);
        s->rx_thread_started = false;
    }

    if (s->cyclic_ring_buf && s->cyclic_ring_buf != MAP_FAILED) {
        munmap(s->cyclic_ring_buf, s->ring_size);
        s->cyclic_ring_buf = NULL;
        s->ring_size = 0;
    }
    if (s->xdma_ctrl_fd >= 0) {
        ioctl(s->xdma_ctrl_fd, XDMA_IOC_DMA32_DB_FREE);
        close(s->xdma_ctrl_fd);
        s->xdma_ctrl_fd = -1;
    }
    if (s->xdma_fd >= 0) {
        close(s->xdma_fd);
        s->xdma_fd = -1;
    }
    printf("[SCOPE PROXY] Device exited and cleaned up.\n");
}

// -----------------------------------------------------------------------------
// QEMU 对象模型 (QOM) 注册
// -----------------------------------------------------------------------------
static void scope_proxy_class_init(ObjectClass *class, const void *data) 
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize = scope_proxy_realize;
    k->exit = scope_proxy_exit;
    k->config_read = scope_proxy_config_read;
    k->config_write = scope_proxy_config_write;
    
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo scope_proxy_info = {
    .name          = TYPE_SCOPE_PROXY,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(ScopeProxyState),
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

