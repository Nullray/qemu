#include "qemu/osdep.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pcie.h"
#include "hw/pci/msix.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h> // 引入标准输入输出以支持 printf

#define TYPE_SCOPE_PROXY "scope-fpga-proxy"
OBJECT_DECLARE_SIMPLE_TYPE(ScopeProxyState, SCOPE_PROXY)

// -----------------------------------------------------------------------------
// 硬件地址映射
// -----------------------------------------------------------------------------
#define HOST_MBX_BASE    0x01000000  
#define HOST_VCONF_BASE  0x01010000  

#define MBX_REG_STATUS   0x00
#define MBX_REG_AWADDR   0x04
#define MBX_REG_WDATA    0x08
#define MBX_REG_WSTRB    0x0C
#define MBX_REG_ACK      0x10

struct ScopeProxyState {
    PCIDevice parent_obj;
    
    int xdma_fd;
    QEMUTimer *poll_timer;
    MemoryRegion dummy_bar0;
};

// -----------------------------------------------------------------------------
// 增加一组 Dummy 的读写回调，加入打印以观察 Linux 是否在操作物理内存
// -----------------------------------------------------------------------------
static uint64_t dummy_bar_read(void *opaque, hwaddr addr, unsigned size) {
    printf("[SCOPE PROXY] Guest OS READ  BAR0: offset 0x%04lX, size %u bytes\n", addr, size);
    return 0;
}
static void dummy_bar_write(void *opaque, hwaddr addr, uint64_t val, unsigned size) {
    printf("[SCOPE PROXY] Guest OS WRITE BAR0: offset 0x%04lX, value 0x%08lX, size %u bytes\n", addr, val, size);
}
static const MemoryRegionOps dummy_bar_ops = {
    .read = dummy_bar_read,
    .write = dummy_bar_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

// -----------------------------------------------------------------------------
// 核心逻辑：定时器回调函数
// -----------------------------------------------------------------------------
static void scope_poll_timer_cb(void *opaque)
{
    ScopeProxyState *s = SCOPE_PROXY(opaque);
    PCIDevice *pci_dev = PCI_DEVICE(s);
    uint32_t status = 0;

    if (pread(s->xdma_fd, &status, 4, HOST_MBX_BASE + MBX_REG_STATUS) != 4) {
        goto rearm;
    }

    if (status == 1) {
        uint32_t awaddr = 0, wdata = 0, wstrb = 0;
        
        if (pread(s->xdma_fd, &awaddr, 4, HOST_MBX_BASE + MBX_REG_AWADDR) != 4 ||
            pread(s->xdma_fd, &wdata,  4, HOST_MBX_BASE + MBX_REG_WDATA) != 4 ||
            pread(s->xdma_fd, &wstrb,  4, HOST_MBX_BASE + MBX_REG_WSTRB) != 4) {
            goto rearm;
        }

        int len = 4;
        uint32_t val = wdata;

        if (wstrb == 0x1) {
            len = 1; val = wdata & 0xFF;
        } else if (wstrb == 0x2) {
            len = 1; val = (wdata >> 8) & 0xFF;
        } else if (wstrb == 0x4) {
            len = 1; val = (wdata >> 16) & 0xFF;
        } else if (wstrb == 0x8) {
            len = 1; val = (wdata >> 24) & 0xFF;
        } else if (wstrb == 0x3) {
            len = 2; val = wdata & 0xFFFF;
        } else if (wstrb == 0xC) {
            len = 2; val = (wdata >> 16) & 0xFFFF;
        }

        printf("\n======================================================\n");
        printf("[SCOPE PROXY] >>> INTERCEPTED CONFIG WRITE >>>\n");
        printf("[SCOPE PROXY] Raw FPGA Data : AWADDR=0x%04X, WDATA=0x%08X, WSTRB=0x%X\n", awaddr, wdata, wstrb);
        printf("[SCOPE PROXY] Decoded Action: Write %d byte(s) [ 0x%X ] to Offset 0x%03X\n", len, val, awaddr);

        // 越界保护
        if (awaddr + len <= 4096) {
            pci_default_write_config(pci_dev, awaddr, val, len);

            // 数据一致性同步
            uint32_t *dyn_config_ptr = (uint32_t *)pci_dev->config;
            for (int i = 0; i < 1024; i++) {
                if (pwrite(s->xdma_fd, &dyn_config_ptr[i], 4, HOST_VCONF_BASE + i * 4) != 4) {
                    qemu_log_mask(LOG_GUEST_ERROR, "SCOPE: Failed to write VCONF BRAM at 0x%x\n", i * 4);
                    break; 
                }
            }
            printf("[SCOPE PROXY] Success       : 4KB Config Space synced back to FPGA BRAM.\n");
        }

        // 向 FPGA 下发 ACK 脉冲，解除硬件写挂起状态
        uint32_t dummy_ack = 1;
        if (pwrite(s->xdma_fd, &dummy_ack, 4, HOST_MBX_BASE + MBX_REG_ACK) != 4) {
            qemu_log_mask(LOG_GUEST_ERROR, "SCOPE: Failed to write ACK\n");
        }
        printf("[SCOPE PROXY] <<< ACK SENT, HARDWARE UNLOCKED <<<\n");
        printf("======================================================\n\n");
    }

rearm:
    timer_mod(s->poll_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1);
}

// -----------------------------------------------------------------------------
// 设备初始化阶段
// -----------------------------------------------------------------------------
static void scope_proxy_realize(PCIDevice *pci_dev, Error **errp)
{
    ScopeProxyState *s = SCOPE_PROXY(pci_dev);

    printf("\n[SCOPE PROXY] Initializing Virtual NVMe Device...\n");

    s->xdma_fd = open("/dev/xdma0_user", O_RDWR | O_SYNC);
    if (s->xdma_fd < 0) {
        error_setg(errp, "Failed to open /dev/xdma0_user. Please check XDMA driver.");
        return;
    }
    printf("[SCOPE PROXY] XDMA Device opened successfully (fd=%d).\n", s->xdma_fd);

    pci_config_set_vendor_id(pci_dev->config, 0x10EE);
    pci_config_set_device_id(pci_dev->config, 0x903F);
    pci_config_set_class(pci_dev->config, PCI_CLASS_STORAGE_EXPRESS); 
    //想让设备挂载驱动时添加：pci_dev->config[PCI_CLASS_PROG] = 0x02;
    pcie_endpoint_cap_init(pci_dev, 0x70);

    // 【修改点】：将 BAR0 缩小为 16KB，防止香山 MMIO 内存池爆仓！
    memory_region_init_io(&s->dummy_bar0, OBJECT(s), &dummy_bar_ops, s, "scope-bar0", 16 * 1024);
    pci_register_bar(pci_dev, 0, 
                     PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_64, 
                     &s->dummy_bar0);

    msix_init_exclusive_bar(pci_dev, 64, 2, errp);

    uint32_t *config_ptr = (uint32_t *)pci_dev->config;
    for (int i = 0; i < 1024; i++) {
        if (pwrite(s->xdma_fd, &config_ptr[i], 4, HOST_VCONF_BASE + i * 4) != 4) {
            error_setg(errp, "Failed to sync initial config at offset 0x%x", i * 4);
            return;
        }
    }
    printf("[SCOPE PROXY] Initial Type 0 Header (4KB) injected into FPGA BRAM.\n");

    uint32_t dummy_ack = 1;
    if (pwrite(s->xdma_fd, &dummy_ack, 4, HOST_MBX_BASE + MBX_REG_ACK) != 4) {
        error_setg(errp, "Failed to reset FPGA Mailbox ACK");
        return;
    }

    s->poll_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, scope_poll_timer_cb, s);
    timer_mod(s->poll_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1);
    
    printf("[SCOPE PROXY] Hardware Mailbox Polling Loop started. Listening for AXI intercepts...\n");
    printf("[SCOPE PROXY] Initialization Complete. Ready for 香山 Linux enumeration!\n\n");
}

// -----------------------------------------------------------------------------
// 设备清理阶段
// -----------------------------------------------------------------------------
static void scope_proxy_exit(PCIDevice *pci_dev)
{
    ScopeProxyState *s = SCOPE_PROXY(pci_dev);
    if (s->poll_timer) {
        timer_free(s->poll_timer);
    }
    if (s->xdma_fd >= 0) {
        close(s->xdma_fd);
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