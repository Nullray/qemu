/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Compile the production guest PCI DMA helper with a mock config/devres API. */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef uint64_t u64;
typedef uint32_t u32;
typedef uint8_t u8;
typedef uint64_t phys_addr_t;
typedef uint64_t dma_addr_t;
#define IS_ENABLED(option) (option)
#ifndef CONFIG_SCOPE_PCIE_P2P_DMA
#define CONFIG_SCOPE_PCIE_P2P_DMA 1
#endif
#define U64_MAX UINT64_MAX
#define PAGE_SIZE 4096U
#define PFN_PHYS(pfn) ((u64)(pfn) * PAGE_SIZE)
#define PCI_CAP_ID_VNDR 9
#define PCI_CFG_SPACE_SIZE 256
#define PCI_CLASS_SERIAL_USB_XHCI 0x0c0330
#define GFP_KERNEL 0
#define dev_err(dev, ...) ((void)(dev))
#define dev_info(dev, ...) ((void)(dev))
static unsigned long min_low_pfn = 0x80000000 / PAGE_SIZE;
static unsigned long max_pfn = 0x100000000ULL / PAGE_SIZE;

struct bus_dma_region { phys_addr_t cpu_start; dma_addr_t dma_start; u64 size, offset; };
struct device {
    const struct bus_dma_region *dma_range_map;
    u64 bus_dma_limit;
    void *ops, *allocation;
    void (*release)(void *);
    void *release_data;
};
struct pci_dev { struct device dev; u32 class; u8 cfg[256]; };
static void *get_dma_ops(struct device *dev) { return dev->ops; }
static void *devm_kzalloc(struct device *dev, size_t size, int flags)
{
    (void)flags;
    return dev->allocation = calloc(1, size);
}
static int devm_add_action_or_reset(struct device *dev,
                                    void (*release)(void *), void *data)
{
    dev->release = release;
    dev->release_data = data;
    return 0;
}
static int pci_read_config_byte(struct pci_dev *p, int off, u8 *value)
{
    if (off < 0 || off >= 256) return -1;
    *value = p->cfg[off]; return 0;
}
static int pci_read_config_dword(struct pci_dev *p, int off, u32 *value)
{
    if (off < 0 || off > 252) return -1;
    *value = (u32)p->cfg[off] | ((u32)p->cfg[off + 1] << 8) |
             ((u32)p->cfg[off + 2] << 16) | ((u32)p->cfg[off + 3] << 24);
    return 0;
}
static int pci_write_config_dword(struct pci_dev *p, int off, u32 value)
{
    if (off < 0 || off > 252) return -1;
    for (unsigned i = 0; i < 4; i++) p->cfg[off + i] = value >> (8 * i);
    return 0;
}
static int find_from(struct pci_dev *p, int start, int id)
{
    for (unsigned count = 0; start >= 0x40 && start <= 0xfc && count < 48; count++) {
        if (p->cfg[start] == id) return start;
        start = p->cfg[start + 1];
    }
    return 0;
}
static int pci_find_capability(struct pci_dev *p, int id)
{
    return find_from(p, p->cfg[0x34], id);
}
static int pci_find_next_capability(struct pci_dev *p, int pos, int id)
{
    /* Matches Linux: pos=0 means read offset 1, NOT capability-list head. */
    return find_from(p, p->cfg[pos + 1], id);
}

#include SCOPE_DMA_HEADER

static void write64(struct pci_dev *p, int off, u64 value)
{
    pci_write_config_dword(p, off, value);
    pci_write_config_dword(p, off + 4, value >> 32);
}
static struct pci_dev fixture(void)
{
    struct pci_dev p = { .class = PCI_CLASS_SERIAL_USB_XHCI };
    p.cfg[0] = 0x22; p.cfg[1] = 0x10;  /* physical AMD ID */
    p.cfg[0x34] = 0x70;
    p.cfg[0x70] = 0x10; p.cfg[0x71] = 0xb0;
    p.cfg[0xb0] = PCI_CAP_ID_VNDR;
    p.cfg[0xb2] = 40; p.cfg[0xb3] = 1;
    pci_write_config_dword(&p, 0xb4, SCOPE_P2P_SIGNATURE);
    pci_write_config_dword(&p, 0xb8, 3);
    write64(&p, 0xc0, 0x80000000);
    write64(&p, 0xc8, 0x980000000);
    write64(&p, 0xd0, 0x80000000);
    return p;
}
static void release(struct pci_dev *p)
{
    if (p->dev.release) p->dev.release(p->dev.release_data);
    free(p->dev.allocation);
    p->dev.release = NULL; p->dev.allocation = NULL;
}

int main(void)
{
    struct pci_dev p = fixture();
    u32 ready;
    if (!CONFIG_SCOPE_PCIE_P2P_DMA) {
        assert(scope_p2p_dma_configure(&p) == -EOPNOTSUPP);
        puts("PASS: disabled guest DMA helper rejects SCOPE controller");
        return 0;
    }
    struct bus_dma_region old[] = {{ .size = 1 }, { 0 }};
    p.dev.dma_range_map = old; p.dev.bus_dma_limit = 0xffffffff;
    assert(scope_p2p_dma_configure(&p) == 0);
    pci_read_config_dword(&p, 0xbc, &ready);
    assert(ready == SCOPE_P2P_READY);
    const struct bus_dma_region *m = p.dev.dma_range_map;
    assert(m[0].cpu_start == 0x80000000 && m[0].dma_start == 0x980000000);
    assert(m[0].size == 0x80000000 && m[1].size == 0);
    for (u64 offset = 0; offset < 0x80000000; offset += 0x12345) {
        u64 pa = m[0].cpu_start + offset;
        u64 dma = pa - m[0].offset;
        assert(dma == m[0].dma_start + offset && dma + m[0].offset == pa);
    }
    release(&p);
    assert(p.dev.dma_range_map == old && p.dev.bus_dma_limit == 0xffffffff);
    pci_read_config_dword(&p, 0xbc, &ready); assert(ready == 0);
    assert(scope_p2p_dma_configure(&p) == 0); release(&p); /* rebind */
    for (unsigned fault = 0; fault < 8; fault++) {
        p = fixture();
        switch (fault) {
        case 0: p.class = 0x010802; break;
        case 1: p.cfg[0xb2] = 32; break;
        case 2: p.cfg[0xb3] = 2; break;
        case 3: pci_write_config_dword(&p, 0xb8, 1); break;
        case 4: p.dev.ops = &p; break;
        case 5: write64(&p, 0xd0, 0x40000000); break;
        case 6: write64(&p, 0xc8, UINT64_MAX - 1024); break;
        case 7: write64(&p, 0xc0, 0x80000001); break;
        }
        assert(scope_p2p_dma_configure(&p) == -EINVAL);
        assert(!p.dev.dma_range_map);
    }
    p = fixture(); p.cfg[0x34] = 0; p.class = 0x010802;
    p.dev.dma_range_map = old;
    assert(scope_p2p_dma_configure(&p) == 0 && p.dev.dma_range_map == old);
    puts("PASS: guest DMA map, inverse, restore/rebind, 8 malformed descriptions, non-SCOPE device");
    return 0;
}
