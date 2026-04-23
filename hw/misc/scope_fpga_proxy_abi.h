#ifndef HW_MISC_SCOPE_FPGA_PROXY_ABI_H
#define HW_MISC_SCOPE_FPGA_PROXY_ABI_H

#include <stdint.h>
#include <sys/ioctl.h>

#define XDMA_IOC_MAGIC 'x'
#define XDMA_DMA32_DB_MMAP_PGOFF 0x100000U
#define XDMA_DMA32_PKT_MAGIC 0x5844504bU

struct scope_xdma_dma32_doorbell {
    uint32_t size;
    uint32_t reserved;
    uint64_t dma_addr;
};

#define XDMA_IOC_DMA32_DB_ALLOC _IOWR(XDMA_IOC_MAGIC, 0x20, struct scope_xdma_dma32_doorbell)
#define XDMA_IOC_DMA32_DB_FREE  _IO(XDMA_IOC_MAGIC, 0x21)
#define XDMA_IOC_DMA32_DB_QUERY _IOWR(XDMA_IOC_MAGIC, 0x22, struct scope_xdma_dma32_doorbell)

struct scope_dma32_packet {
    uint32_t magic;
    uint32_t type;
    uint32_t flags;
    uint32_t len;
    uint32_t seq;
    uint32_t bar_offset;
    uint32_t data;
    /*
     * CFG_WRITE keeps using this last dword as compact metadata.
     * BAR packets reuse it as the upper 32 bits of the 64-bit data lane.
     */
    uint32_t guest_addr_lo;
};

#define SCOPE_PKT_TYPE_CFG_WRITE 1U
#define SCOPE_PKT_TYPE_BAR_WRITE 2U
#define SCOPE_PKT_TYPE_BAR_READ  3U

#endif
