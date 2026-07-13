#ifndef HW_MISC_SCOPE_FPGA_VSWITCH_ABI_H
#define HW_MISC_SCOPE_FPGA_VSWITCH_ABI_H

#include "hw/misc/scope_fpga_proxy_abi.h"

/*
 * vSwitch packets keep the 32-byte DMA32 packet body used by the original
 * proxy.  Only the meaning of flags is extended so the software side can
 * distinguish bridge config writes from endpoint BAR traffic.
 *
 * flags[15:0]  = BDF encoded as bus[15:8], device[7:3], function[2:0]
 * flags[18:16] = BAR index for BAR packets, 7 for config packets
 * flags[23:20] = access size in bytes
 * flags[31:24] = write strobe byte mask, zero for reads
 */
#define SCOPE_VSWITCH_PKT_BDF(flags)   ((uint16_t)((flags) & 0xffffU))
#define SCOPE_VSWITCH_PKT_BAR(flags)   ((uint8_t)(((flags) >> 16) & 0x7U))
#define SCOPE_VSWITCH_PKT_SIZE(flags)  ((uint8_t)(((flags) >> 20) & 0xfU))
#define SCOPE_VSWITCH_PKT_WSTRB(flags) ((uint8_t)(((flags) >> 24) & 0xffU))

#define SCOPE_VSWITCH_CFG_BAR_TAG 7U
#define SCOPE_VSWITCH_MAKE_FLAGS(bdf, bar, size, wstrb) \
    ((((uint32_t)(wstrb) & 0xffU) << 24) | \
     (((uint32_t)(size) & 0xfU) << 20) | \
     (((uint32_t)(bar) & 0x7U) << 16) | \
     ((uint32_t)(bdf) & 0xffffU))

/* SQE_WRITE_DONE uses the legacy monitor layout plus a backend selector. */
#define SCOPE_VSWITCH_SQE_SLOT(flags)       ((uint16_t)((flags) & 0xffffU))
#define SCOPE_VSWITCH_SQE_QID(flags)        ((uint8_t)(((flags) >> 16) & 0xffU))
#define SCOPE_VSWITCH_SQE_BRESP(flags)      ((uint8_t)(((flags) >> 24) & 0x3U))
#define SCOPE_VSWITCH_SQE_BACKEND(flags)    ((uint8_t)(((flags) >> 26) & 0xfU))
#define SCOPE_VSWITCH_SQE_OVERFLOW(flags)   (((flags) & (1U << 31)) != 0)
#define SCOPE_VSWITCH_MAKE_SQE_FLAGS(slot, qid, bresp, backend, overflow) \
    (((overflow) ? (1U << 31) : 0U) | \
     (((uint32_t)(backend) & 0xfU) << 26) | \
     (((uint32_t)(bresp) & 0x3U) << 24) | \
     (((uint32_t)(qid) & 0xffU) << 16) | \
     ((uint32_t)(slot) & 0xffffU))

#endif
