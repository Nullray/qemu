#ifndef HW_MISC_SCOPE_FPGA_VSWITCH_IRQ_LATENCY_H
#define HW_MISC_SCOPE_FPGA_VSWITCH_IRQ_LATENCY_H

#include "qapi/error.h"

#define SCOPE_IRQ_LATENCY_MARKER_ADDR_LO_OFFSET 0xf0U
#define SCOPE_IRQ_LATENCY_MARKER_ADDR_HI_OFFSET 0xf4U
#define SCOPE_IRQ_LATENCY_MARKER_COMMIT_OFFSET  0xf8U
#define SCOPE_IRQ_LATENCY_MARKER_COMMIT_MAGIC   0x534d0001U
#define SCOPE_IRQ_LATENCY_MARKER_PAGE_MAGIC     0x53434d50U
#define SCOPE_IRQ_LATENCY_MARKER_MAGIC_OFFSET   0x00U
#define SCOPE_IRQ_LATENCY_MARKER_SEQ_OFFSET     0x04U

typedef struct ScopeIrqLatencyState ScopeIrqLatencyState;

ScopeIrqLatencyState *scope_irq_latency_create(uint32_t backend_id,
                                               uint32_t qid,
                                               uint16_t target_bdf,
                                               uint32_t target_samples,
                                               const char *output_path,
                                               Error **errp);
void scope_irq_latency_destroy(ScopeIrqLatencyState *s);

bool scope_irq_latency_set_coherent_transport(ScopeIrqLatencyState *s,
                                              int bypass_fd,
                                              uint64_t bypass_bar_size,
                                              uint64_t coherent_alias_base,
                                              uint64_t guest_ddr_base,
                                              uint64_t guest_ddr_size,
                                              size_t host_page_size,
                                              Error **errp);

void scope_irq_latency_record_cqe_absent(ScopeIrqLatencyState *s,
                                         uint32_t backend_id,
                                         uint32_t qid);

void scope_irq_latency_record_cqe(ScopeIrqLatencyState *s,
                                  uint32_t backend_id,
                                  uint32_t qid,
                                  uint16_t cid,
                                  uint16_t cq_tail);
void scope_irq_latency_record_intx(ScopeIrqLatencyState *s,
                                   bool old_level,
                                   bool new_level,
                                   bool write_ok,
                                   int64_t write_start_ns,
                                   int64_t write_done_ns);
bool scope_irq_latency_handle_cfg_write(ScopeIrqLatencyState *s,
                                        uint16_t bdf,
                                        uint32_t offset,
                                        unsigned int len,
                                        uint32_t wstrb,
                                        uint32_t value);
bool scope_irq_latency_poll(ScopeIrqLatencyState *s);
bool scope_irq_latency_sample_pending(const ScopeIrqLatencyState *s);

#endif
