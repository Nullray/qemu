/*
 * Intel 82580 mediated backend for scope-fpga-vswitch.
 *
 * This file is included by scope_fpga_vswitch.c so the backend can use the
 * manager's private coherent-alias and physical-BAR helpers without making
 * those helpers part of QEMU's public device API.
 */

#define SCOPE_IGB_BAR0_SIZE             0x80000U
#define SCOPE_IGB_DESC_SIZE             16U
#define SCOPE_IGB_PENDING_DEPTH         64U
#define SCOPE_IGB_VISIBILITY_RETRY_US   100U
#define SCOPE_IGB_RESET_TIMEOUT_US      200000U
#define SCOPE_IGB_SUPPORTED_CAUSES      E1000_EICR_LEGACY_MASK

/* Linux igb uses the legacy queue-0 aliases for 82580 queues 0..3. */
#define SCOPE_IGB_RDBAL0                E1000_RDBAL_A(0)
#define SCOPE_IGB_RDBAH0                E1000_RDBAH_A(0)
#define SCOPE_IGB_RDLEN0                E1000_RDLEN_A(0)
#define SCOPE_IGB_SRRCTL0               E1000_SRRCTL_A(0)
#define SCOPE_IGB_RDT0                  E1000_RDT_A(0)
#define SCOPE_IGB_RXDCTL0               E1000_RXDCTL_A(0)
#define SCOPE_IGB_TDBAL0                E1000_TDBAL_A(0)
#define SCOPE_IGB_TDBAH0                E1000_TDBAH_A(0)
#define SCOPE_IGB_TDLEN0                E1000_TDLEN_A(0)
#define SCOPE_IGB_TDT0                  E1000_TDT_A(0)
#define SCOPE_IGB_TXDCTL0               E1000_TXDCTL_A(0)

typedef struct ScopeIgbPendingTail {
    bool valid;
    bool bar_done;
    bool tx;
    uint32_t seq;
    uint16_t tail;
    int64_t pending_since_us;
    int64_t next_retry_us;
} ScopeIgbPendingTail;

typedef struct ScopeIgbQueueState {
    uint64_t guest_base;
    uint64_t translated_base;
    uint32_t ring_bytes;
    uint32_t control;
    uint32_t srrctl;
    uint16_t forwarded_tail;
    bool tail_valid;
} ScopeIgbQueueState;

struct ScopeIgbState {
    ScopeIgbQueueState tx;
    ScopeIgbQueueState rx;
    ScopeIgbPendingTail pending[SCOPE_IGB_PENDING_DEPTH];
    uint16_t pending_head;
    uint16_t pending_count;
    uint32_t virtual_icr;
    uint32_t virtual_ims;
    uint32_t last_status;
    uint64_t tx_descriptors_patched;
    uint64_t rx_descriptors_patched;
    uint64_t address_errors;
};

typedef struct ScopeIgbTxDesc {
    uint64_t buffer_addr;
    uint32_t cmd_type_len;
    uint32_t olinfo_status;
} ScopeIgbTxDesc;

static bool scope_igb_reset_real(ScopeProxyState *s, Error **errp);

static bool scope_igb_real_read32(ScopeProxyState *s, uint32_t reg, uint32_t *value)
{
    return scope_real_bar_read32(s, reg, value);
}

static bool scope_igb_real_write32(ScopeProxyState *s, uint32_t reg, uint32_t value)
{
    return scope_real_bar_write32(s, reg, value);
}

static uint32_t scope_igb_rx_packet_size(const ScopeIgbState *igb)
{
    uint32_t size = (igb->rx.srrctl & E1000_SRRCTL_BSIZEPKT_MASK) <<
                    E1000_SRRCTL_BSIZEPKT_SHIFT;

    return size ? size : 2048U;
}

static uint32_t scope_igb_rx_header_size(const ScopeIgbState *igb)
{
    return (igb->rx.srrctl & E1000_SRRCTL_BSIZEHDR_MASK) >>
           E1000_SRRCTL_BSIZEHDRSIZE_SHIFT;
}

static bool scope_igb_sync_ring(ScopeProxyState *s, ScopeIgbQueueState *q,
                                bool tx, Error **errp)
{
    uint64_t translated = 0;
    uint32_t low_reg = tx ? SCOPE_IGB_TDBAL0 : SCOPE_IGB_RDBAL0;
    uint32_t high_reg = tx ? SCOPE_IGB_TDBAH0 : SCOPE_IGB_RDBAH0;
    uint32_t len_reg = tx ? SCOPE_IGB_TDLEN0 : SCOPE_IGB_RDLEN0;
    size_t validate_len = q->ring_bytes ? q->ring_bytes : SCOPE_IGB_DESC_SIZE;

    if (!q->guest_base || !q->ring_bytes) {
        if (errp) {
            error_setg(errp, "82580 %s ring is incomplete: base=0x%016" PRIx64
                       " bytes=0x%x", tx ? "TX" : "RX", q->guest_base,
                       q->ring_bytes);
        }
        return false;
    }
    if (q->ring_bytes &&
        ((q->ring_bytes & 0x7fU) || q->ring_bytes < 80U * SCOPE_IGB_DESC_SIZE ||
         q->ring_bytes > 4096U * SCOPE_IGB_DESC_SIZE)) {
        if (errp) {
            error_setg(errp, "82580 %s ring length 0x%x is invalid",
                       tx ? "TX" : "RX", q->ring_bytes);
        }
        return false;
    }
    if (!scope_translate_guest_pa_for_real_dma(s, q->guest_base,
                                                validate_len, &translated)) {
        if (errp) {
            error_setg(errp, "82580 %s ring base 0x%016" PRIx64
                       " is outside guest DDR", tx ? "TX" : "RX",
                       q->guest_base);
        }
        return false;
    }

    q->translated_base = translated;
    return scope_igb_real_write32(s, low_reg, (uint32_t)translated) &&
           scope_igb_real_write32(s, high_reg, (uint32_t)(translated >> 32)) &&
           scope_igb_real_write32(s, len_reg, q->ring_bytes);
}

static bool scope_igb_sync_ring_report(ScopeProxyState *s,
                                       ScopeIgbQueueState *q, bool tx)
{
    Error *local_err = NULL;
    bool ok = scope_igb_sync_ring(s, q, tx, &local_err);

    if (!ok && local_err) {
        error_report_err(local_err);
    }
    return ok;
}

static void scope_igb_mark_ring_reconfigured(ScopeIgbQueueState *q)
{
    q->translated_base = 0;
    q->forwarded_tail = 0;
    q->tail_valid = false;
}

static bool scope_igb_read_stable_desc(ScopeProxyState *s, uint64_t guest_pa,
                                       void *out)
{
    uint8_t first[SCOPE_IGB_DESC_SIZE];
    uint8_t second[SCOPE_IGB_DESC_SIZE];

    if (!scope_guest_mem_read(s, guest_pa, first, sizeof(first)) ||
        !scope_guest_mem_read(s, guest_pa, second, sizeof(second)) ||
        memcmp(first, second, sizeof(first))) {
        return false;
    }
    memcpy(out, second, sizeof(second));
    return true;
}

static bool scope_igb_patch_dma_addr(ScopeProxyState *s, uint64_t *addr,
                                     size_t len, const char *what)
{
    uint64_t guest_pa = 0;
    uint64_t translated = 0;

    if (!*addr) {
        return true;
    }
    if (!scope_translate_cmd_dma_addr_for_real(s, *addr, MAX(len, (size_t)1),
                                                what, &guest_pa, &translated)) {
        return false;
    }
    *addr = translated;
    return true;
}

static bool scope_igb_patch_tx_range(ScopeProxyState *s, uint16_t new_tail)
{
    ScopeIgbState *igb = s->active->igb;
    ScopeIgbQueueState *q = &igb->tx;
    uint16_t depth;
    uint16_t index;
    unsigned int count = 0;
    uint64_t verify_pa = 0;
    ScopeIgbTxDesc verify_desc = { 0 };
    bool verify_valid = false;

    if (!q->guest_base || !q->ring_bytes ||
        !(q->control & E1000_TXDCTL_QUEUE_ENABLE)) {
        return false;
    }
    depth = q->ring_bytes / SCOPE_IGB_DESC_SIZE;
    if (!depth || new_tail >= depth) {
        return false;
    }
    index = q->tail_valid ? q->forwarded_tail : 0;
    while (index != new_tail && count++ < depth) {
        ScopeIgbTxDesc desc;
        uint64_t guest_pa = q->guest_base + index * SCOPE_IGB_DESC_SIZE;
        uint32_t cmd;
        uint32_t data_len;
        uint64_t buffer_addr;
        bool advanced;
        bool context;

        if (!scope_igb_read_stable_desc(s, guest_pa, &desc)) {
            return false;
        }
        cmd = le32_to_cpu(desc.cmd_type_len);
        advanced = (cmd & E1000_TXD_CMD_DEXT) != 0;
        context = advanced &&
                  ((cmd & E1000_ADVTXD_DTYP_DATA) == E1000_ADVTXD_DTYP_CTXT);
        if (!context) {
            data_len = advanced ? (cmd & 0x000fffffU) : (cmd & 0x0000ffffU);
            buffer_addr = le64_to_cpu(desc.buffer_addr);
            if (!scope_igb_patch_dma_addr(s, &buffer_addr, data_len,
                                           "igb-tx-buffer")) {
                igb->address_errors++;
                igb->virtual_icr |= E1000_ICR_TXQE;
                return false;
            }
            desc.buffer_addr = cpu_to_le64(buffer_addr);
            if (cmd & E1000_ADVTXD_MAC_TSTAMP) {
                cmd &= ~E1000_ADVTXD_MAC_TSTAMP;
                desc.cmd_type_len = cpu_to_le32(cmd);
            }
            if (!scope_guest_mem_write(s, guest_pa, &desc, sizeof(desc))) {
                return false;
            }
            verify_pa = guest_pa;
            verify_desc = desc;
            verify_valid = true;
            igb->tx_descriptors_patched++;
        }
        index = (index + 1U) % depth;
    }
    if (index != new_tail) {
        return false;
    }
    if (verify_valid) {
        ScopeIgbTxDesc visible;

        if (!scope_igb_read_stable_desc(s, verify_pa, &visible) ||
            memcmp(&visible, &verify_desc, sizeof(visible))) {
            return false;
        }
    }
    smp_mb();
    q->forwarded_tail = new_tail;
    q->tail_valid = true;
    return scope_igb_real_write32(s, SCOPE_IGB_TDT0, new_tail);
}

static bool scope_igb_patch_rx_range(ScopeProxyState *s, uint16_t new_tail)
{
    ScopeIgbState *igb = s->active->igb;
    ScopeIgbQueueState *q = &igb->rx;
    uint16_t depth;
    uint16_t index;
    unsigned int count = 0;
    uint64_t verify_pa = 0;
    union e1000_adv_rx_desc verify_desc = { 0 };

    if (!q->guest_base || !q->ring_bytes ||
        !(q->control & E1000_RXDCTL_QUEUE_ENABLE)) {
        return false;
    }
    depth = q->ring_bytes / SCOPE_IGB_DESC_SIZE;
    if (!depth || new_tail >= depth) {
        return false;
    }
    index = q->tail_valid ? q->forwarded_tail : 0;
    while (index != new_tail) {
        union e1000_adv_rx_desc desc;
        uint64_t guest_pa = q->guest_base + index * SCOPE_IGB_DESC_SIZE;
        uint64_t pkt_addr;
        uint64_t hdr_addr;

        if (count++ >= depth ||
            !scope_igb_read_stable_desc(s, guest_pa, &desc)) {
            return false;
        }
        pkt_addr = le64_to_cpu(desc.read.pkt_addr);
        hdr_addr = le64_to_cpu(desc.read.hdr_addr);
        if (!scope_igb_patch_dma_addr(s, &pkt_addr,
                                      scope_igb_rx_packet_size(igb),
                                      "igb-rx-packet") ||
            !scope_igb_patch_dma_addr(s, &hdr_addr,
                                      scope_igb_rx_header_size(igb),
                                      "igb-rx-header")) {
            igb->address_errors++;
            igb->virtual_icr |= E1000_ICR_RXO;
            return false;
        }
        desc.read.pkt_addr = cpu_to_le64(pkt_addr);
        desc.read.hdr_addr = cpu_to_le64(hdr_addr);
        if (!scope_guest_mem_write(s, guest_pa, &desc, sizeof(desc))) {
            return false;
        }
        verify_pa = guest_pa;
        verify_desc = desc;
        igb->rx_descriptors_patched++;
        index = (index + 1U) % depth;
    }
    if (verify_pa) {
        union e1000_adv_rx_desc visible;

        if (!scope_igb_read_stable_desc(s, verify_pa, &visible) ||
            memcmp(&visible, &verify_desc, sizeof(visible))) {
            return false;
        }
    }
    smp_mb();
    q->forwarded_tail = new_tail;
    q->tail_valid = true;
    return scope_igb_real_write32(s, SCOPE_IGB_RDT0, new_tail);
}

static ScopeIgbPendingTail *scope_igb_pending_at(ScopeIgbState *igb,
                                                 unsigned int n)
{
    return &igb->pending[(igb->pending_head + n) % SCOPE_IGB_PENDING_DEPTH];
}

static bool scope_igb_defer_tail(ScopeProxyState *s, uint32_t seq,
                                 uint32_t offset, uint64_t data)
{
    ScopeIgbState *igb = s->active->igb;
    ScopeIgbPendingTail *pending;

    if (!igb || igb->pending_count == SCOPE_IGB_PENDING_DEPTH) {
        return false;
    }
    pending = scope_igb_pending_at(igb, igb->pending_count++);
    *pending = (ScopeIgbPendingTail) {
        .valid = true,
        .tx = offset == SCOPE_IGB_TDT0,
        .seq = seq,
        .tail = scope_extract_dword32(data, offset),
        .pending_since_us = g_get_monotonic_time(),
        .next_retry_us = g_get_monotonic_time(),
    };
    return true;
}

static bool scope_igb_mark_bar_done(ScopeProxyState *s, uint32_t seq)
{
    ScopeIgbState *igb = s->active->igb;
    unsigned int i;

    for (i = 0; igb && i < igb->pending_count; i++) {
        ScopeIgbPendingTail *pending = scope_igb_pending_at(igb, i);
        if (pending->valid && pending->seq == seq) {
            pending->bar_done = true;
            return true;
        }
    }
    return false;
}

static bool scope_igb_is_tail_write(uint32_t offset, uint8_t size_bytes,
                                    uint8_t wstrb)
{
    uint8_t mask = scope_pack_wstrb4_for_offset(offset);

    return size_bytes == 4 && (wstrb & mask) == mask &&
           (offset == SCOPE_IGB_TDT0 || offset == SCOPE_IGB_RDT0);
}

static bool scope_igb_is_ptp_register(uint32_t offset)
{
    return (offset >= 0x0b600U && offset < 0x0b700U) ||
           (offset >= 0x0c000U && offset < 0x0c100U);
}

static bool scope_igb_bar_read(ScopeProxyState *s, uint32_t offset,
                               uint8_t size_bytes, uint64_t *data)
{
    ScopeIgbState *igb = s->active->igb;
    uint32_t value = 0;

    if (!igb || size_bytes != 4) {
        return false;
    }
    switch (offset) {
    case E1000_ICR:
        value = igb->virtual_icr & SCOPE_IGB_SUPPORTED_CAUSES;
        if (value & igb->virtual_ims) {
            value |= E1000_ICR_INT_ASSERTED;
        }
        igb->virtual_icr &= ~(value & ~E1000_ICR_INT_ASSERTED);
        break;
    case E1000_IMS:
        value = igb->virtual_ims;
        break;
    case SCOPE_IGB_TDBAL0: value = (uint32_t)igb->tx.guest_base; break;
    case SCOPE_IGB_TDBAH0: value = (uint32_t)(igb->tx.guest_base >> 32); break;
    case SCOPE_IGB_TDLEN0: value = igb->tx.ring_bytes; break;
    case SCOPE_IGB_TDT0: value = igb->tx.forwarded_tail; break;
    case SCOPE_IGB_TXDCTL0: value = igb->tx.control; break;
    case SCOPE_IGB_RDBAL0: value = (uint32_t)igb->rx.guest_base; break;
    case SCOPE_IGB_RDBAH0: value = (uint32_t)(igb->rx.guest_base >> 32); break;
    case SCOPE_IGB_RDLEN0: value = igb->rx.ring_bytes; break;
    case SCOPE_IGB_RDT0: value = igb->rx.forwarded_tail; break;
    case SCOPE_IGB_RXDCTL0: value = igb->rx.control; break;
    case SCOPE_IGB_SRRCTL0: value = igb->rx.srrctl; break;
    case E1000_MRQC:
    case E1000_EICR:
    case E1000_EIMS:
    case E1000_EIMC:
        value = 0;
        break;
    default:
        if (scope_igb_is_ptp_register(offset)) {
            value = 0;
        } else if (!scope_igb_real_read32(s, offset, &value)) {
            return false;
        }
        break;
    }
    *data = scope_pack_dword32_for_offset(value, offset);
    return true;
}

static bool scope_igb_bar_write(ScopeProxyState *s, uint32_t offset,
                                uint64_t data, uint8_t wstrb,
                                uint8_t size_bytes)
{
    ScopeIgbState *igb = s->active->igb;
    uint32_t value;

    if (!igb || size_bytes != 4 ||
        (wstrb & scope_pack_wstrb4_for_offset(offset)) !=
        scope_pack_wstrb4_for_offset(offset)) {
        return false;
    }
    value = scope_extract_dword32(data, offset);
    switch (offset) {
    case E1000_ICR:
        return true;
    case E1000_CTRL:
        if (value & E1000_CTRL_RST) {
            return scope_igb_reset_real(s, NULL);
        }
        return scope_igb_real_write32(s, offset, value);
    case E1000_IMS:
        igb->virtual_ims |= value & SCOPE_IGB_SUPPORTED_CAUSES;
        return true;
    case E1000_IMC:
        igb->virtual_ims &= ~(value & SCOPE_IGB_SUPPORTED_CAUSES);
        return true;
    case E1000_ICS:
        igb->virtual_icr |= value & SCOPE_IGB_SUPPORTED_CAUSES;
        return true;
    case E1000_EIMS:
    case E1000_EIMC:
        return true;
    case SCOPE_IGB_TDBAL0:
        igb->tx.guest_base = (igb->tx.guest_base & 0xffffffff00000000ULL) |
                             (value & ~0x7fU);
        scope_igb_mark_ring_reconfigured(&igb->tx);
        return true;
    case SCOPE_IGB_TDBAH0:
        igb->tx.guest_base = (igb->tx.guest_base & UINT32_MAX) |
                             ((uint64_t)value << 32);
        scope_igb_mark_ring_reconfigured(&igb->tx);
        return true;
    case SCOPE_IGB_TDLEN0:
        igb->tx.ring_bytes = value;
        scope_igb_mark_ring_reconfigured(&igb->tx);
        return true;
    case SCOPE_IGB_TXDCTL0:
        igb->tx.control = value;
        if ((value & E1000_TXDCTL_QUEUE_ENABLE) &&
            !scope_igb_sync_ring_report(s, &igb->tx, true)) {
            return false;
        }
        return scope_igb_real_write32(s, offset, value);
    case SCOPE_IGB_RDBAL0:
        igb->rx.guest_base = (igb->rx.guest_base & 0xffffffff00000000ULL) |
                             (value & ~0x7fU);
        scope_igb_mark_ring_reconfigured(&igb->rx);
        return true;
    case SCOPE_IGB_RDBAH0:
        igb->rx.guest_base = (igb->rx.guest_base & UINT32_MAX) |
                             ((uint64_t)value << 32);
        scope_igb_mark_ring_reconfigured(&igb->rx);
        return true;
    case SCOPE_IGB_RDLEN0:
        igb->rx.ring_bytes = value;
        scope_igb_mark_ring_reconfigured(&igb->rx);
        return true;
    case SCOPE_IGB_RXDCTL0:
        igb->rx.control = value;
        if ((value & E1000_RXDCTL_QUEUE_ENABLE) &&
            !scope_igb_sync_ring_report(s, &igb->rx, false)) {
            return false;
        }
        return scope_igb_real_write32(s, offset, value);
    case SCOPE_IGB_SRRCTL0:
        igb->rx.srrctl = value;
        return scope_igb_real_write32(s, offset, value);
    case E1000_MRQC:
        return scope_igb_real_write32(s, offset, 0);
    default:
        if (scope_igb_is_ptp_register(offset)) {
            return true;
        }
        return scope_real_bar_write(s, offset, data, wstrb, size_bytes);
    }
}

static bool scope_igb_poll(ScopeProxyState *s, int64_t now_us)
{
    ScopeIgbState *igb = s->active->igb;
    ScopeIgbPendingTail *pending;
    uint32_t causes = 0;
    uint32_t status = 0;
    bool progressed = false;

    if (!igb) {
        return false;
    }
    if (scope_igb_real_read32(s, E1000_ICR, &causes)) {
        igb->virtual_icr |= causes & SCOPE_IGB_SUPPORTED_CAUSES;
        progressed |= (causes & SCOPE_IGB_SUPPORTED_CAUSES) != 0;
    }
    if (scope_igb_real_read32(s, E1000_STATUS, &status) &&
        ((status ^ igb->last_status) & E1000_STATUS_LU)) {
        igb->virtual_icr |= E1000_ICR_LSC;
        igb->last_status = status;
        progressed = true;
    }
    if (!igb->pending_count) {
        s->active->intx_pending =
            (igb->virtual_icr & igb->virtual_ims & SCOPE_IGB_SUPPORTED_CAUSES) != 0;
        return progressed;
    }
    pending = scope_igb_pending_at(igb, 0);
    if (!pending->bar_done && s->bar_done_timeout_us &&
        now_us - pending->pending_since_us >= s->bar_done_timeout_us) {
        pending->bar_done = true;
        SCOPE_PRINTF("[SCOPE IGB][BAR_DONE_INFERRED] backend=%u seq=%u\n",
                     s->active->id, pending->seq);
    }
    if (pending->bar_done && now_us >= pending->next_retry_us) {
        bool ok = pending->tx ? scope_igb_patch_tx_range(s, pending->tail) :
                               scope_igb_patch_rx_range(s, pending->tail);
        if (ok) {
            memset(pending, 0, sizeof(*pending));
            igb->pending_head = (igb->pending_head + 1U) % SCOPE_IGB_PENDING_DEPTH;
            igb->pending_count--;
            progressed = true;
        } else {
            pending->next_retry_us = now_us + SCOPE_IGB_VISIBILITY_RETRY_US;
        }
    }
    s->active->intx_pending =
        (igb->virtual_icr & igb->virtual_ims & SCOPE_IGB_SUPPORTED_CAUSES) != 0;
    return progressed;
}

static void scope_igb_process_bar_packet(
    ScopeProxyState *s, const struct scope_dma32_packet *pkt)
{
    uint8_t size_bytes = SCOPE_VSWITCH_PKT_SIZE(pkt->flags);
    uint8_t wstrb = SCOPE_VSWITCH_PKT_WSTRB(pkt->flags);
    uint64_t lane_data = ((uint64_t)pkt->guest_addr_lo << 32) | pkt->data;
    uint64_t data = UINT64_MAX;
    bool ok = false;

    switch (pkt->type) {
    case SCOPE_PKT_TYPE_BAR_WRITE:
        if (scope_igb_is_tail_write(pkt->bar_offset, size_bytes, wstrb)) {
            ok = scope_igb_defer_tail(s, pkt->seq, pkt->bar_offset,
                                      lane_data);
            if (ok) {
                ok = scope_write_bar_response(s, pkt->seq, 0, 0,
                                              false, true);
            }
        } else {
            ok = scope_igb_bar_write(s, pkt->bar_offset, lane_data,
                                     wstrb, size_bytes);
            if (ok) {
                ok = scope_write_bar_response(s, pkt->seq, 0, 0,
                                              false, false);
            }
        }
        if (!ok) {
            scope_write_bar_response(s, pkt->seq, 0x2U, 0, false, false);
        }
        break;
    case SCOPE_PKT_TYPE_BAR_WRITE_DONE:
        if (!scope_igb_mark_bar_done(s, pkt->seq)) {
            SCOPE_PRINTF("[SCOPE IGB][BAR_DONE][STALE] backend=%u seq=%u\n",
                         s->active->id, pkt->seq);
        }
        break;
    case SCOPE_PKT_TYPE_BAR_READ:
        ok = scope_igb_bar_read(s, pkt->bar_offset, size_bytes, &data);
        scope_write_bar_response(s, pkt->seq, ok ? 0 : 0x2U,
                                 ok ? data : UINT64_MAX, true, false);
        break;
    default:
        break;
    }
}

static bool scope_igb_all_sibling_functions_unbound(const char *bdf, Error **errp)
{
    unsigned int domain, bus, dev, fn;
    unsigned int i;
    char tail;

    if (sscanf(bdf, "%4x:%2x:%2x.%1x%c", &domain, &bus, &dev, &fn, &tail) != 4) {
        error_setg(errp, "invalid 82580 BDF '%s'", bdf);
        return false;
    }
    for (i = 0; i < 4; i++) {
        g_autofree char *driver = g_strdup_printf(
            "/sys/bus/pci/devices/%04x:%02x:%02x.%u/driver", domain, bus, dev, i);
        if (g_file_test(driver, G_FILE_TEST_EXISTS)) {
            error_setg(errp, "82580 sibling %04x:%02x:%02x.%u is still bound; "
                       "unbind all four functions before starting QEMU",
                       domain, bus, dev, i);
            return false;
        }
    }
    return true;
}

static bool scope_igb_backend_preflight(ScopeProxyState *s, Error **errp)
{
    g_autofree char *config_path = NULL;
    uint8_t ids[4];
    uint16_t vendor;
    uint16_t device;
    int fd;

    if (!scope_igb_all_sibling_functions_unbound(
            s->active->real_host_bdf, errp)) {
        return false;
    }
    config_path = g_strdup_printf("/sys/bus/pci/devices/%s/config",
                                  s->active->real_host_bdf);
    fd = open(config_path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        error_setg_errno(errp, errno, "failed to open %s", config_path);
        return false;
    }
    if (pread(fd, ids, sizeof(ids), PCI_VENDOR_ID) != sizeof(ids)) {
        error_setg_errno(errp, errno, "failed to read PCI IDs from %s",
                         config_path);
        close(fd);
        return false;
    }
    close(fd);

    vendor = lduw_le_p(ids);
    device = lduw_le_p(ids + 2);
    if (vendor != 0x8086 || device != 0x150e) {
        error_setg(errp, "IGB backend %s is %04x:%04x, expected 8086:150e",
                   s->active->real_host_bdf, vendor, device);
        return false;
    }
    return true;
}

static bool scope_igb_reset_real(ScopeProxyState *s, Error **errp)
{
    ScopeIgbState *igb = s->active->igb;
    uint32_t ctrl = 0;
    uint32_t status = 0;
    int64_t deadline;

    if (!scope_igb_real_write32(s, E1000_IMC, UINT32_MAX) ||
        !scope_igb_real_write32(s, E1000_RCTL, 0) ||
        !scope_igb_real_write32(s, E1000_TCTL, E1000_TCTL_PSP) ||
        !scope_igb_real_read32(s, E1000_CTRL, &ctrl)) {
        if (errp) {
            error_setg(errp, "failed to start reset of real 82580 %s",
                       s->active->real_host_bdf);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "SCOPE: failed to start reset of real 82580 %s\n",
                          s->active->real_host_bdf);
        }
        return false;
    }
    g_usleep(10000);
    if (!scope_igb_real_write32(s, E1000_CTRL, ctrl | E1000_CTRL_RST)) {
        if (errp) {
            error_setg(errp, "failed to assert reset of real 82580 %s",
                       s->active->real_host_bdf);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "SCOPE: failed to assert reset of real 82580 %s\n",
                          s->active->real_host_bdf);
        }
        return false;
    }

    deadline = g_get_monotonic_time() + SCOPE_IGB_RESET_TIMEOUT_US;
    do {
        if (!scope_igb_real_read32(s, E1000_CTRL, &ctrl)) {
            return false;
        }
        if (!(ctrl & E1000_CTRL_RST)) {
            break;
        }
        g_usleep(1000);
    } while (g_get_monotonic_time() < deadline);
    if (ctrl & E1000_CTRL_RST) {
        if (errp) {
            error_setg(errp, "timed out resetting real 82580 %s",
                       s->active->real_host_bdf);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "SCOPE: timed out resetting real 82580 %s\n",
                          s->active->real_host_bdf);
        }
        return false;
    }

    scope_igb_real_write32(s, E1000_IMC, UINT32_MAX);
    scope_igb_real_read32(s, E1000_ICR, &ctrl);
    scope_igb_real_read32(s, E1000_STATUS, &status);
    if (igb) {
        memset(&igb->tx, 0, sizeof(igb->tx));
        memset(&igb->rx, 0, sizeof(igb->rx));
        memset(igb->pending, 0, sizeof(igb->pending));
        igb->pending_head = 0;
        igb->pending_count = 0;
        igb->virtual_icr = 0;
        igb->virtual_ims = 0;
        igb->last_status = status;
        s->active->intx_pending = false;
    }
    return true;
}

static bool scope_igb_backend_realize(ScopeProxyState *s, Error **errp)
{
    ScopeIgbState *igb;

    if (s->active->real_bar0_size != SCOPE_IGB_BAR0_SIZE) {
        error_setg(errp, "82580 %s BAR0 size is 0x%zx, expected 0x%x",
                   s->active->real_host_bdf, s->active->real_bar0_size,
                   SCOPE_IGB_BAR0_SIZE);
        return false;
    }
    igb = g_new0(ScopeIgbState, 1);
    s->active->igb = igb;
    return scope_igb_reset_real(s, errp);
}

static void scope_igb_backend_cleanup(ScopeProxyState *s, ScopeBackend *be)
{
    ScopeBackend *saved = s->active;

    if (!be->igb) {
        return;
    }
    s->active = be;
    scope_igb_real_write32(s, E1000_IMC, UINT32_MAX);
    scope_igb_real_write32(s, E1000_RCTL, 0);
    scope_igb_real_write32(s, E1000_TCTL, E1000_TCTL_PSP);
    g_free(be->igb);
    be->igb = NULL;
    s->active = saved;
}

static const ScopeBackendOps scope_igb_backend_ops = {
    .name = "igb",
    .preflight = scope_igb_backend_preflight,
    .realize = scope_igb_backend_realize,
    .cleanup = scope_igb_backend_cleanup,
    .process_bar_packet = scope_igb_process_bar_packet,
    .poll = scope_igb_poll,
};
