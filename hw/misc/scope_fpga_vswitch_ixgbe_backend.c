/*
 * Intel 82599 packet-mediated backend for scope-fpga-vswitch.
 *
 * XiangShan sees an 8086:10fb PF and uses the standard ixgbe driver.  The
 * register and descriptor model lives here; Host B only sends and receives
 * complete Ethernet frames through the remote RDMA control connection.
 */

#define SCOPE_IXGBE_REG_COUNT             (SCOPE_IXGBE_BAR0_SIZE / 4U)
#define SCOPE_IXGBE_DESC_SIZE             16U
#define SCOPE_IXGBE_PENDING_DEPTH         64U
#define SCOPE_IXGBE_TX_COMPLETION_DEPTH   64U
#define SCOPE_IXGBE_TX_WB_MAX             64U
#define SCOPE_IXGBE_MAX_FRAME             SCOPE_REMOTE_ETHER_MAX_FRAME
#define SCOPE_IXGBE_REMOTE_EVENT_BUDGET   64U

#define SCOPE_IXGBE_CTRL                  0x00000U
#define SCOPE_IXGBE_STATUS                0x00008U
#define SCOPE_IXGBE_CTRL_EXT              0x00018U
#define SCOPE_IXGBE_ESDP                  0x00020U
#define SCOPE_IXGBE_I2CCTL                0x00028U
#define SCOPE_IXGBE_EICR                  0x00800U
#define SCOPE_IXGBE_EICS                  0x00808U
#define SCOPE_IXGBE_EIMS                  0x00880U
#define SCOPE_IXGBE_EIMC                  0x00888U
#define SCOPE_IXGBE_GPIE                  0x00898U
#define SCOPE_IXGBE_EICS_EX0              0x00a90U
#define SCOPE_IXGBE_EICS_EX1              0x00a94U
#define SCOPE_IXGBE_EIMS_EX0              0x00aa0U
#define SCOPE_IXGBE_EIMS_EX1              0x00aa4U
#define SCOPE_IXGBE_EIMC_EX0              0x00ab0U
#define SCOPE_IXGBE_EIMC_EX1              0x00ab4U
#define SCOPE_IXGBE_RDBAL0                0x01000U
#define SCOPE_IXGBE_RDBAH0                0x01004U
#define SCOPE_IXGBE_RDLEN0                0x01008U
#define SCOPE_IXGBE_RDH0                  0x01010U
#define SCOPE_IXGBE_RDT0                  0x01018U
#define SCOPE_IXGBE_RXDCTL0               0x01028U
#define SCOPE_IXGBE_RDRXCTL               0x02f00U
#define SCOPE_IXGBE_RXCTRL                0x03000U
#define SCOPE_IXGBE_RXPBSIZE0             0x03c00U
#define SCOPE_IXGBE_AUTOC                 0x042a0U
#define SCOPE_IXGBE_LINKS                 0x042a4U
#define SCOPE_IXGBE_DMATXCTL              0x04a80U
#define SCOPE_IXGBE_FCTRL                 0x05080U
#define SCOPE_IXGBE_VLNCTRL               0x05088U
#define SCOPE_IXGBE_RAL0                  0x05400U
#define SCOPE_IXGBE_RAH0                  0x05404U
#define SCOPE_IXGBE_MRQC                  0x05818U
#define SCOPE_IXGBE_TDBAL0                0x06000U
#define SCOPE_IXGBE_TDBAH0                0x06004U
#define SCOPE_IXGBE_TDLEN0                0x06008U
#define SCOPE_IXGBE_TDH0                  0x06010U
#define SCOPE_IXGBE_TDT0                  0x06018U
#define SCOPE_IXGBE_TXDCTL0               0x06028U
#define SCOPE_IXGBE_MTQC                  0x08120U
#define SCOPE_IXGBE_TXPBSIZE0             0x0cc00U
#define SCOPE_IXGBE_EEC                   0x10010U
#define SCOPE_IXGBE_EERD                  0x10014U
#define SCOPE_IXGBE_SWSM                  0x10140U
#define SCOPE_IXGBE_FWSM                  0x10148U
#define SCOPE_IXGBE_GSSR                  0x10160U

#define SCOPE_IXGBE_CTRL_LNK_RST          0x00000008U
#define SCOPE_IXGBE_CTRL_RST              0x04000000U
#define SCOPE_IXGBE_CTRL_RST_MASK         (SCOPE_IXGBE_CTRL_LNK_RST | \
                                           SCOPE_IXGBE_CTRL_RST)
#define SCOPE_IXGBE_CTRL_EXT_PFRSTD       0x00004000U
#define SCOPE_IXGBE_ESDP_SDP2             0x00000004U
#define SCOPE_IXGBE_I2C_CLK_IN            0x00000001U
#define SCOPE_IXGBE_I2C_CLK_OUT           0x00000002U
#define SCOPE_IXGBE_I2C_DATA_IN           0x00000004U
#define SCOPE_IXGBE_I2C_DATA_OUT          0x00000008U
#define SCOPE_IXGBE_EEC_REQ               0x00000040U
#define SCOPE_IXGBE_EEC_GNT               0x00000080U
#define SCOPE_IXGBE_EEC_VIRTUAL_FIXED     0x00002700U
#define SCOPE_IXGBE_FWSM_FW_VAL           0x00008000U
#define SCOPE_IXGBE_EEPROM_CHECKSUM_WORD  0x003fU
#define SCOPE_IXGBE_EEPROM_SUM            0xbabaU
#define SCOPE_IXGBE_EEPROM_FW_PTR         0x000fU
#define SCOPE_IXGBE_FW_PASSTHROUGH_PTR    0x0004U
#define SCOPE_IXGBE_FW_PATCH_VERSION      0x0007U
#define SCOPE_IXGBE_QUEUE_ENABLE          0x02000000U
#define SCOPE_IXGBE_RXCTRL_RXEN           0x00000001U
#define SCOPE_IXGBE_DMATXCTL_TE           0x00000001U
#define SCOPE_IXGBE_LINKS_UP              0x40000000U
#define SCOPE_IXGBE_LINKS_SPEED_10G       0x30000000U
#define SCOPE_IXGBE_EICR_QUEUE0           0x00000001U
#define SCOPE_IXGBE_EICR_RXO              0x00020000U
#define SCOPE_IXGBE_EICR_LSC              0x00100000U
#define SCOPE_IXGBE_SUPPORTED_CAUSES      (SCOPE_IXGBE_EICR_QUEUE0 | \
                                           SCOPE_IXGBE_EICR_RXO | \
                                           SCOPE_IXGBE_EICR_LSC)
#define SCOPE_IXGBE_ADVTXD_DTYP_MASK      0x00f00000U
#define SCOPE_IXGBE_ADVTXD_DTYP_CTXT      0x00200000U
#define SCOPE_IXGBE_ADVTXD_DTYP_DATA      0x00300000U
#define SCOPE_IXGBE_ADVTXD_DCMD_EOP       0x01000000U
#define SCOPE_IXGBE_ADVTXD_DCMD_RS        0x08000000U
#define SCOPE_IXGBE_ADVTXD_DCMD_TSE       0x80000000U
#define SCOPE_IXGBE_ADVTXD_STAT_DD        0x00000001U
#define SCOPE_IXGBE_RXD_STAT_DD           0x00000001U
#define SCOPE_IXGBE_RXD_STAT_EOP          0x00000002U
#define SCOPE_IXGBE_RXPBSIZE_512K         0x00080000U
#define SCOPE_IXGBE_TXPBSIZE_160K         0x00028000U

typedef struct ScopeIxgbePendingTail {
    bool valid;
    uint32_t seq;
    uint16_t tail;
} ScopeIxgbePendingTail;

typedef struct ScopeIxgbePendingQueue {
    ScopeIxgbePendingTail entry[SCOPE_IXGBE_PENDING_DEPTH];
    uint16_t head;
    uint16_t count;
} ScopeIxgbePendingQueue;

typedef struct ScopeIxgbeTxDesc {
    uint64_t buffer_addr;
    uint32_t cmd_type_len;
    uint32_t olinfo_status;
} ScopeIxgbeTxDesc;

typedef struct ScopeIxgbeRxDesc {
    uint64_t pkt_addr;
    uint64_t hdr_addr;
} ScopeIxgbeRxDesc;

typedef struct ScopeIxgbeTxCompletion {
    bool valid;
    uint64_t request_id;
    uint32_t length;
    uint16_t wb_count;
    uint64_t wb_desc_pa[SCOPE_IXGBE_TX_WB_MAX];
    int64_t submitted_us;
} ScopeIxgbeTxCompletion;

struct ScopeIxgbeState {
    uint32_t regs[SCOPE_IXGBE_REG_COUNT];
    uint16_t eeprom[1024];
    uint8_t mac[6];
    uint32_t mtu;
    bool link_up;
    uint32_t causes;
    uint32_t mask;
    ScopeIxgbePendingQueue tx_pending;
    ScopeIxgbePendingQueue rx_pending;
    ScopeIxgbeTxCompletion tx_completion[SCOPE_IXGBE_TX_COMPLETION_DEPTH];
    uint8_t tx_frame[SCOPE_IXGBE_MAX_FRAME];
    uint32_t tx_frame_len;
    uint64_t tx_frame_wb[SCOPE_IXGBE_TX_WB_MAX];
    uint16_t tx_frame_wb_count;
    uint64_t next_tx_id;
    uint64_t tx_submitted;
    uint64_t tx_packets;
    uint64_t tx_completion_stale;
    uint64_t rx_packets;
    uint64_t dropped_packets;
};

static uint32_t scope_ixgbe_reg_get(ScopeIxgbeState *x, uint32_t offset)
{
    return offset < SCOPE_IXGBE_BAR0_SIZE ? x->regs[offset >> 2] : 0;
}

static void scope_ixgbe_reg_set(ScopeIxgbeState *x, uint32_t offset,
                                uint32_t value)
{
    if (offset < SCOPE_IXGBE_BAR0_SIZE) {
        x->regs[offset >> 2] = value;
    }
}

static uint16_t scope_ixgbe_ring_depth(ScopeIxgbeState *x, bool tx)
{
    uint32_t bytes = scope_ixgbe_reg_get(x, tx ? SCOPE_IXGBE_TDLEN0 :
                                                 SCOPE_IXGBE_RDLEN0);

    if (!bytes || (bytes & 0x7fU) || bytes > 4096U * SCOPE_IXGBE_DESC_SIZE) {
        return 0;
    }
    return bytes / SCOPE_IXGBE_DESC_SIZE;
}

static uint64_t scope_ixgbe_ring_base(ScopeIxgbeState *x, bool tx)
{
    uint32_t lo = scope_ixgbe_reg_get(x, tx ? SCOPE_IXGBE_TDBAL0 :
                                              SCOPE_IXGBE_RDBAL0);
    uint32_t hi = scope_ixgbe_reg_get(x, tx ? SCOPE_IXGBE_TDBAH0 :
                                              SCOPE_IXGBE_RDBAH0);

    return ((uint64_t)hi << 32) | (lo & ~0x7fU);
}

static void scope_ixgbe_update_intx(ScopeProxyState *s)
{
    ScopeIxgbeState *x = s->active->ixgbe;

    s->active->intx_pending = x && (x->causes & x->mask &
                                     SCOPE_IXGBE_SUPPORTED_CAUSES);
}

static void scope_ixgbe_raise(ScopeProxyState *s, uint32_t cause)
{
    ScopeIxgbeState *x = s->active->ixgbe;

    x->causes |= cause & SCOPE_IXGBE_SUPPORTED_CAUSES;
    scope_ixgbe_update_intx(s);
}

/*
 * Provide the minimum NVM contents that the upstream 82599 driver checks.
 * The packet-mediated device does not execute firmware, but ixgbe requires a
 * valid firmware-module pointer chain and version >= 0.6 for an SFI device.
 */
static void scope_ixgbe_init_virtual_eeprom(ScopeIxgbeState *x)
{
    const uint16_t fw_block = 0x0080U;
    const uint16_t passthrough_block = 0x0090U;
    uint32_t sum = 0;
    unsigned int i;

    memset(x->eeprom, 0, sizeof(x->eeprom));
    x->eeprom[SCOPE_IXGBE_EEPROM_FW_PTR] = fw_block;
    x->eeprom[fw_block + SCOPE_IXGBE_FW_PASSTHROUGH_PTR] =
        passthrough_block;
    x->eeprom[passthrough_block + SCOPE_IXGBE_FW_PATCH_VERSION] = 0x0006U;

    /* Keep the generic ixgbe NVM checksum valid after adding FW_PTR. */
    for (i = 0; i < SCOPE_IXGBE_EEPROM_CHECKSUM_WORD; i++) {
        sum += x->eeprom[i];
    }
    x->eeprom[SCOPE_IXGBE_EEPROM_CHECKSUM_WORD] =
        (uint16_t)(SCOPE_IXGBE_EEPROM_SUM - (uint16_t)sum);
}

static void scope_ixgbe_reset_model(ScopeProxyState *s)
{
    ScopeIxgbeState *x = s->active->ixgbe;
    uint32_t ral = (uint32_t)x->mac[0] | ((uint32_t)x->mac[1] << 8) |
                   ((uint32_t)x->mac[2] << 16) | ((uint32_t)x->mac[3] << 24);
    uint32_t rah = (uint32_t)x->mac[4] | ((uint32_t)x->mac[5] << 8) |
                   0x80000000U;

    memset(x->regs, 0, sizeof(x->regs));
    memset(&x->tx_pending, 0, sizeof(x->tx_pending));
    memset(&x->rx_pending, 0, sizeof(x->rx_pending));
    memset(x->tx_completion, 0, sizeof(x->tx_completion));
    x->causes = 0;
    x->mask = 0;
    x->tx_frame_len = 0;
    x->tx_frame_wb_count = 0;
    scope_ixgbe_reg_set(x, SCOPE_IXGBE_STATUS, 0);
    scope_ixgbe_reg_set(x, SCOPE_IXGBE_CTRL_EXT,
                        SCOPE_IXGBE_CTRL_EXT_PFRSTD);
    /* The exported 82599 SFP function always represents an occupied cage. */
    scope_ixgbe_reg_set(x, SCOPE_IXGBE_ESDP, SCOPE_IXGBE_ESDP_SDP2);
    /* EEPROM present, auto-read complete, 16-bit addressing, 1024 words. */
    scope_ixgbe_reg_set(x, SCOPE_IXGBE_EEC, SCOPE_IXGBE_EEC_VIRTUAL_FIXED);
    /* Firmware is synthetic but valid; no management mode is advertised. */
    scope_ixgbe_reg_set(x, SCOPE_IXGBE_FWSM, SCOPE_IXGBE_FWSM_FW_VAL);
    scope_ixgbe_reg_set(x, SCOPE_IXGBE_GSSR, 0U);
    scope_ixgbe_reg_set(x, SCOPE_IXGBE_AUTOC, 0x00002000U);
    scope_ixgbe_reg_set(x, SCOPE_IXGBE_LINKS,
                        x->link_up ? SCOPE_IXGBE_LINKS_UP |
                                     SCOPE_IXGBE_LINKS_SPEED_10G : 0);
    scope_ixgbe_reg_set(x, SCOPE_IXGBE_RAL0, ral);
    scope_ixgbe_reg_set(x, SCOPE_IXGBE_RAH0, rah);
    /* 82599 single-TC packet-buffer capacities expected by ixgbe. */
    scope_ixgbe_reg_set(x, SCOPE_IXGBE_RXPBSIZE0,
                        SCOPE_IXGBE_RXPBSIZE_512K);
    scope_ixgbe_reg_set(x, SCOPE_IXGBE_TXPBSIZE0,
                        SCOPE_IXGBE_TXPBSIZE_160K);
    scope_ixgbe_reg_set(x, SCOPE_IXGBE_SWSM, 0U);
    s->active->intx_pending = false;
}

static bool scope_ixgbe_pending_push(ScopeIxgbePendingQueue *q,
                                      uint32_t seq, uint16_t tail)
{
    ScopeIxgbePendingTail *p;
    uint16_t slot;

    if (q->count == SCOPE_IXGBE_PENDING_DEPTH) {
        return false;
    }
    slot = (q->head + q->count) % SCOPE_IXGBE_PENDING_DEPTH;
    p = &q->entry[slot];
    *p = (ScopeIxgbePendingTail) {
        .valid = true,
        .seq = seq,
        .tail = tail,
    };
    q->count++;
    return true;
}

static bool scope_ixgbe_read_desc(ScopeProxyState *s, uint64_t pa, void *desc)
{
    uint8_t a[SCOPE_IXGBE_DESC_SIZE], b[SCOPE_IXGBE_DESC_SIZE];

    return scope_guest_mem_read(s, pa, a, sizeof(a)) &&
           scope_guest_mem_read(s, pa, b, sizeof(b)) &&
           !memcmp(a, b, sizeof(a)) &&
           (memcpy(desc, b, sizeof(b)), true);
}

static bool scope_ixgbe_publish_desc(ScopeProxyState *s, uint64_t pa,
                                     const void *desc)
{
    uint8_t visible[SCOPE_IXGBE_DESC_SIZE];

    if (!scope_guest_mem_write(s, pa, desc, SCOPE_IXGBE_DESC_SIZE)) {
        return false;
    }
    smp_wmb();
    return scope_ixgbe_read_desc(s, pa, visible) &&
           !memcmp(visible, desc, sizeof(visible));
}

static ScopeIxgbeTxCompletion *scope_ixgbe_tx_completion_reserve(
    ScopeIxgbeState *x, uint64_t request_id, uint32_t length, int64_t now_us)
{
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(x->tx_completion); i++) {
        ScopeIxgbeTxCompletion *c = &x->tx_completion[i];

        if (c->valid) {
            continue;
        }
        memset(c, 0, sizeof(*c));
        c->valid = true;
        c->request_id = request_id;
        c->length = length;
        c->wb_count = x->tx_frame_wb_count;
        memcpy(c->wb_desc_pa, x->tx_frame_wb,
               c->wb_count * sizeof(c->wb_desc_pa[0]));
        c->submitted_us = now_us;
        return c;
    }
    return NULL;
}

static ScopeIxgbeTxCompletion *scope_ixgbe_tx_completion_find(
    ScopeIxgbeState *x, uint64_t request_id)
{
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(x->tx_completion); i++) {
        ScopeIxgbeTxCompletion *c = &x->tx_completion[i];

        if (c->valid && c->request_id == request_id) {
            return c;
        }
    }
    return NULL;
}

static bool scope_ixgbe_complete_tx(ScopeProxyState *s,
                                    const ScopeRemoteEvent *event)
{
    ScopeIxgbeState *x = s->active->ixgbe;
    ScopeIxgbeTxCompletion *c =
        scope_ixgbe_tx_completion_find(x, event->request_id);
    unsigned int i;

    if (!c) {
        x->tx_completion_stale++;
        SCOPE_PRINTF("[SCOPE IXGBE][TX_COMPLETE][STALE] backend=%u "
                     "request=%" PRIu64 " length=%u count=%" PRIu64 "\n",
                     s->active->id, event->request_id,
                     event->u.tx_complete.length, x->tx_completion_stale);
        return true;
    }
    if (event->u.tx_complete.length != c->length) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE: ixgbe TX completion length mismatch request=%" PRIu64
                      " expected=%u actual=%u\n", c->request_id, c->length,
                      event->u.tx_complete.length);
        return false;
    }

    for (i = 0; i < c->wb_count; i++) {
        ScopeIxgbeTxDesc desc;

        if (!scope_ixgbe_read_desc(s, c->wb_desc_pa[i], &desc)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "SCOPE: ixgbe cannot reread TX descriptor for "
                          "completion request=%" PRIu64 " pa=0x%016" PRIx64
                          "\n", c->request_id, c->wb_desc_pa[i]);
            return false;
        }
        desc.olinfo_status = cpu_to_le32(
            le32_to_cpu(desc.olinfo_status) | SCOPE_IXGBE_ADVTXD_STAT_DD);
        if (!scope_ixgbe_publish_desc(s, c->wb_desc_pa[i], &desc)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "SCOPE: ixgbe cannot publish TX DD for request=%" PRIu64
                          " pa=0x%016" PRIx64 "\n",
                          c->request_id, c->wb_desc_pa[i]);
            return false;
        }
    }

    x->tx_packets++;
    if (c->wb_count) {
        scope_ixgbe_raise(s, SCOPE_IXGBE_EICR_QUEUE0);
    }
    SCOPE_PRINTF("[SCOPE IXGBE][TX_COMPLETE] backend=%u request=%" PRIu64
                 " length=%u wb=%u completed=%" PRIu64
                 " causes=0x%08x mask=0x%08x intx=%u\n",
                 s->active->id, c->request_id, c->length, c->wb_count,
                 x->tx_packets, x->causes, x->mask,
                 s->active->intx_pending ? 1U : 0U);
    memset(c, 0, sizeof(*c));
    return true;
}

static bool scope_ixgbe_tx_completion_timed_out(ScopeProxyState *s,
                                                int64_t now_us)
{
    ScopeIxgbeState *x = s->active->ixgbe;
    uint64_t timeout_us =
        (uint64_t)scope_remote_request_timeout_ms(s->active->remote) * 1000U;
    unsigned int i;

    if (!timeout_us) {
        return false;
    }
    for (i = 0; i < ARRAY_SIZE(x->tx_completion); i++) {
        ScopeIxgbeTxCompletion *c = &x->tx_completion[i];

        if (!c->valid || now_us - c->submitted_us < (int64_t)timeout_us) {
            continue;
        }
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE: ixgbe TX completion timed out request=%" PRIu64
                      " length=%u elapsed_us=%" PRId64 "\n",
                      c->request_id, c->length, now_us - c->submitted_us);
        return true;
    }
    return false;
}

static bool scope_ixgbe_process_tx(ScopeProxyState *s, uint16_t new_tail)
{
    ScopeIxgbeState *x = s->active->ixgbe;
    uint16_t depth = scope_ixgbe_ring_depth(x, true);
    uint16_t head = scope_ixgbe_reg_get(x, SCOPE_IXGBE_TDH0);
    uint64_t base = scope_ixgbe_ring_base(x, true);
    unsigned int guard = 0;

    if (!depth || new_tail >= depth || !base ||
        !(scope_ixgbe_reg_get(x, SCOPE_IXGBE_TXDCTL0) &
          SCOPE_IXGBE_QUEUE_ENABLE) ||
        !(scope_ixgbe_reg_get(x, SCOPE_IXGBE_DMATXCTL) &
          SCOPE_IXGBE_DMATXCTL_TE)) {
        return false;
    }
    while (head != new_tail && guard++ < depth) {
        ScopeIxgbeTxDesc desc;
        uint64_t desc_pa = base + (uint64_t)head * sizeof(desc);
        uint32_t cmd;
        uint32_t len;
        uint32_t frame_len_before;
        uint16_t wb_count_before;

        if (!scope_ixgbe_read_desc(s, desc_pa, &desc)) {
            return false;
        }
        cmd = le32_to_cpu(desc.cmd_type_len);
        if ((cmd & SCOPE_IXGBE_ADVTXD_DTYP_MASK) ==
            SCOPE_IXGBE_ADVTXD_DTYP_CTXT) {
            head = (head + 1U) % depth;
            scope_ixgbe_reg_set(x, SCOPE_IXGBE_TDH0, head);
            continue;
        }
        if ((cmd & SCOPE_IXGBE_ADVTXD_DTYP_MASK) !=
            SCOPE_IXGBE_ADVTXD_DTYP_DATA ||
            (cmd & SCOPE_IXGBE_ADVTXD_DCMD_TSE)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "SCOPE: ixgbe unsupported TX descriptor cmd=0x%08x\n",
                          cmd);
            return false;
        }

        frame_len_before = x->tx_frame_len;
        wb_count_before = x->tx_frame_wb_count;
        len = cmd & 0xffffU;
        if (!len || x->tx_frame_len + len > sizeof(x->tx_frame) ||
            !scope_guest_mem_read(s, le64_to_cpu(desc.buffer_addr),
                                  x->tx_frame + x->tx_frame_len, len)) {
            return false;
        }
        x->tx_frame_len += len;

        if (cmd & SCOPE_IXGBE_ADVTXD_DCMD_RS) {
            if (x->tx_frame_wb_count == ARRAY_SIZE(x->tx_frame_wb)) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "SCOPE: ixgbe too many RS descriptors in one "
                              "packet\n");
                x->tx_frame_len = frame_len_before;
                return false;
            }
            x->tx_frame_wb[x->tx_frame_wb_count++] = desc_pa;
        }

        if (cmd & SCOPE_IXGBE_ADVTXD_DCMD_EOP) {
            Error *local_err = NULL;
            uint64_t request_id = x->next_tx_id++;
            ScopeIxgbeTxCompletion *completion;

            completion = scope_ixgbe_tx_completion_reserve(
                x, request_id, x->tx_frame_len, g_get_monotonic_time());
            if (!completion) {
                x->tx_frame_len = frame_len_before;
                x->tx_frame_wb_count = wb_count_before;
                return false;
            }

            net_checksum_calculate(x->tx_frame, x->tx_frame_len, CSUM_ALL);
            if (!scope_remote_submit_ixgbe_tx(s->active->remote,
                                               x->tx_frame, x->tx_frame_len,
                                               request_id, &local_err)) {
                memset(completion, 0, sizeof(*completion));
                x->tx_frame_len = frame_len_before;
                x->tx_frame_wb_count = wb_count_before;
                if (local_err) {
                    error_report_err(local_err);
                }
                return false;
            }
            x->tx_submitted++;
            SCOPE_PRINTF("[SCOPE IXGBE][TX_SUBMIT] backend=%u request=%" PRIu64
                         " length=%u wb=%u submitted=%" PRIu64 "\n",
                         s->active->id, request_id, completion->length,
                         completion->wb_count, x->tx_submitted);
            x->tx_frame_len = 0;
            x->tx_frame_wb_count = 0;
        }

        /*
         * Do not write DD here.  The descriptor remains guest-owned until
         * Host B confirms that AF_PACKET accepted the corresponding frame.
         * IXGBE_TX_COMPLETE performs DD writeback and raises queue-0 INTx.
         */
        head = (head + 1U) % depth;
        scope_ixgbe_reg_set(x, SCOPE_IXGBE_TDH0, head);
    }
    scope_ixgbe_reg_set(x, SCOPE_IXGBE_TDT0, new_tail);
    return head == new_tail;
}

static bool scope_ixgbe_deliver_rx(ScopeProxyState *s,
                                   const uint8_t *frame, uint32_t length)
{
    ScopeIxgbeState *x = s->active->ixgbe;
    uint16_t depth = scope_ixgbe_ring_depth(x, false);
    uint16_t head = scope_ixgbe_reg_get(x, SCOPE_IXGBE_RDH0);
    uint16_t tail = scope_ixgbe_reg_get(x, SCOPE_IXGBE_RDT0);
    uint64_t base = scope_ixgbe_ring_base(x, false);
    ScopeIxgbeRxDesc desc;
    uint64_t desc_pa;
    uint32_t status_error = SCOPE_IXGBE_RXD_STAT_DD |
                            SCOPE_IXGBE_RXD_STAT_EOP;

    if (!length || length > x->mtu + 18U || !depth || head == tail || !base ||
        !(scope_ixgbe_reg_get(x, SCOPE_IXGBE_RXDCTL0) &
          SCOPE_IXGBE_QUEUE_ENABLE) ||
        !(scope_ixgbe_reg_get(x, SCOPE_IXGBE_RXCTRL) &
          SCOPE_IXGBE_RXCTRL_RXEN)) {
        x->dropped_packets++;
        if (x->dropped_packets <= 8U ||
            !(x->dropped_packets & (x->dropped_packets - 1U))) {
            SCOPE_PRINTF("[SCOPE IXGBE][RX_DROP] backend=%u length=%u "
                         "depth=%u head=%u tail=%u rxdctl=0x%08x "
                         "rxctrl=0x%08x dropped=%" PRIu64 "\n",
                         s->active->id, length, depth, head, tail,
                         scope_ixgbe_reg_get(x, SCOPE_IXGBE_RXDCTL0),
                         scope_ixgbe_reg_get(x, SCOPE_IXGBE_RXCTRL),
                         x->dropped_packets);
        }
        scope_ixgbe_raise(s, SCOPE_IXGBE_EICR_RXO);
        return false;
    }
    desc_pa = base + (uint64_t)head * sizeof(desc);
    if (!scope_ixgbe_read_desc(s, desc_pa, &desc) ||
        !le64_to_cpu(desc.pkt_addr) ||
        !scope_guest_mem_write(s, le64_to_cpu(desc.pkt_addr), frame, length)) {
        x->dropped_packets++;
        scope_ixgbe_raise(s, SCOPE_IXGBE_EICR_RXO);
        return false;
    }
    memset(&desc, 0, sizeof(desc));
    stl_le_p((uint8_t *)&desc + 8, status_error);
    stw_le_p((uint8_t *)&desc + 12, length);
    if (!scope_ixgbe_publish_desc(s, desc_pa, &desc)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE: ixgbe cannot publish RX descriptor "
                      "pa=0x%016" PRIx64 "\n", desc_pa);
        return false;
    }
    head = (head + 1U) % depth;
    scope_ixgbe_reg_set(x, SCOPE_IXGBE_RDH0, head);
    x->rx_packets++;
    scope_ixgbe_raise(s, SCOPE_IXGBE_EICR_QUEUE0);
    if (x->rx_packets <= 16U || !(x->rx_packets & (x->rx_packets - 1U))) {
        SCOPE_PRINTF("[SCOPE IXGBE][RX_DELIVER] backend=%u length=%u "
                     "head=%u tail=%u received=%" PRIu64
                     " causes=0x%08x mask=0x%08x intx=%u\n",
                     s->active->id, length, head, tail, x->rx_packets,
                     x->causes, x->mask,
                     s->active->intx_pending ? 1U : 0U);
    }
    return true;
}

static bool scope_ixgbe_bar_read(ScopeProxyState *s, uint32_t offset,
                                 uint8_t size, uint64_t *data)
{
    ScopeIxgbeState *x = s->active->ixgbe;
    uint32_t value;

    if (!x || size != 4 || (offset & 3U) ||
        offset >= SCOPE_IXGBE_BAR0_SIZE) {
        return false;
    }
    switch (offset) {
    case SCOPE_IXGBE_EICR:
        value = x->causes;
        x->causes = 0;
        scope_ixgbe_update_intx(s);
        break;
    case SCOPE_IXGBE_EIMS:
        value = x->mask;
        break;
    case SCOPE_IXGBE_EICS_EX0:
        value = x->causes & SCOPE_IXGBE_EICR_QUEUE0;
        break;
    case SCOPE_IXGBE_EIMS_EX0:
        value = x->mask & SCOPE_IXGBE_EICR_QUEUE0;
        break;
    case SCOPE_IXGBE_EICS_EX1:
    case SCOPE_IXGBE_EIMS_EX1:
        value = 0;
        break;
    case SCOPE_IXGBE_LINKS:
        value = x->link_up ? SCOPE_IXGBE_LINKS_UP |
                             SCOPE_IXGBE_LINKS_SPEED_10G : 0;
        break;
    case SCOPE_IXGBE_I2CCTL:
        /*
         * 82599 I2C is bit-banged.  Reflect the driven open-drain line
         * levels into the input pins so ixgbe does not spend 500 BAR
         * round trips in each clock-stretch poll.  With no virtual SFP
         * EEPROM slave, released SDA remains high and therefore NACKs.
         */
        value = scope_ixgbe_reg_get(x, offset);
        if (value & SCOPE_IXGBE_I2C_CLK_OUT) {
            value |= SCOPE_IXGBE_I2C_CLK_IN;
        } else {
            value &= ~SCOPE_IXGBE_I2C_CLK_IN;
        }
        if (value & SCOPE_IXGBE_I2C_DATA_OUT) {
            value |= SCOPE_IXGBE_I2C_DATA_IN;
        } else {
            value &= ~SCOPE_IXGBE_I2C_DATA_IN;
        }
        break;
    default:
        value = scope_ixgbe_reg_get(x, offset);
        break;
    }
    *data = scope_pack_dword32_for_offset(value, offset);
    return true;
}

static bool scope_ixgbe_bar_write(ScopeProxyState *s, uint32_t offset,
                                  uint64_t data, uint8_t wstrb, uint8_t size)
{
    ScopeIxgbeState *x = s->active->ixgbe;
    uint32_t value;

    if (!x || size != 4 || (offset & 3U) ||
        offset >= SCOPE_IXGBE_BAR0_SIZE ||
        (wstrb & scope_pack_wstrb4_for_offset(offset)) !=
         scope_pack_wstrb4_for_offset(offset)) {
        return false;
    }
    value = scope_extract_dword32(data, offset);
    switch (offset) {
    case SCOPE_IXGBE_CTRL:
        /*
         * 82599 uses link reset while the link is down and software reset
         * while it is up.  Both bits are self-clearing in real hardware.
         * Retaining LNK_RST made ixgbe_reset_hw_82599() time out with
         * IXGBE_ERR_RESET_FAILED.
         */
        if (value & SCOPE_IXGBE_CTRL_RST_MASK) {
            SCOPE_PRINTF("[SCOPE IXGBE][RESET] backend=%u value=0x%08x "
                         "kind=%s\n", s->active->id, value,
                         (value & SCOPE_IXGBE_CTRL_LNK_RST) ?
                         "link" : "software");
            scope_ixgbe_reset_model(s);
        } else {
            scope_ixgbe_reg_set(x, offset, value);
        }
        return true;
    case SCOPE_IXGBE_EIMS:
        x->mask |= value & SCOPE_IXGBE_SUPPORTED_CAUSES;
        scope_ixgbe_update_intx(s);
        return true;
    case SCOPE_IXGBE_EIMC:
        x->mask &= ~(value & SCOPE_IXGBE_SUPPORTED_CAUSES);
        scope_ixgbe_update_intx(s);
        return true;
    case SCOPE_IXGBE_EIMS_EX0:
        if (value & 1U) {
            x->mask |= SCOPE_IXGBE_EICR_QUEUE0;
        }
        scope_ixgbe_update_intx(s);
        return true;
    case SCOPE_IXGBE_EIMC_EX0:
        if (value & 1U) {
            x->mask &= ~SCOPE_IXGBE_EICR_QUEUE0;
        }
        scope_ixgbe_update_intx(s);
        return true;
    case SCOPE_IXGBE_EICS_EX0:
        if (value & 1U) {
            scope_ixgbe_raise(s, SCOPE_IXGBE_EICR_QUEUE0);
        }
        return true;
    case SCOPE_IXGBE_EICS_EX1:
    case SCOPE_IXGBE_EIMS_EX1:
    case SCOPE_IXGBE_EIMC_EX1:
        return true;
    case SCOPE_IXGBE_EICS:
        scope_ixgbe_raise(s, value);
        return true;
    case SCOPE_IXGBE_EICR:
        x->causes &= ~value;
        scope_ixgbe_update_intx(s);
        return true;
    case SCOPE_IXGBE_EERD: {
        uint16_t addr = (value >> 2) & 0x3ffU;
        scope_ixgbe_reg_set(x, offset, ((uint32_t)x->eeprom[addr] << 16) |
                                       (value & 0xffffU) | 0x2U);
        return true;
    }
    case SCOPE_IXGBE_EEC: {
        uint32_t next = value | SCOPE_IXGBE_EEC_VIRTUAL_FIXED;

        /* Grant software EEPROM ownership immediately in the virtual PF. */
        if (value & SCOPE_IXGBE_EEC_REQ) {
            next |= SCOPE_IXGBE_EEC_GNT;
        } else {
            next &= ~SCOPE_IXGBE_EEC_GNT;
        }
        scope_ixgbe_reg_set(x, offset, next);
        return true;
    }
    case SCOPE_IXGBE_FWSM:
        /* FWSM is firmware-owned/read-only from the guest's perspective. */
        return true;
    case SCOPE_IXGBE_MRQC:
        /* One mediated RX queue: RSS is deliberately disabled. */
        scope_ixgbe_reg_set(x, offset, 0);
        return true;
    default:
        scope_ixgbe_reg_set(x, offset, value);
        return true;
    }
}

static bool scope_ixgbe_is_tail(uint32_t offset, uint8_t size, uint8_t wstrb)
{
    return size == 4 && wstrb == 0x0f &&
           (offset == SCOPE_IXGBE_TDT0 || offset == SCOPE_IXGBE_RDT0);
}

/*
 * TDT/RDT are ordinary device doorbells in the packet-mediated 82599 model.
 *
 * Do not wait for FPGA BAR_WRITE_DONE here. BAR_WRITE_DONE is generated only
 * after the guest AXI B response is consumed; it does not provide additional
 * ordering for descriptor memory. Linux ixgbe orders descriptor stores before
 * publishing a tail, while scope_ixgbe_process_tx() uses stable descriptor
 * reads and leaves the pending tail queued until ring state/data are visible.
 *
 * During ndo_open(), ixgbe initializes TDT=0, RDT=0 and later publishes the
 * populated RX ring with another RDT write. Treating these initialization
 * writes as BAR_DONE-dependent caused the false 5 ms timeout diagnostics.
 */
static bool scope_ixgbe_poll_pending(ScopeProxyState *s,
                                     ScopeIxgbePendingQueue *q, bool tx)
{
    ScopeIxgbePendingTail *p;
    bool ok;

    if (!q->count) {
        return false;
    }

    p = &q->entry[q->head];
    if (tx) {
        ok = scope_ixgbe_process_tx(s, p->tail);
    } else {
        scope_ixgbe_reg_set(s->active->ixgbe, SCOPE_IXGBE_RDT0, p->tail);
        ok = true;
    }
    if (!ok) {
        /*
         * Ring setup or descriptor contents may not be visible yet. Keep the
         * tail queued and retry from the normal backend poll loop.
         */
        return false;
    }

    memset(p, 0, sizeof(*p));
    q->head = (q->head + 1U) % SCOPE_IXGBE_PENDING_DEPTH;
    q->count--;
    return true;
}

static bool scope_ixgbe_poll(ScopeProxyState *s, int64_t now_us)
{
    ScopeIxgbeState *x = s->active->ixgbe;
    bool progressed = false;
    unsigned int events;

    if (!x || !s->active->remote || s->active->remote_failed_state) {
        return false;
    }
    for (events = 0; events < SCOPE_IXGBE_REMOTE_EVENT_BUDGET; events++) {
        ScopeRemoteEvent event;
        Error *local_err = NULL;

        if (!scope_remote_poll(s->active->remote, &event, &local_err)) {
            if (local_err) {
                error_report_err(local_err);
                s->active->remote_failed_state = true;
            }
            break;
        }
        progressed = true;
        if (event.status != SCOPE_REMOTE_STATUS_OK) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "SCOPE: remote ixgbe operation failed: status=%d\n",
                          event.status);
            s->active->remote_failed_state = true;
            x->link_up = false;
            scope_ixgbe_raise(s, SCOPE_IXGBE_EICR_LSC);
            break;
        }
        if (event.type == SCOPE_REMOTE_EVENT_IXGBE_TX_COMPLETE) {
            if (!scope_ixgbe_complete_tx(s, &event)) {
                s->active->remote_failed_state = true;
                x->link_up = false;
                scope_ixgbe_raise(s, SCOPE_IXGBE_EICR_LSC);
                break;
            }
        } else if (event.type == SCOPE_REMOTE_EVENT_IXGBE_RX) {
            scope_ixgbe_deliver_rx(s, event.u.frame.data,
                                   event.u.frame.length);
        } else if (event.type == SCOPE_REMOTE_EVENT_IXGBE_LINK) {
            bool old = x->link_up;
            x->link_up = event.u.link.link_up != 0;
            if (old != x->link_up) {
                scope_ixgbe_raise(s, SCOPE_IXGBE_EICR_LSC);
            }
        } else if (event.type == SCOPE_REMOTE_EVENT_FAILED) {
            s->active->remote_failed_state = true;
            x->link_up = false;
            scope_ixgbe_raise(s, SCOPE_IXGBE_EICR_LSC);
            break;
        }
    }
    if (scope_ixgbe_tx_completion_timed_out(s, now_us)) {
        s->active->remote_failed_state = true;
        x->link_up = false;
        scope_ixgbe_raise(s, SCOPE_IXGBE_EICR_LSC);
        return true;
    }
    progressed |= scope_ixgbe_poll_pending(s, &x->tx_pending, true);
    progressed |= scope_ixgbe_poll_pending(s, &x->rx_pending, false);
    scope_ixgbe_update_intx(s);
    return progressed;
}

static void scope_ixgbe_process_bar_packet(
    ScopeProxyState *s, const struct scope_dma32_packet *pkt)
{
    ScopeIxgbeState *x = s->active->ixgbe;
    uint8_t size = SCOPE_VSWITCH_PKT_SIZE(pkt->flags);
    uint8_t wstrb = SCOPE_VSWITCH_PKT_WSTRB(pkt->flags);
    uint64_t lane = ((uint64_t)pkt->guest_addr_lo << 32) | pkt->data;
    uint64_t data = UINT64_MAX;
    bool ok = false;

    switch (pkt->type) {
    case SCOPE_PKT_TYPE_BAR_WRITE:
        if (scope_ixgbe_is_tail(pkt->bar_offset, size, wstrb)) {
            ScopeIxgbePendingQueue *q = pkt->bar_offset == SCOPE_IXGBE_TDT0 ?
                &x->tx_pending : &x->rx_pending;
            ok = scope_ixgbe_pending_push(q, pkt->seq,
                                          scope_extract_dword32(lane,
                                                                pkt->bar_offset));
            if (ok) {
                /*
                 * Packet-mediated ixgbe does not need the post-B-response
                 * BAR_WRITE_DONE notification. Queue the tail for normal
                 * backend polling and complete the MMIO write normally.
                 */
                ok = scope_write_bar_response(s, pkt->seq, 0, 0,
                                              false, false);
            }
        } else {
            ok = scope_ixgbe_bar_write(s, pkt->bar_offset, lane, wstrb, size);
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
        /*
         * No ixgbe BAR write requests this follow-up notification anymore.
         * Ignore a stale packet from an older producer generation.
         */
        break;
    case SCOPE_PKT_TYPE_BAR_READ:
        ok = scope_ixgbe_bar_read(s, pkt->bar_offset, size, &data);
        scope_write_bar_response(s, pkt->seq, ok ? 0 : 0x2U,
                                 ok ? data : UINT64_MAX, true, false);
        break;
    default:
        break;
    }
}

static bool scope_ixgbe_backend_preflight(ScopeProxyState *s, Error **errp)
{
    if (s->active->transport != SCOPE_TRANSPORT_REMOTE_RDMA ||
        s->active->remote_memory_mode !=
            (s->active->remote_ixgbe_shadow_ring ?
             SCOPE_REMOTE_MEMORY_INLINE : SCOPE_REMOTE_MEMORY_HOST_STAGING)) {
        error_setg(errp, "ixgbe backend requires remote-rdma and the memory "
                   "mode selected by remote-device-mode");
        return false;
    }
    return true;
}

static bool scope_ixgbe_backend_realize(ScopeProxyState *s, Error **errp)
{
    ScopeBackend *be = s->active;
    ScopeRemoteConfig config = {
        .host = be->remote_host,
        .service = be->remote_service,
        .device_id = be->remote_device_id,
        .rdma_device = be->rdma_device,
        .memory_mode = be->remote_memory_mode,
        .host_staging_size = be->remote_staging_size,
        .device_type = SCOPE_REMOTE_DEVICE_IXGBE_PACKET,
        .guest_ddr_base = s->guest_ddr_base,
        .guest_ddr_size = s->guest_ddr_size,
        .coherent_alias_base = s->bypass_coherent_alias_base,
        .ixgbe_shadow_ring = be->remote_ixgbe_shadow_ring,
    };
    struct scope_remote_device_info info = { 0 };
    ScopeIxgbeState *x;

    be->remote = scope_remote_connect(&config, &info, errp);
    if (!be->remote) {
        return false;
    }
    x = g_new0(ScopeIxgbeState, 1);
    be->ixgbe = x;
    memcpy(x->mac, info.mac, sizeof(x->mac));
    x->mtu = info.mtu;
    x->link_up = info.link_up != 0;
    x->next_tx_id = 1;
    scope_ixgbe_init_virtual_eeprom(x);
    be->real_bar0_size = SCOPE_IXGBE_BAR0_SIZE;
    be->real_bar0_flags = IORESOURCE_MEM;
    scope_ixgbe_reset_model(s);
    SCOPE_PRINTF("[SCOPE REMOTE][IXGBE][READY] backend=%u virtual=%02x:00.0 "
                 "device=8086:10fb mac=%02x:%02x:%02x:%02x:%02x:%02x "
                 "mtu=%u link=%s host=%s service=%s mode=%s\n", be->id,
                 3U + be->id, x->mac[0], x->mac[1], x->mac[2], x->mac[3],
                 x->mac[4], x->mac[5], x->mtu, x->link_up ? "up" : "down",
                 be->remote_host, be->remote_service,
                 be->remote_ixgbe_shadow_ring ? "shadow-ring" : "packet");
    return true;
}

static void scope_ixgbe_backend_cleanup(ScopeProxyState *s, ScopeBackend *be)
{
    scope_remote_disconnect(be->remote);
    be->remote = NULL;
    g_free(be->ixgbe);
    be->ixgbe = NULL;
}

static const ScopeBackendOps scope_ixgbe_backend_ops = {
    .name = "ixgbe",
    .requires_real_pci = false,
    .preflight = scope_ixgbe_backend_preflight,
    .realize = scope_ixgbe_backend_realize,
    .cleanup = scope_ixgbe_backend_cleanup,
    .process_bar_packet = scope_ixgbe_process_bar_packet,
    .poll = scope_ixgbe_poll,
};
