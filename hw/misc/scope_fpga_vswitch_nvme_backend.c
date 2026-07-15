/*
 * NVMe backend dispatch for scope-fpga-vswitch.
 *
 * The mature queue, PRP and CQ helpers remain private to the translation
 * unit.  This file provides the backend boundary consumed by the generic
 * vSwitch manager, matching the independently implemented 82580 backend.
 */

static void scope_nvme_backend_process_bar_packet(
    ScopeProxyState *s, const struct scope_dma32_packet *pkt)
{
    uint16_t bdf = SCOPE_VSWITCH_PKT_BDF(pkt->flags);
    uint8_t size_bytes = SCOPE_VSWITCH_PKT_SIZE(pkt->flags);
    uint8_t wstrb = SCOPE_VSWITCH_PKT_WSTRB(pkt->flags);
    uint64_t lane_data = ((uint64_t)pkt->guest_addr_lo << 32) | pkt->data;
    uint32_t resp = 0x2U;
    uint64_t data = 0;
    bool ok;

    switch (pkt->type) {
    case SCOPE_PKT_TYPE_BAR_WRITE: {
        uint16_t qid = 0;
        bool early_resp = scope_is_early_sq_doorbell_write(
            s, pkt->bar_offset, wstrb, size_bytes, &qid);

        if (early_resp &&
            !scope_write_bar_response(s, pkt->seq, 0, 0, false, true)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "SCOPE: Failed to write early BAR doorbell response\n");
        }
        if (early_resp) {
            scope_defer_sq_doorbell_write(s, pkt->seq, pkt->bar_offset,
                                          lane_data, wstrb, size_bytes, qid);
            scope_log_bar_write(s, pkt->seq, pkt->bar_offset, pkt->flags,
                                size_bytes, wstrb, lane_data, true);
            break;
        }

        ok = scope_handle_nvme_bar_write(s, pkt->bar_offset, lane_data,
                                         wstrb, size_bytes);
        resp = ok ? 0x0U : 0x2U;
        scope_log_bar_write(s, pkt->seq, pkt->bar_offset, pkt->flags,
                            size_bytes, wstrb, lane_data, ok);
        if (!scope_write_bar_response(s, pkt->seq, resp, 0, false, false)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "SCOPE: Failed to write BAR write response\n");
        }
        break;
    }
    case SCOPE_PKT_TYPE_BAR_WRITE_DONE:
        scope_process_deferred_sq_doorbell(s, pkt);
        break;
    case SCOPE_PKT_TYPE_BAR_READ:
        ok = scope_handle_nvme_bar_read(s, pkt->bar_offset, size_bytes, &data);
        resp = ok ? 0x0U : 0x2U;
        scope_log_bar_read(s, pkt->seq, pkt->bar_offset, pkt->flags,
                           size_bytes, data, resp);
        if (!scope_write_bar_response(s, pkt->seq, resp, data, true, false)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "SCOPE: Failed to write BAR read response\n");
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE: unsupported NVMe packet type %u for %02x:%02x.%u\n",
                      pkt->type, bdf >> 8, (bdf >> 3) & 0x1f, bdf & 0x7);
        break;
    }
}

static bool scope_nvme_backend_poll(ScopeProxyState *s, int64_t now_us)
{
    ScopeBackend *be = s->active;
    bool progressed = false;

    if (!be->pending_sq_db.valid) {
        return false;
    }
    if (now_us - be->last_pending_log_us >= 1000000) {
        SCOPE_PRINTF("[SCOPE PROXY][DB][PENDING] backend=%u qid=%u seq=%u "
                     "bar_done=%d off=0x%04x new_tail=%u\n",
                     be->id, be->pending_sq_db.qid, be->pending_sq_db.seq,
                     be->pending_sq_db.bar_done, be->pending_sq_db.offset,
                     be->pending_sq_db.new_tail);
        SCOPE_FFLUSH(stdout);
        be->last_pending_log_us = now_us;
    }
    if (!be->pending_sq_db.bar_done) {
        progressed = scope_recover_missing_bar_done(s, now_us);
    }
    if (be->pending_sq_db.valid && be->pending_sq_db.bar_done &&
        now_us >= be->pending_sq_db.next_visibility_retry_us) {
        scope_try_process_pending_sq_doorbell(s);
        progressed = true;
    }
    return progressed;
}

static const ScopeBackendOps scope_nvme_backend_ops = {
    .name = "nvme",
    .realize = scope_nvme_backend_realize,
    .cleanup = scope_nvme_backend_cleanup,
    .process_bar_packet = scope_nvme_backend_process_bar_packet,
    .poll = scope_nvme_backend_poll,
};
