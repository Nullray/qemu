/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Physical xHCI ownership. Included by scope_fpga_vswitch.c. */

#define SCOPE_XHCI_DMA_CAP       0xb0U
#define SCOPE_XHCI_DMA_CAP_LEN   40U
#define SCOPE_XHCI_DMA_SIGNATURE 0x53435045U
#define SCOPE_XHCI_DMA_FLAGS     3U /* translated DMA addresses, coherent */
#define SCOPE_XHCI_DMA_READY     0x444d4150U
#define SCOPE_XHCI_SBRN          0x60U
#define SCOPE_XHCI_RS            (1U << 0)
#define SCOPE_XHCI_INTE          (1U << 2)
#define SCOPE_XHCI_HCH           (1U << 0)
#define SCOPE_XHCI_EINT          (1U << 3)
#define SCOPE_XHCI_CNR           (1U << 11)

struct ScopeXhciState {
    uint8_t pci_config[256];
    int pci_fd;
    uint32_t operational;
    uint32_t runtime;
    uint32_t doorbell;
    uint32_t doorbell_end;
    bool dma_ready;
    bool initialized;
    bool failed;
    bool bus_master;
    bool doorbell_pending;
    int64_t doorbell_since_us;
    struct scope_dma32_packet pending_doorbell;
};

static void scope_vcfg_init_xhci_endpoint(ScopeProxyState *s,
                                         ScopeVswitchConfigFn *fn,
                                         ScopeBackend *be)
{
    const uint8_t pcie = 0x70;
    const uint8_t cap = SCOPE_XHCI_DMA_CAP;
    uint64_t dma_start = s->fpga_bypass_bar_base +
        s->bypass_coherent_alias_base + s->guest_ddr_base;

    memset(fn, 0, sizeof(*fn));
    fn->bdf = be->virtual_bdf;
    memcpy(fn->config, be->xhci->pci_config, PCI_COMMAND);
    memcpy(fn->config + PCI_REVISION_ID,
           be->xhci->pci_config + PCI_REVISION_ID, 4);
    memcpy(fn->config + PCI_SUBSYSTEM_VENDOR_ID,
           be->xhci->pci_config + PCI_SUBSYSTEM_VENDOR_ID, 4);
    /* xhci-pci reads the physical Serial Bus Release Number during setup. */
    fn->config[SCOPE_XHCI_SBRN] = be->xhci->pci_config[SCOPE_XHCI_SBRN];
    fn->config[PCI_INTERRUPT_PIN] = 1;
    fn->config[PCI_INTERRUPT_LINE] = 0xff;
    pci_set_long(fn->config + PCI_BASE_ADDRESS_0,
                 PCI_BASE_ADDRESS_MEM_TYPE_64);
    pci_set_word(fn->config + PCI_STATUS, PCI_STATUS_CAP_LIST);
    fn->config[PCI_CAPABILITY_LIST] = pcie;
    fn->config[pcie + PCI_CAP_LIST_ID] = PCI_CAP_ID_EXP;
    fn->config[pcie + PCI_CAP_LIST_NEXT] = cap;
    pci_set_word(fn->config + pcie + PCI_EXP_FLAGS,
                 PCI_EXP_FLAGS_VER2 | (PCI_EXP_TYPE_ENDPOINT << 4));
    pci_set_long(fn->config + pcie + PCI_EXP_LNKCAP,
                 PCI_EXP_LNKCAP_SLS_2_5GB |
                 QEMU_PCI_EXP_LNKCAP_MLW(QEMU_PCI_EXP_LNK_X1));
    pci_set_word(fn->config + pcie + PCI_EXP_LNKSTA,
                 PCI_EXP_LNKSTA_CLS_2_5GB |
                 QEMU_PCI_EXP_LNKSTA_NLW(QEMU_PCI_EXP_LNK_X1));

    /* Per-function firmware description consumed by guest PCI DMA setup. */
    fn->config[cap] = PCI_CAP_ID_VNDR;
    fn->config[cap + 2] = SCOPE_XHCI_DMA_CAP_LEN;
    fn->config[cap + 3] = 1;
    pci_set_long(fn->config + cap + 4, SCOPE_XHCI_DMA_SIGNATURE);
    pci_set_long(fn->config + cap + 8, SCOPE_XHCI_DMA_FLAGS);
    stq_le_p(fn->config + cap + 16, s->guest_ddr_base);
    stq_le_p(fn->config + cap + 24, dma_start);
    stq_le_p(fn->config + cap + 32, s->guest_ddr_size);

    scope_vcfg_allow_range(fn, PCI_COMMAND, 2);
    scope_vcfg_w1c_range(fn, PCI_STATUS, 2);
    scope_vcfg_allow_range(fn, PCI_INTERRUPT_LINE, 1);
    scope_vcfg_allow_range(fn, cap + 12, 4);
    pci_set_long(fn->wmask + PCI_BASE_ADDRESS_0,
                 ~(be->real_bar0_size - 1U) & PCI_BASE_ADDRESS_MEM_MASK);
    pci_set_long(fn->wmask + PCI_BASE_ADDRESS_0 + 4, UINT32_MAX);
    scope_vcfg_allow_range(fn, pcie + PCI_EXP_DEVCTL, 2);
    scope_vcfg_w1c_range(fn, pcie + PCI_EXP_DEVSTA, 2);
}

static bool scope_xhci_halt(ScopeProxyState *s, Error **errp)
{
    ScopeXhciState *x = s->active->xhci;
    uint32_t cmd, status;
    int64_t deadline = g_get_monotonic_time() + 1000000;

    if (!scope_real_bar_read32(s, x->operational, &cmd) ||
        !scope_real_bar_write32(s, x->operational,
                                cmd & ~(SCOPE_XHCI_RS | SCOPE_XHCI_INTE))) {
        error_setg(errp, "xhci: cannot stop physical controller");
        return false;
    }
    do {
        if (!scope_real_bar_read32(s, x->operational + 4, &status)) {
            break;
        }
        if (status & SCOPE_XHCI_HCH) {
            return true;
        }
        g_usleep(100);
    } while (g_get_monotonic_time() < deadline);
    error_setg(errp, "xhci: physical controller did not halt");
    return false;
}

static bool scope_xhci_set_pci_command(ScopeBackend *be, uint16_t command)
{
    uint8_t buf[2];
    uint16_t verify;

    stw_le_p(buf, command);
    if (pwrite(be->xhci->pci_fd, buf, 2, PCI_COMMAND) != 2 ||
        pread(be->xhci->pci_fd, buf, 2, PCI_COMMAND) != 2) {
        return false;
    }
    verify = lduw_le_p(buf);
    return (verify & (PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER |
                      PCI_COMMAND_INTX_DISABLE)) == command;
}

static bool scope_xhci_prepare_pci(ScopeProxyState *s, Error **errp)
{
    ScopeBackend *be = s->active;

    be->original_pci_command = lduw_le_p(be->xhci->pci_config + PCI_COMMAND);
    be->pci_command_saved = true;
    /* Do not enable DMA until guest firmware mapping has been acknowledged. */
    if (!scope_xhci_set_pci_command(be, PCI_COMMAND_MEMORY |
                                       PCI_COMMAND_INTX_DISABLE)) {
        error_setg_errno(errp, errno,
                         "xhci: cannot disable physical bus master");
        return false;
    }
    return true;
}

static void scope_xhci_fail(ScopeProxyState *s)
{
    ScopeBackend *be = s->active;

    be->xhci->failed = true;
    be->xhci->doorbell_pending = false;
    be->xhci->bus_master = false;
    be->intx_pending = false;
    if (!scope_xhci_set_pci_command(be, PCI_COMMAND_MEMORY |
                                      PCI_COMMAND_INTX_DISABLE)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE xhci: failed to disable physical bus master\n");
    }
}

static bool scope_xhci_preflight(ScopeProxyState *s, Error **errp)
{
    ScopeBackend *be = s->active;
    ScopeXhciState *x = g_new0(ScopeXhciState, 1);
    g_autofree char *base = g_strdup_printf("/sys/bus/pci/devices/%s",
                                           be->real_host_bdf);
    g_autofree char *driver = g_strdup_printf("%s/driver", base);
    g_autofree char *group = g_strdup_printf("%s/iommu_group", base);
    g_autofree char *config = g_strdup_printf("%s/config", base);
    unsigned pos, count;

    be->xhci = x;
    x->pci_fd = -1;
    if (!s->allow_unsafe_xhci_p2p) {
        error_setg(errp, "physical xhci requires allow-unsafe-xhci-p2p=on "
                   "and a verified FPGA P2P route");
        return false;
    }
    if (!s->bypass_coherent_alias_base || s->guest_mem_raw_fallback ||
        s->fpga_bypass_bar_base > UINT64_MAX - s->fpga_bypass_bar_size) {
        error_setg(errp,
                   "xhci requires a coherent FPGA BAR without raw fallback");
        return false;
    }
    if (g_file_test(driver, G_FILE_TEST_EXISTS)) {
        error_setg(errp, "xhci %s is still bound; "
                   "unbind only the target function",
                   be->real_host_bdf);
        return false;
    }
    if (g_file_test(group, G_FILE_TEST_EXISTS)) {
        g_autofree char *path = g_strdup_printf("%s/type", group);
        g_autofree char *type = NULL;

        if (!g_file_get_contents(path, &type, NULL, NULL) ||
            strcmp(g_strstrip(type), "identity")) {
            error_setg(errp, "xhci requires an identity host IOMMU domain "
                       "or no active IOMMU; translated/unknown domain rejected");
            return false;
        }
    }
    x->pci_fd = open(config, O_RDWR | O_CLOEXEC);
    if (x->pci_fd < 0 ||
        pread(x->pci_fd, x->pci_config, sizeof(x->pci_config), 0) !=
        sizeof(x->pci_config)) {
        error_setg_errno(errp, errno, "xhci: cannot read PCI configuration");
        return false;
    }
    if (x->pci_config[PCI_CLASS_PROG] != 0x30 ||
        lduw_le_p(x->pci_config + PCI_CLASS_DEVICE) != PCI_CLASS_SERIAL_USB) {
        error_setg(errp, "%s is not a PCI xHCI function", be->real_host_bdf);
        return false;
    }
    /* Host IRQs must already have been released by the Linux driver. */
    pos = x->pci_config[PCI_CAPABILITY_LIST] & ~3U;
    for (count = 0; pos && count < 48; count++) {
        uint16_t flags;
        if (pos < 0x40 || pos > 0xfc) {
            break;
        }
        flags = lduw_le_p(x->pci_config + pos + 2);
        if ((x->pci_config[pos] == PCI_CAP_ID_MSI && (flags & 1)) ||
            (x->pci_config[pos] == PCI_CAP_ID_MSIX && (flags & 0x8000))) {
            error_setg(errp, "xhci host MSI/MSI-X is still enabled");
            return false;
        }
        pos = x->pci_config[pos + 1] & ~3U;
    }
    if (pos) {
        error_setg(errp, "xhci: malformed physical PCI capability chain");
        return false;
    }
    return true;
}

static bool scope_xhci_realize(ScopeProxyState *s, Error **errp)
{
    ScopeXhciState *x = s->active->xhci;
    uint32_t cap, hcc, hcs, runtime, doorbell, status;
    uint32_t ext;
    unsigned count;

    if (!scope_real_bar_read32(s, 0, &cap) ||
        !scope_real_bar_read32(s, 4, &hcs) ||
        !scope_real_bar_read32(s, 0x10, &hcc) ||
        !scope_real_bar_read32(s, 0x14, &doorbell) ||
        !scope_real_bar_read32(s, 0x18, &runtime)) {
        error_setg(errp, "xhci: cannot read physical capability registers");
        return false;
    }
    x->operational = cap & 0xff;
    x->runtime = runtime & ~0x1fU;
    x->doorbell = doorbell & ~3U;
    x->doorbell_end = x->doorbell + 4U * ((hcs & 0xff) + 1U);
    if (!(hcc & 1) || x->operational < 0x20 ||
        x->operational + 0x400U + ((hcs >> 24) * 16U) >
            s->active->real_bar0_size ||
        x->runtime < 0x20 ||
        (uint64_t)x->runtime + 0x40U > s->active->real_bar0_size ||
        x->doorbell < 0x20 ||
        x->doorbell_end < x->doorbell ||
        x->doorbell_end > s->active->real_bar0_size) {
        error_setg(errp, "xhci: invalid BAR layout or no 64-bit DMA support");
        return false;
    }
    if (!scope_real_bar_read32(s, x->operational + 4, &status) ||
        (status & SCOPE_XHCI_CNR)) {
        error_setg(errp, "xhci: physical controller is not ready");
        return false;
    }
    /* Host Linux did handoff; do not force ownership away from firmware. */
    ext = (hcc >> 16) * 4U;
    for (count = 0; ext && count < 256; count++) {
        uint32_t value;
        if (!scope_real_bar_read32(s, ext, &value)) {
            break;
        }
        if ((value & 0xff) == 1 && (value & (1U << 16))) {
            error_setg(errp, "xhci: BIOS still owns the physical controller");
            return false;
        }
        if ((value & 0xff) == 1) {
            uint32_t control;
            const uint32_t reserved = (7U << 1) | (0xffU << 5) | (7U << 17);

            if (!scope_real_bar_read32(s, ext + 4, &control) ||
                !scope_real_bar_write32(s, ext, value | (1U << 24)) ||
                !scope_real_bar_write32(s, ext + 4,
                                        (control & reserved) | (7U << 29))) {
                error_setg(errp,
                           "xhci: cannot assert OS ownership/disable SMI");
                return false;
            }
        }
        if (!(value & 0xff00)) {
            ext = 0;
            break;
        }
        ext += ((value >> 8) & 0xff) * 4U;
    }
    if (ext) {
        error_setg(errp, "xhci: invalid extended capability chain");
        return false;
    }
    /* Never touch USBCMD if firmware owns xHC or its layout is invalid. */
    x->initialized = true;
    if (!scope_xhci_halt(s, errp)) {
        return false;
    }
    SCOPE_PRINTF("[SCOPE XHCI] physical %s -> guest backend=%u "
                 "DMA bus base=0x%016" PRIx64 " legacy INTx\n",
                 s->active->real_host_bdf, s->active->id,
                 s->fpga_bypass_bar_base + s->bypass_coherent_alias_base +
                 s->guest_ddr_base);
    return true;
}

static void scope_xhci_config_changed(ScopeProxyState *s, ScopeBackend *be,
                                     const ScopeVswitchConfigFn *fn)
{
    ScopeXhciState *x = be->xhci;
    ScopeBackend *saved = s->active;
    uint16_t command = PCI_COMMAND_MEMORY | PCI_COMMAND_INTX_DISABLE;
    bool ready = ldl_le_p(fn->config + SCOPE_XHCI_DMA_CAP + 12) ==
                 SCOPE_XHCI_DMA_READY;

    if (ready && (lduw_le_p(fn->config + PCI_COMMAND) & PCI_COMMAND_MASTER)) {
        command |= PCI_COMMAND_MASTER;
    }
    s->active = be;
    if (x->bus_master && !(command & PCI_COMMAND_MASTER)) {
        Error *err = NULL;
        if (!scope_xhci_halt(s, &err)) {
            error_report_err(err);
            x->failed = true;
        }
        x->doorbell_pending = false;
    }
    if (x->failed) {
        command &= ~PCI_COMMAND_MASTER;
    }
    if (!scope_xhci_set_pci_command(be, command)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE xhci: physical PCI command failed\n");
        scope_xhci_fail(s);
        command &= ~PCI_COMMAND_MASTER;
    }
    x->dma_ready = ready;
    x->bus_master = !!(command & PCI_COMMAND_MASTER);
    s->active = saved;
}

static bool scope_xhci_commit_write(ScopeProxyState *s,
                                    const struct scope_dma32_packet *pkt)
{
    ScopeXhciState *x = s->active->xhci;
    uint64_t value = ((uint64_t)pkt->guest_addr_lo << 32) | pkt->data;
    uint32_t fence;

    return scope_real_bar_write(s, pkt->bar_offset, value,
                                SCOPE_VSWITCH_PKT_WSTRB(pkt->flags),
                                SCOPE_VSWITCH_PKT_SIZE(pkt->flags)) &&
           scope_real_bar_read32(s, x->operational + 4, &fence);
}

static void scope_xhci_process_bar_packet(
    ScopeProxyState *s, const struct scope_dma32_packet *pkt)
{
    ScopeXhciState *x = s->active->xhci;
    uint8_t size = SCOPE_VSWITCH_PKT_SIZE(pkt->flags);
    uint8_t strobe = SCOPE_VSWITCH_PKT_WSTRB(pkt->flags);
    uint64_t value = ((uint64_t)pkt->guest_addr_lo << 32) | pkt->data;
    uint64_t data = UINT64_MAX;
    bool read = pkt->type == SCOPE_PKT_TYPE_BAR_READ;
    bool ok = false;

    if (pkt->type == SCOPE_PKT_TYPE_BAR_WRITE_DONE) {
        if (x->doorbell_pending && pkt->seq == x->pending_doorbell.seq &&
            pkt->bar_offset == x->pending_doorbell.bar_offset) {
            x->doorbell_pending = false;
            if (!x->dma_ready || !x->bus_master ||
                !scope_xhci_commit_write(s, &x->pending_doorbell)) {
                scope_xhci_fail(s);
                qemu_log_mask(LOG_GUEST_ERROR,
                              "SCOPE xhci: deferred physical doorbell failed\n");
            }
        }
        return;
    }
    if (!x->failed && (size == 4 || size == 8) &&
        !(pkt->bar_offset & (size - 1U))) {
        if (read) {
            ok = scope_real_bar_read(s, pkt->bar_offset, size, &data);
            /* Legacy mode uses only interrupter zero. */
            if (ok && ((pkt->bar_offset == 4 && size == 4) ||
                       (pkt->bar_offset == 0 && size == 8))) {
                uint32_t hcs = (uint32_t)(data >> 32);
                hcs = (hcs & ~(0x7ffU << 8)) | (1U << 8);
                data = (data & UINT32_MAX) | ((uint64_t)hcs << 32);
            }
        } else if (x->dma_ready && x->bus_master) {
            if (pkt->bar_offset >= x->doorbell &&
                pkt->bar_offset < x->doorbell_end) {
                if (!x->doorbell_pending && size == 4 &&
                    strobe == scope_pack_wstrb4_for_offset(pkt->bar_offset)) {
                    x->pending_doorbell = *pkt;
                    x->doorbell_pending = true;
                    x->doorbell_since_us = g_get_monotonic_time();
                    /* Same ordering handshake used by the local NVMe path. */
                    scope_write_bar_response(s, pkt->seq, 0, 0, false, true);
                    return;
                }
            } else {
                ok = scope_xhci_commit_write(s, pkt);
            }
        } else if (pkt->bar_offset == x->operational && size == 4 &&
                   !(scope_extract_dword32(value, pkt->bar_offset) &
                     SCOPE_XHCI_RS)) {
            /* Halt/reset is allowed before mapping, but running/DMA is not. */
            ok = scope_real_bar_write(s, pkt->bar_offset, value, strobe, size);
        }
    }
    scope_write_bar_response(s, pkt->seq, ok ? 0 : 2,
                             read ? data : 0, read, false);
}

static bool scope_xhci_poll(ScopeProxyState *s, int64_t now_us)
{
    ScopeXhciState *x = s->active->xhci;
    uint32_t command, status, iman;
    const ScopeVswitchConfigFn *fn =
        &s->vcfg[SCOPE_VSWITCH_EP_SLOT(s->active->id)];

    if (x->doorbell_pending && now_us - x->doorbell_since_us > 1000000) {
        scope_xhci_fail(s);
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SCOPE xhci: timed out waiting for BAR_WRITE_DONE\n");
    }

    s->active->intx_pending = !x->failed && x->dma_ready && x->bus_master &&
        !(lduw_le_p(fn->config + PCI_COMMAND) & PCI_COMMAND_INTX_DISABLE) &&
        scope_real_bar_read32(s, x->operational, &command) &&
        (command & SCOPE_XHCI_INTE) &&
        scope_real_bar_read32(s, x->operational + 4, &status) &&
        (status & SCOPE_XHCI_EINT) &&
        scope_real_bar_read32(s, x->runtime + 0x20, &iman) &&
        ((iman & 3U) == 3U);
    return false;
}

static void scope_xhci_cleanup(ScopeProxyState *s, ScopeBackend *be)
{
    ScopeXhciState *x = be->xhci;
    ScopeBackend *saved = s->active;
    Error *err = NULL;

    if (!x) {
        return;
    }
    s->active = be;
    if (x->initialized && !scope_xhci_halt(s, &err)) {
        error_report_err(err);
    }
    if (be->pci_command_saved && x->pci_fd >= 0) {
        scope_xhci_set_pci_command(be, PCI_COMMAND_MEMORY |
                                      PCI_COMMAND_INTX_DISABLE);
        /* Never restore BME into a controller holding guest queue addresses. */
        be->original_pci_command &= ~PCI_COMMAND_MASTER;
    }
    if (x->pci_fd >= 0) {
        close(x->pci_fd);
    }
    g_free(x);
    be->xhci = NULL;
    s->active = saved;
}

static const ScopeBackendOps scope_xhci_backend_ops = {
    .name = "xhci",
    .requires_real_pci = true,
    .preflight = scope_xhci_preflight,
    .realize = scope_xhci_realize,
    .cleanup = scope_xhci_cleanup,
    .process_bar_packet = scope_xhci_process_bar_packet,
    .poll = scope_xhci_poll,
};
