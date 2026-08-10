#include "qemu/osdep.h"
#include "qemu/atomic.h"
#include "qemu/bswap.h"
#include "qemu/timer.h"
#include "hw/misc/scope_fpga_vswitch_irq_latency.h"

#define SCOPE_IRQ_LATENCY_DEFAULT_OUTPUT "/tmp/scope_irq_latency.csv"
#define SCOPE_IRQ_LATENCY_TIMEOUT_NS (5LL * NANOSECONDS_PER_SECOND)
#define SCOPE_IRQ_LATENCY_MAX_SAMPLES 1000000U

typedef enum ScopeIrqLatencyPhase {
    SCOPE_IRQ_LAT_IDLE = 0,
    SCOPE_IRQ_LAT_WAIT_INTX,
    SCOPE_IRQ_LAT_WAIT_MARKER,
} ScopeIrqLatencyPhase;

typedef struct ScopeIrqLatencySample {
    uint32_t index;
    uint16_t cid;
    uint16_t cq_tail;
    uint32_t marker_seq;
    int64_t cqe_last_absent_ns;
    int64_t cqe_first_present_ns;
    int64_t intx_write_start_ns;
    int64_t intx_write_done_ns;
    int64_t marker_last_old_ns;
    int64_t marker_first_new_ns;
} ScopeIrqLatencySample;

struct ScopeIrqLatencyState {
    uint32_t backend_id;
    uint32_t qid;
    uint16_t target_bdf;
    uint32_t target_samples;
    char *output_path;

    ScopeIrqLatencySample *samples;
    uint32_t sample_count;
    ScopeIrqLatencySample current;
    ScopeIrqLatencyPhase phase;
    int64_t phase_start_ns;
    int64_t last_cqe_absent_ns;
    bool dumped;

    int bypass_fd;
    uint64_t bypass_bar_size;
    uint64_t coherent_alias_base;
    uint64_t guest_ddr_base;
    uint64_t guest_ddr_size;
    size_t host_page_size;
    void *marker_map;
    size_t marker_map_len;
    size_t marker_map_delta;
    uint64_t marker_guest_pa;
    uint32_t marker_addr_lo;
    uint32_t marker_addr_hi;
    bool marker_addr_lo_valid;
    bool marker_addr_hi_valid;
    bool transport_ready;
    bool marker_ready;
    uint32_t observed_marker_seq;

    uint64_t overlap_count;
    uint64_t coalesced_count;
    uint64_t timeout_count;
    uint64_t intx_write_error_count;
    uint64_t marker_registration_error_count;
    uint64_t marker_not_ready_count;
    uint64_t marker_seq_jump_count;
    uint64_t unbracketed_cqe_count;
};

static int64_t scope_irq_latency_now_ns(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_HOST);
}

static void scope_irq_latency_reset_current(ScopeIrqLatencyState *s)
{
    memset(&s->current, 0, sizeof(s->current));
    s->phase = SCOPE_IRQ_LAT_IDLE;
    s->phase_start_ns = 0;
}

static int scope_irq_latency_cmp_i64(const void *a, const void *b)
{
    int64_t av = *(const int64_t *)a;
    int64_t bv = *(const int64_t *)b;

    return (av > bv) - (av < bv);
}

static int64_t scope_irq_latency_percentile(const int64_t *sorted,
                                            uint32_t count,
                                            unsigned int permille)
{
    uint64_t rank;

    if (!count) {
        return 0;
    }
    rank = ((uint64_t)permille * count + 999U) / 1000U;
    rank = MAX(rank, 1U);
    return sorted[MIN(rank - 1U, (uint64_t)count - 1U)];
}

static void scope_irq_latency_print_distribution(const char *name,
                                                  int64_t *values,
                                                  uint32_t count)
{
    long double sum = 0;
    uint32_t i;

    qsort(values, count, sizeof(*values), scope_irq_latency_cmp_i64);
    for (i = 0; i < count; i++) {
        sum += values[i];
    }

    fprintf(stdout,
            "[SCOPE IRQ LAT][SUMMARY] metric=%s count=%u mean_ns=%.2Lf "
            "p50_ns=%" PRId64 " p90_ns=%" PRId64 " p95_ns=%" PRId64
            " p99_ns=%" PRId64 " p999_ns=%" PRId64 " max_ns=%" PRId64 "\n",
            name, count, count ? sum / count : 0.0L,
            scope_irq_latency_percentile(values, count, 500),
            scope_irq_latency_percentile(values, count, 900),
            scope_irq_latency_percentile(values, count, 950),
            scope_irq_latency_percentile(values, count, 990),
            scope_irq_latency_percentile(values, count, 999),
            count ? values[count - 1] : 0);
}

static int64_t scope_irq_latency_midpoint(int64_t lower, int64_t upper)
{
    return lower + (upper - lower) / 2;
}

static bool scope_irq_latency_dump(ScopeIrqLatencyState *s)
{
    int64_t *software;
    int64_t *mailbox;
    int64_t *irq_lower;
    int64_t *irq_estimate;
    int64_t *irq_upper;
    int64_t *full_lower;
    int64_t *full_estimate;
    int64_t *full_upper;
    int64_t *full_interval;
    int64_t *cqe_window;
    int64_t *marker_window;
    FILE *fp;
    uint32_t i;

    if (s->dumped || !s->sample_count) {
        return true;
    }

    fp = fopen(s->output_path, "w");
    if (!fp) {
        fprintf(stderr, "SCOPE IRQ LAT: failed to open %s: %s\n",
                s->output_path, strerror(errno));
        return false;
    }

    fprintf(fp, "sample,backend,qid,cid,cq_tail,marker_seq,"
            "cqe_last_absent_ns,cqe_first_present_ns,intx_write_start_ns,"
            "intx_write_done_ns,marker_last_old_ns,marker_first_new_ns,"
            "software_ns,mailbox_write_ns,interrupt_lower_ns,"
            "interrupt_estimate_ns,interrupt_upper_ns,full_lower_ns,"
            "full_estimate_ns,full_upper_ns,full_interval_ns,"
            "cqe_visibility_window_ns,marker_visibility_window_ns\n");

    software = g_new(int64_t, s->sample_count);
    mailbox = g_new(int64_t, s->sample_count);
    irq_lower = g_new(int64_t, s->sample_count);
    irq_estimate = g_new(int64_t, s->sample_count);
    irq_upper = g_new(int64_t, s->sample_count);
    full_lower = g_new(int64_t, s->sample_count);
    full_estimate = g_new(int64_t, s->sample_count);
    full_upper = g_new(int64_t, s->sample_count);
    full_interval = g_new(int64_t, s->sample_count);
    cqe_window = g_new(int64_t, s->sample_count);
    marker_window = g_new(int64_t, s->sample_count);

    for (i = 0; i < s->sample_count; i++) {
        ScopeIrqLatencySample *sample = &s->samples[i];

        software[i] = MAX(sample->intx_write_start_ns -
                          sample->cqe_first_present_ns, 0);
        mailbox[i] = MAX(sample->intx_write_done_ns -
                         sample->intx_write_start_ns, 0);
        irq_lower[i] = MAX(sample->marker_last_old_ns -
                           sample->intx_write_done_ns, 0);
        irq_upper[i] = MAX(sample->marker_first_new_ns -
                           sample->intx_write_start_ns, 0);
        irq_estimate[i] = scope_irq_latency_midpoint(irq_lower[i],
                                                     irq_upper[i]);
        full_lower[i] = MAX(sample->marker_last_old_ns -
                            sample->cqe_first_present_ns, 0);
        full_upper[i] = MAX(sample->marker_first_new_ns -
                            sample->cqe_last_absent_ns, 0);
        full_estimate[i] = scope_irq_latency_midpoint(full_lower[i],
                                                      full_upper[i]);
        full_interval[i] = full_upper[i] - full_lower[i];
        cqe_window[i] = sample->cqe_first_present_ns -
                        sample->cqe_last_absent_ns;
        marker_window[i] = sample->marker_first_new_ns -
                           sample->marker_last_old_ns;

        fprintf(fp,
                "%u,%u,%u,%u,%u,%u,%" PRId64 ",%" PRId64 ",%" PRId64
                ",%" PRId64 ",%" PRId64 ",%" PRId64 ",%" PRId64
                ",%" PRId64 ",%" PRId64 ",%" PRId64 ",%" PRId64
                ",%" PRId64 ",%" PRId64 ",%" PRId64 ",%" PRId64
                ",%" PRId64 ",%" PRId64 "\n",
                sample->index, s->backend_id, s->qid, sample->cid,
                sample->cq_tail, sample->marker_seq,
                sample->cqe_last_absent_ns, sample->cqe_first_present_ns,
                sample->intx_write_start_ns, sample->intx_write_done_ns,
                sample->marker_last_old_ns, sample->marker_first_new_ns,
                software[i], mailbox[i], irq_lower[i], irq_estimate[i],
                irq_upper[i], full_lower[i], full_estimate[i], full_upper[i],
                full_interval[i], cqe_window[i], marker_window[i]);
    }
    if (fclose(fp) != 0) {
        fprintf(stderr, "SCOPE IRQ LAT: failed to close %s: %s\n",
                s->output_path, strerror(errno));
    }

    s->dumped = true;
    fprintf(stdout,
            "[SCOPE IRQ LAT][COUNTS] valid=%u target=%u overlap=%" PRIu64
            " coalesced=%" PRIu64 " timeout=%" PRIu64
            " intx_write_error=%" PRIu64 " marker_registration_error=%" PRIu64
            " marker_not_ready=%" PRIu64 " marker_seq_jump=%" PRIu64
            " unbracketed_cqe=%" PRIu64 " output=%s\n",
            s->sample_count, s->target_samples, s->overlap_count,
            s->coalesced_count, s->timeout_count,
            s->intx_write_error_count, s->marker_registration_error_count,
            s->marker_not_ready_count, s->marker_seq_jump_count,
            s->unbracketed_cqe_count, s->output_path);
    fprintf(stdout,
            "[SCOPE IRQ LAT][METHOD] CQ completion and DUT-handler marker are "
            "bracketed by coherent-alias reads; estimate is each sample's "
            "interval midpoint and lower/upper are conservative bounds.\n");
    scope_irq_latency_print_distribution("software", software,
                                         s->sample_count);
    scope_irq_latency_print_distribution("mailbox_write", mailbox,
                                         s->sample_count);
    scope_irq_latency_print_distribution("interrupt_delivery_lower",
                                         irq_lower, s->sample_count);
    scope_irq_latency_print_distribution("interrupt_delivery_estimate",
                                         irq_estimate, s->sample_count);
    scope_irq_latency_print_distribution("interrupt_delivery_upper",
                                         irq_upper, s->sample_count);
    scope_irq_latency_print_distribution("full_device_to_handler_lower",
                                         full_lower, s->sample_count);
    scope_irq_latency_print_distribution("full_device_to_handler_estimate",
                                         full_estimate, s->sample_count);
    scope_irq_latency_print_distribution("full_device_to_handler_upper",
                                         full_upper, s->sample_count);
    scope_irq_latency_print_distribution("full_interval_width",
                                         full_interval, s->sample_count);
    scope_irq_latency_print_distribution("cqe_visibility_window",
                                         cqe_window, s->sample_count);
    scope_irq_latency_print_distribution("marker_visibility_window",
                                         marker_window, s->sample_count);
    fflush(stdout);

    g_free(marker_window);
    g_free(cqe_window);
    g_free(full_interval);
    g_free(full_upper);
    g_free(full_estimate);
    g_free(full_lower);
    g_free(irq_upper);
    g_free(irq_estimate);
    g_free(irq_lower);
    g_free(mailbox);
    g_free(software);
    return true;
}

ScopeIrqLatencyState *scope_irq_latency_create(uint32_t backend_id,
                                               uint32_t qid,
                                               uint16_t target_bdf,
                                               uint32_t target_samples,
                                               const char *output_path,
                                               Error **errp)
{
    ScopeIrqLatencyState *s;

    if (!target_samples || target_samples > SCOPE_IRQ_LATENCY_MAX_SAMPLES) {
        error_setg(errp, "irq-latency-samples must be in [1, %u]",
                   SCOPE_IRQ_LATENCY_MAX_SAMPLES);
        return NULL;
    }
    if (qid > UINT16_MAX) {
        error_setg(errp, "irq-latency-qid must fit in 16 bits");
        return NULL;
    }

    s = g_new0(ScopeIrqLatencyState, 1);
    s->backend_id = backend_id;
    s->qid = qid;
    s->target_bdf = target_bdf;
    s->target_samples = target_samples;
    s->bypass_fd = -1;
    s->output_path = g_strdup(output_path && output_path[0] ? output_path :
                              SCOPE_IRQ_LATENCY_DEFAULT_OUTPUT);
    s->samples = g_new0(ScopeIrqLatencySample, target_samples);
    return s;
}

void scope_irq_latency_destroy(ScopeIrqLatencyState *s)
{
    if (!s) {
        return;
    }
    scope_irq_latency_dump(s);
    if (s->marker_map) {
        munmap(s->marker_map, s->marker_map_len);
    }
    g_free(s->samples);
    g_free(s->output_path);
    g_free(s);
}

bool scope_irq_latency_set_coherent_transport(ScopeIrqLatencyState *s,
                                              int bypass_fd,
                                              uint64_t bypass_bar_size,
                                              uint64_t coherent_alias_base,
                                              uint64_t guest_ddr_base,
                                              uint64_t guest_ddr_size,
                                              size_t host_page_size,
                                              Error **errp)
{
    uint64_t guest_end;

    if (!s) {
        return true;
    }
    if (bypass_fd < 0 || !bypass_bar_size || !coherent_alias_base ||
        !guest_ddr_size || !host_page_size ||
        (host_page_size & (host_page_size - 1U))) {
        error_setg(errp, "invalid coherent marker transport configuration");
        return false;
    }
    if (guest_ddr_base > UINT64_MAX - guest_ddr_size) {
        error_setg(errp, "guest DDR range overflows for IRQ latency marker");
        return false;
    }
    guest_end = guest_ddr_base + guest_ddr_size;
    if (coherent_alias_base > UINT64_MAX - guest_end ||
        coherent_alias_base + guest_end > bypass_bar_size) {
        error_setg(errp, "coherent alias does not cover guest DDR marker range");
        return false;
    }

    s->bypass_fd = bypass_fd;
    s->bypass_bar_size = bypass_bar_size;
    s->coherent_alias_base = coherent_alias_base;
    s->guest_ddr_base = guest_ddr_base;
    s->guest_ddr_size = guest_ddr_size;
    s->host_page_size = host_page_size;
    s->transport_ready = true;
    return true;
}

static uint32_t scope_irq_latency_read_marker32(ScopeIrqLatencyState *s,
                                                size_t offset)
{
    volatile uint32_t *reg = (volatile uint32_t *)
        ((uint8_t *)s->marker_map + s->marker_map_delta + offset);
    uint32_t value = *reg;

    smp_rmb();
    return le32_to_cpu(value);
}

static bool scope_irq_latency_refresh_marker_ready(ScopeIrqLatencyState *s)
{
    uint32_t magic;

    if (s->marker_ready) {
        return true;
    }
    if (!s->marker_map) {
        return false;
    }
    magic = scope_irq_latency_read_marker32(
        s, SCOPE_IRQ_LATENCY_MARKER_MAGIC_OFFSET);
    if (magic != SCOPE_IRQ_LATENCY_MARKER_PAGE_MAGIC) {
        return false;
    }
    s->observed_marker_seq = scope_irq_latency_read_marker32(
        s, SCOPE_IRQ_LATENCY_MARKER_SEQ_OFFSET);
    s->marker_ready = true;
    fprintf(stdout,
            "[SCOPE IRQ LAT][MARKER_READY] guest_pa=0x%016" PRIx64
            " seq=%u\n", s->marker_guest_pa, s->observed_marker_seq);
    fflush(stdout);
    return true;
}

static bool scope_irq_latency_map_marker(ScopeIrqLatencyState *s)
{
    uint64_t guest_end;
    uint64_t alias_offset;
    uint64_t map_base;
    size_t map_delta;
    size_t map_len;
    void *map;

    if (!s->transport_ready || !s->marker_addr_lo_valid ||
        !s->marker_addr_hi_valid) {
        return false;
    }
    s->marker_guest_pa = ((uint64_t)s->marker_addr_hi << 32) |
                         s->marker_addr_lo;
    guest_end = s->guest_ddr_base + s->guest_ddr_size;
    if (s->marker_guest_pa < s->guest_ddr_base ||
        s->marker_guest_pa > guest_end - 8U ||
        s->coherent_alias_base > UINT64_MAX - s->marker_guest_pa) {
        return false;
    }
    alias_offset = s->coherent_alias_base + s->marker_guest_pa;
    if (alias_offset > s->bypass_bar_size - 8U) {
        return false;
    }
    map_base = alias_offset & ~((uint64_t)s->host_page_size - 1U);
    map_delta = alias_offset - map_base;
    map_len = s->host_page_size;
    if (map_delta + 8U > map_len) {
        map_len += s->host_page_size;
    }
    if (map_base > (uint64_t)INT64_MAX) {
        return false;
    }

    map = mmap(NULL, map_len, PROT_READ | PROT_WRITE, MAP_SHARED,
               s->bypass_fd, (off_t)map_base);
    if (map == MAP_FAILED) {
        fprintf(stderr,
                "SCOPE IRQ LAT: marker mmap failed guest_pa=0x%016" PRIx64
                " alias=0x%016" PRIx64 ": %s\n",
                s->marker_guest_pa, alias_offset, strerror(errno));
        return false;
    }
    if (s->marker_map) {
        munmap(s->marker_map, s->marker_map_len);
    }
    s->marker_map = map;
    s->marker_map_len = map_len;
    s->marker_map_delta = map_delta;
    s->marker_ready = false;
    scope_irq_latency_refresh_marker_ready(s);
    return true;
}

bool scope_irq_latency_handle_cfg_write(ScopeIrqLatencyState *s,
                                        uint16_t bdf,
                                        uint32_t offset,
                                        unsigned int len,
                                        uint32_t wstrb,
                                        uint32_t value)
{
    bool registration_offset;

    registration_offset = offset == SCOPE_IRQ_LATENCY_MARKER_ADDR_LO_OFFSET ||
                          offset == SCOPE_IRQ_LATENCY_MARKER_ADDR_HI_OFFSET ||
                          offset == SCOPE_IRQ_LATENCY_MARKER_COMMIT_OFFSET;
    if (!registration_offset) {
        return false;
    }
    /* These dwords are reserved for marker registration on every endpoint. */
    if (!s || bdf != s->target_bdf) {
        return true;
    }
    if (len != 4 || (wstrb & 0xfU) != 0xfU) {
        s->marker_registration_error_count++;
        return true;
    }

    if (offset == SCOPE_IRQ_LATENCY_MARKER_ADDR_LO_OFFSET) {
        s->marker_addr_lo = value;
        s->marker_addr_lo_valid = true;
    } else if (offset == SCOPE_IRQ_LATENCY_MARKER_ADDR_HI_OFFSET) {
        s->marker_addr_hi = value;
        s->marker_addr_hi_valid = true;
    } else if (value != SCOPE_IRQ_LATENCY_MARKER_COMMIT_MAGIC ||
               !scope_irq_latency_map_marker(s)) {
        s->marker_registration_error_count++;
        fprintf(stderr,
                "SCOPE IRQ LAT: invalid marker registration commit=0x%08x "
                "addr=0x%08x%08x transport_ready=%d\n",
                value, s->marker_addr_hi, s->marker_addr_lo,
                s->transport_ready);
    }
    return true;
}

void scope_irq_latency_record_cqe_absent(ScopeIrqLatencyState *s,
                                         uint32_t backend_id,
                                         uint32_t qid)
{
    if (!s || s->dumped || !s->marker_ready ||
        backend_id != s->backend_id || qid != s->qid ||
        s->phase != SCOPE_IRQ_LAT_IDLE) {
        return;
    }
    s->last_cqe_absent_ns = scope_irq_latency_now_ns();
}

void scope_irq_latency_record_cqe(ScopeIrqLatencyState *s,
                                  uint32_t backend_id,
                                  uint32_t qid,
                                  uint16_t cid,
                                  uint16_t cq_tail)
{
    int64_t now_ns;

    if (!s || s->dumped || s->sample_count >= s->target_samples ||
        backend_id != s->backend_id || qid != s->qid) {
        return;
    }
    if (!scope_irq_latency_refresh_marker_ready(s)) {
        s->marker_not_ready_count++;
        return;
    }

    now_ns = scope_irq_latency_now_ns();
    if (s->phase != SCOPE_IRQ_LAT_IDLE) {
        if (now_ns - s->phase_start_ns > SCOPE_IRQ_LATENCY_TIMEOUT_NS) {
            s->timeout_count++;
            scope_irq_latency_reset_current(s);
        } else {
            s->overlap_count++;
            return;
        }
    }

    s->current.cid = cid;
    s->current.cq_tail = cq_tail;
    s->current.cqe_first_present_ns = now_ns;
    if (s->last_cqe_absent_ns && s->last_cqe_absent_ns <= now_ns) {
        s->current.cqe_last_absent_ns = s->last_cqe_absent_ns;
    } else {
        s->current.cqe_last_absent_ns = now_ns;
        s->unbracketed_cqe_count++;
    }
    s->last_cqe_absent_ns = 0;
    s->phase = SCOPE_IRQ_LAT_WAIT_INTX;
    s->phase_start_ns = now_ns;
}

void scope_irq_latency_record_intx(ScopeIrqLatencyState *s,
                                   bool old_level,
                                   bool new_level,
                                   bool write_ok,
                                   int64_t write_start_ns,
                                   int64_t write_done_ns)
{
    if (!s || s->dumped || !new_level ||
        s->phase != SCOPE_IRQ_LAT_WAIT_INTX) {
        return;
    }

    if (old_level) {
        s->coalesced_count++;
        scope_irq_latency_reset_current(s);
        return;
    }
    if (!write_ok) {
        s->intx_write_error_count++;
        scope_irq_latency_reset_current(s);
        return;
    }

    s->current.intx_write_start_ns = write_start_ns;
    s->current.intx_write_done_ns = write_done_ns;
    s->current.marker_last_old_ns = write_start_ns;
    s->phase = SCOPE_IRQ_LAT_WAIT_MARKER;
    s->phase_start_ns = write_done_ns;
}

bool scope_irq_latency_poll(ScopeIrqLatencyState *s)
{
    ScopeIrqLatencySample *sample;
    uint32_t marker_seq;
    int64_t now_ns;

    if (!s || s->dumped) {
        return false;
    }
    if (!scope_irq_latency_refresh_marker_ready(s) ||
        s->phase == SCOPE_IRQ_LAT_IDLE) {
        return false;
    }

    now_ns = scope_irq_latency_now_ns();
    if (now_ns - s->phase_start_ns > SCOPE_IRQ_LATENCY_TIMEOUT_NS) {
        s->timeout_count++;
        marker_seq = scope_irq_latency_read_marker32(
            s, SCOPE_IRQ_LATENCY_MARKER_SEQ_OFFSET);
        s->observed_marker_seq = marker_seq;
        scope_irq_latency_reset_current(s);
        return false;
    }
    if (s->phase != SCOPE_IRQ_LAT_WAIT_MARKER) {
        return true;
    }

    marker_seq = scope_irq_latency_read_marker32(
        s, SCOPE_IRQ_LATENCY_MARKER_SEQ_OFFSET);
    now_ns = scope_irq_latency_now_ns();
    if (marker_seq == s->observed_marker_seq) {
        s->current.marker_last_old_ns = now_ns;
        return true;
    }
    if (marker_seq != s->observed_marker_seq + 1U) {
        s->marker_seq_jump_count++;
    }

    s->current.marker_seq = marker_seq;
    s->current.marker_first_new_ns = now_ns;
    s->observed_marker_seq = marker_seq;
    s->current.index = s->sample_count;
    sample = &s->samples[s->sample_count++];
    *sample = s->current;
    scope_irq_latency_reset_current(s);

    if (s->sample_count == s->target_samples) {
        scope_irq_latency_dump(s);
    }
    return false;
}

bool scope_irq_latency_sample_pending(const ScopeIrqLatencyState *s)
{
    return s && !s->dumped && s->phase != SCOPE_IRQ_LAT_IDLE;
}
