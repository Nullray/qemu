/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Minimal environment for the actual production PRP and CQ admission code.
 * run-scope-remote-nvme.sh extracts these functions, not a duplicate parser.
 * This test does not model FPGA cache coherence or the RDMA transport.
 */
#include <assert.h>
#include <endian.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hw/misc/scope_remote_proto.h"

#define SCOPE_NVME_DEFAULT_CTRL_PAGE_SIZE 4096U
#define SCOPE_MAX_NVME_QUEUES 4U
#define SCOPE_REMOTE_MAX_INFLIGHT 8U
#define NVME_CMD_FLAGS_PSDT(flags) (((flags) >> 6) & 3)
#define NVME_PSDT_PRP 0
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define le64_to_cpu(x) le64toh(x)
#define g_new(type, n) ((type *)malloc(sizeof(type) * (n)))
#define g_free free
#define scope_min_u64(a, b) MIN(a, b)
#define scope_div_round_up_u64(a, b) (((a) + (b) - 1) / (b))
#define MEM_BASE UINT64_C(0x82000000)
#define MEM_SIZE (64U * 1024 * 1024)

typedef struct NvmeCmd {
    uint8_t flags;
    struct { uint64_t prp1, prp2; } dptr;
} NvmeCmd;
typedef struct ScopeRemotePrpError {
    const char *reason;
    uint32_t entry;
    uint64_t address;
} ScopeRemotePrpError;
typedef struct ScopeCqState {
    bool valid;
    uint16_t depth, shadow_tail, last_guest_head;
} ScopeCqState;
typedef struct ScopeBackend {
    uint32_t ctrl_page_size;
    ScopeCqState cq[SCOPE_MAX_NVME_QUEUES];
    struct { bool valid; uint16_t cqid; } remote_req[SCOPE_REMOTE_MAX_INFLIGHT];
} ScopeBackend;
typedef struct ScopeProxyState {
    ScopeBackend *active;
} ScopeProxyState;

static uint8_t memory[MEM_SIZE];
static uint64_t fail_read;
static unsigned int read_calls;
static bool scope_guest_range_to_bar_offset(ScopeProxyState *s, uint64_t pa,
                                            uint64_t length, uint64_t *offset)
{
    (void)s;
    if (pa < MEM_BASE || pa - MEM_BASE >= MEM_SIZE ||
        length > MEM_SIZE - (pa - MEM_BASE)) {
        return false;
    }
    *offset = pa - MEM_BASE;
    return true;
}
static bool scope_guest_mem_read(ScopeProxyState *s, uint64_t pa,
                                 void *buf, uint32_t length)
{
    uint64_t offset;
    read_calls++;
    if (pa == fail_read ||
        !scope_guest_range_to_bar_offset(s, pa, length, &offset)) {
        return false;
    }
    /* A list read must never cross its controller page boundary. */
    assert((pa & (s->active->ctrl_page_size - 1)) + length <=
           s->active->ctrl_page_size);
    memcpy(buf, memory + offset, length);
    return true;
}

#include "scope-prp.inc"

static ScopeBackend backend = { .ctrl_page_size = 4096 };
static ScopeProxyState state = { .active = &backend };
static unsigned int cases;

static void put_entry(uint64_t pa, uint64_t value)
{
    uint64_t offset;
    assert(scope_guest_range_to_bar_offset(&state, pa, 8, &offset));
    value = htole64(value);
    memcpy(memory + offset, &value, 8);
}

static void check(uint64_t prp1, uint64_t prp2, uint64_t length,
                  uint8_t flags, const char *expected_error)
{
    NvmeCmd cmd = { .flags = flags,
                   .dptr = { htole64(prp1), htole64(prp2) } };
    struct scope_remote_nvme_submit submit = { 0 };
    ScopeRemotePrpError error = { 0 };
    bool ok = scope_remote_collect_prps(&state, &cmd, length, &submit, &error);
    cases++;
    if (expected_error) {
        assert(!ok && error.reason);
        if (strcmp(error.reason, expected_error)) {
            fprintf(stderr, "expected %s, got %s\n", expected_error, error.reason);
            abort();
        }
    } else {
        uint64_t total = 0;
        if (!ok) {
            fprintf(stderr, "unexpected %s at %#lx\n", error.reason, error.address);
            abort();
        }
        for (uint32_t i = 0; i < submit.segment_count; i++) {
            total += submit.segments[i].length;
        }
        assert(total == length);
    }
}

static void test_original_failure(void)
{
    struct scope_remote_nvme_submit submit = { 0 };
    ScopeRemotePrpError error = { 0 };
    NvmeCmd cmd = { .dptr = { htole64(0x8297c000), htole64(0x84ea3100) } };
    for (unsigned int i = 0; i < 31; i++) {
        put_entry(0x84ea3100 + i * 8, 0x83000000 + i * 8192);
    }
    read_calls = 0;
    assert(scope_remote_collect_prps(&state, &cmd, 131072, &submit, &error));
    assert(submit.segment_count == 32 && read_calls == 1);
    assert(submit.segments[0].guest_pa == 0x8297c000);
    for (unsigned int i = 1; i < 32; i++) {
        assert(submit.segments[i].guest_pa == 0x83000000 + (i - 1) * 8192);
        assert(submit.segments[i].length == 4096);
    }
    cases++;
}

static void test_offsets_and_chains(void)
{
    const unsigned int sizes[] = { 2, 31, 511, 512, 513 };
    for (unsigned int offset = 0; offset < 4096; offset += 8) {
        for (unsigned int t = 0; t < sizeof(sizes) / sizeof(sizes[0]); t++) {
            unsigned int left = sizes[t], page = 0, index = 0;
            uint64_t list = 0x84000000 + offset;
            uint64_t first = list;
            while (left) {
                unsigned int room = (4096 - (list & 4095)) / 8;
                bool chain = left > room;
                unsigned int data = chain ? room - 1 : left;
                for (unsigned int i = 0; i < data; i++) {
                    put_entry(list + i * 8, 0x83000000 + index++ * 4096);
                }
                left -= data;
                if (chain) {
                    uint64_t next = 0x84100000 + page++ * 4096;
                    put_entry(list + data * 8, next);
                    list = next;
                }
            }
            check(0x8297c000, first, (sizes[t] + 1) * 4096, 0, NULL);
        }
    }
}

static void test_invalid_and_short(void)
{
    check(0, 0, 0, 0, NULL);
    check(0x8297c004, 0, 1024, 0, NULL);
    check(0x8297cffc, 0x83000000, 8, 0, NULL);
    check(0x8297c000, 0x83000000, 8192, 0, NULL);
    check(0, 0, 4096, 0, "prp1-zero");
    check(UINT64_MAX - 8, 0, 16, 0, "prp1-range");
    check(0x8297c000, 0, 8192, 0, "prp2-zero");
    check(0x8297c000, 0x83000100, 8192, 0, "prp2-data-unaligned");
    check(0x8297c000, 0x84ea3104, 12288, 0, "prp-list-unaligned");
    check(0x8297c000, 0, 4096, 0x40, "unsupported-psdt");
    put_entry(0x84000000, 0);
    check(0x8297c000, 0x84000000, 12288, 0, "prp-entry-zero");
    put_entry(0x84000000, 0x83000008);
    check(0x8297c000, 0x84000000, 12288, 0, "prp-entry-unaligned");
    put_entry(0x84000ff8, 0);
    check(0x8297c000, 0x84000ff8, 12288, 0, "prp-chain-zero");
    put_entry(0x84000ff8, 0x84100008);
    check(0x8297c000, 0x84000ff8, 12288, 0, "prp-chain-unaligned");
    fail_read = 0x84000000;
    check(0x8297c000, fail_read, 12288, 0, "prp-list-read");
    fail_read = 0;
    for (unsigned int i = 0; i < 64; i++) {
        put_entry(0x84000000 + i * 8, 0x83000000 + i * 8192);
    }
    check(0x8297c000, 0x84000000, 64 * 4096, 0, NULL);
    check(0x8297c000, 0x84000000, 65 * 4096, 0, "prp-entry-range-or-limit");
    backend.ctrl_page_size = 6144;
    check(0x8297c000, 0, 4096, 0, "invalid-page-size");
    backend.ctrl_page_size = 8192;
    put_entry(0x84ea3100, 0x83000000);
    put_entry(0x84ea3108, 0x83004000);
    check(0x8297c000, 0x84ea3100, 3 * 8192, 0, NULL);
    backend.ctrl_page_size = 4096;
}

static void test_cq_backpressure(void)
{
    ScopeCqState *cq = &backend.cq[1];
    assert(!scope_remote_cq_has_room(&backend, 1));
    assert(!scope_remote_cq_has_room(&backend, SCOPE_MAX_NVME_QUEUES));
    *cq = (ScopeCqState){ .valid = true, .depth = 4 };
    assert(scope_remote_cq_has_room(&backend, 1));
    cq->shadow_tail = 2;
    backend.remote_req[0].valid = true;
    backend.remote_req[0].cqid = 1;
    assert(!scope_remote_cq_has_room(&backend, 1));
    backend.remote_req[0].cqid = 0;
    assert(scope_remote_cq_has_room(&backend, 1));
    cq->shadow_tail = 3;
    assert(!scope_remote_cq_has_room(&backend, 1));
    cq->last_guest_head = 1;
    assert(scope_remote_cq_has_room(&backend, 1));
    cq->last_guest_head = 2;
    cq->shadow_tail = 1;
    assert(!scope_remote_cq_has_room(&backend, 1));
    cq->last_guest_head = 3;
    assert(scope_remote_cq_has_room(&backend, 1));
    cases++;
}

int main(void)
{
    test_original_failure();
    test_offsets_and_chains();
    test_invalid_and_short();
    test_cq_backpressure();
    printf("PASS: %u PRP/CQ cases (including the logged 128 KiB request)\n", cases);
    return 0;
}
