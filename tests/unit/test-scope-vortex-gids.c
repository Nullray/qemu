/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "hw/misc/scope_vortex_gids_queue.h"

static void test_abi_layout(void)
{
    g_assert_cmpuint(sizeof(struct scope_gids_queue_header), ==, 64);
    g_assert_cmpuint(sizeof(struct scope_gids_sqe), ==, 64);
    g_assert_cmpuint(sizeof(struct scope_gids_cqe), ==, 64);
    g_assert_cmpuint(offsetof(struct scope_gids_sqe, user_token), ==, 56);
    g_assert_cmpuint(offsetof(struct scope_gids_cqe, generation), ==, 48);
}

static void test_layout(void)
{
    struct scope_gids_queue_layout layout;

    g_assert_false(scope_gids_queue_layout_init(0, &layout));
    g_assert_false(scope_gids_queue_layout_init(3, &layout));
    g_assert_false(scope_gids_queue_layout_init(2048, &layout));
    g_assert_true(scope_gids_queue_layout_init(64, &layout));
    g_assert_cmpuint(layout.header_offset, ==, 0);
    g_assert_cmpuint(layout.sq_offset, ==, 4096);
    g_assert_cmpuint(layout.cq_offset, ==, 8192);
    g_assert_cmpuint(layout.total_size, ==, 12288);
}

static void test_ring_basic(void)
{
    const uint32_t depth = 8;

    g_assert_true(scope_gids_ring_empty(10, 10));
    g_assert_true(scope_gids_ring_valid(18, 10, depth));
    g_assert_true(scope_gids_ring_full(18, 10, depth));
    g_assert_false(scope_gids_ring_valid(19, 10, depth));
    g_assert_cmpuint(scope_gids_slot(15, depth), ==, 7);
    g_assert_cmpuint(scope_gids_slot(16, depth), ==, 0);
}

static void test_counter_wrap(void)
{
    const uint32_t depth = 8;
    uint32_t consumer = UINT32_MAX - 3U;
    uint32_t producer = 2U;

    g_assert_cmpuint(scope_gids_counter_distance(producer, consumer), ==, 6);
    g_assert_true(scope_gids_ring_valid(producer, consumer, depth));
    g_assert_cmpuint(scope_gids_ready_sequence(UINT32_MAX), ==, 0);
    g_assert_true(scope_gids_entry_ready(0, UINT32_MAX));
    g_assert_false(scope_gids_entry_ready(1, UINT32_MAX));
}

static void test_range_validation(void)
{
    g_assert_true(scope_gids_range_valid(0, 4096, 8192));
    g_assert_true(scope_gids_range_valid(4096, 4096, 8192));
    g_assert_false(scope_gids_range_valid(8192, 1, 8192));
    g_assert_false(scope_gids_range_valid(0, 0, 8192));
    g_assert_false(scope_gids_range_valid(UINT64_MAX - 1, 4, UINT64_MAX));
}

static void test_header_validation(void)
{
    struct scope_gids_queue_header h = {
        .magic = SCOPE_GIDS_ABI_MAGIC,
        .abi_major = SCOPE_GIDS_ABI_MAJOR,
        .abi_minor = SCOPE_GIDS_ABI_MINOR,
        .header_size = sizeof(h),
        .sqe_size = sizeof(struct scope_gids_sqe),
        .cqe_size = sizeof(struct scope_gids_cqe),
        .depth = 64,
        .features = SCOPE_GIDS_FEATURE_READ |
                    SCOPE_GIDS_FEATURE_SINGLE_PRODUCER,
        .state = SCOPE_GIDS_QUEUE_READY,
        .generation = 7,
        .sq_producer = 3,
        .sq_consumer = 2,
    };

    g_assert_true(scope_gids_header_valid(&h));
    h.reserved1 = 1;
    g_assert_false(scope_gids_header_valid(&h));
    h.reserved1 = 0;
    h.sq_producer = 100;
    g_assert_false(scope_gids_header_valid(&h));
}

static void test_sqe_contract(void)
{
    struct scope_gids_sqe sqe = {
        .ready_sequence = 1,
        .logical_cid = 4,
        .opcode = SCOPE_GIDS_OP_READ,
        .nsid = 1,
        .slba = 1024,
        .block_count = 8,
        .buffer_handle = 3,
        .buffer_generation = 9,
        .buffer_offset = 4096,
        .byte_count = 4096,
        .user_token = 0x1234,
    };

    g_assert_true(scope_gids_sqe_reserved_zero(&sqe));
    sqe.flags = 1;
    g_assert_false(scope_gids_sqe_reserved_zero(&sqe));
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/scope/gids/abi-layout", test_abi_layout);
    g_test_add_func("/scope/gids/layout", test_layout);
    g_test_add_func("/scope/gids/ring-basic", test_ring_basic);
    g_test_add_func("/scope/gids/counter-wrap", test_counter_wrap);
    g_test_add_func("/scope/gids/range-validation", test_range_validation);
    g_test_add_func("/scope/gids/header-validation", test_header_validation);
    g_test_add_func("/scope/gids/sqe-contract", test_sqe_contract);
    return g_test_run();
}
