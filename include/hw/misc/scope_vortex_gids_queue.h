/* SPDX-License-Identifier: Apache-2.0 */
#ifndef HW_MISC_SCOPE_VORTEX_GIDS_QUEUE_H
#define HW_MISC_SCOPE_VORTEX_GIDS_QUEUE_H

#include "hw/misc/scope_vortex_gids_abi.h"

#include <stdbool.h>

struct scope_gids_queue_layout {
    uint64_t header_offset;
    uint64_t sq_offset;
    uint64_t cq_offset;
    uint64_t total_size;
};

static inline bool scope_gids_depth_valid(uint32_t depth)
{
    return depth >= SCOPE_GIDS_MIN_DEPTH &&
           depth <= SCOPE_GIDS_MAX_DEPTH &&
           (depth & (depth - 1U)) == 0;
}

static inline uint32_t scope_gids_counter_distance(uint32_t producer,
                                                   uint32_t consumer)
{
    return producer - consumer;
}

static inline bool scope_gids_ring_valid(uint32_t producer,
                                         uint32_t consumer,
                                         uint32_t depth)
{
    return scope_gids_depth_valid(depth) &&
           scope_gids_counter_distance(producer, consumer) <= depth;
}

static inline bool scope_gids_ring_empty(uint32_t producer, uint32_t consumer)
{
    return producer == consumer;
}

static inline bool scope_gids_ring_full(uint32_t producer, uint32_t consumer,
                                        uint32_t depth)
{
    return scope_gids_depth_valid(depth) &&
           scope_gids_counter_distance(producer, consumer) == depth;
}

static inline uint32_t scope_gids_slot(uint32_t counter, uint32_t depth)
{
    return counter & (depth - 1U);
}

/* Sequence zero is valid when the monotonic counter wraps. */
static inline uint32_t scope_gids_ready_sequence(uint32_t counter)
{
    return counter + 1U;
}

static inline bool scope_gids_entry_ready(uint32_t ready_sequence,
                                          uint32_t counter)
{
    return ready_sequence == scope_gids_ready_sequence(counter);
}

static inline bool scope_gids_range_valid(uint64_t offset, uint64_t length,
                                          uint64_t total)
{
    return length != 0 && offset <= total && length <= total - offset;
}

static inline bool scope_gids_queue_layout_init(
    uint32_t depth, struct scope_gids_queue_layout *layout)
{
    uint64_t entries;

    if (!layout || !scope_gids_depth_valid(depth)) {
        return false;
    }
    entries = (uint64_t)depth * SCOPE_GIDS_ENTRY_SIZE;
    layout->header_offset = 0;
    layout->sq_offset = SCOPE_GIDS_QUEUE_ALIGN;
    layout->cq_offset = layout->sq_offset + entries;
    layout->cq_offset = (layout->cq_offset + SCOPE_GIDS_QUEUE_ALIGN - 1U) &
                        ~(uint64_t)(SCOPE_GIDS_QUEUE_ALIGN - 1U);
    layout->total_size = layout->cq_offset + entries;
    layout->total_size = (layout->total_size + SCOPE_GIDS_QUEUE_ALIGN - 1U) &
                         ~(uint64_t)(SCOPE_GIDS_QUEUE_ALIGN - 1U);
    return true;
}

static inline bool scope_gids_header_valid(
    const struct scope_gids_queue_header *header)
{
    if (!header || header->magic != SCOPE_GIDS_ABI_MAGIC ||
        header->abi_major != SCOPE_GIDS_ABI_MAJOR ||
        header->header_size != sizeof(*header) ||
        header->sqe_size != sizeof(struct scope_gids_sqe) ||
        header->cqe_size != sizeof(struct scope_gids_cqe) ||
        header->reserved0 || header->reserved1 ||
        !scope_gids_depth_valid(header->depth)) {
        return false;
    }
    return scope_gids_ring_valid(header->sq_producer, header->sq_consumer,
                                 header->depth) &&
           scope_gids_ring_valid(header->cq_producer, header->cq_consumer,
                                 header->depth);
}

static inline bool scope_gids_sqe_reserved_zero(
    const struct scope_gids_sqe *sqe)
{
    return sqe && !sqe->flags && !sqe->reserved0 && !sqe->reserved1;
}

#endif
