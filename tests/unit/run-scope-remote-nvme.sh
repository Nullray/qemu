#!/usr/bin/env bash
# Compile the production functions with mock guest memory and DMA/RDMA I/O.
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
source_file=${SCOPE_VSWITCH_SOURCE:-$root/hw/misc/scope_fpga_vswitch.c}
sed -n '/^static bool scope_remote_add_segment(/,/^static uint32_t scope_remote_admin_data_len(/p' \
    "$source_file" | sed '$d' > "$tmp/scope-prp.inc"
sed -n '/^static bool scope_remote_cq_has_room(/,/^static ScopeSqeReadStatus scope_process_remote_sq_entries(/p' \
    "$source_file" | sed '$d' >> "$tmp/scope-prp.inc"
grep -q '^static bool scope_remote_collect_prps(' "$tmp/scope-prp.inc"
grep -q '^static bool scope_remote_cq_has_room(' "$tmp/scope-prp.inc"
"${CC:-cc}" -std=gnu11 -O2 -g -Wall -Wextra -Werror \
    ${SANITIZER_FLAGS:--fsanitize=address,undefined -fno-omit-frame-pointer} \
    -I"$root/include" -I"$tmp" \
    "$root/tests/unit/test-scope-remote-nvme.c" -o "$tmp/test-prp"
"$tmp/test-prp"
