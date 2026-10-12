#!/usr/bin/env bash
set -euo pipefail
qemu_root=$(cd "$(dirname "$0")/../.." && pwd)
qemu_binary=${1:-$qemu_root/build-rdma-repo17-review/qemu-system-x86_64}
test_dir=$(mktemp -d)
trap 'rm -rf "$test_dir"' EXIT
cc -shared -fPIC -Wall -Wextra -Werror -I"$qemu_root" -I"$qemu_root/include" \
  "$qemu_root/tests/scope-usb/fake-xdma.c" -ldl -o "$test_dir/fake-xdma.so"
python3 "$qemu_root/tests/scope-usb/test-physical-xhci.py" \
  "$qemu_binary" "$test_dir/fake-xdma.so"
kernel_source=${SCOPE_GUEST_KERNEL:-$qemu_root/../work_farm/software/linux}
mkdir -p "$test_dir/linux"
touch "$test_dir/linux/dma-direct.h"
for enabled in 1 0; do
  cc -std=gnu11 -O2 -g -Wall -Wextra -Werror \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -DCONFIG_SCOPE_PCIE_P2P_DMA="$enabled" \
    -DSCOPE_DMA_HEADER="\"$kernel_source/drivers/pci/scope-p2p-dma.h\"" \
    -I"$test_dir" "$qemu_root/tests/scope-usb/test-dma-window.c" \
    -o "$test_dir/test-dma-window"
  "$test_dir/test-dma-window"
done
