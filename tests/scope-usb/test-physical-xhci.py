#!/usr/bin/env python3
"""Exercise production scope-fpga-vswitch against file-backed PCI/XDMA I/O.

Checks control-plane behavior, never claims physical USB DMA works.
"""
import json
import mmap
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import time

QEMU = Path(sys.argv[1]).resolve()
SHIM = Path(sys.argv[2]).resolve()
FPGA = "fffe:00:01.0"
XHCI = "fffe:00:02.0"
MBX = 0x1000000
ECAM = 0x1010000 + 3 * 4096
RING_OFF = 0x100000000


def put32(mem, off, value):
    struct.pack_into("<I", mem, off, value)


def get32(mem, off):
    return struct.unpack_from("<I", mem, off)[0]


def wait_until(predicate, proc, description):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if predicate():
            return
        if proc.poll() is not None:
            raise AssertionError(f"QEMU exited waiting for {description}")
        time.sleep(0.001)
    raise AssertionError(f"timed out waiting for {description}")


def run_case(mode):
    with tempfile.TemporaryDirectory(prefix="scope-xhci-test-") as tmp:
        root = Path(tmp)
        for bdf in (FPGA, XHCI):
            (root / bdf).mkdir()
        (root / FPGA / "resource").write_text(
            "0 0 0\n0 0 0\n800000000 9ffffffff 200\n0 0 0\n0 0 0\n0 0 0\n")
        (root / XHCI / "resource").write_text(
            "10000000 1000ffff 200\n0 0 0\n0 0 0\n0 0 0\n0 0 0\n0 0 0\n")
        config = bytearray(256)
        struct.pack_into("<HH", config, 0, 0x1022, 0x43f7)
        config[8:12] = bytes([1, 0x30, 3, 0x0c])
        config[0x60] = 0x31  # physical Serial Bus Release Number
        if mode == "wrong-class":
            config[11] = 1
        if mode == "msi":
            config[0x34] = 0x40
            config[0x40] = 5
            config[0x42] = 1
        (root / XHCI / "config").write_bytes(config)
        if mode == "bound":
            (root / XHCI / "driver").mkdir()
        if mode == "iommu":
            (root / XHCI / "iommu_group").mkdir()
            (root / XHCI / "iommu_group/type").write_text("DMA\n")
        regs = bytearray(65536)
        put32(regs, 0, 0x1100040)
        put32(regs, 4, (2 << 24) | (8 << 8) | 32)
        put32(regs, 0x10, 0 if mode == "dma32" else 1)
        put32(regs, 0x14, 0x2000)
        put32(regs, 0x18, 0x1000)
        put32(regs, 0x44, 1)
        if mode == "bad-runtime":
            put32(regs, 0x18, 0xffffffe0)
        if mode == "not-ready":
            put32(regs, 0x44, 1 | (1 << 11))
            put32(regs, 0x40, 5)
        if mode in ("legacy", "bios-owned"):
            put32(regs, 0x10, 1 | (0xc00 << 16))
            put32(regs, 0x3000, 1 | ((1 << 16) if mode == "bios-owned" else 0))
            put32(regs, 0x3004, 0x1111)
            put32(regs, 0x40, 5)
        (root / XHCI / "resource0").write_bytes(regs)
        (root / "backends.json").write_text(json.dumps({
            "version": 1, "devices": [{"type": "xhci", "real-host-bdf": XHCI}]
        }))
        for name, size in (("user", 0x1030000), ("ctrl", RING_OFF + 65536),
                           ("bypass", 0x200000000)):
            with (root / name).open("wb") as handle:
                handle.truncate(size)
        env = dict(os.environ, LD_PRELOAD=str(SHIM),
                   SCOPE_TEST_SYSFS=tmp, SCOPE_TEST_CTRL=str(root / "ctrl"))
        device = (
            f"scope-fpga-vswitch,id=scope,bus=rp,backend-config={root / 'backends.json'},"
            f"fpga-host-bdf={FPGA},xdma-user-dev={root / 'user'},"
            f"xdma-ctrl-dev={root / 'ctrl'},xdma-bypass-dev={root / 'bypass'},"
            "guest-ddr-base=0x80000000,guest-ddr-size=0x80000000"
        )
        if mode != "no-opt-in":
            device += ",allow-unsafe-xhci-p2p=on"
        logfile = (root / "qemu.log").open("w+")
        proc = subprocess.Popen(
            [str(QEMU), "-machine", "q35", "-S", "-m", "128M",
             "-display", "none", "-monitor", "none", "-serial", "none",
             "-device", "pcie-root-port,id=rp", "-device", device],
            env=env, stdout=logfile, stderr=subprocess.STDOUT)
        mappings = []
        files = []
        try:
            if mode not in ("normal", "legacy", "single-slot", "packet-ring"):
                proc.wait(timeout=6)
                assert proc.returncode != 0, mode
                logfile.seek(0)
                message = logfile.read()
                expected = {
                    "bound": "still bound", "iommu": "identity host IOMMU",
                    "wrong-class": "not a PCI xHCI", "msi": "MSI/MSI-X",
                    "dma32": "64-bit DMA", "no-opt-in": "allow-unsafe-xhci",
                    "bad-runtime": "invalid BAR layout", "not-ready": "not ready",
                    "bios-owned": "BIOS still owns",
                }[mode]
                assert expected in message, message
                if mode in ("bios-owned", "not-ready"):
                    physical = (root / XHCI / "resource0").read_bytes()
                    assert get32(physical, 0x40) == 5  # no USBCMD takeover
                return
            files = [(root / path).open("r+b") for path in
                     ("user", "ctrl", f"{XHCI}/resource0", f"{XHCI}/config")]
            user = mmap.mmap(files[0].fileno(), 0)
            ring = mmap.mmap(files[1].fileno(), 65536, offset=RING_OFF)
            bar = mmap.mmap(files[2].fileno(), 0)
            pci = mmap.mmap(files[3].fileno(), 0)
            mappings.extend((user, ring, bar, pci))
            wait_until(lambda: get32(user, MBX + 0x30) & 3 == 3,
                       proc, "proxy startup")
            if mode == "legacy":
                assert get32(bar, 0x3000) & (1 << 24)
                assert get32(bar, 0x3004) == 0xe0001100  # reserved bits preserved
                assert not get32(bar, 0x40) & 5
            assert struct.unpack_from("<HH", user, ECAM) == (0x1022, 0x43f7)
            assert get32(user, ECAM + 0xb4) == 0x53435045
            assert struct.unpack_from("<QQQ", user, ECAM + 0xc0) == (
                0x80000000, 0x980000000, 0x80000000)
            assert user[ECAM + 0x70] == 0x10  # PCIe, no MSI capability
            assert user[ECAM + 0x71] == 0xb0
            assert user[ECAM + 0x60] == 0x31
            seq = 1

            def packet(kind, offset, value=0, size=4, cookie=None, wait=True):
                nonlocal seq
                number = cookie if cookie is not None else seq
                seq += 1
                strobe = ((1 << size) - 1) << (offset & 7)
                if kind == 3:
                    strobe = 0
                if kind == 1:
                    strobe = 0xf  # config data uses a 32-bit lane
                flags = 0x300 | ((7 if kind == 1 else 0) << 16) | (size << 20)
                flags |= strobe << 24
                lane_value = value if kind == 1 else value << (8 * (offset & 7))
                old = get32(user, MBX + 0x3c)
                wire = struct.pack("<8I", 0x5844504b, kind, flags, 32, number,
                                   offset, lane_value & 0xffffffff, lane_value >> 32)
                # The old cyclic XDMA setup has one descriptor, so each TLAST
                # reuses slot zero. The fixed setup gives every packet a
                # separate 64-byte descriptor (C2H minimum buffer length).
                slot = (0 if mode == "single-slot" else
                        ((seq - 1) % 1024) * 64 if mode == "packet-ring" else
                        ((seq - 1) % 128) * 32)
                ring[slot + 8:slot + 32] = wire[8:]
                ring[slot:slot + 8] = wire[:8]
                if not wait:
                    return number
                if kind == 1:
                    wait_until(lambda: get32(user, MBX + 0x10) == number,
                               proc, "CFG acknowledgement")
                    return number
                if kind == 4:
                    return number
                wait_until(lambda: get32(user, MBX + 0x38) == number and
                           get32(user, MBX + 0x3c) != old,
                           proc, "BAR response")
                data = get32(user, MBX + 0x34) | (get32(user, MBX + 0x4c) << 32)
                return number, get32(user, MBX + 0x3c), data >> (8 * (offset & 7))

            packet(1, 0x10, 0x50000004)
            packet(1, 4, 6)
            assert not get32(pci, 4) & 4  # MASTER denied without guest DMA ready
            _, response, _ = packet(2, 0x40, 5)
            assert response & 3 == 2
            _, response, _ = packet(2, 0x40, 2)  # reset is permitted
            assert response & 3 == 0
            packet(1, 0xbc, 0x444d4150)
            assert get32(pci, 4) & 4
            _, response, hcs = packet(3, 4)
            assert response & 3 == 0 and (hcs >> 8) & 0x7ff == 1
            _, response, _ = packet(2, 0x58, 0x980001001, size=8)
            assert response & 3 == 0
            assert struct.unpack_from("<Q", bar, 0x58)[0] == 0x980001001
            # Linux xhci_write_64() uses two ordered dword stores, low then high.
            pointer = 0x9a0002001
            _, response, _ = packet(2, 0x58, pointer & 0xffffffff)
            assert response & 3 == 0
            _, response, _ = packet(2, 0x5c, pointer >> 32)
            assert response & 3 == 0
            assert struct.unpack_from("<Q", bar, 0x58)[0] == pointer
            _, response, low = packet(3, 0x58)
            assert response & 3 == 0 and low & 0xffffffff == pointer & 0xffffffff
            _, response, high = packet(3, 0x5c)
            assert response & 3 == 0 and high & 0xffffffff == pointer >> 32
            _, response, _ = packet(2, 0x40, 5)
            assert response & 3 == 0 and get32(bar, 0x40) == 5
            number, response, _ = packet(2, 0x2004, 0x101)
            assert response & 8 and get32(bar, 0x2004) == 0
            packet(4, 0x2004, cookie=number)
            wait_until(lambda: get32(bar, 0x2004) == 0x101, proc, "doorbell DONE")
            # Pending port events must not interrupt an unstarted controller,
            # including the transient RUN=0/HCHalted=0 state during stopping.
            packet(2, 0x40, 4)
            put32(bar, 0x44, 8)
            put32(bar, 0x1020, 3)

            def assert_no_intx(description):
                deadline = time.monotonic() + 0.03
                while time.monotonic() < deadline:
                    assert get32(user, MBX + 0x50) == 0, description
                    assert proc.poll() is None
                    time.sleep(0.001)

            assert_no_intx("RUN=0 with pending port event")
            put32(bar, 0x44, 9)
            packet(2, 0x40, 5)
            assert_no_intx("RUN=1 while physical HCHalted is still set")
            # Preserve genuine error reporting when hardware halts itself.
            packet(2, 0x40, 4)
            for error in (1 << 2, 1 << 12):
                put32(bar, 0x44, 9 | error)
                wait_until(lambda: get32(user, MBX + 0x50) == 1,
                           proc, "halted controller error interrupt")
                put32(bar, 0x44, 9)
                wait_until(lambda: get32(user, MBX + 0x50) == 0,
                           proc, "error interrupt cleared")
            # An inaccessible BAR reads as all ones, not a valid error IRQ.
            for register in (0x40, 0x44, 0x1020):
                put32(bar, 0x1020, 2)
                wait_until(lambda: get32(user, MBX + 0x50) == 0,
                           proc, "interrupter temporarily masked")
                put32(bar, 0x44, 8)
                packet(2, 0x40, 5)
                put32(bar, register, 0xffffffff)
                if register != 0x1020:
                    put32(bar, 0x1020, 3)
                assert_no_intx(f"inaccessible register {register:#x}")
            put32(bar, 0x1020, 2)
            put32(bar, 0x44, 8)
            packet(2, 0x40, 5)
            put32(bar, 0x1020, 3)
            wait_until(lambda: get32(user, MBX + 0x50) == 1, proc, "INTx assertion")
            packet(1, 4, 0x406)
            wait_until(lambda: get32(user, MBX + 0x50) == 0, proc, "INTx mask")
            packet(1, 4, 6)
            wait_until(lambda: get32(user, MBX + 0x50) == 1, proc, "INTx unmask")
            packet(2, 0x40, 4)
            wait_until(lambda: get32(user, MBX + 0x50) == 0,
                       proc, "stopping controller deasserts INTx")
            packet(2, 0x40, 5)
            wait_until(lambda: get32(user, MBX + 0x50) == 1,
                       proc, "restarting controller preserves pending event")
            # File-backed registers have no hardware W1C; simulate xHC clearing IP.
            put32(bar, 0x1020, 2)
            wait_until(lambda: get32(user, MBX + 0x50) == 0, proc, "INTx clear")
            packet(1, 0xb4, 0)  # firmware metadata is read-only
            assert get32(user, ECAM + 0xb4) == 0x53435045
            put32(bar, 0x44, 1)
            packet(1, 0xbc, 0)
            assert not get32(pci, 4) & 4 and not get32(bar, 0x40) & 1
            packet(1, 0xbc, 0x444d4150)
            assert get32(pci, 4) & 4
            number, response, _ = packet(2, 0x2008, 0x102)
            assert response & 8
            if mode in ("single-slot", "packet-ring"):
                # Pause only this file-backed QEMU, then publish the hardware
                # order DONE -> readback before its polling thread resumes.
                # This makes loss in the single-descriptor layout deterministic.
                proc.send_signal(signal.SIGSTOP)
                _, stopped = os.waitpid(proc.pid, os.WUNTRACED)
                assert os.WIFSTOPPED(stopped)
                try:
                    packet(4, 0x2008, cookie=number)
                    read_number = packet(3, 0x2008, wait=False)
                finally:
                    proc.send_signal(signal.SIGCONT)
                wait_until(lambda: get32(user, MBX + 0x38) == read_number,
                           proc, "back-to-back doorbell readback")
                if mode == "packet-ring":
                    wait_until(lambda: get32(bar, 0x2008) == 0x102,
                               proc, "retained DONE commits physical doorbell")
                    assert get32(pci, 4) & 4
                    return
            # Missing or overwritten DONE must stop DMA without ringing xHC.
            wait_until(lambda: not get32(pci, 4) & 4, proc, "timeout DMA disable")
            assert get32(bar, 0x2008) == 0
            logfile.seek(0)
            failure_log = logfile.read()
            assert "timed out waiting for BAR_WRITE_DONE" in failure_log
            assert "off=0x2008" in failure_log and "disabling physical DMA" in failure_log
            packet(1, 4, 6)
            assert not get32(pci, 4) & 4  # failure cannot be re-enabled by config
            _, response, hcs = packet(3, 4)
            assert response & 3 == 2 and hcs & 0xffffffff == 0xffffffff
            assert hcs & 0xff == 255  # guest's misleading MaxSlots diagnostic
            _, response, crcr = packet(3, 0x58, size=8)
            assert response & 3 == 2 and crcr == 0xffffffffffffffff
            assert get32(bar, 4) & 0xff == 32  # physical xHC is still accessible
        except Exception:
            logfile.flush()
            logfile.seek(0)
            print(logfile.read(), file=sys.stderr)
            raise
        finally:
            if proc.poll() is None:
                proc.terminate()
                proc.wait(timeout=6)
            for mapping in mappings:
                mapping.close()
            for handle in files:
                handle.close()
            logfile.close()


for case in ("normal", "legacy", "single-slot", "packet-ring", "bound", "iommu", "wrong-class", "msi",
             "dma32", "no-opt-in", "bad-runtime", "not-ready", "bios-owned"):
    run_case(case)
    print(f"PASS: {case}")
