#!/usr/bin/env python3
"""Exercise production scope-fpga-vswitch against file-backed PCI/XDMA I/O.

Checks control-plane behavior, never claims physical USB DMA works.
"""
import ctypes
import json
import mmap
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
import threading

QEMU = Path(sys.argv[1]).resolve()
SHIM = Path(sys.argv[2]).resolve()
ATOMIC = ctypes.CDLL(str(SHIM))
ATOMIC.scope_test_store32.argtypes = (ctypes.c_void_p, ctypes.c_uint32)
ATOMIC.scope_test_store16.argtypes = (ctypes.c_void_p, ctypes.c_uint16)
FPGA = "fffe:00:01.0"
XHCI = "fffe:00:02.0"
TRANSPORT = sys.argv[3] if len(sys.argv) > 3 else "xdma-dma32"
BAR_FIFO = TRANSPORT == "bar-fifo"
BRIDGE = TRANSPORT == "qdma-bridge"
QDMA = TRANSPORT in ("qdma-cmpt", "qdma-bridge")
MBX = 0x200000 if BAR_FIFO or QDMA else 0x1000000
ECAM_BASE = 0x220000 if BAR_FIFO or QDMA else 0x1010000
ECAM = ECAM_BASE + 3 * 4096
FIFO = 0x240000
RING_OFF = 0x100000000


def put32(mem, off, value):
    ATOMIC.scope_test_store32(ctypes.addressof(ctypes.c_char.from_buffer(mem, off)), value)


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
        devices = [{"type": "xhci", "real-host-bdf": XHCI}]
        tap_pair = None
        if mode == "with-nic":
            devices.append({"type": "host-net", "tap-name": "scope-test"})
            tap_pair = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        (root / "backends.json").write_text(json.dumps({"version": 1, "devices": devices}))
        for name, size in (("user", 0x1030000), ("ctrl", RING_OFF + 65536),
                           ("bypass", 0x200000000)):
            with (root / name).open("wb") as handle:
                handle.truncate(size)
        if BAR_FIFO or QDMA:
            with (root / "user").open("r+b") as handle:
                handle.seek(FIFO)
                handle.write(struct.pack("<6I", 0x53434231 if BRIDGE else 0x53435131 if QDMA else 0x53465131,
                                         1 if QDMA else 32, 0, 1, 0, 1))
                if BRIDGE:
                    handle.seek(FIFO + 28)
                    handle.write(struct.pack("<I", 3))
        env = dict(os.environ, LD_PRELOAD=str(SHIM),
                   SCOPE_TEST_SYSFS=tmp, SCOPE_TEST_CTRL=str(root / "ctrl"))
        if tap_pair:
            env["SCOPE_TEST_TAP_FD"] = str(tap_pair[1].fileno())
        if QDMA:
            (root / "notify").write_bytes(bytes(4096))
            env["SCOPE_TEST_QDMA"] = str(root / "notify")
        device = (
            f"scope-fpga-vswitch,id=scope,bus=rp,backend-config={root / 'backends.json'},"
            f"fpga-host-bdf={FPGA},xdma-user-dev={root / 'user'},"
            f"xdma-ctrl-dev={root / 'ctrl'},xdma-bypass-dev={root / 'bypass'},"
            "guest-ddr-base=0x80000000,guest-ddr-size=0x80000000"
        )
        if BAR_FIFO or QDMA:
            device += (f",control-transport={TRANSPORT},mailbox-offset={MBX},"
                       f"ecam-shadow-offset={ECAM_BASE},fifo-offset={FIFO}")
        if QDMA:
            device += f",qdma-notify-dev={root / 'notify'}"
            if BRIDGE:
                device += ",qdma-ring-entries=8"
        if mode != "no-opt-in":
            device += ",allow-unsafe-xhci-p2p=on"
        logfile = (root / "qemu.log").open("w+")
        proc = subprocess.Popen(
            [str(QEMU), "-L", str(Path(__file__).resolve().parents[2] / "pc-bios"),
             "-machine", "q35", "-S", "-m", "128M",
             "-display", "none", "-monitor", "none", "-serial", "none",
             "-device", "pcie-root-port,id=rp", "-device", device],
            env=env, stdout=logfile, stderr=subprocess.STDOUT,
            pass_fds=(tap_pair[1].fileno(),) if tap_pair else ())
        mappings = []
        stopped = threading.Event()
        hardware = None
        files = []
        produced = 0
        try:
            if mode not in ("normal", "legacy", "single-slot", "packet-ring",
                            "fifo-malformed", "fifo-pop-stuck", "ordered-done",
                            "cmpt-malformed", "bridge-producer-invalid", "with-nic"):
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
            if QDMA:
                files.append((root / "notify").open("r+b"))
                ring = mmap.mmap(files[-1].fileno(), 4096)
            else:
                ring = None if BAR_FIFO else mmap.mmap(files[1].fileno(), 65536,
                                                       offset=RING_OFF)
            bar = mmap.mmap(files[2].fileno(), 0)
            pci = mmap.mmap(files[3].fileno(), 0)
            mappings.extend((user, bar, pci))
            if ring is not None:
                mappings.append(ring)
            if BAR_FIFO:
                def emulate_fifo():
                    while not stopped.wait(0.00005):
                        ticket = get32(user, FIFO + 12)
                        if get32(user, FIFO + 8) and get32(user, FIFO + 16) == ticket:
                            put32(user, FIFO + 8, 0)
                            put32(user, FIFO + 12, ticket + 1)
                hardware = threading.Thread(target=emulate_fifo)
                hardware.start()
            wait_until(lambda: get32(user, MBX + 0x30) & 3 == 3,
                       proc, "proxy startup")
            if QDMA:
                wait_until(lambda: get32(ring, 4084) == 1, proc, "QDMA ARM")
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

            def packet(kind, offset, value=0, size=4, cookie=None, wait=True, bdf=0x300):
                nonlocal seq, produced
                number = cookie if cookie is not None else seq
                seq += 1
                strobe = ((1 << size) - 1) << (offset & 7)
                if kind == 3:
                    strobe = 0
                if kind == 1:
                    strobe = 0xf  # config data uses a 32-bit lane
                flags = bdf | ((7 if kind == 1 else 0) << 16) | (size << 20)
                flags |= strobe << 24
                lane_value = value if kind == 1 else value << (8 * (offset & 7))
                old = get32(user, MBX + 0x3c)
                wire = struct.pack("<8I", 0x5844504b, kind, flags, 32, number,
                                   offset, lane_value & 0xffffffff, lane_value >> 32)
                # XDMA regression covers the old overwritten descriptor and
                # the packet-sized descriptor ring added upstream.
                slot = (0 if mode == "single-slot" else
                        ((seq - 1) % 1024) * 64 if mode == "packet-ring" else
                        ((seq - 1) % 128) * 32)
                if BRIDGE:
                    wait_until(lambda: produced - get32(ring, 4080) < 8,
                               proc, "bridge host ring credit")
                    slot = 64 + (produced % 8) * 64
                    ring[slot:slot + 32] = wire
                    produced += 1
                    put32(ring, 0, produced)
                elif QDMA:
                    wait_until(lambda: produced - get32(ring, 4080) < 7,
                               proc, "QDMA completion credit")
                    slot = (produced % 8) * 64
                    color = 1 ^ ((produced // 8) & 1)
                    ring[slot + 8:slot + 40] = wire
                    put32(ring, slot, color << 1)
                    produced += 1
                    put32(ring, 4088, produced)
                    # Publish DMA writeback PIDX after the entry is complete.
                    ATOMIC.scope_test_store16(ctypes.addressof(ctypes.c_char.from_buffer(ring, 8 * 64)), produced % 8)
                elif BAR_FIFO:
                    wait_until(lambda: get32(user, FIFO + 8) == 0, proc, "FIFO empty")
                    user[FIFO + 32:FIFO + 64] = wire
                    put32(user, FIFO + 8, 1)
                else:
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

            if mode == "with-nic":
                nic = ECAM_BASE + 5 * 4096  # compact shadow: DP1 slot 4, EP1 slot 5
                assert struct.unpack_from("<HH", user, nic) == (0x8086, 0x10fb)
                packet(1, 0x10, 0x50100000, bdf=0x400)
                packet(1, 4, 6, bdf=0x400)
                _, response, value = packet(3, 0x42a4, bdf=0x400)
                assert response & 3 == 0 and value & 0x40000000
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
            if mode in ("cmpt-malformed", "bridge-producer-invalid", "with-nic"):
                wait_until(lambda: produced == get32(ring, 4080), proc,
                           "QDMA ring drained before failure injection")
                if mode == "bridge-producer-invalid":
                    produced += 9
                    put32(ring, 0, produced)
                elif BRIDGE:
                    slot = 64 + (produced % 8) * 64
                    ring[slot:slot + 32] = bytes(32)
                    produced += 1
                    put32(ring, 0, produced)
                else:
                    slot = (produced % 8) * 64
                    color = 1 ^ ((produced // 8) & 1)
                    ring[slot + 8:slot + 40] = bytes(32)
                    put32(ring, slot, color << 1)
                    produced += 1
                    put32(ring, 4088, produced)
                    ATOMIC.scope_test_store16(ctypes.addressof(ctypes.c_char.from_buffer(ring, 8 * 64)), produced % 8)
                wait_until(lambda: not get32(pci, 4) & 4, proc,
                           "invalid QDMA notification disables physical USB DMA")
                assert get32(user, MBX + 0x50) == 0
                logfile.seek(0)
                failure_log = logfile.read()
                assert "control notification transport failed" in failure_log
                assert "disabling physical DMA" in failure_log
                return
            if mode in ("fifo-malformed", "fifo-pop-stuck"):
                wait_until(lambda: get32(user, FIFO + 8) == 0, proc, "FIFO empty")
                if mode == "fifo-malformed":
                    user[FIFO + 32:FIFO + 64] = bytes(32)
                    put32(user, FIFO + 8, 1)
                else:
                    stopped.set()
                    hardware.join()
                    packet(1, 4, 6)  # the mock hardware no longer commits POP
                wait_until(lambda: not get32(pci, 4) & 4, proc,
                           "FIFO failure disables physical USB DMA")
                assert get32(user, MBX + 0x50) == 0
                for handle in files:
                    handle.close()
                return
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
            if mode in ("single-slot", "packet-ring", "ordered-done"):
                # Pause only this file-backed QEMU, then publish the hardware
                # order DONE -> readback before its polling thread resumes.
                # This makes loss in the single-descriptor layout deterministic.
                proc.send_signal(signal.SIGSTOP)
                _, wait_status = os.waitpid(proc.pid, os.WUNTRACED)
                assert os.WIFSTOPPED(wait_status)
                try:
                    packet(4, 0x2008, cookie=number)
                    read_number = packet(3, 0x2008, wait=False)
                finally:
                    proc.send_signal(signal.SIGCONT)
                wait_until(lambda: get32(user, MBX + 0x38) == read_number,
                           proc, "back-to-back doorbell readback")
                if mode in ("packet-ring", "ordered-done"):
                    wait_until(lambda: get32(bar, 0x2008) == 0x102,
                               proc, "retained DONE commits physical doorbell")
                    assert get32(pci, 4) & 4
                    if QDMA:
                        # Repeated real BAR requests cross the eight-entry
                        # completion ring while checking color and credits.
                        for _ in range(128):
                            _, response, value = packet(3, 0x2008)
                            assert response & 3 == 0 and value == 0x102
                        wait_until(lambda: produced == get32(ring, 4080),
                                   proc, "QDMA completion ACK")
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
            stopped.set()
            if hardware:
                hardware.join()
            for mapping in mappings:
                mapping.close()
            for handle in files:
                handle.close()
            logfile.close()
            if tap_pair:
                for endpoint in tap_pair:
                    endpoint.close()


cases = ["normal", "legacy", "bound", "iommu", "wrong-class", "msi",
         "dma32", "no-opt-in", "bad-runtime", "not-ready", "bios-owned"]
if BAR_FIFO:
    cases.extend(["fifo-malformed", "fifo-pop-stuck"])
elif QDMA:
    cases.extend(["ordered-done", "cmpt-malformed"])
    if BRIDGE:
        cases.extend(["bridge-producer-invalid", "with-nic"])
else:
    cases.extend(["single-slot", "packet-ring"])
for case in cases:
    run_case(case)
    print(f"PASS: {TRANSPORT} xHCI {case}")
