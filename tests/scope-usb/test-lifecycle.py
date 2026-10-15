#!/usr/bin/env python3
"""Real QEMU lifecycle regression using only temporary regular files.

No physical device or kernel module is opened. Tests ownership on allocation
failure, normal signal exit, and exclusion of a second proxy.
"""
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

QEMU = Path(sys.argv[1]).resolve()
SHIM = Path(sys.argv[2]).resolve()
FPGA, XHCI = "fffe:00:01.0", "fffe:00:02.0"
MBX, RING_OFF = 0x1000000, 0x100000000


def wait_for_main_loop(proc, path):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        assert proc.poll() is None
        with socket.socket(socket.AF_UNIX) as connection:
            connection.settimeout(5)
            try:
                connection.connect(str(path))
            except (FileNotFoundError, ConnectionRefusedError):
                time.sleep(0.001)
                continue
            with connection.makefile("rwb", buffering=0) as monitor:
                assert "QMP" in json.loads(monitor.readline())
                for execute in ("qmp_capabilities", "query-status"):
                    monitor.write((json.dumps({"execute": execute}) + "\n").encode())
                    while True:
                        response = json.loads(monitor.readline())
                        if "event" not in response:
                            assert "return" in response, response
                            break
            return
    raise AssertionError("QEMU main loop did not start")


def run_case(mode):
    with tempfile.TemporaryDirectory(prefix="scope-lifecycle-") as directory:
        root = Path(directory)
        for bdf in (FPGA, XHCI):
            (root / bdf).mkdir()
        (root / FPGA / "resource").write_text(
            "0 0 0\n0 0 0\n800000000 9ffffffff 200\n0 0 0\n0 0 0\n0 0 0\n")
        (root / XHCI / "resource").write_text(
            "10000000 1000ffff 200\n0 0 0\n0 0 0\n0 0 0\n0 0 0\n0 0 0\n")
        config = bytearray(256)
        struct.pack_into("<HH", config, 0, 0x1022, 0x43f7)
        config[8:12] = bytes([1, 0x30, 3, 0x0c])
        struct.pack_into("<H", config, 4, 0x406)
        (root / XHCI / "config").write_bytes(config)
        regs = bytearray(65536)
        for off, value in ((0, 0x1100040), (4, (2 << 24) | (8 << 8) | 32),
                           (0x10, 1), (0x14, 0x2000), (0x18, 0x1000), (0x44, 1)):
            struct.pack_into("<I", regs, off, value)
        (root / XHCI / "resource0").write_bytes(regs)
        (root / "backends.json").write_text(json.dumps({"version": 1, "devices": [
            {"type": "xhci", "real-host-bdf": XHCI}]}))
        for name, size in (("user", 0x1030000), ("ctrl", RING_OFF + 65536),
                           ("bypass", 0x200000000)):
            with (root / name).open("wb") as stream:
                stream.truncate(size)
        env = dict(os.environ, LD_PRELOAD=str(SHIM), SCOPE_TEST_SYSFS=directory,
                   SCOPE_TEST_CTRL=str(root / "ctrl"),
                   SCOPE_TEST_IOCTL_LOG=str(root / "ioctl.log"),
                   SCOPE_TEST_RING_MODE=mode)
        device = (f"scope-fpga-vswitch,bus=rp,backend-config={root / 'backends.json'},"
                  f"fpga-host-bdf={FPGA},xdma-user-dev={root / 'user'},"
                  f"xdma-ctrl-dev={root / 'ctrl'},xdma-bypass-dev={root / 'bypass'},"
                  "guest-ddr-base=0x80000000,guest-ddr-size=0x80000000,"
                  "allow-unsafe-xhci-p2p=on")
        command = [str(QEMU), "-machine", "q35", "-S", "-m", "128M",
                   "-display", "none", "-monitor", "none", "-serial", "none",
                   "-qmp", f"unix:{root / 'qmp'},server=on,wait=off",
                   "-device", "pcie-root-port,id=rp", "-device", device]
        with (root / "qemu.log").open("w+") as log:
            proc = subprocess.Popen(command, env=env, stdout=log, stderr=log)
            try:
                if mode in ("busy", "stale"):
                    proc.wait(timeout=6)
                    assert proc.returncode != 0
                    freed = (root / "ioctl.log").exists()
                    assert not freed, "failed allocation freed another session's ring"
                    if mode == "stale":
                        assert (root / XHCI / "config").read_bytes() == config
                        assert (root / XHCI / "resource0").read_bytes() == regs
                    return
                with (root / "user").open("r+b") as user_file, \
                        (root / "ctrl").open("r+b") as ctrl_file:
                    with mmap.mmap(user_file.fileno(), 0) as user, \
                            mmap.mmap(ctrl_file.fileno(), 65536, offset=RING_OFF) as ring:
                        def wait_for(predicate):
                            deadline = time.monotonic() + 5
                            while time.monotonic() < deadline:
                                assert proc.poll() is None
                                if predicate():
                                    return
                                time.sleep(0.001)
                            raise AssertionError("proxy response timed out")

                        wait_for(lambda: struct.unpack_from("<I", user, MBX + 0x30)[0] & 3 == 3)
                        # Guest accepts the DMA capability then enables bus mastering.
                        for seq, off, val in ((1, 0xbc, 0x444d4150), (2, 4, 6)):
                            struct.pack_into("<8I", ring, seq * 64, 0x5844504b, 1,
                                             0x0f470300, 32, seq, off, val, 0)
                            wait_for(lambda: struct.unpack_from("<I", user, MBX + 0x10)[0] == seq)
                        before = (root / XHCI / "config").read_bytes()
                        assert struct.unpack_from("<H", before, 4)[0] & 4
                        # Device realize precedes installation of QEMU's
                        # termination signal handlers. A QMP response proves
                        # initialization finished and the main loop is active.
                        wait_for_main_loop(proc, root / "qmp")
                        if mode == "concurrent":
                            with (root / "second.log").open("w+") as second_log:
                                second_command = list(command)
                                second_command[second_command.index("-qmp") + 1] = \
                                    f"unix:{root / 'second-qmp'},server=on,wait=off"
                                second = subprocess.Popen(second_command, env=env,
                                                          stdout=second_log, stderr=second_log)
                                try:
                                    second.wait(timeout=3)
                                    assert second.returncode != 0
                                    assert (root / XHCI / "config").read_bytes() == before
                                    assert not (root / "ioctl.log").exists()
                                finally:
                                    if second.poll() is None:
                                        second.kill()
                                        second.wait(timeout=3)
                        proc.send_signal(signal.SIGINT if mode == "sigint" else signal.SIGTERM)
                        proc.wait(timeout=6)
                        after = (root / XHCI / "config").read_bytes()
                        assert not struct.unpack_from("<H", after, 4)[0] & 4, "exit left physical DMA enabled"
                        assert (root / "ioctl.log").read_text() == "FREE\n", "ring not freed exactly once"
                        if mode == "restart":
                            proc = subprocess.Popen(command, env=env, stdout=log, stderr=log)
                            wait_for_main_loop(proc, root / "qmp")
                            proc.terminate()
                            proc.wait(timeout=6)
                            assert (root / "ioctl.log").read_text() == "FREE\nFREE\n"
            except Exception:
                log.flush()
                log.seek(0)
                print(log.read(), file=sys.stderr)
                raise
            finally:
                if proc.poll() is None:
                    proc.kill()
                    proc.wait(timeout=3)


cases = sys.argv[3:] or ("busy", "stale", "sigint", "sigterm", "concurrent", "restart")
for case in cases:
    run_case(case)
    print(f"PASS: lifecycle {case}")
