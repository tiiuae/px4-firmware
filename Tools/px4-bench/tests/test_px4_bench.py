import os
import pty
import threading
import time
from pathlib import Path

import pytest

from px4_bench import firmware
from px4_bench.config import DEFAULT_PROMPT
from px4_bench.console import SerialConsole
from px4_bench.safety import check_command

REPO = Path(__file__).resolve().parents[3]

RISCV_DUMP = """\
[boot] some earlier output 0x80001000
riscv_exception: EXCEPTION: Load access fault. MCAUSE: 0000000000000005, EPC: 0000000080012345, MTVAL: 0000000000000000
riscv_exception: Segmentation fault in commander (PID 42: commander)
dump_assert_info: Assertion failed panic: at file: :0 task: commander process: commander 0xc0001234
up_dump_register: EPC: 0000000080012345
up_dump_register: SP: 00000000c0100000 FP: 0000000000000000 TP: 0000000000000000 RA: 00000000c0004321
stack_dump: 0xc0100000: c0004400 00000000 ffffffff 80020000
"""


class FakeNsh:
    """Minimal NSH emulation on a pty: echoes input, answers commands, prints the Saluki prompt."""

    PROMPT = b"saluki> \x1b[K"

    def __init__(self, responses):
        self.master, slave = pty.openpty()
        self.slave_path = os.ttyname(slave)
        self.responses = responses
        self.received = bytearray()
        self._stop = False
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        line = b""
        while not self._stop:
            try:
                data = os.read(self.master, 1024)
            except OSError:
                return
            self.received += data
            for ch in data:
                ch = bytes([ch])
                if ch == b"\n":
                    cmd = line.decode()
                    line = b""
                    os.write(self.master, b"\r\n")
                    reply = self.responses.get(cmd)
                    if reply is None and cmd:
                        reply = f"nsh: {cmd}: command not found"
                    if reply == "<hang>":
                        continue
                    if reply:
                        os.write(self.master, reply.replace("\n", "\r\n").encode() + b"\r\n")
                    os.write(self.master, self.PROMPT)
                elif ch == b"\x03":
                    os.write(self.master, b"\r\n" + self.PROMPT)
                else:
                    line += ch
                    os.write(self.master, ch)

    def close(self):
        self._stop = True
        os.close(self.master)


@pytest.fixture
def nsh(tmp_path):
    fake = FakeNsh({"ver git": "FW git-hash: 06fcf5aa0fe825426dcba80a7c317bd1719f781c", "top": "<hang>",
                    "listener sensor_accel": "TOPIC: sensor_accel\n    x: 0.1\n    y: 0.2"})
    console = SerialConsole(fake.slave_path, 115200, DEFAULT_PROMPT, tmp_path)
    yield fake, console
    console.close()
    fake.close()


def test_exec_returns_output_between_echo_and_prompt(nsh):
    fake, console = nsh
    r = console.exec("listener sensor_accel", timeout=3)
    assert not r["timed_out"]
    assert r["output"] == "TOPIC: sensor_accel\n    x: 0.1\n    y: 0.2"
    r = console.exec("ver git", timeout=3)
    assert r["output"] == "FW git-hash: 06fcf5aa0fe825426dcba80a7c317bd1719f781c"
    assert console.buffer.log_path.read_bytes().count(b"saluki> ") >= 4


def test_exec_timeout_interrupts_with_ctrl_c(nsh):
    fake, console = nsh
    r = console.exec("top", timeout=0.5)
    assert r["timed_out"]
    for _ in range(50):
        if b"\x03" in fake.received:
            break
        time.sleep(0.02)
    assert b"\x03" in fake.received
    assert console.exec("ver git", timeout=3)["output"].startswith("FW git-hash:")


@pytest.mark.parametrize("cmd, blocked, confirm", [
    ("listener sensor_accel", False, False),
    ("param show SYS_*", False, False),
    ("param set SYS_AUTOSTART 4001", False, True),
    ("ver all; param save", False, True),
    ("motor_test test -m 1 -p 10", False, True),
    ("echo 1 > /fs/microsd/x", False, True),
    ("reboot -b", True, False),
])
def test_safety(cmd, blocked, confirm):
    b, c = check_command(cmd)
    assert bool(b) == blocked
    assert bool(c) == confirm


def test_crash_extraction_and_addresses():
    text = "noise\n" * 100 + RISCV_DUMP + "saluki> \n"
    dump = firmware.extract_crash(text)
    assert dump.splitlines()[0].startswith("noise") and "stack_dump" in dump
    assert firmware.crash_tasks(dump) == ["commander"]
    registers, candidates = firmware.crash_addresses(dump, 100)
    assert ("EPC", "0000000080012345") in registers
    assert ("RA", "00000000c0004321") in registers
    assert "c0004400" in candidates and "80020000" in candidates
    assert "ffffffff" not in candidates and "0000000000000000" not in candidates
    assert firmware.extract_crash("all good\nsaluki> ") is None


def test_parse_addr2line():
    out = ("0x0000000080012345: px4_main at /px4-firmware/sources/src/foo.cpp:12\n"
           " (inlined by) run at /px4-firmware/sources/src/bar.cpp:34\n"
           "0x00000000c0004400: ?? ??:0\n")
    parsed = firmware.parse_addr2line(out)
    assert parsed["80012345"][1].startswith("(inlined by)")
    assert parsed["c0004400"][0].startswith("??")


def test_run_process_output_and_timeout():
    rc, out = firmware.run_process(["sh", "-c", "echo hello; echo err >&2"], timeout=10)
    assert rc == 0 and "hello" in out and "err" in out
    t0 = time.monotonic()
    rc, out = firmware.run_process(["sleep", "30"], timeout=0.5)
    assert rc is None and "timeout" in out and time.monotonic() - t0 < 10


def test_parse_ver_git_hash():
    assert firmware.parse_ver_git_hash("HW arch: SSRC_SALUKI_PI\nPX4 git-hash: 98ea2bb98ebd44de6e838c00a0a570f9b1d53110\n"
                                       "OS git-hash: af99d1f223c31419345f8ba02d2136864d190961") == \
        "98ea2bb98ebd44de6e838c00a0a570f9b1d53110"
    assert firmware.parse_ver_git_hash("FW git-hash: 06fcf5aa0fe8") == "06fcf5aa0fe8"
    assert firmware.parse_ver_git_hash("OS git-hash: af99d1f223c3") is None


def test_resolve_target():
    assert firmware.resolve_target(REPO, "saluki-pi") == "ssrc_saluki-pi_default"
    assert firmware.resolve_target(REPO, "saluki-v3_flat") == "ssrc_saluki-v3_flat"
    assert firmware.resolve_target(REPO, "ssrc_salukini_default") == "ssrc_salukini_default"
    with pytest.raises(ValueError):
        firmware.resolve_target(REPO, "no-such-board")
