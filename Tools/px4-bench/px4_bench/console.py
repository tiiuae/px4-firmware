"""Byte-stream capture and NSH command execution over a UART console or MAVLink shell."""

import re
import threading
import time
from datetime import datetime
from pathlib import Path

import serial

ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[ -/]*[@-~]|\x1b[@-Z\\-_]")
CTRL_RE = re.compile(r"[\x00-\x08\x0b-\x1f\x7f]")

MAX_BUFFER = 16 * 1024 * 1024
TRIM_TO = 8 * 1024 * 1024


def clean_text(data: bytes) -> str:
    text = data.decode("utf-8", "replace")
    text = ANSI_RE.sub("", text)
    text = text.replace("\r\n", "\n").replace("\r", "\n")
    return CTRL_RE.sub("", text)


class StreamBuffer:
    """Thread-safe, offset-addressed capture of everything a transport receives."""

    def __init__(self, log_path: Path | None = None):
        self._buf = bytearray()
        self._base = 0
        self._cond = threading.Condition()
        self._log = open(log_path, "ab") if log_path else None
        self.log_path = log_path

    @property
    def offset(self) -> int:
        with self._cond:
            return self._base + len(self._buf)

    def append(self, data: bytes):
        with self._cond:
            self._buf += data
            if len(self._buf) > MAX_BUFFER:
                drop = len(self._buf) - TRIM_TO
                del self._buf[:drop]
                self._base += drop
            self._cond.notify_all()
        if self._log:
            self._log.write(data)
            self._log.flush()

    def raw_since(self, offset: int) -> bytes:
        with self._cond:
            start = max(offset - self._base, 0)
            return bytes(self._buf[start:])

    def text_since(self, offset: int) -> str:
        return clean_text(self.raw_since(offset))

    def first_offset(self) -> int:
        with self._cond:
            return self._base

    def wait(self, timeout: float):
        with self._cond:
            self._cond.wait(timeout)

    def wait_for(self, pattern: re.Pattern, offset: int, timeout: float) -> re.Match | None:
        deadline = time.monotonic() + timeout
        while True:
            m = pattern.search(self.text_since(offset))
            if m:
                return m
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self.wait(min(remaining, 0.2))

    def close(self):
        if self._log:
            self._log.close()
            self._log = None


class Shell:
    """Base for transports that run an NSH session. Subclasses provide write() and feed self.buffer."""

    name = "shell"

    def __init__(self, prompt: str, buffer: StreamBuffer):
        self.prompt_re = re.compile(prompt)
        self.buffer = buffer
        self._exec_lock = threading.Lock()

    def write(self, data: bytes):
        raise NotImplementedError

    def close(self):
        self.buffer.close()

    def exec(self, command: str, timeout: float = 10.0) -> dict:
        """Run one command and return its output once the prompt is printed again."""
        with self._exec_lock:
            # Wake the shell and sync on a fresh prompt so that stale output is not mixed in.
            start = self.buffer.offset
            self.write(b"\n")
            got_prompt = self.buffer.wait_for(self.prompt_re, start, min(timeout, 3.0)) is not None

            start = self.buffer.offset
            self.write(command.encode() + b"\n")
            t0 = time.monotonic()
            deadline = t0 + timeout
            while True:
                text = self.buffer.text_since(start)
                echo = text.find(command)
                body = text[echo + len(command):] if echo >= 0 else text
                m = self.prompt_re.search(body)
                if m:
                    return {
                        "command": command,
                        "output": body[:m.start()].strip("\n"),
                        "timed_out": False,
                        "duration_s": round(time.monotonic() - t0, 2),
                        "transport": self.name,
                    }
                if time.monotonic() >= deadline:
                    # Interrupt commands that never return (e.g. `top` without `once`).
                    self.write(b"\x03")
                    note = "" if got_prompt else " No shell prompt seen before the command; is the board at NSH?"
                    return {
                        "command": command,
                        "output": body.strip("\n"),
                        "timed_out": True,
                        "duration_s": round(time.monotonic() - t0, 2),
                        "transport": self.name,
                        "note": f"Prompt not seen within {timeout}s; sent Ctrl-C.{note}",
                    }
                self.buffer.wait(min(deadline - time.monotonic(), 0.2))


class SerialConsole(Shell):
    """Raw NuttX UART console. Captures continuously, including boot output and fault dumps."""

    name = "uart"

    def __init__(self, port: str, baud: int, prompt: str, log_dir: Path):
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        super().__init__(prompt, StreamBuffer(log_dir / f"console-{stamp}.log"))
        self.port = port
        self.baud = baud
        self.error = ""
        self._ser = serial.Serial(port, baud, timeout=0.1)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="console-reader", daemon=True)
        self._thread.start()

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def _run(self):
        while not self._stop.is_set():
            try:
                if not self.is_open:
                    self._ser.open()
                    self.error = ""
                data = self._ser.read(self._ser.in_waiting or 1)
                if data:
                    self.buffer.append(data)
            except (serial.SerialException, OSError) as e:
                # USB-UART adapters may disappear when the board is power-cycled; keep retrying.
                self.error = str(e)
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._stop.wait(0.5)

    def write(self, data: bytes):
        self._ser.write(data)
        self._ser.flush()

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2)
        try:
            self._ser.close()
        finally:
            super().close()
