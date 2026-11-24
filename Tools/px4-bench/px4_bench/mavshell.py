"""NSH over MAVLink SERIAL_CONTROL (same protocol as Tools/mavlink_shell.py)."""

import re
import threading
import time
from datetime import datetime
from pathlib import Path

import serial.tools.list_ports

from .console import Shell, StreamBuffer

SERIAL_CONTROL_DEV_SHELL = 10


def find_mavlink_port(port_match: str, exclude: str = "") -> str | None:
    exclude_real = str(Path(exclude).resolve()) if exclude else ""
    for p in serial.tools.list_ports.comports():
        if exclude_real and str(Path(p.device).resolve()) == exclude_real:
            continue
        if re.search(port_match, f"{p.description} {p.manufacturer or ''} {p.product or ''}"):
            return p.device
    return None


class MavlinkShell(Shell):
    name = "mavlink"

    def __init__(self, port: str, baud: int, prompt: str, log_dir: Path, heartbeat_timeout: float = 5.0):
        try:
            from pymavlink import mavutil
        except ImportError as e:
            raise RuntimeError("pymavlink is not installed; MAVLink transport unavailable") from e
        self._mavutil = mavutil
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        super().__init__(prompt, StreamBuffer(log_dir / f"mavlink-shell-{stamp}.log"))
        self.port = port
        self.mav = mavutil.mavlink_connection(port, baud=baud, autoreconnect=True, source_system=254)
        self._heartbeat()
        if self.mav.wait_heartbeat(timeout=heartbeat_timeout) is None:
            self.mav.close()
            raise RuntimeError(f"no MAVLink heartbeat on {port} within {heartbeat_timeout}s")
        self._tx_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="mavlink-shell-reader", daemon=True)
        self._thread.start()

    def _heartbeat(self):
        m = self._mavutil.mavlink
        self.mav.mav.heartbeat_send(m.MAV_TYPE_GENERIC, m.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def _run(self):
        last_hb = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            if now - last_hb > 1.0:
                with self._tx_lock:
                    self._heartbeat()
                last_hb = now
            msg = self.mav.recv_match(condition="SERIAL_CONTROL.count!=0", type="SERIAL_CONTROL",
                                      blocking=True, timeout=0.05)
            if msg is not None:
                self.buffer.append(bytes(msg.data[:msg.count]))

    def write(self, data: bytes):
        m = self._mavutil.mavlink
        flags = m.SERIAL_CONTROL_FLAG_EXCLUSIVE | m.SERIAL_CONTROL_FLAG_RESPOND
        with self._tx_lock:
            while data:
                chunk = data[:70]
                data = data[70:]
                payload = list(chunk) + [0] * (70 - len(chunk))
                self.mav.mav.serial_control_send(SERIAL_CONTROL_DEV_SHELL, flags, 0, 0, len(chunk), payload)

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2)
        try:
            with self._tx_lock:
                self.mav.mav.serial_control_send(SERIAL_CONTROL_DEV_SHELL, 0, 0, 0, 0, [0] * 70)
            self.mav.close()
        except Exception:
            pass
        super().close()
