"""MCP server exposing bench tools for PX4/NuttX flight controller bring-up."""

import functools
import re
import sys
import threading
import time
from pathlib import Path

import anyio
import serial.tools.list_ports

try:
    from mcp.server.mcpserver import MCPServer
except ImportError:  # mcp 1.x
    from mcp.server.fastmcp import FastMCP as MCPServer
from mcp.types import ToolAnnotations

from . import firmware
from .config import load_config
from .console import SerialConsole
from .mavshell import MavlinkShell, find_mavlink_port
from .safety import check_command

INSTRUCTIONS = """\
Bench tools for PX4/NuttX flight controllers (any boards/<vendor>/<board> target, e.g. ssrc saluki-*).
- The UART console captures everything continuously (boot log, fault dumps); read it with console_read.
- Run shell commands with nsh_exec / nsh_batch (UART console, or MAVLink shell over USB).
- Never use `top` without `once` or other never-ending commands; they are interrupted at timeout.
- Commands that change parameters, storage, vehicle state or actuators require confirm=true: ask the user first.
- Typical loop: build -> flash -> console_read/nsh_batch -> verify_firmware -> decode_crash on faults.
- See Tools/px4-bench/USE_CASES.md for workflows.
"""

CLIP = 30000


def _annotations(read_only=False, destructive=False, idempotent=False) -> ToolAnnotations:
    try:
        return ToolAnnotations(read_only_hint=read_only, destructive_hint=destructive, idempotent_hint=idempotent)
    except Exception:
        return ToolAnnotations(readOnlyHint=read_only, destructiveHint=destructive, idempotentHint=idempotent)


def _clip(text: str, limit: int = CLIP) -> str:
    if len(text) <= limit:
        return text
    half = limit // 2
    return f"{text[:half]}\n... [{len(text) - limit} chars clipped] ...\n{text[-half:]}"


BOOT_ISSUE_RE = re.compile(r"(?i)(\berror\b|\bfail(ed|ure)?\b|panic|assert|not found|timed? ?out|\binvalid\b|"
                           r"missing|unable|\bWARN)")


def boot_summary(text: str, max_issues: int = 60) -> dict:
    lines = text.splitlines()
    issues = [line for line in lines if BOOT_ISSUE_RE.search(line)]
    return {"lines": len(lines), "issue_lines": issues[:max_issues], "issue_count": len(issues),
            "crash_detected": firmware.extract_crash(text) is not None}


class Bench:
    def __init__(self):
        self.cfg = load_config()
        self.console: SerialConsole | None = None
        self.mav: MavlinkShell | None = None
        self._lock = threading.Lock()

    # ---- transports
    def open_console(self, port: str = "", baud: int = 0) -> SerialConsole:
        with self._lock:
            port = port or self.cfg.console_port
            if not port:
                raise RuntimeError("no console port: pass `port` or set [console] port in Tools/px4-bench/bench.toml "
                                   "(see list_serial_ports)")
            if self.console and self.console.port != port:
                self.console.close()
                self.console = None
            if not self.console:
                self.console = SerialConsole(port, baud or self.cfg.console_baud, self.cfg.prompt, self.cfg.log_dir)
            return self.console

    def get_console(self, required: bool = True) -> SerialConsole | None:
        if self.console:
            return self.console
        if self.cfg.console_port and self.cfg.console_autoopen:
            return self.open_console()
        if required:
            raise RuntimeError("UART console is not open: call console_open first")
        return None

    def get_mav(self) -> MavlinkShell:
        with self._lock:
            if not self.mav:
                exclude = self.console.port if self.console else self.cfg.console_port
                port = self.cfg.mavlink_port or find_mavlink_port(self.cfg.mavlink_port_match, exclude)
                if not port:
                    raise RuntimeError(f"no MAVLink USB port matching '{self.cfg.mavlink_port_match}' found")
                self.mav = MavlinkShell(port, self.cfg.mavlink_baud, self.cfg.prompt, self.cfg.log_dir)
            return self.mav

    def close_mav(self):
        with self._lock:
            if self.mav:
                self.mav.close()
                self.mav = None

    def shell(self, transport: str):
        if transport == "uart":
            return self.get_console()
        if transport == "mavlink":
            return self.get_mav()
        console = self.get_console(required=False)
        return console if console else self.get_mav()

    def guard(self, command: str, confirm: bool) -> str | None:
        blocked, needs_confirm = check_command(command, self.cfg.extra_confirm_patterns)
        if blocked:
            return "Blocked: " + "; ".join(blocked)
        if needs_confirm and not confirm:
            return ("Confirmation required: " + "; ".join(needs_confirm) +
                    ". Ask the user, then call again with confirm=true.")
        return None


bench: Bench | None = None


def get_bench() -> Bench:
    global bench
    if bench is None:
        bench = Bench()
    return bench


def _threaded(fn):
    """Run blocking tool bodies in a worker thread; report errors as tool results."""
    @functools.wraps(fn)
    async def wrapper(*args, **kwargs):
        def call():
            try:
                return fn(*args, **kwargs)
            except Exception as e:
                return {"status": "error", "message": f"{type(e).__name__}: {e}"}
        return await anyio.to_thread.run_sync(call)
    return wrapper


server = MCPServer("px4-bench", instructions=INSTRUCTIONS)


# --------------------------------------------------------------------------- status / discovery

@server.tool(annotations=_annotations(read_only=True))
@_threaded
def bench_status() -> dict:
    """Show bench configuration and transport state (console, MAVLink, uploader, build image)."""
    b = get_bench()
    c = b.cfg
    return {
        "repo": str(c.repo),
        "config_file": str(c.config_file) if c.config_file else None,
        "console": {"configured_port": c.console_port, "baud": c.console_baud, "prompt_regex": c.prompt,
                    "open": bool(b.console), "error": b.console.error if b.console else "",
                    "offset": b.console.buffer.offset if b.console else None,
                    "log_file": str(b.console.buffer.log_path) if b.console else None},
        "mavlink": {"configured_port": c.mavlink_port or f"auto ({c.mavlink_port_match})", "open": bool(b.mav)},
        "uploader": {"path": c.uploader, "exists": bool(c.uploader) and Path(c.uploader).is_file()},
        "docker_image": {"name": c.docker_image, "exists": firmware.docker_image_exists(c.docker_image)},
        "log_dir": str(c.log_dir),
        "boards": firmware.list_boards(c.repo),
    }


@server.tool(annotations=_annotations(read_only=True))
@_threaded
def list_serial_ports() -> list[dict]:
    """List serial ports with descriptions and stable /dev/serial/by-id names (use those in bench.toml)."""
    by_id = {}
    for link in Path("/dev/serial/by-id").glob("*"):
        by_id[str(link.resolve())] = str(link)
    return [{"device": p.device, "by_id": by_id.get(str(Path(p.device).resolve())), "description": p.description,
             "hwid": p.hwid} for p in serial.tools.list_ports.comports()]


@server.tool(annotations=_annotations(read_only=True))
@_threaded
def board_info(target: str) -> dict:
    """Describe a build target: arch, toolchain, kernel/flat build, artifact paths and git hash of the local build.

    target: 'saluki-pi', 'saluki-pi_flat', 'ssrc_saluki-pi_default' or any '<vendor>_<board>_<label>'."""
    return firmware.target_info(get_bench().cfg, target).to_dict()


# --------------------------------------------------------------------------- console

@server.tool(annotations=_annotations())
@_threaded
def console_open(port: str = "", baud: int = 0) -> dict:
    """Open the UART debug console (defaults from bench.toml) and start continuous capture."""
    c = get_bench().open_console(port, baud)
    return {"status": "ok", "port": c.port, "baud": c.baud, "offset": c.buffer.offset,
            "log_file": str(c.buffer.log_path)}


@server.tool(annotations=_annotations())
@_threaded
def console_close() -> dict:
    """Close the UART console (frees the port for other tools such as minicom)."""
    b = get_bench()
    if b.console:
        b.console.close()
        b.console = None
    return {"status": "ok"}


@server.tool(annotations=_annotations(read_only=True))
@_threaded
def console_read(since: int | None = None, last_lines: int = 200, pattern: str = "", context: int = 2,
                 max_chars: int = 20000) -> dict:
    """Read captured UART console output.

    since: byte offset returned earlier as `next_offset` (or by flash/reboot); omit to read the last `last_lines`.
    pattern: optional regex; only matching lines (with `context` lines around) are returned.
    Keep `next_offset` and pass it as `since` to read only new output next time."""
    c = get_bench().get_console()
    start = c.buffer.first_offset() if since is None else since
    text = c.buffer.text_since(start)
    lines = text.splitlines()
    if since is None:
        lines = lines[-last_lines:]
    if pattern:
        regex = re.compile(pattern)
        keep = set()
        for i, line in enumerate(lines):
            if regex.search(line):
                keep.update(range(max(i - context, 0), min(i + context + 1, len(lines))))
        lines = [lines[i] for i in sorted(keep)]
    out = "\n".join(lines)
    truncated = len(out) > max_chars
    return {"text": out[-max_chars:] if truncated else out, "truncated": truncated, "next_offset": c.buffer.offset,
            "console_error": c.error}


@server.tool(annotations=_annotations(destructive=True))
@_threaded
def console_write(text: str = "", newline: bool = True, ctrl_c: bool = False, confirm: bool = False) -> dict:
    """Send raw text to the UART console without waiting for a prompt (e.g. to answer a bootloader prompt).

    Prefer nsh_exec for shell commands. ctrl_c sends Ctrl-C first."""
    b = get_bench()
    if text:
        msg = b.guard(text, confirm)
        if msg:
            return {"status": "rejected", "message": msg}
    c = b.get_console()
    offset = c.buffer.offset
    if ctrl_c:
        c.write(b"\x03")
    if text or newline:
        c.write(text.encode() + (b"\n" if newline else b""))
    return {"status": "ok", "offset": offset}


# --------------------------------------------------------------------------- shell

@server.tool(annotations=_annotations(destructive=True))
@_threaded
def nsh_exec(command: str, timeout: float = 10.0, transport: str = "auto", confirm: bool = False) -> dict:
    """Run one NSH/PX4 shell command and return its output.

    transport: 'uart' (debug console), 'mavlink' (MAVLink shell over USB) or 'auto' (UART if available).
    Commands that modify parameters, storage, vehicle state or actuators need confirm=true after asking the user.
    Use the reboot tool instead of `reboot`."""
    b = get_bench()
    msg = b.guard(command, confirm)
    if msg:
        return {"status": "rejected", "message": msg}
    result = b.shell(transport).exec(command, timeout)
    result["output"] = _clip(result["output"])
    return result


@server.tool(annotations=_annotations(destructive=True))
@_threaded
def nsh_batch(commands: list[str], timeout: float = 10.0, transport: str = "auto", confirm: bool = False) -> dict:
    """Run several shell commands in order (e.g. a bring-up snapshot) and return all outputs.

    The whole batch is rejected if any command needs confirmation and confirm is false."""
    b = get_bench()
    for cmd in commands:
        msg = b.guard(cmd, confirm)
        if msg:
            return {"status": "rejected", "command": cmd, "message": msg}
    sh = b.shell(transport)
    results = []
    for cmd in commands:
        r = sh.exec(cmd, timeout)
        r["output"] = _clip(r["output"], CLIP // max(len(commands), 1) + 2000)
        results.append(r)
    return {"status": "ok", "transport": sh.name, "results": results}


@server.tool(annotations=_annotations(read_only=True))
@_threaded
def verify_firmware(target: str, transport: str = "auto") -> dict:
    """Compare the git hash reported by the board (`ver all`) with the local build of `target`."""
    b = get_bench()
    info = firmware.target_info(b.cfg, target)
    r = b.shell(transport).exec("ver all", 10)
    board_hash = firmware.parse_ver_git_hash(r["output"])
    match = bool(board_hash and info.git_hash and
                 (info.git_hash.startswith(board_hash) or board_hash.startswith(info.git_hash)))
    return {"match": match, "board_git_hash": board_hash, "build_git_hash": info.git_hash,
            "build_time": info.build_time, "target": info.target, "ver_all": r["output"]}


@server.tool(annotations=_annotations(read_only=True))
@_threaded
def fetch_file(remote_path: str, timeout: float = 60.0, transport: str = "auto") -> dict:
    """Fetch a TEXT file from the board (via `cat`) into the local log dir, e.g. fault logs or boot logs.

    Binary files such as .ulg are not supported; download those with QGroundControl / MAVLink FTP."""
    b = get_bench()
    r = b.shell(transport).exec(f"cat {remote_path}", timeout)
    out_dir = b.cfg.log_dir / "fetched"
    out_dir.mkdir(exist_ok=True)
    local = out_dir / f"{time.strftime('%Y%m%d-%H%M%S')}-{Path(remote_path).name}"
    local.write_text(r["output"] + "\n")
    return {"status": "timeout" if r["timed_out"] else "ok", "local_path": str(local),
            "bytes": len(r["output"]), "head": "\n".join(r["output"].splitlines()[:40])}


# --------------------------------------------------------------------------- reboot / flash

def _wait_boot(b: Bench, mark: int, timeout: float, after: str = "") -> dict:
    c = b.console
    if not c:
        return {"boot_log": "UART console not open; boot log not captured"}
    deadline = time.monotonic() + timeout
    prompt_seen = False
    while time.monotonic() < deadline:
        text = c.buffer.text_since(mark)
        pos = text.find(after) + len(after) if after and after in text else 0
        if c.prompt_re.search(text[pos:]):
            prompt_seen = True
            break
        c.buffer.wait(0.5)
    text = c.buffer.text_since(mark)
    return {"prompt_seen": prompt_seen, "boot_log_offset": mark, "boot": boot_summary(text),
            "hint": "read the full boot log with console_read(since=boot_log_offset)"}


@server.tool(annotations=_annotations(destructive=True))
@_threaded
def reboot(bootloader: bool = False, wait_for_boot: bool = True, transport: str = "auto",
           timeout: float = 0) -> dict:
    """Reboot the board via the shell (`reboot`, or `reboot -b` to stay in the bootloader).

    With the UART console open, waits for the shell prompt and returns a boot summary plus the offset of the
    boot log for console_read."""
    b = get_bench()
    sh = b.shell(transport)
    mark = b.console.buffer.offset if b.console else 0
    cmd = "reboot -b" if bootloader else "reboot"
    sh.write(b"\n" + cmd.encode() + b"\n")
    if sh is b.mav:
        b.close_mav()
    if bootloader or not wait_for_boot:
        time.sleep(2)
        return {"status": "ok", "command": cmd, "boot_log_offset": mark}
    return {"status": "ok", "command": cmd, **_wait_boot(b, mark, timeout or b.cfg.boot_timeout, after=cmd)}


def _run_uploader(b: Bench, args: list[str], timeout: float, log_name: str) -> tuple[int | None, str]:
    up = b.cfg.uploader
    if not up or not Path(up).is_file():
        raise RuntimeError("uploader not found: set [uploader] path in bench.toml or SALUKI_UPLOADER "
                           "(the upstream Tools/px_uploader.py does not work with Saluki bootloaders)")
    # The uploader talks to the board's USB CDC port; release our MAVLink connection on it first.
    b.close_mav()
    logs = b.cfg.log_dir / "flash"
    logs.mkdir(exist_ok=True)
    return firmware.run_process([sys.executable, up, *args], timeout=timeout, cwd=Path(up).parent,
                                log_path=logs / f"{time.strftime('%Y%m%d-%H%M%S')}-{log_name}.log")


def _uploader_hint(out: str) -> str:
    if "sudo" in out and ("password" in out or "terminal is required" in out or "execute failed" in out):
        return ("The uploader needed sudo to write the block device and cannot prompt from here. Give your user "
                "write access to the board's USB mass-storage device (udev rule / 'disk' group) or a NOPASSWD sudo rule.")
    if "Wait for connection" in out and "Found" not in out:
        return "No Saluki bootloader/USB device appeared. Check the USB cable and that the board is powered."
    return ""


@server.tool(annotations=_annotations(destructive=True))
@_threaded
def flash(target: str = "", file: str = "", second_partition: bool = False, wait_for_boot: bool = True,
          timeout: float = 0) -> dict:
    """Flash a .px4 image with the configured Saluki uploader (px_uploader.py from saluki_bootloader_v2).

    target: flash build/<target>/<target>.px4 of a local build; or file: explicit .px4 path (e.g. CI artifact).
    second_partition writes the image to partition 2 (-2). Bootloader (-b) and keystore (-k) writes are not
    exposed on purpose. With the UART console open, returns a boot summary after flashing."""
    b = get_bench()
    if file:
        px4 = Path(file).expanduser().resolve()
    elif target:
        info = firmware.target_info(b.cfg, target)
        if not info.px4_file:
            return {"status": "error", "message": f"{info.target} has no .px4; run build first"}
        px4 = Path(info.px4_file)
    else:
        return {"status": "error", "message": "pass target or file"}
    if not px4.is_file():
        return {"status": "error", "message": f"{px4} does not exist"}

    mark = b.console.buffer.offset if b.console else 0
    args = [str(px4)] + (["-2"] if second_partition else [])
    rc, out = _run_uploader(b, args, timeout or b.cfg.uploader_timeout, "flash")
    result = {"status": "ok" if rc == 0 else "failed", "file": str(px4), "exit_code": rc,
              "uploader_output": firmware.tail(out, 40)}
    hint = _uploader_hint(out)
    if hint and rc != 0:
        result["hint"] = hint
    if rc == 0 and wait_for_boot:
        result.update(_wait_boot(b, mark, b.cfg.boot_timeout))
    return result


@server.tool(annotations=_annotations(destructive=True))
@_threaded
def detect_board(timeout: float = 30.0) -> dict:
    """Identify the connected board via the bootloader (uploader -d): HW type, serial, FPGA and bootloader version.

    Note: this reboots a running board into the bootloader."""
    b = get_bench()
    rc, out = _run_uploader(b, ["-d"], timeout, "detect")
    result = {"status": "ok" if rc == 0 else "failed", "exit_code": rc, "output": firmware.tail(out, 30)}
    hint = _uploader_hint(out)
    if hint and rc != 0:
        result["hint"] = hint
    return result


# --------------------------------------------------------------------------- build / decode

@server.tool(annotations=_annotations())
@_threaded
def build(target: str, incremental: bool = True, rebuild_image: bool = False) -> dict:
    """Build a target inside the tii_px4_build container (same flow as build.sh / packaging/build_px4fw.sh).

    incremental=false does a clean build. rebuild_image re-creates the container image. Takes minutes."""
    return firmware.build(get_bench().cfg, target, incremental=incremental, rebuild_image=rebuild_image)


@server.tool(annotations=_annotations(read_only=True))
@_threaded
def decode_crash(target: str = "", dump: str = "", elf: str = "", app: str = "", arch: str = "",
                 max_addresses: int = 300) -> dict:
    """Decode a NuttX crash/assert dump to functions and source lines with addr2line (host or build container).

    dump: dump text; omit to take the latest crash from the UART console buffer.
    target: local build to decode against. elf: alternative ELF, directory or CI *_app_elfs-*.tar.gz.
    app: userspace app name, if not detectable from the dump (kernel builds have per-app ELFs in bin_debug/).
    arch: risc-v | arm64 | arm, only needed with `elf` and no `target`."""
    b = get_bench()
    source = "argument"
    if not dump:
        c = b.get_console()
        dump = firmware.extract_crash(c.buffer.text_since(c.buffer.first_offset())) or ""
        source = "console"
        if not dump:
            return {"status": "error", "message": "no crash dump found in the console buffer; pass `dump`"}
    result = firmware.decode_crash(b.cfg, dump, target=target or None, elf=elf or None, app=app or None,
                                   arch=arch or None, max_addresses=max_addresses)
    result["dump_source"] = source
    if source == "console":
        result["dump"] = _clip(dump, 8000)
    return result


def main():
    server.run()


if __name__ == "__main__":
    main()
