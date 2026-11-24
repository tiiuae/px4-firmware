"""Build targets, artifacts, build container and crash decoding."""

import os
import re
import shutil
import signal
import subprocess
import tarfile
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path

from .config import Config

CONTAINER_SRC = "/px4-firmware/sources"


# --------------------------------------------------------------------------- targets

def list_boards(repo: Path, vendor: str = "ssrc") -> list[str]:
    root = repo / "boards" / vendor
    return sorted(p.name for p in root.iterdir() if (p / "nuttx-config").is_dir()) if root.is_dir() else []


def resolve_target(repo: Path, name: str) -> str:
    """Accept 'saluki-pi', 'saluki-pi_flat', 'ssrc_saluki-pi_default' or any '<vendor>_<board>_<label>'."""
    name = name.strip()
    if not name:
        raise ValueError("target name is empty")
    parts = name.split("_")
    if len(parts) >= 3 and (repo / "boards" / parts[0] / parts[1]).is_dir():
        return name
    if (repo / "build" / name).is_dir():
        return name
    board, label = (parts[0], "default") if len(parts) == 1 else (parts[0], "_".join(parts[1:]))
    vendors = sorted((repo / "boards").iterdir(), key=lambda p: (p.name != "ssrc", p.name))
    for vendor in vendors:
        if (vendor / board).is_dir():
            return f"{vendor.name}_{board}_{label}"
    raise ValueError(f"unknown target '{name}'. ssrc boards: {', '.join(list_boards(repo))}")


def _read_kconfig(path: Path) -> dict:
    values = {}
    if path.is_file():
        for line in path.read_text(errors="replace").splitlines():
            m = re.match(r'^(CONFIG_[A-Z0-9_]+)=(.*)$', line)
            if m:
                values[m.group(1)] = m.group(2).strip('"')
    return values


@dataclass
class TargetInfo:
    target: str
    board_dir: str
    build_dir: str
    built: bool
    arch: str
    toolchain_prefix: str
    kernel_build: bool
    px4_file: str | None
    elf: str | None
    kernel_elf: str | None
    app_debug_dir: str | None
    map_file: str | None
    git_hash: str | None
    build_time: int | None
    nsh_prompt: str | None

    def to_dict(self) -> dict:
        return asdict(self)


def _existing(p: Path) -> str | None:
    return str(p) if p.exists() else None


def target_info(cfg: Config, name: str) -> TargetInfo:
    target = resolve_target(cfg.repo, name)
    parts = target.split("_")
    board_dir = cfg.repo / "boards" / parts[0] / parts[1]
    build_dir = cfg.repo / "build" / target

    kconfig = _read_kconfig(build_dir / "NuttX" / "nuttx" / ".config")
    if not kconfig:
        label_cfg = board_dir / "nuttx-config" / parts[2] / "defconfig" if len(parts) > 2 else None
        kconfig = _read_kconfig(label_cfg) if label_cfg and label_cfg.is_file() else {}
    if not kconfig:
        kconfig = _read_kconfig(board_dir / "nuttx-config" / "nsh" / "defconfig")
    arch = kconfig.get("CONFIG_ARCH", "")

    git_hash = build_time = None
    version_h = build_dir / "src" / "lib" / "version" / "build_git_version.h"
    if version_h.is_file():
        text = version_h.read_text()
        m = re.search(r'PX4_GIT_VERSION_STR\s+"([0-9a-f]+)"', text)
        git_hash = m.group(1) if m else None
        m = re.search(r"PX4_BUILD_TIME\s+(\d+)", text)
        build_time = int(m.group(1)) if m else None

    return TargetInfo(
        target=target,
        board_dir=str(board_dir),
        build_dir=str(build_dir),
        built=(build_dir / f"{target}.px4").is_file(),
        arch=arch,
        toolchain_prefix=cfg.toolchain_prefix.get(arch, ""),
        kernel_build=kconfig.get("CONFIG_BUILD_KERNEL") == "y",
        px4_file=_existing(build_dir / f"{target}.px4"),
        elf=_existing(build_dir / f"{target}.elf"),
        kernel_elf=_existing(build_dir / f"{target}_kernel.elf"),
        app_debug_dir=_existing(build_dir / "bin_debug"),
        map_file=_existing(build_dir / f"{target}.map"),
        git_hash=git_hash,
        build_time=build_time,
        nsh_prompt=kconfig.get("CONFIG_NSH_PROMPT_STRING"),
    )


def parse_ver_git_hash(ver_output: str) -> str | None:
    """Git hash from `ver all`/`ver git` output (upstream prints 'FW git-hash', this fork 'PX4 git-hash')."""
    m = re.search(r"(?:FW|PX4) git-hash:\s*([0-9a-f]{7,40})", ver_output)
    return m.group(1) if m else None


# --------------------------------------------------------------------------- processes

def run_process(cmd: list[str], timeout: float, log_path: Path | None = None, cwd: Path | None = None,
                env: dict | None = None, on_timeout=None) -> tuple[int | None, str]:
    """Run a command, stream output to an optional log file, kill the process group on timeout."""
    log = open(log_path, "w") if log_path else None
    proc = subprocess.Popen(cmd, cwd=cwd, env=env, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, errors="replace", start_new_session=True)
    lines = []

    def reader():
        for line in proc.stdout:
            lines.append(line)
            if log:
                log.write(line)
                log.flush()

    t = threading.Thread(target=reader, daemon=True)
    t.start()
    timed_out = False
    try:
        proc.wait(timeout)
    except subprocess.TimeoutExpired:
        timed_out = True
        if on_timeout:
            on_timeout()
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
    t.join(5)
    if log:
        log.close()
    output = "".join(lines)
    if timed_out:
        output += f"\n[px4-bench] killed after {timeout:.0f}s timeout\n"
        return None, output
    return proc.returncode, output


def tail(text: str, n: int) -> str:
    return "\n".join(text.rstrip("\n").splitlines()[-n:])


def docker_image_exists(image: str) -> bool:
    return subprocess.run(["docker", "image", "inspect", image], stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL).returncode == 0


def build_docker_image(cfg: Config, log_path: Path) -> tuple[int | None, str]:
    # Same invocation as build.sh, so the image is interchangeable with the official build flow.
    cmd = ["docker", "build", "--build-arg", f"UID={os.getuid()}", "--build-arg", f"GID={os.getgid()}",
           "--pull", "-f", "./packaging/Dockerfile.build_env", "-t", cfg.docker_image, "."]
    return run_process(cmd, timeout=cfg.build_timeout, log_path=log_path, cwd=cfg.repo)


class PathMapper:
    """Maps host paths to paths inside the build container (repo is mounted at CONTAINER_SRC)."""

    def __init__(self, repo: Path):
        self.repo = repo
        self.mounts: dict[str, str] = {}

    def to_container(self, path: str) -> str:
        p = Path(path).resolve()
        try:
            return f"{CONTAINER_SRC}/{p.relative_to(self.repo)}"
        except ValueError:
            parent = str(p.parent)
            if parent not in self.mounts:
                self.mounts[parent] = f"/ext/{len(self.mounts)}"
            return f"{self.mounts[parent]}/{p.name}"

    def docker_args(self) -> list[str]:
        args = ["-v", f"{self.repo}:{CONTAINER_SRC}"]
        for host, cont in self.mounts.items():
            args += ["-v", f"{host}:{cont}:ro"]
        return args


def docker_run(cfg: Config, args: list[str], mapper: PathMapper | None = None, env: dict | None = None,
               timeout: float = 600, log_path: Path | None = None) -> tuple[int | None, str]:
    mapper = mapper or PathMapper(cfg.repo)
    name = f"px4-bench-{os.getpid()}-{int(time.time() * 1000)}"
    cmd = ["docker", "run", "--rm", "--name", name, *mapper.docker_args(), "-w", CONTAINER_SRC]
    for k, v in (env or {}).items():
        cmd += ["-e", f"{k}={v}"]
    cmd += [cfg.docker_image, *args]

    def kill():
        subprocess.run(["docker", "kill", name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    return run_process(cmd, timeout=timeout, log_path=log_path, on_timeout=kill)


def run_toolchain(cfg: Config, tool: str, args: list[str], paths: list[str], timeout: float = 120) -> tuple[int | None, str]:
    """Run a cross toolchain binary on the host if available, otherwise inside the build container.

    `args` may contain '{0}', '{1}', ... placeholders that are replaced by the mapped `paths`."""
    if cfg.use_host_toolchain and shutil.which(tool):
        return run_process([tool, *[a.format(*paths) for a in args]], timeout=timeout)
    if not docker_image_exists(cfg.docker_image):
        return 1, f"{tool} not found on host and docker image '{cfg.docker_image}' is missing (run the build tool once)"
    mapper = PathMapper(cfg.repo)
    mapped = [mapper.to_container(p) for p in paths]
    return docker_run(cfg, [tool, *[a.format(*mapped) for a in args]], mapper=mapper, timeout=timeout)


def map_source_path(cfg: Config, path: str) -> str:
    if path.startswith(CONTAINER_SRC + "/"):
        return path[len(CONTAINER_SRC) + 1:]
    try:
        return str(Path(path).relative_to(cfg.repo))
    except ValueError:
        return path


# --------------------------------------------------------------------------- build

ERROR_LINE_RE = re.compile(r"(error:|FAILED:|undefined reference|Error \d+|No rule to make target|ninja: build stopped)")


def build(cfg: Config, name: str, incremental: bool = True, rebuild_image: bool = False) -> dict:
    target = resolve_target(cfg.repo, name)
    logs = cfg.log_dir / "builds"
    logs.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")

    if rebuild_image or not docker_image_exists(cfg.docker_image):
        rc, out = build_docker_image(cfg, logs / f"docker-image-{stamp}.log")
        if rc != 0:
            return {"status": "failed", "stage": "docker image", "tail": tail(out, 40)}

    log_path = logs / f"{target}-{stamp}.log"
    env = {"PX4_INCREMENTAL": "1" if incremental else "0"}
    for k in ("SIGNING_KEY", "SIGNING_ARGS", "SIGNING_TOOL"):
        if os.environ.get(k):
            env[k] = os.environ[k]

    start = time.time()
    rc, out = docker_run(cfg, ["./packaging/build_px4fw.sh", target], env=env,
                         timeout=cfg.build_timeout, log_path=log_path)
    info = target_info(cfg, target)
    # build_px4fw.sh does not propagate make's exit status, so judge success by a fresh .px4.
    fresh = bool(info.px4_file) and Path(info.px4_file).stat().st_mtime >= start - 1
    errors = [line for line in out.splitlines() if ERROR_LINE_RE.search(line)][:30]
    return {
        "status": "ok" if fresh and rc == 0 else "failed",
        "target": target,
        "incremental": incremental,
        "duration_s": round(time.time() - start),
        "log_file": str(log_path),
        "errors": errors,
        "tail": tail(out, 25),
        "artifacts": info.to_dict(),
    }


# --------------------------------------------------------------------------- crash decoding

CRASH_MARKER_RE = re.compile(
    r"(Assertion failed|EXCEPTION:|PANIC|Hard ?[Ff]ault|Segmentation fault|Unhandled .*exception|"
    r"up_assert|dump_assert_info|Kernel panic|arm64_fatal|riscv_exception)")
REGISTER_RE = re.compile(r"\b(PC|LR|EPC|MEPC|SEPC|RA|ELR|x30)\s*:\s*(?:0x)?([0-9a-fA-F]{6,16})\b")
HEX_RE = re.compile(r"(?<![0-9a-zA-Z_])(?:0x)?([0-9a-fA-F]{6,16})(?![0-9a-zA-Z_])")
TASK_RES = [
    re.compile(r"\btask:\s*([\w.\-/]+)"),
    re.compile(r"Segmentation fault in ([\w.\-/]+)"),
    re.compile(r"exception in PID \d+:\s*([\w.\-/]+)"),
    re.compile(r"PID \d+:\s*([\w.\-/]+)"),
]


def extract_crash(text: str, before_lines: int = 5, max_lines: int = 400) -> str | None:
    lines = text.splitlines()
    for i in range(len(lines) - 1, -1, -1):
        if CRASH_MARKER_RE.search(lines[i]):
            # Walk back to the first marker of this dump (dumps often contain several marker lines).
            first = i
            j = i - 1
            while j >= 0 and i - j < 60:
                if CRASH_MARKER_RE.search(lines[j]):
                    first = j
                j -= 1
            start = max(first - before_lines, 0)
            return "\n".join(lines[start:start + max_lines])
    return None


def crash_tasks(dump: str) -> list[str]:
    names = []
    for regex in TASK_RES:
        for m in regex.finditer(dump):
            n = m.group(1).split("/")[-1]
            if n not in names:
                names.append(n)
    return names


def crash_addresses(dump: str, limit: int) -> tuple[list[tuple[str, str]], list[str]]:
    registers = list(dict.fromkeys((m.group(1), m.group(2).lower()) for m in REGISTER_RE.finditer(dump)))
    seen = {v for _, v in registers}
    candidates = []
    for m in HEX_RE.finditer(dump):
        v = m.group(1).lower()
        if v in seen or set(v) <= {"0"} or set(v) <= {"f"}:
            continue
        seen.add(v)
        candidates.append(v)
    return registers, candidates[:limit]


def parse_addr2line(output: str) -> dict[str, list[str]]:
    """Parse `addr2line -a -p -f -i -C` output into {address: [frame, inlined-by...]}."""
    result: dict[str, list[str]] = {}
    current = None
    for line in output.splitlines():
        m = re.match(r"^0x([0-9a-fA-F]+):\s*(.*)$", line)
        if m:
            current = m.group(1).lstrip("0").lower() or "0"
            result[current] = [m.group(2).strip()]
        elif current and line.strip().startswith("(inlined by)"):
            result[current].append(line.strip())
    return result


def _resolved(frames: list[str]) -> bool:
    return bool(frames) and not frames[0].startswith("??")


def _clean_frame(cfg: Config, frame: str) -> str:
    return re.sub(r"(/\S+?):", lambda m: map_source_path(cfg, m.group(1)) + ":", frame)


def prepare_artifact(cfg: Config, path: str) -> Path:
    """Accept an ELF, a directory or a CI *_app_elfs-*.tar.gz; returns a file or directory."""
    p = Path(path).expanduser().resolve()
    if p.is_file() and p.name.endswith((".tar.gz", ".tgz")):
        out = cfg.log_dir / "artifacts" / p.name.replace(".tar.gz", "").replace(".tgz", "")
        if not out.is_dir():
            out.mkdir(parents=True)
            with tarfile.open(p) as tar:
                try:
                    tar.extractall(out, filter="data")
                except TypeError:
                    tar.extractall(out)
        return out
    if not p.exists():
        raise FileNotFoundError(path)
    return p


def _elfs_from_dir(d: Path) -> tuple[Path | None, Path | None]:
    kernel = next(iter(sorted(d.glob("*_kernel.elf"))), None)
    main = kernel or next((e for e in sorted(d.glob("*.elf")) if e.name != "toc.elf"), None)
    debug = d / "bin_debug"
    return main, debug if debug.is_dir() else None


def decode_crash(cfg: Config, dump: str, target: str | None = None, elf: str | None = None,
                 app: str | None = None, arch: str | None = None, max_addresses: int = 300) -> dict:
    tasks = crash_tasks(dump)
    if app and app not in tasks:
        tasks.insert(0, app)

    notes = []
    info = None
    if target:
        info = target_info(cfg, target)
        arch = arch or info.arch
    if not arch:
        return {"status": "error", "message": "pass `target` (to look up the architecture) or `arch` "
                                               "(risc-v, arm64, arm)"}
    prefix = cfg.toolchain_prefix.get(arch)
    if not prefix:
        return {"status": "error", "message": f"no toolchain prefix configured for arch '{arch}'"}

    elfs: list[tuple[str, Path]] = []
    if elf:
        p = prepare_artifact(cfg, elf)
        main, debug_dir = _elfs_from_dir(p) if p.is_dir() else (p, None)
    elif info:
        if not info.elf and not info.kernel_elf:
            return {"status": "error", "message": f"{info.target} is not built; build it or pass `elf`"}
        main = Path(info.kernel_elf or info.elf)
        debug_dir = Path(info.app_debug_dir) if info.app_debug_dir else None
    else:
        return {"status": "error", "message": "pass `target` or `elf`"}
    if main:
        elfs.append(("kernel" if main.name.endswith("_kernel.elf") else "firmware", main))
    for t in tasks:
        if debug_dir and (debug_dir / t).is_file():
            elfs.append((f"app:{t}", debug_dir / t))
    if debug_dir and tasks and len(elfs) == 1:
        notes.append(f"no app debug ELF matched tasks {tasks} in {debug_dir}; userspace frames may not resolve")

    registers, candidates = crash_addresses(dump, max_addresses)
    addresses = list(dict.fromkeys([v for _, v in registers] + candidates))
    if not addresses:
        return {"status": "error", "message": "no addresses found in dump"}

    resolved_by_elf = {}
    for label, path in elfs:
        rc, out = run_toolchain(cfg, f"{prefix}addr2line", ["-a", "-p", "-f", "-i", "-C", "-e", "{0}", *addresses],
                                [str(path)])
        if rc != 0:
            notes.append(f"addr2line failed for {label}: {tail(out, 5)}")
            continue
        resolved_by_elf[label] = parse_addr2line(out)

    def lookup(addr: str) -> list[dict]:
        key = addr.lstrip("0") or "0"
        hits = []
        for label, table in resolved_by_elf.items():
            frames = table.get(key)
            if frames and _resolved(frames):
                hits.append({"elf": label, "frames": [_clean_frame(cfg, f) for f in frames]})
        return hits

    reg_out = [{"register": r, "address": "0x" + v, "resolved": lookup(v)} for r, v in registers]
    stack_out = []
    for v in candidates:
        hits = lookup(v)
        if hits:
            stack_out.append({"address": "0x" + v, "resolved": hits})

    summary_lines = [line.strip() for line in dump.splitlines()
                     if re.search(r"Assertion failed|EXCEPTION:|PANIC|Segmentation fault|Reason:|Hard ?[Ff]ault", line)]
    notes.append("Stack/backtrace entries come from scanning every hex word in the dump; entries that are "
                 "not on a call path (stale stack data) are possible. Register entries are authoritative.")
    if info and info.git_hash:
        notes.append(f"Decoded against local build {info.target} @ {info.git_hash[:12]}; confirm the board runs "
                     "the same build (verify_firmware) or results are meaningless.")
    return {
        "status": "ok",
        "arch": arch,
        "elfs": {label: str(p) for label, p in elfs},
        "tasks": tasks,
        "summary": summary_lines[:10],
        "registers": reg_out,
        "code_addresses": stack_out,
        "notes": notes,
    }
