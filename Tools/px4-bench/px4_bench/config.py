"""Bench configuration: bench.toml + environment variable overrides."""

import os
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent
TOOL_DIR = PACKAGE_DIR.parent

DEFAULT_PROMPT = r"(?:saluki|nsh)> "

DEFAULT_TOOLCHAIN_PREFIX = {
    "risc-v": "riscv-none-elf-",
    "arm64": "aarch64-none-elf-",
    "arm": "arm-none-eabi-",
}


def _find_repo(start: Path) -> Path:
    for p in [start, *start.parents]:
        if (p / "boards").is_dir() and (p / "Makefile").is_file() and (p / "platforms").is_dir():
            return p
    raise RuntimeError(f"cannot locate px4-firmware repository root above {start}")


@dataclass
class Config:
    repo: Path
    config_file: Path | None = None
    console_port: str = ""
    console_baud: int = 115200
    console_autoopen: bool = True
    prompt: str = DEFAULT_PROMPT
    mavlink_port: str = ""
    mavlink_baud: int = 57600
    mavlink_port_match: str = r"Saluki|PX4"
    uploader: str = ""
    uploader_timeout: float = 240.0
    boot_timeout: float = 60.0
    docker_image: str = "tii_px4_build"
    build_timeout: float = 3600.0
    use_host_toolchain: bool = True
    toolchain_prefix: dict = field(default_factory=lambda: dict(DEFAULT_TOOLCHAIN_PREFIX))
    log_dir: Path = Path("~/.cache/px4-bench").expanduser()
    extra_confirm_patterns: list = field(default_factory=list)


def load_config() -> Config:
    path_env = os.environ.get("PX4_BENCH_CONFIG")
    path = Path(path_env).expanduser() if path_env else TOOL_DIR / "bench.toml"
    data = {}
    if path.is_file():
        with open(path, "rb") as f:
            data = tomllib.load(f)
    else:
        path = None

    repo = Path(data["repo"]).expanduser() if data.get("repo") else _find_repo(TOOL_DIR)
    cfg = Config(repo=repo.resolve(), config_file=path)

    console = data.get("console", {})
    cfg.console_port = console.get("port", cfg.console_port)
    cfg.console_baud = int(console.get("baud", cfg.console_baud))
    cfg.console_autoopen = bool(console.get("autoopen", cfg.console_autoopen))
    cfg.prompt = console.get("prompt", cfg.prompt)

    mav = data.get("mavlink", {})
    cfg.mavlink_port = mav.get("port", cfg.mavlink_port)
    cfg.mavlink_baud = int(mav.get("baud", cfg.mavlink_baud))
    cfg.mavlink_port_match = mav.get("port_match", cfg.mavlink_port_match)

    up = data.get("uploader", {})
    cfg.uploader = up.get("path", cfg.uploader)
    cfg.uploader_timeout = float(up.get("timeout", cfg.uploader_timeout))
    cfg.boot_timeout = float(up.get("boot_timeout", cfg.boot_timeout))

    build = data.get("build", {})
    cfg.docker_image = build.get("docker_image", cfg.docker_image)
    cfg.build_timeout = float(build.get("timeout", cfg.build_timeout))

    tc = data.get("toolchain", {})
    cfg.use_host_toolchain = bool(tc.get("use_host", cfg.use_host_toolchain))
    cfg.toolchain_prefix.update(tc.get("prefix", {}))

    safety = data.get("safety", {})
    cfg.extra_confirm_patterns = list(safety.get("extra_confirm_patterns", []))

    logs = data.get("logs", {})
    if logs.get("dir"):
        cfg.log_dir = Path(logs["dir"]).expanduser()

    cfg.console_port = os.environ.get("PX4_BENCH_CONSOLE_PORT", cfg.console_port)
    cfg.mavlink_port = os.environ.get("PX4_BENCH_MAVLINK_PORT", cfg.mavlink_port)
    cfg.uploader = os.environ.get("SALUKI_UPLOADER", cfg.uploader)
    cfg.uploader = str(Path(cfg.uploader).expanduser()) if cfg.uploader else ""

    cfg.log_dir.mkdir(parents=True, exist_ok=True)
    return cfg
