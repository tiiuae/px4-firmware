# px4-bench

px4-bench is an MCP server that lets an AI agent (Copilot in VS Code, Copilot CLI, or any MCP client) work with a PX4/NuttX flight controller on the bench. The agent can capture the UART console, run shell commands, flash and reboot the board, build in the `tii_px4_build` container, and decode crash dumps.

The code is generic across boards. It works for any `boards/<vendor>/<board>` target. Board details (architecture, toolchain, ELF layout, prompt) are read from the board directory and the build output, not hard-coded.

Workflows are described in [USE_CASES.md](USE_CASES.md). The agent skills in [`.github/skills/`](../../.github/skills) build on these tools.

## Requirements

- Linux host with Python ≥ 3.11 and Docker (your user must be able to run `docker` without sudo).
- The board's UART debug console connected through a USB-UART adapter (NSH prompt `saluki> `).
- The board's USB port connected. It is used for flashing and for the optional MAVLink shell.
- The Saluki uploader: `tools/px_uploader.py` from the `saluki_bootloader_v2` repository. `Tools/px_uploader.py` in this repo is the upstream version and does **not** work with Saluki bootloaders.
- Write access to the board's USB mass-storage device while flashing. The uploader falls back to `sudo`, which can't prompt for a password from inside an MCP server, so set up a udev rule, add your user to the `disk` group, or use a NOPASSWD sudo rule.

You don't need cross toolchains on the host. `addr2line` and the builds run inside the `tii_px4_build` image. If the image is missing, the `build` tool creates it the same way as `build.sh`.

## Setup

```sh
cd Tools/px4-bench
cp bench.example.toml bench.toml   # git-ignored, per-bench settings
$EDITOR bench.toml                 # console port, uploader path
```

`run.sh` creates `.venv` and installs [requirements.txt](requirements.txt) the first time it runs, then starts the server over stdio.

### VS Code

The repository ships [`.vscode/mcp.json`](../../.vscode/mcp.json), which registers the server as `px4-bench`. Start it from the MCP view, or from the prompt the first time a tool is used.

### Copilot CLI and other clients

Add a stdio server that runs `bash <repo>/Tools/px4-bench/run.sh`. For example, in `~/.copilot/mcp-config.json`:

```json
{
  "mcpServers": {
    "px4-bench": {
      "type": "local",
      "command": "bash",
      "args": ["/home/<user>/code/px4-firmware/Tools/px4-bench/run.sh"],
      "tools": ["*"]
    }
  }
}
```

## Configuration

See [bench.example.toml](bench.example.toml). These environment variables override the file:

| Variable | Meaning |
|---|---|
| `PX4_BENCH_CONFIG` | Path to the config file (default `Tools/px4-bench/bench.toml`) |
| `PX4_BENCH_CONSOLE_PORT` | UART console device |
| `PX4_BENCH_MAVLINK_PORT` | MAVLink USB device (default: auto-detect by USB description) |
| `SALUKI_UPLOADER` | Path to the Saluki `px_uploader.py` |
| `SIGNING_KEY`, `SIGNING_ARGS`, `SIGNING_TOOL` | Passed to the container build, as in `build.sh` |

Use stable `/dev/serial/by-id/...` names for ports. The `list_serial_ports` tool shows them.

Console captures, build and flash logs, fetched files and extracted CI artifacts are stored in `~/.cache/px4-bench` (setting `[logs] dir`).

## Tools

| Tool | Purpose |
|---|---|
| `bench_status` | Configuration, transport state, uploader and image availability, list of boards |
| `list_serial_ports` | Serial ports with descriptions and `/dev/serial/by-id` names |
| `board_info(target)` | Arch, toolchain prefix, kernel or flat build, artifact paths, git hash of the local build |
| `console_open` / `console_close` | Start or stop continuous UART capture. Also logged to a file |
| `console_read(since, last_lines, pattern, context)` | Read captured output. Pass `next_offset` as `since` to get only new output |
| `console_write(text, ctrl_c)` | Send raw text without waiting for a prompt (for example, bootloader prompts) |
| `nsh_exec(command, timeout, transport)` | Run a command and return its output once the prompt comes back |
| `nsh_batch(commands)` | Run several commands, e.g. a bring-up snapshot |
| `verify_firmware(target)` | Compare the `ver all` git hash with the local build |
| `fetch_file(remote_path)` | Copy a text file from the board (via `cat`) to the log directory |
| `reboot(bootloader, wait_for_boot)` | `reboot` / `reboot -b`. Returns a boot summary and the offset of the boot log |
| `flash(target \| file, second_partition)` | Flash a `.px4` with the Saluki uploader, then summarize the boot |
| `detect_board` | Board, FPGA and bootloader identification (`px_uploader.py -d`). Reboots the board into the bootloader |
| `build(target, incremental, rebuild_image)` | Container build through `packaging/build_px4fw.sh`, with test-key signing |
| `decode_crash(target \| elf, dump, app, arch)` | `addr2line` on registers and stack words, using the kernel ELF and `bin_debug/<app>` ELFs |

Targets can be given as `saluki-pi`, `saluki-v3_flat`, `ssrc_saluki-pi_default`, or any `<vendor>_<board>_<label>`.

`transport` is `uart` (debug console), `mavlink` (MAVLink shell over USB, using the same protocol as `Tools/mavlink_shell.py`), or `auto` (UART if configured, otherwise MAVLink).

## Safety

- Commands that change parameters (`param set/save/reset/import`), storage (`rm`, `mtd`, `>` redirects, ...), vehicle state (`commander arm/takeoff/...`) or actuators (`actuator_test`, `motor_test`, `pwm`, `dshot`) are rejected unless `confirm=true` is passed. The agent is told to ask you first. You can add more patterns with `[safety] extra_confirm_patterns`.
- `reboot` and `shutdown` must go through the `reboot` tool.
- `flash` only writes firmware partitions (`-2` is optional). Bootloader (`-b`) and keystore (`-k`) writes are not exposed.
- Commands that never return (e.g. `top` without `once`) are interrupted with Ctrl-C at the timeout.
- Assume propellers are attached. Remove them before running anything that can drive outputs.

## Limitations

- `fetch_file` handles only text files. Download `.ulg` logs with QGroundControl or MAVLink FTP.
- No JTAG/SWD probe support and no CI/HIL integration yet.
- One board per server instance. To use several boards, run several instances, each with its own `PX4_BENCH_CONFIG`.

## Development

```sh
cd Tools/px4-bench
.venv/bin/pip install pytest
.venv/bin/python -m pytest -q tests
```

The tests emulate NSH on a pseudo-terminal, so they don't need hardware.
