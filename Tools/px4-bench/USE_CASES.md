# px4-bench use cases

These are the workflows the [px4-bench](README.md) tools and the [skills](../../.github/skills) are built for. Each one can be a single request to the agent. Requests are just examples; any wording works.

All of them are generic: they work on any `boards/ssrc/*` target (and other PX4 NuttX targets) because board details come from the repository and the build output.

## Prerequisites

- The board is on the bench with its UART debug console and USB connected, and `bench.toml` is configured (see [README](README.md#setup)).
- The `tii_px4_build` Docker image is available, or will be built on first use.
- The Saluki `px_uploader.py` (from `saluki_bootloader_v2`) is configured.
- The shell prompt is `saluki> ` (upstream boards: `nsh> `).
- **Propellers are off.** Commands that can drive outputs always require confirmation.

## Contents

- Bring-up: [1](#1-first-boot-check) · [2](#2-peripheral-verification) · [3](#3-pin-bus-and-driver-iteration) · [4](#4-porting-a-feature-between-boards)
- Crash debugging: [5](#5-hard-fault-triage) · [6](#6-crashes-on-released-images) · [7](#7-intermittent-issue-statistics)
- Regressions: [8](#8-beforeafter-comparison) · [9](#9-hardware-git-bisect) · [10](#10-resource-budgeting)
- Daily work: [11](#11-build-flash-and-boot-log-in-one-step) · [12](#12-log-and-file-retrieval) · [13](#13-parameter-and-state-inspection) · [14](#14-comparing-boards)
- Knowledge: [15](#15-onboarding-on-a-live-board) · [16](#16-reusable-checklists)

---

## Bring-up

### 1. First boot check

**When:** A new board revision, new board directory or large config change boots for the first time.
**Ask the agent:** "Flash saluki-pro and tell me what's broken."
**Tools used:** `build`, `console_open`, `flash`, `console_read`, `nsh_batch`, `verify_firmware`. Skills: `build-flash-verify`, `bringup-check`
**What happens:**
1. Builds the target in the container and flashes it while the UART capture runs, so the boot log is recorded from the first byte.
2. Summarizes the boot log: error and warning lines, failed `rc.board_*` commands, and any crash.
3. Takes a snapshot (`ver all`, `free`, `top once`, `work_queue status`, `uorb top -1`, `sensors status`, ...).
4. Compares it with what `init/rc.board_sensors` and `default.px4board` say should be running.
**Output:** An OK/missing/degraded table per expected device and service, boot issues with quoted log lines, and hypotheses for each failure.
**Limits / safety:** Read-only on the board apart from flashing. A board that doesn't reach the bootloader over USB can't be flashed by this tool.

### 2. Peripheral verification

**When:** You need to know whether all sensors and buses on a board work.
**Ask the agent:** "Are all sensors on saluki-nxp93 alive and publishing at the right rate?"
**Tools used:** `nsh_batch`, `nsh_exec`. Skill: `bringup-check`
**What happens:**
1. Reads the expected sensors, buses and addresses from `init/rc.board_sensors`.
2. Runs `<driver> status`, `uorb top -1` and `listener <topic> -n 1` for each, plus `i2cdetect -b <bus>` for missing I2C devices.
3. Checks publication rates and that values are plausible (e.g. accel ≈ 9.8 m/s² at rest).
**Output:** The expected-vs-actual table with rates and sample values.
**Limits / safety:** Read-only.

### 3. Pin, bus and driver iteration

**When:** A device isn't detected, or works intermittently, and the cause is probably in the board configuration.
**Ask the agent:** "The IMU on SPI2 isn't detected on saluki-micro. Find and fix the problem."
**Tools used:** `build`, `flash`, `console_read`, `nsh_exec`, `verify_firmware`. Skills: `board-port`, `build-flash-verify`, `bringup-check`
**What happens:**
1. Compares the board's `spi.cpp`, `board_config.h`, defconfig and `rc.board_sensors` with a working board of the same SoC family.
2. Proposes a change, builds incrementally, flashes, and checks the device again.
3. Repeats with the next hypothesis, keeping a log of what was tried.
**Output:** A working change (as a diff) or a list of rejected hypotheses with their evidence.
**Limits / safety:** The agent edits files in your working tree. Review the diff. Hardware faults (soldering, wiring) can only be inferred.

### 4. Porting a feature between boards

**When:** A UART, sensor, PWM output or driver that works on one board is needed on another.
**Ask the agent:** "Enable the same telemetry UART setup on salukini as on saluki-nxp93."
**Tools used:** `build`, `flash`, `nsh_exec`. Skills: `board-port`, `build-flash-verify`
**What happens:**
1. Diffs the relevant files of the two board directories.
2. Applies the change, adapted to the target SoC's peripheral naming.
3. Builds, and if the board is on the bench, flashes and checks the feature (`mavlink status`, `listener`, driver `status`).
**Output:** A diff, plus a note on what was verified on hardware and what was only compile-tested.
**Limits / safety:** Changes only the boards that were named.

## Crash debugging

### 5. Hard fault triage

**When:** The board crashes at boot or at runtime and prints a fault dump on the UART console.
**Ask the agent:** "The board just crashed. Decode the dump."
**Tools used:** `decode_crash`, `console_read`, `verify_firmware`. Skill: `crash-triage`
**What happens:**
1. Extracts the latest fault dump (registers, stack, task name) from the console buffer.
2. Checks that the board's `ver all` git hash matches the local build.
3. Picks the ELF: `ssrc_<board>_<label>_kernel.elf` if it exists, otherwise `ssrc_<board>_<label>.elf`. Userspace faults also use `bin_debug/<task>`.
4. Runs `<prefix>-addr2line` (`riscv-none-elf-`, `aarch64-none-elf-` or `arm-none-eabi-`, from `CONFIG_ARCH`), on the host or inside the `tii_px4_build` container.
5. Reads the resolved source lines and explains the likely cause.
**Output:** A decoded call stack with source lines, the fault type, a likely root cause and a suggested fix.
**Limits / safety:** Read-only on the board. If the hash doesn't match, the result can't be trusted. Stack-scan entries can include old values left on the stack; register entries are reliable.

### 6. Crashes on released images

**When:** A crash is reported on a released or CI-built image (e.g. v1.17.x), and you have no local build of it.
**Ask the agent:** "Decode this dump against ssrc_saluki-v3_default_app_elfs-1.17.2.tar.gz."
**Tools used:** `decode_crash` (with `elf` and `arch`). Skill: `crash-triage`
**What happens:**
1. Extracts the CI artifact (`*_app_elfs-<version>.tar.gz`, produced by `build.sh`) into the cache.
2. Mounts it into the build container and decodes the dump against the kernel ELF and the matching `bin_debug` app ELF.
3. Maps source paths to the repository. Check out the release tag to read the exact code.
**Output:** The same as use case 5.
**Limits / safety:** No board needed. The version in the report must match the artifact version.

### 7. Intermittent issue statistics

**When:** A problem shows up only sometimes, such as sensor init failing, a hang at boot, or an occasional fault.
**Ask the agent:** "Reboot the board 30 times and tell me how often the baro fails to start."
**Tools used:** `reboot`, `console_read`, `nsh_exec`, `decode_crash`
**What happens:**
1. Loops `reboot(wait_for_boot=true)` and records whether the prompt appeared, the boot issue lines and any crash.
2. After each boot, runs the specific check (e.g. `sensors status` or `<driver> status`).
3. Keeps the boot log offsets of failing iterations and decodes any crashes.
**Output:** A failure rate, the distribution of symptoms, and logs from the failing boots. The console log file has everything.
**Limits / safety:** The board reboots repeatedly, so don't do this while it's connected to anything that can move.

## Regressions

### 8. Before/after comparison

**When:** You need to know whether a change affects behaviour on hardware.
**Ask the agent:** "Compare main and my branch on saluki-v3: boot log, CPU, RAM and sensor rates."
**Tools used:** `build`, `flash`, `nsh_batch`, `verify_firmware`. Skills: `build-flash-verify`, `bringup-check`
**What happens:**
1. Builds and flashes the baseline and runs a fixed snapshot (`nsh_batch`).
2. Does the same for the change.
3. Diffs the boot issue lines, `free`, `top once`, `work_queue status`, `uorb top -1` rates and driver status.
**Output:** A side-by-side table of the differences.
**Limits / safety:** The agent switches git branches in your working tree. Commit or stash your work first.

### 9. Hardware git bisect

**When:** Something broke on hardware between two known versions.
**Ask the agent:** "Boot to shell broke between v1.17.2 and main on saluki-pi. Bisect it."
**Tools used:** `build`, `flash`, `reboot`, `console_read`, `verify_firmware`
**What happens:**
1. Runs `git bisect` with a clear pass/fail criterion agreed with you (e.g. "prompt appears within 60 s and IMU is publishing").
2. For each candidate, does a clean or incremental build in the container, flashes, checks, and marks the commit good or bad.
**Output:** The first bad commit, plus the evidence log for every tested commit.
**Limits / safety:** It takes several minutes per step, and it changes the checked-out commit. Submodule changes can force clean builds.

### 10. Resource budgeting

**When:** Before a release, or after adding modules, to see how much RAM, CPU and stack headroom is left.
**Ask the agent:** "How much RAM and stack headroom do we have on saluki-nxp93 compared with the last release?"
**Tools used:** `nsh_batch` (`free`, `top once`, `ps`, `work_queue status`, `perf`), `board_info`
**What happens:**
1. Captures resource usage at idle, after boot.
2. Flags tasks with little free stack, high CPU load and low free heap.
3. Optionally compares with the `.map` of another build to see what grew.
**Output:** A budget table with warnings.
**Limits / safety:** Read-only. Idle figures don't show worst-case load in flight.

## Daily work

### 11. Build, flash and boot log in one step

**When:** Every edit-build-test cycle.
**Ask the agent:** "Build and flash saluki-pi and show me the boot log."
**Tools used:** `build`, `flash`, `console_read`, `verify_firmware`. Skill: `build-flash-verify`
**What happens:**
1. Incremental container build (`PX4_INCREMENTAL=1`). Stops and reports compile errors if there are any.
2. Flashes with the Saluki uploader while the console is captured.
3. Checks the running git hash, so you never debug a stale image.
**Output:** The build result, flash result, boot summary and hash match.
**Limits / safety:** Writes firmware partitions only.

### 12. Log and file retrieval

**When:** You need files from the board's file system.
**Ask the agent:** "Get the boot log and any fault logs from the SD card and summarize them."
**Tools used:** `nsh_exec` (`ls`), `fetch_file`, `console_read`
**What happens:**
1. Lists candidate directories (e.g. `/fs/microsd`, `/fs/microsd/log`).
2. Fetches text files into `~/.cache/px4-bench/fetched/`.
3. Summarizes them, or uses them in use case 5.
**Output:** Local file paths and a summary.
**Limits / safety:** Text only. Download `.ulg` logs with QGroundControl or MAVLink FTP.

### 13. Parameter and state inspection

**When:** You need to understand why the vehicle behaves the way it does.
**Ask the agent:** "Why isn't the EKF using GPS?"
**Tools used:** `nsh_exec` (`param show`, `listener estimator_status -n 1`, `listener vehicle_gps_position -n 1`, `ekf2 status`, `commander status`)
**What happens:**
1. Reads the relevant parameters and topics.
2. Explains them against the PX4 code paths in the repository.
3. Proposes parameter changes but does not apply them.
**Output:** An explanation and suggested changes.
**Limits / safety:** `param set/save` requires your explicit confirmation.

### 14. Comparing boards

**When:** Two boards (e.g. two revisions, or two SoC families) behave differently.
**Ask the agent:** "Run the bring-up snapshot on both bench boards and show the differences."
**Tools used:** `nsh_batch`, `board_info`, on one px4-bench instance per board
**What happens:**
1. Runs the same snapshot on each board (configure one server instance per board, each with its own `PX4_BENCH_CONFIG`).
2. Diffs the outputs and the board directories (`default.px4board`, `rc.board_*`).
**Output:** The differences in runtime state and in configuration.
**Limits / safety:** Read-only.

## Knowledge

### 15. Onboarding on a live board

**When:** A new developer needs to understand how a board is put together and starts up.
**Ask the agent:** "How does saluki-micro start its sensors? Show me on the board."
**Tools used:** Repository reading, plus `nsh_exec` / `console_read`
**What happens:**
1. Explains the startup path: NuttX init, `rcS`, `rc.board_defaults`, `rc.board_sensors`, and the modules involved.
2. Shows the corresponding boot log lines and live state (`ps`, `uorb top -1`) on the board.
**Output:** An explanation linked to files and to live evidence.
**Limits / safety:** Read-only.

### 16. Reusable checklists

**When:** Any of the above, done by any team member.
**Ask the agent:** "Run the bring-up check." / "Triage this crash."
**Tools used:** The skills in [`.github/skills/`](../../.github/skills): `bringup-check`, `crash-triage`, `build-flash-verify`, `board-port`
**What happens:** The skill gives the agent the standard steps, expectations and report format, so results are consistent between people and boards.
**Output:** The same structured report every time.
**Limits / safety:** Update the skills when the process changes. They are plain Markdown.

---

## Not covered yet

- CI / hardware-in-the-loop runs on self-hosted runners.
- JTAG/SWD probes (J-Link, OpenOCD): early-boot debugging before the UART works, live GDB.
- Binary file transfer (`.ulg`) and parameter file import/export.
