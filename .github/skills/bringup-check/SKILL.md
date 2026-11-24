---
name: bringup-check
description: Check the health of a PX4/NuttX flight controller on the bench (boot log, drivers, sensors, buses, CPU/RAM) and report what is missing or broken. Use for first boot of new hardware, after board config changes, or when asked whether a board "works". Works with any boards/ssrc target and uses the px4-bench MCP tools.
---

# Bring-up check

Goal: a short report of what works, what's missing and what's suspicious on the connected board, compared against what the board's own startup scripts expect. Don't use a hard-coded per-board list.

## Steps

1. **Identify the target.** Ask the user which board is connected if it's unclear. Call `board_info(target)` for arch, kernel/flat build and the local build hash.
2. **Learn what to expect** from the repository for that board (`boards/<vendor>/<board>/`):
   - `init/rc.board_sensors` lists the sensor drivers started, with their buses and addresses.
   - `init/rc.board_defaults`, `init/rc.board_mavlink` and `init/rc.board_extras` cover the other services.
   - `default.px4board` (or the label's `.px4board`) lists the enabled drivers and modules.
   - `src/board_config.h` gives bus numbers and GPIOs when needed.
3. **Get the boot log.** If the board was just flashed or rebooted, use the returned `boot_log_offset` with `console_read(since=...)`. Otherwise, with the user's agreement, call `reboot()`. Look at `boot.issue_lines` and `crash_detected`. If a crash is detected, switch to the `crash-triage` skill.
4. **Snapshot** with `nsh_batch`. These commands exist on all Saluki builds:
   ```
   ver all
   free
   ps
   top once
   work_queue status
   uorb top -1
   sensors status
   perf
   mtd status
   ```
   Add `dmesg` (RAMLOG syslog, enabled on most boards) and `mft query` if relevant. Use `nsh_exec` with a longer timeout for slow commands.
5. **Check each expected device:**
   - Started? Its `<driver> status` succeeds and it appears in `ps`, `work_queue status` and `uorb top -1`.
   - Publishing at a sane rate? Use `uorb top -1` and `listener <topic> -n 1` (e.g. `sensor_accel`, `sensor_gyro`, `sensor_baro`, `sensor_mag`, `sensor_gps`).
   - If it's missing, look on the bus: `i2cdetect -b <bus>` for I2C devices, and the driver's start error in the boot log for SPI devices.
6. **Check resources.** Free RAM from `free`, CPU load and idle from `top once`, and stack headroom per task from `top once` / `ps`. Flag tasks with less than ~300 bytes of free stack, and CPU load above ~70% at idle.
7. **Firmware identity.** Call `verify_firmware(target)` and report it if the hash differs from the local build.

## Report format

- **Summary:** one line, e.g. "Boots to shell; 2 of 7 expected sensors missing; no crashes."
- **Table:** expected item | evidence | status (OK / missing / degraded).
- **Boot log issues:** quote the relevant lines.
- **Resources:** RAM free, CPU idle, the tightest stacks.
- **Next steps:** concrete hypotheses, such as wrong bus or address in `rc.board_sensors`, a missing driver in `.px4board`, or a pin mux issue in `board_config.h`.

## Rules

- Read-only by default. Don't run `param set`, `actuator_test`, `motor_test`, storage writes or `commander` state changes without asking the user. The tools enforce this with `confirm`.
- Never run `top` without `once`, or `listener` without `-n`.
- Base expectations only on files from the repository, so the check works the same for every board.
