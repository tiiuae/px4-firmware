# Copilot instructions for tiiuae/px4-firmware

This is TII's fork of PX4 that builds NuttX/PX4 images for in-house flight controller hardware.

## Targets
- Custom boards live in `boards/ssrc/<board>/`. Build target names are `ssrc_<board>_<label>`, e.g. `ssrc_saluki-pi_default`. `build.sh` takes `<board>_<label>` (e.g. `saluki-pi_default`).
- The CPU architecture comes from `CONFIG_ARCH` in `boards/ssrc/<board>/nuttx-config/nsh/defconfig`:
  - `risc-v` (Microchip PolarFire SoC: saluki-v1/v2/v3/pi) uses the `riscv-none-elf-` toolchain.
  - `arm64` (i.MX93: saluki-nxp93/micro/ft/pro) uses `aarch64-none-elf-`.
  - `arm` (i.MXRT1189: salukini) uses `arm-none-eabi-`.
- Most boards use `CONFIG_BUILD_KERNEL=y`, where userspace apps are separate ELFs. Debug ELFs per app are in `build/<target>/bin_debug/<app>`, the kernel is `build/<target>/<target>_kernel.elf`, and flat builds have only `build/<target>/<target>.elf`.
- Tooling must stay generic across all `boards/ssrc/*` boards. Do not add board-specific special cases unless a board really needs one.

## Board directory layout
- `default.px4board` (plus variants like `flat.px4board`): the PX4 modules/drivers enabled for the build (Kconfig).
- `nuttx-config/<config>/defconfig`: NuttX configuration (console UART, USB CDC-ACM, prompt `CONFIG_NSH_PROMPT_STRING="saluki> "`).
- `src/board_config.h`, `src/*.c|cpp`: GPIOs, buses (`spi.cpp`, `i2c.cpp`), timers/PWM, board init.
- `init/rc.board_sensors`, `rc.board_defaults`, `rc.board_mavlink`, `rc.board_extras`: startup scripts. `rc.board_sensors` lists which sensors are expected.
- `firmware.prototype`: board metadata for `.px4` packaging.

## Building
- Builds run inside the `tii_px4_build` Docker image (`packaging/Dockerfile.build_env`); host machines usually don't have the cross toolchains.
- Full release-style build: `./build.sh <out-dir> <board>_<label>`. It rebuilds the image and the target from clean.
- Inside the container, `./packaging/build_px4fw.sh ssrc_<board>_<label>` builds and signs with the board's test keys. Set `PX4_INCREMENTAL=1` to keep the existing build directory.
- Outputs: `build/<target>/<target>.px4` (flashable), `.elf`/`_kernel.elf`, `.map`, `bin_debug/`, and `src/lib/version/build_git_version.h` (git hash).

## Flashing
- Saluki boards are flashed with `px_uploader.py` from the separate `saluki_bootloader_v2` repository (`tools/px_uploader.py`). **Do not use `Tools/px_uploader.py` from this repo for Saluki boards**; it is the upstream PX4 uploader and is not compatible.

## Hardware bench tooling
- `Tools/px4-bench/` is an MCP server for working with a board on the bench: UART console capture, NSH commands, flash, reboot, container builds and crash decoding. Setup is in `Tools/px4-bench/README.md`, workflows in `Tools/px4-bench/USE_CASES.md`.
- Skills in `.github/skills/` (`bringup-check`, `crash-triage`, `build-flash-verify`, `board-port`) describe the standard workflows.
- Safety: never run commands that change parameters, storage, vehicle state or actuators without asking the user. Assume propellers may be attached.
