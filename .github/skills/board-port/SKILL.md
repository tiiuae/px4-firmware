---
name: board-port
description: Add or change hardware support in an existing boards/ssrc board directory (UART/serial port, I2C/SPI sensor, PWM/actuator output, GPIO, driver or module enablement), or copy a feature from one board to another. Use when the request is about editing board configuration rather than generic PX4 code.
---

# Board port and configuration changes

## Where things live (`boards/ssrc/<board>/`)

| Change | Files |
|---|---|
| Enable or disable a PX4 driver or module | `default.px4board` (or the label's `<label>.px4board`) |
| Start a sensor at boot (bus, address, rotation) | `init/rc.board_sensors` |
| Default parameters | `init/rc.board_defaults` |
| MAVLink instances and serial ports | `init/rc.board_mavlink`, `default.px4board` (`CONFIG_BOARD_SERIAL_*`) |
| Other services at boot | `init/rc.board_extras` |
| Bus definitions, chip selects, DRDY pins | `src/spi.cpp`, `src/i2c.cpp`, `src/board_config.h` |
| PWM and timer outputs | `src/timer_config.cpp`, `src/board_config.h`, SoC-specific `src/*_pwm.c` |
| Storage (NOR/eMMC/SD partitions) | `src/mtd.cpp`, SoC-specific `src/*_emmcsd.c` / `*_usdhc.c` / `*_spinor.c` |
| GPIOs, power enables, LEDs | `src/board_config.h`, `src/init.c`, `src/led.c`, SoC-specific bring-up (`src/imx9_bringup.c`, ...) |
| NuttX peripherals (UART, I2C, SPI enable, console, buffers) | `nuttx-config/nsh/defconfig` (and `flat/`, `amp/`, `hyperram/` variants if they exist) |
| Firmware metadata | `firmware.prototype` |

The SoC family changes how pins and peripherals are named: MPFS (`risc-v`), i.MX93 (`arm64`) or i.MXRT1189 (`arm`). Read `CONFIG_ARCH` and `CONFIG_ARCH_CHIP_*` in the defconfig, and copy the patterns already used by another board of the same family.

## Steps

1. **Find a reference.** Look for the same feature on another board with `grep` over `boards/ssrc/*/`, ideally a board with the same `CONFIG_ARCH_CHIP_*`. Show the user the diff between the reference board and the target board for the relevant files.
2. **Make the smallest consistent change.** Keep the Kconfig (`.px4board`/defconfig), the C/C++ board files and the `init/rc.board_*` scripts in sync. If the board has several NuttX configs (`nsh`, `flat`, `amp`, `hyperram`), update all of them that use the peripheral.
3. **Edit defconfigs carefully.** Keep the alphabetical order and don't add derived options. If the build container is available, check the result with the normal build. The NuttX `olddefconfig` step reports problems in `build/<target>/nuttx_olddefconfig.log`.
4. **Build** using the `build-flash-verify` skill. If the board is on the bench, flash and run the `bringup-check` skill to confirm the new device or port works: driver `status`, `listener`, `i2cdetect -b <bus>`, and `uorb top -1`.
5. **Summarize** the files changed, why, and what was verified on hardware versus only compile-tested.

## Rules

- Don't change other boards unless asked. If the same fix clearly applies to sibling boards, mention it.
- Keep changes generic. Prefer existing PX4 mechanisms (Kconfig, rc scripts, parameters) over board-specific code.
