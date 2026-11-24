---
name: build-flash-verify
description: Build a PX4/NuttX target in the tii_px4_build container, flash it with the Saluki uploader, capture the boot log and confirm the board runs exactly that build. Use for the everyday edit-build-flash loop, or before any hardware test whose result depends on the running firmware.
---

# Build, flash, verify

## Steps

1. **Resolve the target.** Short names work, e.g. `saluki-pi` becomes `ssrc_saluki-pi_default`, and `saluki-v3_flat` becomes `ssrc_saluki-v3_flat`. Use `board_info(target)` if unsure.
2. **Build** with `build(target)`.
   - The default is an incremental build (`PX4_INCREMENTAL=1`). Use `incremental=false` after Kconfig/defconfig changes or strange build errors.
   - If `status` is `failed`, report the `errors` lines and the end of `log_file`. Fix the compile errors and build again. Don't flash.
3. **Open the console** (`console_open`, or automatically if configured) before flashing, so the boot log is captured from the first byte.
4. **Flash** with `flash(target=target)`, or `flash(file=...)` for a CI `.px4`.
   - This uses the Saluki `px_uploader.py` from `saluki_bootloader_v2`, never `Tools/px_uploader.py`.
   - If it fails, show `uploader_output` and `hint`. Common causes: no USB connection, sudo needed for the block device, or the wrong uploader path in `bench.toml`.
   - Writing the bootloader or keystore is not available through the tool on purpose. Tell the user to do it by hand.
5. **Check the boot.** `flash` returns `prompt_seen`, a `boot` summary and `boot_log_offset`. Read the full log with `console_read(since=boot_log_offset)` if there are issues. If `crash_detected` is set, continue with the `crash-triage` skill.
6. **Verify identity** with `verify_firmware(target)`. Treat `match=false` as a failure: the board did not boot the new image (a wrong partition, a failed flash, or a stale build).
7. **Report** the build duration, flash result, boot issues and the hash match, and say what to do next (for example, run the `bringup-check` skill).

## Notes

- The build matches `build.sh` (same container and signing with the board's test keys), so the images are comparable to CI builds.
- Builds can take several minutes. Tell the user before starting a clean build.
