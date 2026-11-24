---
name: crash-triage
description: Decode and explain a NuttX/PX4 crash (hard fault, exception, assertion, panic, segmentation fault) from the UART console or a pasted dump, using the matching ELF files from a local build or CI artifacts. Use when a board crashes, reboots unexpectedly or prints a fault dump.
---

# Crash triage

Goal: turn a fault dump into a call stack with source lines, a likely root cause and a fix proposal, without trusting results from a mismatched build.

## Steps

1. **Get the dump.**
   - Board on the bench: call `decode_crash(target=...)` with no `dump`. It takes the latest crash from the UART console buffer, which records continuously from `console_open`, so dumps printed during boot are kept too.
   - Pasted or logged dump: pass it as `dump`.
   - If you can't find a dump, use `console_read(pattern="Assertion|EXCEPTION|PANIC|fault|Segmentation", context=40)`.
2. **Pick the right ELFs.**
   - Local build: `target="<board>"`. The tool uses `<target>_kernel.elf` (kernel builds) or `<target>.elf` (flat builds), plus `bin_debug/<app>` for the faulting task (found from `task:`, `Segmentation fault in <name>` or `PID n: <name>`). If the task name doesn't match an app ELF, pass `app="<module>"`.
   - Released or CI image: pass `elf` as the `*_app_elfs-<version>.tar.gz` artifact (or an extracted directory) and `arch` (`risc-v`, `arm64` or `arm`). You can find the arch with `board_info(target)`.
3. **Check that the ELFs match.** The decode is only valid if the board ran exactly that build. With the board available, call `verify_firmware(target)`. Otherwise compare the version in the dump or report with the artifact version. Say clearly in the report if this couldn't be confirmed.
4. **Interpret the result.**
   - `registers` are authoritative: PC/EPC/ELR is where the fault happened, LR/RA/x30 is the caller.
   - `code_addresses` come from scanning every hex word in the dump (stack dump or backtrace). They show call-path candidates, but old values left on the stack can appear too. Prefer frames that form a plausible call chain.
   - Check the fault type: RISC-V `MCAUSE` (5 = load access fault, 7 = store access fault, 2 = illegal instruction); arm64 `ESR`/reason text; Arm Cortex-M CFSR bits.
   - For assertions, the summary line has the file and line. Read that code first.
5. **Read the code.** Open the resolved source lines, look for null pointers or uninitialised objects, stack overflow (SP near the stack base in the dump, `top once` stack headroom), use-after-free, and a wrong board configuration (bus, IRQ or DMA in `boards/<vendor>/<board>/`).
6. **Reproduce if needed.** Use `reboot` and `console_read` to check whether it happens again. For intermittent faults, see the "Intermittent issue statistics" use case in `Tools/px4-bench/USE_CASES.md`.

## Report format

- Fault type and faulting task.
- Decoded stack (innermost first): `function at file:line`, labelled kernel or `app:<name>`.
- Whether the build match was verified.
- Most likely root cause, with supporting evidence from the code.
- A suggested fix or the next debugging step.

## Notes

- Host machines usually don't have the cross toolchains. `decode_crash` falls back to the `tii_px4_build` container automatically.
- Source paths are reported relative to the repository, with the container prefix `/px4-firmware/sources` removed.
