# px4-firmware, `skunkworks`

TII's PX4 for the Saluki flight controllers. `skunkworks` is the shared line
for the secure link, secure OTA and trial-boot work: every submodule this
branch moves is pinned at its own `skunkworks`, and `.gitmodules` tracks it.

PX4 itself: <https://docs.px4.io>, <https://github.com/PX4/PX4-Autopilot>.
Design and proof: [RFC, secure MAVLink](https://github.com/tiiuae/ZTCS/blob/main/docs/rfc-secure-mavlink.md).

## What is where

| Path                               | Repository                                                                             | Carries                                |
| ---------------------------------- | -------------------------------------------------------------------------------------- | -------------------------------------- |
| `boards/ssrc/saluki-nxp93`         | [saluki-nxp93](https://github.com/tiiuae/saluki-nxp93/tree/skunkworks)                 | board config, NuttX config, rc scripts |
| `boards/ssrc/common`               | [px4_boards_ssrc](https://github.com/tiiuae/px4_boards_ssrc/tree/skunkworks)           | board code shared by the SSRC boards   |
| `platforms/nuttx/NuttX/nuttx`      | [nuttx](https://github.com/tiiuae/nuttx/tree/skunkworks)                               | NuttX with the i.MX93 enclave drivers  |
| `src/drivers/ssrc_crypto`          | [pfsoc_crypto](https://github.com/tiiuae/pfsoc_crypto/tree/skunkworks)                 | keystore-backed crypto                 |
| `src/modules/moi_agent`            | [px4_moi_agent](https://github.com/tiiuae/px4_moi_agent/tree/skunkworks)               | signed mode of operation               |
| `src/modules/px4_fw_update_client` | [px4-fw-update-client](https://github.com/tiiuae/px4-fw-update-client/tree/skunkworks) | OTA client                             |
| `src/lib/ztcs_secure_link`         | this repository, [README](src/lib/ztcs_secure_link/README.md)                          | the aircraft end of the secure link    |

On the ground: [px4-update-server](https://github.com/tiiuae/px4-update-server/tree/skunkworks),
[ZTCS](https://github.com/tiiuae/ZTCS) for the gateway,
[fmu-tools](https://github.com/tiiuae/fmu-tools) to flash, provision and update.

## Build

```sh
git clone -b skunkworks --recurse-submodules https://github.com/tiiuae/px4-firmware.git
cd px4-firmware
./build.sh out saluki-nxp93_default
```

| Output in `out/`                                      | Use                    |
| ----------------------------------------------------- | ---------------------- |
| `ssrc_saluki-nxp93_default-<version>.px4`             | `fmu flash`, `fmu ota` |
| `ssrc_saluki-nxp93_default-<version>.map`             | symbols                |
| `ssrc_saluki-nxp93_default_app_elfs-<version>.tar.gz` | ELFs for debugging     |

`build.sh` builds the `tii_px4_build` container and runs
`packaging/build_px4fw.sh` in it, which sets the signing variables and removes
the previous build of the target. `./build.sh` with no arguments lists the
targets: `saluki-{v1,v2,v3,pi,nxp93,micro,ft}_default`, their `_flat`, `_amp`
and `_custom_keys` variants, `fmu-v6xrt`, `pixhawk`.

Signing keys come from `SIGNING_ARGS`; unset, the test keys in
`Tools/saluki-sec-scripts/test_keys/` are used. `SIGNING_KEY=hsm` signs through
PKCS#11 instead.

## Do not

| Do not                                                           | Because                                                                                                 |
| ---------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------- |
| run `make` on the host, or `docker run` without `build_px4fw.sh` | without `SIGNING_TOOL` at configure time the image has no table of contents: it flashes and never boots |
| trust an incremental build after the module set or env changed   | the cached configure keeps the old signing path                                                         |
| build an image for OTA from a dirty tree                         | the version gets `-dirty`, which the update server cannot parse and `fmu ota` refuses                   |
| serve `_signed.bin` for OTA                                      | it has no table of contents, so the trial boot fails. `fmu ota` stages the right bytes from the `.px4`  |
| `git commit -a` here                                             | it sweeps every moved submodule pointer into the commit                                                 |
| point a submodule at a commit not on its `skunkworks`            | the pin stops being reproducible from the shared line                                                   |
| force-push `skunkworks`, here or in a submodule                  | it is shared                                                                                            |

## When the build fails

| Symptom                                                     | Cause, fix                                                                                                  |
| ----------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------- |
| `exec /ros_entrypoint.sh: resource temporarily unavailable` | `RLIMIT_NPROC` is per uid across the host. `build.sh` raises it; a hand-rolled `docker run` does not        |
| `The dependency target "rust_bindings" does not exist`      | `src/lib/rust_px4_nuttx` is not checked out: `git submodule update --init --recursive`                      |
| `olddefconfig` cannot find a `Kconfig`                      | a host build left absolute paths in `apps/*/Kconfig`: `git clean -xfd` in the `apps` and `nuttx` submodules |

## Change a submodule

1. Commit in the submodule on its `skunkworks` and push.
2. Here: `git add <path>`, commit, push `skunkworks`.

`git submodule update --remote <path>` pulls a submodule's `skunkworks` head.

## Then

| Step                               | Where                                                                                      |
| ---------------------------------- | ------------------------------------------------------------------------------------------ |
| flash, provision                   | [fmu-tools](https://github.com/tiiuae/fmu-tools)                                           |
| enrol, secure MAVLink, mode change | [ZTCS bench demo](https://github.com/tiiuae/ZTCS/tree/main/docs/demos/secure-mavlink)      |
| update over the link               | [ZTCS `ota.md`](https://github.com/tiiuae/ZTCS/blob/main/docs/demos/secure-mavlink/ota.md) |
