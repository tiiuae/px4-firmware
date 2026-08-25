{
  description = "PX4 Autopilot development environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";

    # Rust toolchains with arbitrary target support. Needed because this fork
    # builds Rust modules (assembly_agent, enroll_agent, moi_agent) for the
    # bare-metal aarch64-unknown-none target, and nixpkgs' rustc ships rust-std
    # only for bpf/wasm/x86_64-linux.
    fenix = {
      url = "github:nix-community/fenix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      fenix,
      ...
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ fenix.overlays.default ];
        };

        # Rust target triple comes from
        # platforms/nuttx/cmake/Toolchain-aarch64-none-elf.cmake:33
        # (set(CMAKE_RUST_TARGET aarch64-unknown-none)). No toolchain version is
        # pinned for aarch64 -- only the riscv toolchain file pins +1.81.0 -- so
        # plain stable is what `cargo` resolves to here.
        rustToolchain = pkgs.fenix.combine [
          pkgs.fenix.stable.cargo
          pkgs.fenix.stable.rustc
          pkgs.fenix.targets.aarch64-unknown-none.stable.rust-std
        ];

        # Nix owns the Python interpreter.
        # uv owns the Python packages inside .venv.
        python = pkgs.python312;

        nativeTools = with pkgs; [
          # General PX4 build tools
          git
          cmake
          ninja
          gnumake
          ccache
          clang-tools
          astyle
          cppcheck
          file
          rsync
          shellcheck
          unzip
          zip

          # Build/debug tooling
          gdb
          lcov
          openssl
          libxml2

          # ARM / NuttX toolchain (Cortex-M/R targets: arm-none-eabi-*)
          gcc-arm-embedded

          # AArch64 bare-metal toolchain (aarch64-none-elf-*), required by the
          # i.MX93 / Cortex-A55 NuttX boards such as ssrc_saluki-nxp93_*.
          # binutils adds the prefixed objcopy/size/ar/nm that the NuttX link
          # and image steps invoke.
          #
          # Pinned to GCC 13 deliberately. PX4 builds this tree with -Werror,
          # and each newer GCC adds a warning that vendored submodule code
          # trips on:
          #
          #   GCC 15  -Wunterminated-string-initialization
          #           NuttX crypto/chacha_private.h:61
          #             static const char sigma[16] = "expand 32-byte k";
          #
          #   GCC 14  -Wcalloc-transposed-args
          #           libatt/ree/gp/att_ca.c:133
          #             node = libatt_calloc(sizeof(prv_session_t), 1);
          #
          # Both live in submodules (platforms/nuttx/NuttX, src/lib/libatt/libatt),
          # so the fix is to use the compiler generation this tree was written
          # against rather than to suppress the diagnostics or patch vendored
          # code. Bump this when those submodules are updated.
          pkgsCross.aarch64-embedded.buildPackages.gcc13
          pkgsCross.aarch64-embedded.buildPackages.binutils

          # Rust bridge codegen: src/lib/rust_px4_nuttx/nuttx_bindings/gen_bindings.sh
          # invokes `bindgen` directly.
          rust-bindgen

          # NuttX build dependencies
          automake
          bison
          flex
          genromfs
          xxd # ROMFS/CMakeLists.txt:403 hard-fails without it
          gettext
          gperf
          kconfig-frontends

          libelf
          expat.dev
          gmp.dev
          isl
          libmpc
          mpfr.dev
          ncurses.dev
          zlib
        ];

        px4PythonSetup = pkgs.writeShellApplication {
          name = "px4-python-setup";

          runtimeInputs = [
            pkgs.coreutils
            pkgs.git
            pkgs.gnused
            pkgs.uv
          ];

          text = ''
            set -euo pipefail

            root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
            requirements="$root/Tools/setup/requirements.txt"

            if [ ! -f "$requirements" ]; then
              echo "error: cannot find Tools/setup/requirements.txt" >&2
              echo "run this command from inside the PX4 checkout" >&2
              exit 1
            fi

            cd "$root"

            if [ ! -x .venv/bin/python ]; then
              echo "Creating .venv with Nix Python..."
              uv venv \
                --python "$UV_PYTHON" \
                .venv
            fi

            #
            # Two compatibility fixes are needed against modern tooling:
            #
            # 1. Older PX4 revisions contain:
            #
            #      matplotlib>=3.0.*
            #
            #    This is invalid under modern PEP 440 parsing. Current uv
            #    correctly rejects it.
            #
            # 2. requirements.txt pins only setuptools>=39.2.0, which now
            #    resolves to setuptools 81+. setuptools 81 removed
            #    pkg_resources, and the vendored UAVCAN dsdl compiler
            #    (src/drivers/uavcan/libuavcan/.../pyuavcan/dronecan) still
            #    imports it, so the build fails at "Running dsdl compiler".
            #    Cap it below 81.
            #
            # Don't modify PX4's tracked requirements.txt. Instead create
            # a temporary compatibility copy for uv.
            #
            patched_requirements="$(mktemp)"
            trap 'rm -f "$patched_requirements"' EXIT

            sed \
              -e 's/^matplotlib>=3\.0\.\*$/matplotlib>=3.0/' \
              -e 's/^setuptools>=\(.*\)$/setuptools>=\1,<81/' \
              "$requirements" \
              > "$patched_requirements"

            if grep -q '^matplotlib>=3\.0\.\*$' "$requirements"; then
              echo "Applying compatibility fix:"
              echo "  matplotlib>=3.0.* -> matplotlib>=3.0"
              echo
            fi

            if grep -q '^setuptools>=' "$requirements"; then
              echo "Applying compatibility fix:"
              echo "  setuptools -> <81 (pkg_resources removed in 81)"
              echo
            fi

            echo "Installing PX4 Python requirements..."
            uv pip install \
              --python .venv/bin/python \
              --requirement "$patched_requirements"

            echo
            echo "Checking Python environment..."
            uv pip check \
              --python .venv/bin/python

            echo
            echo "PX4 Python environment is ready."
          '';
        };
      in
      {
        devShells.default = pkgs.mkShell {
          packages = nativeTools ++ [
            python
            rustToolchain
            pkgs.uv
            px4PythonSetup
          ];

          env = {
            UV_PYTHON = python.interpreter;
            UV_PYTHON_DOWNLOADS = "never";
            UV_PROJECT_ENVIRONMENT = ".venv";
            PYTHONNOUSERSITE = "1";

            CMAKE_EXPORT_COMPILE_COMMANDS = "ON";

            # This PX4 revision contains subprojects declaring CMake compatibility
            # older than 3.5. CMake 4 removed those compatibility modes.
            #
            # CMake explicitly provides this variable for consumers building older
            # projects without modifying their CMakeLists.txt.
            CMAKE_POLICY_VERSION_MINIMUM = "3.5";
          };

          shellHook = ''
            #
            # Creating the venv is cheap and doesn't install anything.
            # Dependency installation remains explicit:
            #
            #   px4-python-setup
            #
            if [ ! -x .venv/bin/python ]; then
              echo "Creating PX4 .venv..."

              uv venv \
                --python "$UV_PYTHON" \
                .venv >/dev/null
            fi

            source .venv/bin/activate

            echo "PX4 development shell"
            echo "  ARM GCC:     $(arm-none-eabi-gcc --version | head -1)"
            echo "  AArch64 GCC: $(aarch64-none-elf-gcc --version | head -1)"
            echo "  Rust:        $(rustc --version) / $(cargo --version)"
            echo "  bindgen:     $(bindgen --version)"
            echo "  Python:      $(python --version)"
            echo "  uv:          $(uv --version)"
            echo
            echo "First time / requirements changed:"
            echo "  px4-python-setup"
            echo
            echo "Then try:"
            echo "  make px4_sitl"
            echo "  make <board>_default"
          '';
        };
      }
    );
}