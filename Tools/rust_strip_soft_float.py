#!/usr/bin/env python3
"""
Remove archive members with a smaller RISC-V float ABI than the rest of a Rust staticlib.

The precompiled compiler_builtins of riscv64gc-unknown-none-elf ships a few C-built
intrinsics (e.g. __popcountdi2) labelled with the soft-float ABI, while the rest of the
library (and PX4) uses lp64d. GNU ld refuses to link such objects together:
"can't link soft-float modules with double-float modules".

Dropping these members lets the linker take the intrinsics from libgcc instead, which
is built for the ABI used by PX4. Any intrinsic libgcc does not provide shows up as a
regular undefined reference.
"""

import argparse
import os
import struct
import subprocess
import sys
import tempfile

EM_RISCV = 243
EF_RISCV_FLOAT_ABI = 0x6


def riscv_float_abi(path):
    """Return the float ABI bits of a RISC-V ELF64 object, or None for anything else."""
    with open(path, 'rb') as f:
        header = f.read(64)

    if len(header) < 64 or header[:4] != b'\x7fELF' or header[4] != 2 or header[5] != 1:
        return None

    e_machine, = struct.unpack_from('<H', header, 18)
    e_flags, = struct.unpack_from('<I', header, 48)

    if e_machine != EM_RISCV:
        return None

    return e_flags & EF_RISCV_FLOAT_ABI


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('archive', help='static library to fix in place')
    parser.add_argument('--ar', default='ar', help='ar tool of the target toolchain')
    args = parser.parse_args()

    archive = os.path.abspath(args.archive)

    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run([args.ar, 'x', archive], cwd=tmp, check=True)

        abis = {}

        for name in sorted(os.listdir(tmp)):
            abi = riscv_float_abi(os.path.join(tmp, name))

            if abi is not None:
                abis[name] = abi

        if not abis:
            return 0

        wanted = max(abis.values())
        stale = [name for name, abi in abis.items() if abi < wanted]

        for i in range(0, len(stale), 64):
            subprocess.run([args.ar, 'ds', archive] + stale[i:i + 64], check=True)

        if stale:
            print(f"{os.path.basename(archive)}: removed {len(stale)} members with a smaller float ABI")

    return 0


if __name__ == '__main__':
    sys.exit(main())
