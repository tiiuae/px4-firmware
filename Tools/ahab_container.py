#!/usr/bin/env python3
"""Wrap an application in a signed AHAB container for an i.MX9 ELE.

The container is what NXP mkimage writes for one A55 image, byte for byte;
NXP CST then signs it under the SRK whose hash the part trusts. The saluki
bootloader hands it to the ELE, which checks it before the application runs.
"""

import argparse
import hashlib
import os
import struct
import subprocess
import sys
import tempfile

HDR_TAG = 0x87
SIG_TAG = 0x90
IMAGE_OFFSET = 0x2000
IMAGE_ALIGN = 0x400
HDR_FLAGS = 0x10  # SRK set: OEM
IMG_FLAGS = 0x123  # executable, A55, SHA-384
IMG_META = 0x2
SIG_BLOCK_OFFSET = 0x90


def container(app: bytes, load: int, fuse_version: int) -> bytes:
    size = -(-len(app) // IMAGE_ALIGN) * IMAGE_ALIGN
    padded = app.ljust(size, b"\0")
    length = SIG_BLOCK_OFFSET + 0x10

    hdr = struct.pack("<BHBIHBBHH", 0, length, HDR_TAG, HDR_FLAGS, 0,
                      fuse_version, 1, SIG_BLOCK_OFFSET, 0)
    img = struct.pack("<IIQQII", IMAGE_OFFSET, size, load, load, IMG_FLAGS,
                      IMG_META)
    img += hashlib.sha384(padded).digest().ljust(64, b"\0") + bytes(32)
    sig = struct.pack("<BHB", 0, 0x10, SIG_TAG) + bytes(12)

    return (hdr + img + sig).ljust(IMAGE_OFFSET, b"\0") + padded


def sign(path: str, keys: str, cst: str, pkcs11: str = "") -> None:
    crts = os.path.join(keys, "bootloader", "crts")
    source = f"{crts}/SRK1_sha384_secp384r1_v3_usr_crt.pem"
    backend = []
    if pkcs11:
        # The key never leaves the token; CST signs through the pkcs11 engine.
        token, pin = pkcs11.split(",", 1)
        source = (f"pkcs11:token={token};object=./SRK1_sha384_secp384r1_v3_usr;"
                  f"type=cert;pin-value={pin}")
        backend = ["-b", "pkcs11"]
    csf = f"""[Header]
Target = AHAB
Version = 1.0

[Install SRK]
File = "{crts}/SRK_1_2_3_4_table.bin"
Source = "{source}"
Source index = 0
Source set = OEM
Revocations = 0x0

[Authenticate Data]
File = "{path}"
Offsets = 0x0 {SIG_BLOCK_OFFSET:#x}
"""
    with tempfile.NamedTemporaryFile("w", suffix=".csf", delete=False) as f:
        f.write(csf)

    try:
        # CST finds each private key beside its certificate, from the keys dir.
        r = subprocess.run([cst, *backend, "-i", f.name, "-o", path], cwd=keys,
                           capture_output=True, text=True)
        if r.returncode != 0:
            sys.exit(f"cst failed:\n{r.stdout}{r.stderr}")
    finally:
        os.unlink(f.name)


def main() -> int:
    p = argparse.ArgumentParser(description="Wrap an application in a signed AHAB container.")
    p.add_argument("--load", type=lambda s: int(s, 0), required=True,
                   help="load and entry address")
    p.add_argument("--fuse-version", type=int, default=0)
    p.add_argument("--keys", required=True,
                   help="key set directory holding bootloader/crts and keys")
    p.add_argument("--cst", required=True, help="NXP CST binary")
    p.add_argument("--pkcs11", default="",
                   help="TOKEN,PIN: sign with the SRK in this PKCS#11 token")
    p.add_argument("--unsigned", action="store_true",
                   help="write the container without signing it")
    p.add_argument("app")
    p.add_argument("out")
    a = p.parse_args()

    with open(a.app, "rb") as f:
        data = container(f.read(), a.load, a.fuse_version)

    with open(a.out, "wb") as f:
        f.write(data)

    if not a.unsigned:
        sign(os.path.abspath(a.out), os.path.abspath(a.keys),
             os.path.abspath(a.cst), a.pkcs11)

    return 0


if __name__ == "__main__":
    sys.exit(main())
