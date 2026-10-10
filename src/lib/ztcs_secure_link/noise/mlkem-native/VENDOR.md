# mlkem-native, vendored

Upstream `pq-code-package/mlkem-native`, tag **v2.0.0**, Apache-2.0 OR ISC OR MIT.

## Why this one

| Wanted | mlkem-native |
| --- | --- |
| one implementation for both parts | C90 portable for the RT1176's Cortex-M7, AArch64 and Neon for the i.MX93's Cortex-A55 |
| no heap, no libc surprises | C90 plus `stdint.h` and a 64-bit `unsigned long long`; everything on the stack or static |
| the FC's own TRNG, not the library's | the deterministic API takes its coins as an argument, and `MLK_CONFIG_NO_RANDOMIZED_API` drops the rest so no `randombytes` has to exist |
| assurance to match the rest of the stack | every C file proved memory-safe and type-safe with CBMC; the AArch64 assembly proved functionally correct and constant-time with HOL-Light |
| not a one-off | the default ML-KEM in liboqs and AWS-LC, and in rustls through AWS-LC; under the Post-Quantum Cryptography Alliance |

The Rust side takes ML-KEM from RustCrypto's `ml-kem` instead. Two independent
implementations agreeing on the vectors is worth more than one shared.

## What was dropped

`mlkem/src/native/{x86_64,ppc64le,riscv64}`, backends for parts this project
will not run on. Nothing else is modified.

## How to update

Clone the new tag, copy its `mlkem/` and `LICENSE` here, delete those three
directories again, then run the vectors. Never patch in place: a local change
would be invisible the next time someone copies a release over it.

## How to build

One compilation unit, `mlkem/mlkem_native.c`, with `mlkem/` on the include
path and:

```
-DMLK_CONFIG_PARAMETER_SET=768
-DMLK_CONFIG_NAMESPACE_PREFIX=ztcs_mlkem
-DMLK_CONFIG_NO_RANDOMIZED_API
```

The AArch64 backend additionally needs `mlkem/mlkem_native_asm.S` and the
backend selected in the config header. It is off until it is measured on the
i.MX93.

## Measured, not quoted

Peak stack, x86-64, portable C, `-O2`, by painting a thread stack and reading
the high-water mark:

| Operation | Bytes |
| --- | --- |
| keypair_derand | 18 560 |
| enc_derand | 21 728 |
| dec | 22 880 |

Arm will be lower, but not by the factor that would make these fit the 8 KB
the FC's syscalls run on. The KEM therefore runs on a worker with a stack of
its own, sized from a measurement on each part rather than from this table.
