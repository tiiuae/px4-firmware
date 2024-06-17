//  Demo Program for Rust on NuttX
#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>

//  Rust Function defined in rust/src/lib.rs
void rust_main(void);

int rust_test_main(int argc, FAR char *argv[]) {
    rust_main();
    return 0;
}
