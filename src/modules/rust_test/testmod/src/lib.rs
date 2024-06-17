#![no_main]
#![no_std]  //  Use the Rust Core Library instead of the Rust Standard Library, which is not compatible with embedded systems
mod syslib;

use core::panic::PanicInfo;
use syslib::{px4_print, hrt_time};
use numtoa::NumToA;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

#[no_mangle]  //  Don't mangle the function name
extern "C" fn rust_main() {  //  Declare `extern "C"` because it will be called by NuttX
    run();
}

fn run() {
    let mut num_buffer = [0u8; 128];
    px4_print("Rust test app\n");

    let val: u64 = hrt_time();
    let val_str = val.numtoa_str(10, &mut num_buffer);

    px4_print("hrt_time: ");
    px4_print(val_str);
}
