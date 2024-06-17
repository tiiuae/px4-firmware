#![no_std]
#![no_main]

extern crate alloc;
mod galloc;
mod nuttxsyslib;

#[macro_use]
extern crate nuttx_rs;

use alloc::vec::Vec;
use alloc::vec;
use nuttxsyslib::hrt_time;

#[no_mangle]  //  Don't mangle the function name
extern "C" fn rust_main() {  //  Declare `extern "C"` because it will be called by NuttX
    run();
}

fn run() {

    println!("INFO  [{:010}] Hello from Rust.", hrt_time());

    let v2: Vec<u8> = vec![1, 2, 3, 4, 5];
    println!("INFO  [{:010}] vector2 {:?}", hrt_time(), v2);
}
