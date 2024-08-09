#![no_std]
#![no_main]

extern crate px4_nuttx;
extern crate px4_nuttx_macros;

use log::{info, warn};
use px4_nuttx::px4_module_main;
use px4_nuttx::nuttx::net::UdpSocket;
use px4_nuttx::nuttx::time::hrt_time;
use px4_nuttx::alloc::vec::Vec;


#[px4_module_main]
pub fn run(_args: &[&str]) -> Result<(), ()> {
    info!("Hello from Rust module test!");

    let mut count: u32 = 0;
    for arg in _args {
        info!("  arg[{}]: {}", count, arg);
        count += 1;
    }

    if _args.len() > 1 {
        let mode: u32 = _args[1].parse::<u32>().unwrap_or(0);
        match mode {
            1 => vector_test(),
            2 => timer_test(),
            3 => udp_send_test(),
            _ => warn!("Invalid test. Use 1-3"),
        }

    } else {
        vector_test();
        timer_test();
        udp_send_test();
    }

    Ok(())
}

fn vector_test() {
    // Test vector
    info!("TEST: Vector");
    let mut v1: Vec<u8> = Vec::new();
    for i in 0..10 {
        v1.push(i+3);
    }
    info!("This is new vector: {:?}", v1);
}

fn timer_test() {
    // Test hrt_timer
    info!("TEST: timer");
    info!("System timer: {}", hrt_time());
}

fn udp_send_test() {
    // Test udp socket send
    info!("TEST: UDP socket send");
    let socket = UdpSocket::bind("192.168.200.101:12222").unwrap();
    info!("Created a socket which is bound to address 192.168.200.101:12222");
    socket.send_to(b"Hello from Rust!\n", "192.168.200.100:12223").unwrap();
    info!("Message sent to 192.168.200.100:12223");
}

