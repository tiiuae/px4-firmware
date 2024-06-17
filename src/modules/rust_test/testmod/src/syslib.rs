
extern "C" {  //  Import C Function
    /// Print a message to the serial console (from C stdio library)
    fn puts(s: *const u8) -> i32;
  }

pub fn px4_print(msg: &str) {
    let mut buf: [u8; 128] = [0; 128];
    // Leave one byte for null termination
    let len = if msg.len() > 126 { 126 } else { msg.len() };

    for i in 0..len {
        buf[i] = msg.as_bytes()[i];
    }

    unsafe {
        puts(buf.as_ptr());
    }
}

extern "C" {
    fn hrt_absolute_time() -> u64;
}

pub fn hrt_time() -> u64 {
    unsafe {
        hrt_absolute_time()
    }
}
