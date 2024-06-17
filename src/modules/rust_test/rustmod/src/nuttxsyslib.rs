
extern "C" {
    fn hrt_absolute_time() -> u64;
}

pub fn hrt_time() -> u64 {
    unsafe {
        hrt_absolute_time()
    }
}
