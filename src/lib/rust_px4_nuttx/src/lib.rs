#![no_std]
#![no_main]

pub mod nuttx;
pub mod px4;

pub use nuttx::alloc;
pub use px4::logger;
pub use px4_nuttx_macros::px4_module_main;

use nuttx::alloc::vec::Vec;
use nuttx::alloc::string::String;
use core::fmt::Write;
use core::ffi::CStr;

#[cfg(not(target_arch="x86_64"))]
use core::panic::PanicInfo;


#[cfg(not(target_arch="x86_64"))]
#[panic_handler]
fn panic(info: &PanicInfo<'_>) -> ! {
    let payload: &str = if let Some(s) = info.payload().downcast_ref::<&'static str>() {
        s
    } else if let Some(s) = info.payload().downcast_ref::<String>() {
        &s
    } else {
        "[unknown]"
    };
    let mut message = String::new();
    write!(message, "panicked at '{}'", payload).unwrap();
    if let Some(loc) = info.location() {
        write!(message, ", {}", loc).unwrap();
    }
    message.push('\0');
    logger::log_raw(
        logger::LogLevel::Panic,
        &message,
    );
    loop {}
}

pub fn init() {
    px4::logger::init();
}


#[doc(hidden)]
pub unsafe fn _run<F, R>(_modulename: &'static [u8], argc: u32, argv: *mut *mut u8, f: F) -> i32
where
	F: Fn(&[&str]) -> R + core::panic::UnwindSafe,
	R: MainStatusCode,
{
	logger::init();
	let mut args = Vec::with_capacity(argc as usize);
	for i in 0..argc {
		args.push(
			CStr::from_ptr(*argv.offset(i as isize) as *const i8)
				.to_str()
				.unwrap_or_else(|_| panic!("Invalid UTF-8 in arguments.")),
		);
	}
	f(&args).to_status_code()
}

/// The return type of your `#[px4_module_main]` function.
pub trait MainStatusCode {
	/// The status code to return.
	fn to_status_code(self) -> i32;

	/// The status code to return in case of a panic.
	///
	/// −1 by default.
	fn panic_status_code() -> i32 {
		-1
	}
}

/// Returns 0.
impl MainStatusCode for () {
	fn to_status_code(self) -> i32 {
		0
	}
}

/// Returns the `i32` itself.
impl MainStatusCode for i32 {
	fn to_status_code(self) -> i32 {
		self
	}
}

/// Returns 0 for `Ok`, and 1 for `Err`.
impl MainStatusCode for Result<(), ()> {
	fn to_status_code(self) -> i32 {
		match self {
			Ok(()) => 0,
			Err(()) => 1,
		}
	}
}

/// Returns 0 for `Ok`, and the `i32` itself for `Err`.
impl MainStatusCode for Result<(), i32> {
	fn to_status_code(self) -> i32 {
		match self {
			Ok(()) => 0,
			Err(s) => s,
		}
	}
}
