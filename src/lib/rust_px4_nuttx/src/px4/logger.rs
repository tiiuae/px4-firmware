use log::{Metadata, Record};
use crate::nuttx::alloc::format;

#[allow(dead_code)]
extern "C" {
	fn printf(fmt: *const u8, ...) -> i32;
}

pub enum LogLevel {
	Debug = 0,
	Info = 1,
	Warn = 2,
	Error = 3,
	Panic = 4,
}

#[allow(dead_code)]
pub fn log_raw(level: LogLevel, message: &str) {
	let lvlstr = match level {
		LogLevel::Panic => "PANIC",
		LogLevel::Error => "ERROR",
		LogLevel::Warn => "WARN ",
		LogLevel::Info => "INFO ",
		LogLevel::Debug => "DEBUG",
	};
	let msg = format!("{} {}\n\0", lvlstr, message);
	unsafe {
		let _ = printf("%s\0".as_ptr(), msg.as_ptr());
	}
}

#[macro_export]
macro_rules! info_raw {
	($arg:expr) => (
		$crate::log_raw($crate::LogLevel::Info, $arg)
	);
	($($arg:tt)+) => (
		$crate::log_raw($crate::LogLevel::Info, &format!($($arg)+))
	);
}

struct Px4Logger;

impl log::Log for Px4Logger {
	fn enabled(&self, metadata: &Metadata) -> bool {
		metadata.level() <= log::Level::Info
	}

	fn log(&self, record: &Record) {
		if !self.enabled(record.metadata()) {
			return;
		}

		let level = match record.level() {
			log::Level::Error => "ERROR",
			log::Level::Warn => "WARN ",
			log::Level::Info => "INFO ",
			log::Level::Debug | log::Level::Trace  => "DEBUG",
		};

		let target = record.target();
		let message = format!("{} [{}] {}\0", level, target, record.args());

		unsafe {
			let _ = printf("%s\n\0".as_ptr(), message.as_ptr());
		}
	}

	fn flush(&self) {}
}

static LOGGER: Px4Logger = Px4Logger;

pub fn init() {
	/*
	unsafe {
	        printf(b"INFO  [rust_px4_nuttx] initializing\n\0".as_ptr());
	}
	*/
	if log::set_logger(&LOGGER).is_ok() {
		log::set_max_level(log::LevelFilter::Info);
	}
}
