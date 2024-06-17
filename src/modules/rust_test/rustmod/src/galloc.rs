//use no_std_compat::alloc::{GlobalAlloc, Layout};
extern crate alloc;
use alloc::alloc::{GlobalAlloc, Layout};

#[global_allocator]
static ALLOCATOR: Gallocator = Gallocator;

extern "C" {  //  Import C Function
	fn aligned_alloc(align: usize, size: usize) -> *mut u8;
	fn free(p: *const u8);
}

struct Gallocator;
unsafe impl GlobalAlloc for Gallocator {
	unsafe fn alloc(&self, l: Layout) -> *mut u8 {
		unsafe {
			aligned_alloc(l.align(), l.size())
		}
	}
	unsafe fn dealloc(&self, p: *mut u8, _: Layout) {
		free(p);
	}
}
