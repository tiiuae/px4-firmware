use proc_macro::TokenStream;
use proc_macro2::{Ident, Span};
use quote::quote;
use syn::parse_macro_input;

pub fn px4_module_main(attr: TokenStream, input: TokenStream) -> TokenStream {
	if !attr.is_empty() {
		panic!("px4_module_main does not take any arguments");
	}
	let crate_name = std::env::var("CARGO_PKG_NAME").unwrap();
	let main_fndef = Ident::new(&format!("rust_{}_main", crate_name), Span::call_site());

	let fndef = parse_macro_input!(input as syn::ItemFn);
	let name = &fndef.sig.ident;
	let expanded = quote! {
		#fndef

		#[no_mangle]
		pub extern "C" fn #main_fndef(argc: u32, argv: *mut *mut u8) -> i32 {
			unsafe { px4_nuttx::_run(concat!(module_path!(), "\0").as_bytes(), argc, argv, #name) }
		}
	};
	expanded.into()
}
