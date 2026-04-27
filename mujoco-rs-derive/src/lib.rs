//! Procedural macros for the [`mujoco-rs`](https://crates.io/crates/mujoco-rs) crate.

extern crate proc_macro;

mod three_way_merge;

use proc_macro::TokenStream;

/// Derives `ThreeWayMerge` for a struct with named fields.
///
/// This macro is an implementation detail of `mujoco-rs` and is not intended
/// for direct use outside of that crate.
#[proc_macro_derive(ThreeWayMerge)]
pub fn derive_three_way_merge(input: TokenStream) -> TokenStream {
    three_way_merge::derive(input)
}
