use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, Data, DeriveInput, Fields};

pub fn derive(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let ident = input.ident;

    let fields = match input.data {
        Data::Struct(s) => match s.fields {
            Fields::Named(fields) => fields.named,
            Fields::Unnamed(_) => {
                return syn::Error::new_spanned(
                    ident,
                    "ThreeWayMerge only supports structs with named fields",
                )
                .to_compile_error()
                .into();
            }
            Fields::Unit => {
                return syn::Error::new_spanned(
                    ident,
                    "ThreeWayMerge cannot be derived for unit structs",
                )
                .to_compile_error()
                .into();
            }
        },
        _ => {
            return syn::Error::new_spanned(
                ident,
                "ThreeWayMerge can only be derived for structs",
            )
            .to_compile_error()
            .into();
        }
    };

    let field_merges = fields.iter().map(|field| {
        let field_ident = field.ident.as_ref().unwrap();

        quote! {
            crate::viewer::ThreeWayMerge::merge(
                &mut self.#field_ident,
                &mut other.#field_ident,
                &mut other_prev.#field_ident,
            );
        }
    });

    let output = quote! {
        impl crate::viewer::ThreeWayMerge for #ident {
            fn merge(&mut self, other: &mut Self, other_prev: &mut Self) {
                #(#field_merges)*
            }
        }
    };

    output.into()
}
