
#[macro_export]
macro_rules! const_assert_size {
    ($expr:tt, $size:tt) => {
    const _: fn(a: $expr) -> [u8; $size] = |a| unsafe { core::mem::transmute::<$expr, [u8; $size]>(a) };
    };
}

#[macro_export]
macro_rules! request_type {
    ($dir:ident, $typ:ident, $rec:ident) => {
        $crate::items::TypeTriple($crate::items::TransferDirection::$dir, $crate::items::RequestType::$typ, $crate::items::Recipient::$rec)
    };
}

#[macro_export]
macro_rules! request_type_u8 {
    ($dir:ident, $typ:ident, $rec:ident) => {{
        $crate::items::TypeTriple($crate::items::TransferDirection::$dir, $crate::items::RequestType::$typ, $crate::items::Recipient::$rec).encode()
    }};
}