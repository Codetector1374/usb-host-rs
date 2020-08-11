
#[macro_export]
macro_rules! const_assert_size {
    ($expr:tt, $size:tt) => {
    const _: fn(a: $expr) -> [u8; $size] = |a| unsafe { core::mem::transmute::<$expr, [u8; $size]>(a) };
    };
}

