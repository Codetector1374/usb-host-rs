
#[derive(Debug, Clone, Eq, PartialEq)]
pub enum USBError {
    InvalidArgument,
    NoFreeDeviceAddress,
}