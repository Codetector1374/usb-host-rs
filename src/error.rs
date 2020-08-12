
#[derive(Debug, Clone, Eq, PartialEq)]
pub enum USBError {
    InvalidArgument,
    NoFreeDeviceAddress,
    InvalidState,
    Timeout
}