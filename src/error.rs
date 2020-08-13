
#[derive(Debug, Clone, Eq, PartialEq)]
pub enum USBErrorKind {
    Other,
    InvalidArgument,
    NoFreeDeviceAddress,
    InvalidState,
    InvalidDescriptor,
    Timeout
}

impl USBErrorKind {
    pub fn msg(&self, msg: &'static str) -> USBError {
        USBError::new(self.clone(), msg)
    }

    pub fn err<R>(&self, msg: &'static str) -> Result<R, USBError> {
        Err(self.msg(msg))
    }
}

#[derive(Clone, Debug)]
pub struct USBError {
    pub kind: USBErrorKind,
    pub msg: &'static str,
}

impl USBError {
    pub fn new(kind: USBErrorKind, msg: &'static str) -> Self {
        USBError { kind, msg }
    }
}

pub type USBResult<R> = Result<R, USBError>;