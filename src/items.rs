use alloc::boxed::Box;
use core::fmt;

#[derive(Clone, Debug, Default)]
pub struct Port {
    pub port_id: u8,
    pub slot_id: u8,
    pub parent: Option<Box<Port>>,
    pub is_low_or_full_speed: bool,
}

impl Port {
    pub fn new_from_root(port_id: u8) -> Self {
        Port {
            port_id,
            slot_id: 0,
            parent: None,
            is_low_or_full_speed: false,
        }
    }

    pub fn child_port(&self, port_id: u8) -> Self {
        Port {
            port_id,
            slot_id: 0,
            parent: Some(Box::new(self.clone())),
            is_low_or_full_speed: false,
        }
    }

    pub fn get_root_port_id(&self) -> u8 {
        match &self.parent {
            None => self.port_id,
            Some(parent) => parent.get_root_port_id(),
        }
    }

    pub fn construct_route_string(&self) -> u32 {
        let mut string = 0u32;
        let mut me = self;
        loop {
            match &me.parent {
                Some(p) => {
                    string <<= 4;
                    string |= me.port_id as u32;
                    me = p.as_ref();
                }
                None => return string,
            }
        }
    }
}

pub enum TransferBuffer<'a> {
    Read(&'a mut [u8]),
    Write(&'a [u8]),
    None
}

impl<'a> TransferBuffer<'a> {
    pub fn len(&self) -> usize {
        match &self {
            TransferBuffer::Read(r) => r.len(),
            TransferBuffer::Write(w) => w.len(),
            TransferBuffer::None => 0,
        }
    }

    pub fn clone_mut(&mut self) -> TransferBuffer<'_> {
        match self {
            TransferBuffer::Read(r) => TransferBuffer::Read(r),
            TransferBuffer::Write(w) => TransferBuffer::Write(w),
            TransferBuffer::None => TransferBuffer::None,
        }
    }
}

impl<'a> fmt::Debug for TransferBuffer<'a> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TransferBuffer::Read(s) => write!(f, "Read(len: {})", s.len()),
            TransferBuffer::Write(s) => write!(f, "Write(len: {})", s.len()),
            TransferBuffer::None => write!(f, "None"),
        }
    }
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub enum TransferDirection {
    HostToDevice = 0,
    DeviceToHost,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub enum RequestType {
    Standard = 0,
    Class,
    Vendor,
    Reserved,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub enum Recipient {
    Device = 0,
    Interface,
    Endpoint,
    Other,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub struct TypeTriple(pub TransferDirection, pub RequestType, pub Recipient);

impl TypeTriple {
    pub const fn encode(&self) -> u8 {
        ((self.0 as u8) << 7) | ((self.1 as u8) << 5) | ((self.2 as u8) << 0)
    }

    pub fn assert_is(self, value: u8) -> Self {
        assert_eq!(Into::<u8>::into(self), value);
        self
    }
}

impl Into<u8> for TypeTriple {
    fn into(self) -> u8 {
        self.encode()
    }
}

#[derive(Clone, Debug)]
pub enum EndpointType {
    Control,
    Isochronous,
    Bulk,
    Interrupt,
}

#[derive(Clone, Debug)]
pub enum Error {
    Str(&'static str),
}

/*
fn send_control_command(&mut self, slot_id: u8, request_type: TypeTriple, request: u8,
                            value: u16, index: u16, length: u16,
                            write_to_usb: Option<&[u8]>, read_from_usb: Option<&mut [u8]>)
                            -> Result<usize, Error>
 */

#[derive(Debug)]
pub struct ControlCommand<'a> {
    pub request_type: TypeTriple,
    pub request: u8,
    pub value: u16,
    pub index: u16,
    pub length: u16,
    pub buffer: TransferBuffer<'a>,
}
