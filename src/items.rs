use alloc::boxed::Box;
use alloc::sync::Arc;
use core::fmt;

use modular_bitfield::prelude::*;
use spin::RwLock;

use crate::consts::USBSpeed;
use crate::structs::USBDevice;
use crate::traits::USBHostController;

pub enum TransferBuffer<'a> {
    Read(&'a mut [u8]),
    Write(&'a [u8]),
    None,
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

#[derive(Clone, Debug, Eq, PartialEq, Hash)]
pub enum EndpointType {
    Control,
    Isochronous,
    Bulk,
    Interrupt,
}

#[derive(Debug)]
pub struct ControlCommand<'a> {
    pub request_type: TypeTriple,
    pub request: u8,
    pub value: u16,
    pub index: u16,
    pub length: u16,
    pub buffer: TransferBuffer<'a>,
}

#[bitfield]
#[derive(Clone, Default)]
pub struct PortStatus {
    device_connected: bool,
    port_enable: bool,
    suspend: bool,
    over_current: bool,
    reset: bool,
    __res0: B3,
    port_power: bool,
    low_speed: bool,
    // low speed bit takes precedence over high speed bit
    high_speed: bool,
    port_test: bool,
    port_indicator: bool,
    __res1: B3,
    change_device_connected: bool,
    change_port_enable: bool,
    change_suspend: bool,
    change_over_current: bool,
    change_reset: bool,
    __res2: B11,
}

impl fmt::Debug for PortStatus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "PortStatus[")?;
        
        let fields = [
            ("device_connected", self.get_device_connected()),
            ("port_enable", self.get_port_enable()),
            ("suspend", self.get_suspend()),
            ("over_current", self.get_over_current()),
            ("reset", self.get_reset()),
            ("port_power", self.get_port_power()),
            ("low_speed", self.get_low_speed()),
            ("high_speed", self.get_high_speed()),
            ("port_test", self.get_port_test()),
            ("port_indicator", self.get_port_indicator()),
            ("change_device_connected", self.get_change_device_connected()),
            ("change_port_enable", self.get_change_port_enable()),
            ("change_suspend", self.get_change_suspend()),
            ("change_over_current", self.get_change_over_current()),
            ("change_reset", self.get_change_reset()),
        ];

        let mut is_first = true;
        for (name, val) in fields.iter() {
            if *val {
                if !is_first {
                    write!(f, ", ")?;
                }
                is_first = false;
                write!(f, "{}", name)?;
            }
        }

        write!(f, "]")?;
        Ok(())
    }
}

const_assert_size!(PortStatus, 4);

impl PortStatus {
    pub fn get_speed(&self) -> USBSpeed {
        if self.get_low_speed() {
            USBSpeed::Low
        } else if self.get_high_speed() {
            USBSpeed::High
        } else {
            USBSpeed::Full
        }
    }
}

