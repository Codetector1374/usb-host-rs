use alloc::vec::Vec;
use core::fmt;

use crate::macros;

#[derive(Debug)]
#[repr(C)]
pub struct USBDeviceDescriptor {
    length: u8,
    pub descriptor_type: u8,
    /// Version in bcd Major/Minor
    pub version: u16,
    pub class: u8,
    pub sub_class: u8,
    pub protocol: u8,
    max_packet_size: u8,
    pub vid: u16,
    pub pid: u16,
    /// BCD Device Release Version
    pub dev_version: u16,
    pub manufacturer_index: u8,
    pub product_index: u8,
    pub serial_index: u8,
    pub config_count: u8,
}

impl USBDeviceDescriptor {
    pub fn get_max_packet_size(&self) -> u32 {
        match self.max_packet_size {
            8 => 8,
            16 => 16,
            32 => 32,
            64 => 64,
            _ => 1u32 << self.max_packet_size,
        }
    }
}

#[derive(Debug)]
pub struct USBConfigurationDescriptorSet {
    pub config: USBConfigurationDescriptor,
    pub ifsets: Vec<USBInterfaceDescriptorSet>,
}

// NOT A Descriptor
#[derive(Debug)]
pub struct USBInterfaceDescriptorSet {
    pub interface: USBInterfaceDescriptor,
    pub endpoints: Vec<USBEndpointDescriptor>,
}

impl USBInterfaceDescriptorSet {
    pub fn new(ifdesc: USBInterfaceDescriptor) -> Self {
        Self {
            interface: ifdesc,
            endpoints: Default::default(),
        }
    }
}

#[derive(Debug)]
#[repr(C)]
pub struct USBInterfaceDescriptor {
    length: u8,
    pub descriptor_type: u8,
    pub interface_number: u8,
    pub alt_set: u8,
    pub num_ep: u8,
    pub class: u8,
    pub sub_class: u8,
    pub protocol: u8,
    pub ifstr_index: u8,
}

#[repr(u8)]
#[derive(Debug, Copy, Clone)]
pub enum USBEndpointTransferType {
    Control = 0,
    Isochronous = 1,
    Bulk = 2,
    Interrupt = 3,
}

#[repr(C, packed)]
#[derive(Debug, Copy, Clone)]
pub struct USBEndpointDescriptor {
    length: u8,
    pub descriptor_type: u8,
    pub address: u8,
    pub attr: u8,
    pub max_packet_size: u16,
    pub interval: u8,
}
const_assert_size!(USBEndpointDescriptor, 7);

impl USBEndpointDescriptor {
    pub fn get_endpoint_id(&self) -> u8 {
        self.address & 0xF
    }

    pub fn is_input(&self) -> bool {
        (self.address & 0x80) != 0
    }

    pub fn transfer_type(&self) -> USBEndpointTransferType {
        unsafe { core::mem::transmute(self.attr & 0x3 as u8) }
    }
}

#[derive(Debug)]
#[repr(C)]
pub struct USBConfigurationDescriptor {
    length: u8,
    pub descriptor_type: u8,
    total_length: [u8; 2],
    pub num_interfaces: u8,
    pub config_val: u8,
    pub config_string: u8,
    pub attrs: u8,
    pub max_power: u8,
}

impl USBConfigurationDescriptor {
    pub fn get_total_length(&self) -> u16 {
        u16::from_le_bytes(self.total_length)
    }
}

#[repr(C)]
struct USBHubDescriptorData([u8; 64]);

impl Default for USBHubDescriptorData {
    fn default() -> Self {
        Self([0; 64])
    }
}

impl fmt::Debug for USBHubDescriptorData {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self.0.as_ref())
    }
}


#[derive(Debug, Default)]
#[repr(C)]
pub struct USBHubDescriptor {
    pub length: u8,
    pub descriptor_type: u8,
    pub num_ports: u8,
    hub_chars: [u8; 2],
    // Power On to Power Good Time
    pub potpgt: u8,
    max_hub_current: u8,

    // "removable" and "power control mask" bitfields
    data: USBHubDescriptorData,
}


