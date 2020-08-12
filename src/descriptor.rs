#![allow(non_snake_case)]

use alloc::vec::Vec;
use core::fmt;
use modular_bitfield::prelude::*;

use crate::macros;

#[derive(Debug, Default, Copy, Clone)]
#[repr(C, packed)]
pub struct USBDeviceDescriptor {
    pub bLength: u8,
    pub bDescriptorType: u8,
    /// Version in bcd Major/Minor
    pub bcdUSB: u16,
    pub bDeviceClass: u8,
    pub bDeviceSubClass: u8,
    pub bDeviceProtocol: u8,
    pub bMaxPacketSize: u8,
    pub idVendor: u16,
    pub idProduct: u16,
    /// BCD Device Release Version
    pub bcdDevice: u16,
    pub iManufacturer: u8,
    pub iProduct: u8,
    pub iSerialNumber: u8,
    pub bNumConfigurations: u8,
}

const_assert_size!(USBDeviceDescriptor, 18);

impl USBDeviceDescriptor {
    pub fn get_max_packet_size(&self) -> u32 {
        match self.bMaxPacketSize {
            8 => 8,
            16 => 16,
            32 => 32,
            64 => 64,
            9 => 512,
            _ => panic!("oofed"),
        }
    }
}

#[derive(Debug, Clone)]
pub struct USBConfigurationDescriptorSet {
    pub config: USBConfigurationDescriptor,
    pub ifsets: Vec<USBInterfaceDescriptorSet>,
}

// NOT A Descriptor
#[derive(Debug, Clone)]
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

#[derive(Default, Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct USBInterfaceDescriptor {
    pub bLength: u8,
    pub bDescriptorType: u8,
    pub bInterfaceNumber: u8,
    pub bAlternateSetting: u8,
    pub bNumEndpoints: u8,
    pub bInterfaceClass: u8,
    pub bInterfaceSubClass: u8,
    pub bInterfaceProtocol: u8,
    pub iInterface: u8,
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
#[derive(Default, Debug, Copy, Clone)]
pub struct USBEndpointDescriptor {
    pub bLength: u8,
    pub bDescriptorType: u8,
    pub bEndpointAddress: u8,
    pub bmAttributes: u8,
    pub wMaxPacketSize: u16,
    pub bInterval: u8,
}
const_assert_size!(USBEndpointDescriptor, 7);

impl USBEndpointDescriptor {
    pub fn get_endpoint_id(&self) -> u8 {
        self.bEndpointAddress & 0xF
    }

    pub fn is_input(&self) -> bool {
        (self.bEndpointAddress & 0x80) != 0
    }

    pub fn transfer_type(&self) -> USBEndpointTransferType {
        unsafe { core::mem::transmute(self.bmAttributes & 0x3 as u8) }
    }
}

#[repr(C, packed)]
#[derive(Default, Debug, Copy, Clone)]
pub struct USBConfigurationDescriptor {
    pub bLength: u8,
    pub bDescriptorType: u8,
    pub wTotalLength: u16,
    pub bNumInterface: u8,
    pub bConfigurationValue: u8,
    pub iConfiguration: u8,
    pub bmAttributes: u8,
    pub bMaxPower: u8,
}

#[bitfield]
#[derive(Debug, Clone, Copy, Default)]
pub struct USBHubDescriptorHubCharacteristics {
    logical_pwr_switching_mode: B2,
    is_compound_device: bool,
    over_current_protection_mode: B2,
    tt_think_time: B2,
    port_indicator_support: bool,
    __reserved: B8,
}

#[derive(Debug, Default, Copy, Clone)]
#[repr(C, packed)]
pub struct USBHubDescriptor {
    pub bLength: u8,
    pub bDescriptorType: u8,
    pub bNbrPorts: u8,
    pub wHubCharacteristics: USBHubDescriptorHubCharacteristics,
    /// Power On to Power Good Time in 2ms interval
    pub bPwrOn2PwrGood: u8,
    pub bHubContrCurrent: u8,
    pub deviceRemovable: [u8; 32],
}

// #[repr(C, packed)]
// #[derive(Default, Copy, Clone)]
// pub struct USBStringDescriptor {
//     pub bLength: u8,
//     pub bDescriptorType: u8,
//     pub bString: [u16; 126],
// }
