//! USB Host Device Structures

use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicU32, Ordering};

use hashbrown::HashMap;
use spin::{Mutex, RwLock};

use crate::consts::{DESCRIPTOR_TYPE_ENDPOINT, USBSpeed};
use crate::descriptor::{USBConfigurationDescriptor, USBConfigurationDescriptorSet, USBDeviceDescriptor, USBEndpointDescriptor};
use crate::items::{ControlCommand, EndpointType, TransferBuffer};
use crate::{items, error};
use crate::traits::{USBHostController, USBMeta};

/// Describes a Generic USB Device
pub struct USBDevice {
    pub bus: Arc<RwLock<USBBus>>,
    pub addr: u32,
    pub port: u8,
    pub speed: USBSpeed,
    pub depth: u8,
    pub langid: u16,
    pub ddesc: USBDeviceDescriptor,
    pub config_desc: Option<USBConfigurationDescriptorSet>,

    pub parent: Option<Arc<RwLock<USBDevice>>>,
    pub def_ep: USBEndpoint,

    pub prv: Option<Box<dyn USBMeta>>,
}

impl USBDevice {
    pub fn new(parent: Option<Arc<RwLock<USBDevice>>>, bus: Arc<RwLock<USBBus>>, max_packet_size: u16, addr: u32, port: u8, speed: USBSpeed, depth: u8) -> Self {
        let mut device = Self {
            bus,
            addr,
            port,
            speed,
            depth,
            parent,
            config_desc: None,
            langid: 0,
            ddesc: USBDeviceDescriptor::default(),
            def_ep: USBEndpoint::default_ep(max_packet_size),
            prv: None,
        };
        device.ddesc.bMaxPacketSize = match max_packet_size {
            8 | 32 | 64 => max_packet_size as u8,
            512 => 9,
            _ => panic!("oofed"),
        };
        device
    }
}

pub struct USBBus {
    pub controller: Arc<dyn USBHostController>,
    pub devices: Vec<Option<Arc<RwLock<USBDevice>>>>,
}

impl USBBus {
    pub fn new(controller: Arc<dyn USBHostController>) -> Self {
        let mut devices = Vec::new();
        devices.resize(256, None);

        Self {
            controller,
            devices,
        }
    }

    pub fn get_new_addr(&self) -> Option<u32> {
        for i in 0..self.devices.len() {
            if self.devices[i].is_none() {
                return Some(i as u32);
            }
        }
        return None;
    }
}

pub struct USBEndpoint {
    pub descriptor: USBEndpointDescriptor,
}

impl USBEndpoint {
    pub fn default_ep(max_packet_size: u16) -> USBEndpoint {
        USBEndpoint {
            descriptor: USBEndpointDescriptor {
                bLength: core::mem::size_of::<USBEndpointDescriptor>() as u8,
                bDescriptorType: DESCRIPTOR_TYPE_ENDPOINT,
                bEndpointAddress: 0, // USB_CONTROL_ENDPOINT
                bmAttributes: 0, // CONTROL ENDPOINT
                wMaxPacketSize: max_packet_size,
                bInterval: 0,
            }
        }
    }
}


pub struct USBPipe {
    pub device: Arc<RwLock<USBDevice>>,
    pub controller: Arc<dyn USBHostController>,
    pub index: u8,
    pub endpoint_type: EndpointType,

    // true for control endpoints but control endpoints are bidirectional
    pub is_input: bool,
}

impl USBPipe {
    pub fn control_transfer(&self, command: ControlCommand) {
        assert!(matches!(self.endpoint_type, EndpointType::Control));
        self.controller.control_transfer(&self, command);
    }

    pub fn bulk_write(&self, buf: &[u8]) -> Result<usize, error::USBError> {
        assert!(matches!(self.endpoint_type, EndpointType::Bulk | EndpointType::Interrupt));
        assert!(!self.is_input);
        self.controller.bulk_transfer(&self, TransferBuffer::Write(buf))
    }

    pub fn bulk_read(&self, buf: &mut [u8]) -> Result<usize, error::USBError> {
        assert!(matches!(self.endpoint_type, EndpointType::Bulk | EndpointType::Interrupt));
        assert!(self.is_input);
        self.controller.bulk_transfer(&self, TransferBuffer::Read(buf))
    }
}

