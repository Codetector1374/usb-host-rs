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
use crate::{items, error, USBResult};
use crate::traits::{USBHostController, USBMeta, USBAsyncReadFn};
use downcast_rs::DowncastSync;

pub trait USBDeviceDriver: DowncastSync {

    fn on_disconnect(&self) {}

}

impl_downcast!(sync USBDeviceDriver);

pub enum DeviceState {
    Unconfigured,
    Idle,
    Owned(Arc<dyn USBDeviceDriver>),
    Disconnected,
}

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

    pub device_state: DeviceState,
    pub user_meta: Option<Box<dyn USBMeta>>,
    pub protocol_meta: Option<Box<dyn USBMeta>>,
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
            device_state: DeviceState::Unconfigured,
            user_meta: None,
            protocol_meta: None,
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
    pub max_packet_size: usize,

    // true for control endpoints but control endpoints are bidirectional
    pub is_input: bool,
}

impl USBPipe {
    pub fn control_transfer(&self, command: ControlCommand) {
        assert!(matches!(self.endpoint_type, EndpointType::Control));
        self.controller.control_transfer(&self, command);
    }

    pub fn bulk_write(&self, buf: &[u8]) -> USBResult<usize> {
        assert!(matches!(self.endpoint_type, EndpointType::Bulk));
        assert!(!self.is_input);
        self.controller.bulk_transfer(&self, TransferBuffer::Write(buf))
    }

    pub fn bulk_read(&self, buf: &mut [u8]) -> USBResult<usize> {
        assert!(matches!(self.endpoint_type, EndpointType::Bulk));
        assert!(self.is_input);
        self.controller.bulk_transfer(&self, TransferBuffer::Read(buf))
    }

    pub fn async_read(&self, buf: Vec<u8>, int_callback: USBAsyncReadFn) -> USBResult<()> {
        self.controller.async_read(&self, buf, int_callback)
    }
}

