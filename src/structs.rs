//! USB Host Device Structures

use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::ops::Deref;
use core::sync::atomic::{AtomicU32, Ordering};

use downcast_rs::DowncastSync;
use hashbrown::HashMap;
use spin::{Mutex, RwLock};

use crate::{error, items, USBErrorKind, USBResult};
use crate::consts::{DESCRIPTOR_TYPE_ENDPOINT, USBSpeed};
use crate::descriptor::{USBConfigurationDescriptor, USBConfigurationDescriptorSet, USBDeviceDescriptor, USBEndpointDescriptor};
use crate::items::{ControlCommand, EndpointType, TransferBuffer};
use crate::traits::{USBAsyncReadFn, USBHostController, USBMeta};
use crate::collection::SyncArray;

pub trait USBDeviceDriver: DowncastSync {
    fn on_attach(&self) -> USBResult<()> {
        Ok(())
    }

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
    pub bus: Arc<USBBus>,
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
    pub fn new(parent: Option<Arc<RwLock<USBDevice>>>, bus: Arc<USBBus>, max_packet_size: u16, addr: u32, port: u8, speed: USBSpeed, depth: u8) -> Self {
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

    pub fn attach_driver(device: &Arc<RwLock<USBDevice>>, driver: Arc<dyn USBDeviceDriver>) -> USBResult<()> {
        {
            let mut d = device.write();
            if !matches!(d.device_state, DeviceState::Idle) {
                return USBErrorKind::InvalidArgument.err("device is not idle");
            }
            d.device_state = DeviceState::Owned(driver.clone());
        }

        if let Err(e) = driver.on_attach() {
            let mut d = device.write();
            d.device_state = DeviceState::Idle;
            return Err(e);
        }

        Ok(())
    }

    pub fn get_driver<T: Deref<Target=USBDevice>>(val: T) -> Option<Arc<dyn USBDeviceDriver>> {
        if let DeviceState::Owned(v) = &val.device_state {
            Some(v.clone())
        } else {
            None
        }
    }

    pub fn handle_disconnect(&mut self) {
        match core::mem::replace(&mut self.device_state, DeviceState::Disconnected) {
            DeviceState::Idle => {},
            DeviceState::Owned(driver) => {
                driver.on_disconnect();
            }
            DeviceState::Disconnected => {
                error!("Disconnecting a device multiple times!");
                return;
            }
            _ => {},
        }

        self.bus.devices.replace(self.addr as usize, None);
    }
}

impl Drop for USBDevice {
    fn drop(&mut self) {
        self.bus.controller.free_slot(self.addr as u8);
    }
}

pub struct USBBus {
    pub controller: Arc<dyn USBHostController>,
    pub devices: SyncArray<Option<Arc<RwLock<USBDevice>>>>,
}

impl USBBus {
    pub fn new(controller: Arc<dyn USBHostController>) -> Self {
        Self {
            controller,
            devices: SyncArray::new(256),
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
    pub fn control_transfer(&self, command: ControlCommand) -> USBResult<()> {
        assert!(matches!(self.endpoint_type, EndpointType::Control));
        self.controller.control_transfer(&self, command)
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

