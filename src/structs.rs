//! USB Host Device Structures

use alloc::boxed::Box;
use alloc::sync::Arc;

use spin::{RwLock, Mutex};

use crate::traits::{USBHostController, USBMeta, USBPipe};
use crate::descriptor::{USBDeviceDescriptor, USBEndpointDescriptor};
use hashbrown::HashMap;
use core::sync::atomic::{AtomicU32, Ordering};
use crate::consts::{DESCRIPTOR_TYPE_ENDPOINT, USBSpeed};

/// Describes a Generic USB Device
pub struct USBDevice {
    pub bus: Arc<RwLock<USBBus>>,
    pub addr: u32,
    pub speed: USBSpeed,
    pub depth: u8,
    pub langid: u16,

    pub def_ep: USBEndpoint,

    pub prv: Option<Box<dyn USBMeta>>,
}

impl USBDevice {
    pub fn new(bus: Arc<RwLock<USBBus>>, max_packet_size: u16, addr: u32, speed: USBSpeed, depth: u8) -> Self {
        Self {
            bus,
            addr,
            speed,
            depth,
            langid: 0,
            def_ep: USBEndpoint::default_ep(max_packet_size),
            prv: None,
        }
    }
}

pub struct USBBus {
    pub controller: Arc<dyn USBHostController>,
    pub devices: Vec<Option<Arc<Mutex<USBDevice>>>>,
}

impl USBBus {
    pub fn get_new_addr(&self) -> Option<u32> {
        for i in 0..self.devices.len() {
            if self.devices[i].is_none() {
                return Some(i as u32);
            }
        }
        return None
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