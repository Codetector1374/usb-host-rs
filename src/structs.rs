//! USB Host Device Structures

use alloc::boxed::Box;
use alloc::sync::Arc;

use spin::{RwLock, Mutex};

use crate::traits::{USBHostController, USBMeta, USBPipe};
use crate::descriptor::USBDeviceDescriptor;
use hashbrown::HashMap;
use core::sync::atomic::{AtomicU32, Ordering};

/// Describes a Generic USB Device
pub struct USBDevice {
    pub bus: Arc<RwLock<USBBus>>,
    pub addr: u32,

    pub desc: Option<USBDeviceDescriptor>,
    pub speed: u8,
    pub max_packet_size: u16,

    pub prv: Option<Box<dyn USBMeta>>,
}

impl USBDevice {
    pub fn new(bus: Arc<RwLock<USBBus>>, addr: u32) -> Self {
        Self {
            bus,
            addr,
            desc: None,
            prv: None,
            speed: 0,
            max_packet_size: 0,
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