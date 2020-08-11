//! USB Host Device Structures

use alloc::boxed::Box;
use alloc::sync::Arc;

use spin::RwLock;

use crate::traits::{USBHostController, USBMeta};
use crate::descriptor::USBDeviceDescriptor;

/// Describes a Generic USB Device
pub struct USBDevice {
    pub controller: Arc<dyn USBHostController>,
    pub desc: USBDeviceDescriptor,

    pub speed: u8,
    pub max_packet_size: u16,

    pub prv: Option<Box<dyn USBMeta>>,
}

impl USBDevice {
    pub fn new(controller: Arc<dyn USBHostController>) -> Self {
        Self {
            controller,
            prv: None,
        }
    }
}

pub struct USBBus {}