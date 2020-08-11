//! USB Host Device Structures

use alloc::boxed::Box;
use alloc::sync::Arc;

use spin::RwLock;

use crate::traits::{USBHostController, USBMeta, USBPipe};
use crate::descriptor::USBDeviceDescriptor;
use hashbrown::HashMap;

/// Describes a Generic USB Device
pub struct USBDevice {
    pub controller: Arc<dyn USBHostController>,
    pub desc: Option<USBDeviceDescriptor>,

    pub speed: u8,
    pub max_packet_size: u16,

    pub prv: Option<Box<dyn USBMeta>>,
}

impl USBDevice {
    pub fn new(controller: Arc<dyn USBHostController>) -> Self {
        Self {
            controller,
            desc: None,
            prv: None,
            speed: 0,
            max_packet_size: 0,
        }
    }
}

pub struct USBBus {}