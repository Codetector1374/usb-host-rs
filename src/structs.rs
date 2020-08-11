//! USB Host Device Structures

use std::sync::Arc;
use spin::RwLock;
use crate::traits::USBHostController;

/// Describes a Generic USB Device
pub struct USBDevice {
    controller: Arc<RwLock<dyn USBHostController>>

}

pub struct USBBus {

}