use alloc::sync::Arc;

use spin::RwLock;

use crate::items::{Port, EndpointType, Error, ControlCommand};
use crate::structs::USBDevice;
use downcast_rs::Downcast;
use crate::descriptor::USBEndpointDescriptor;

pub trait USBMeta : Downcast {}
impl_downcast!(USBMeta);

pub struct USBPipe {
    pub index: u8,
    pub endpoint_type: EndpointType,
}

pub trait USBHostController {

    fn register_root_hub(&self, device: &Arc<RwLock<USBDevice>>);

    fn pipe_open(&self, device: &Arc<RwLock<USBDevice>>, endpoint: Option<&USBEndpointDescriptor>) -> Result<Arc<RwLock<USBPipe>>, Error>;

    fn set_address(&self, device: &Arc<RwLock<USBDevice>>);

    fn control_transfer(&self, device: &Arc<RwLock<USBDevice>>, endpoint: &USBPipe, command: ControlCommand);



}

