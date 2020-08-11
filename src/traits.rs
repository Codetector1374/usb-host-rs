use alloc::sync::Arc;

use spin::RwLock;

use crate::items::{Port, EndpointType, Error};
use crate::structs::USBDevice;
use downcast_rs::Downcast;
use crate::descriptor::USBEndpointDescriptor;

pub trait USBMeta : Downcast {}
impl_downcast!(USBMeta);

pub struct USBPipe {
    pub endpoint_type: EndpointType,
}

pub trait USBHostController {

    fn register_root_hub(&self, device: &Arc<RwLock<USBDevice>>);

    fn pipe_open(&self, device: &Arc<RwLock<USBDevice>>, endpoint: Option<&USBEndpointDescriptor>) -> Result<Arc<RwLock<USBPipe>>, Error>;

    fn set_address(&self, device: &Arc<RwLock<USBDevice>>);

    fn reset_port(&self, device: &Arc<RwLock<USBDevice>>, port: u8);

    fn control_transfer(&self, endpoint: Arc<RwLock<USBPipe>>);



}

