use alloc::sync::Arc;

use spin::RwLock;

use crate::items::{Port, EndpointType, Error, ControlCommand, TransferBuffer};
use crate::structs::USBDevice;
use downcast_rs::Downcast;
use crate::descriptor::USBEndpointDescriptor;

pub trait USBMeta : Downcast {}
impl_downcast!(USBMeta);

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

    pub fn bulk_write(&self, buf: &[u8]) -> Result<usize, Error> {
        assert!(matches!(self.endpoint_type, EndpointType::Bulk));
        assert!(!self.is_input);
        self.controller.bulk_transfer(&self, TransferBuffer::Write(buf))
    }

    pub fn bulk_read(&self, buf: &mut [u8]) -> Result<usize, Error> {
        assert!(matches!(self.endpoint_type, EndpointType::Bulk));
        assert!(self.is_input);
        self.controller.bulk_transfer(&self, TransferBuffer::Read(buf))
    }

}

pub trait USBHostController {

    fn register_root_hub(&self, device: &Arc<RwLock<USBDevice>>);

    fn pipe_open(&self, device: &Arc<RwLock<USBDevice>>, endpoint: Option<&USBEndpointDescriptor>) -> Result<Arc<RwLock<USBPipe>>, Error>;

    fn set_address(&self, device: &Arc<RwLock<USBDevice>>, addr: u32);

    fn control_transfer(&self, endpoint: &USBPipe, command: ControlCommand);

    fn bulk_transfer(&self, endpoint: &USBPipe, buffer: TransferBuffer) -> Result<usize, Error>;

    fn allocate_slot(&self) -> u8;

    fn free_slot(&self, slot: u8);

}

