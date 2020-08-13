use alloc::sync::Arc;

use downcast_rs::DowncastSync;
use spin::RwLock;

use crate::descriptor::USBEndpointDescriptor;
use crate::error::{USBErrorKind, USBError};
use crate::items::{ControlCommand, EndpointType, TransferBuffer};
use crate::structs::{USBDevice, USBPipe};
use crate::USBResult;

pub trait USBMeta: DowncastSync {}
impl_downcast!(USBMeta);

pub trait USBHostController: Send + Sync {
    fn register_root_hub(&self, device: &Arc<RwLock<USBDevice>>);

    fn pipe_open(&self, device: &Arc<RwLock<USBDevice>>, endpoint: Option<&USBEndpointDescriptor>) -> USBResult<Arc<RwLock<USBPipe>>>;

    fn set_address(&self, device: &Arc<RwLock<USBDevice>>, addr: u32) -> USBResult<()>;

    fn configure_hub(&self, device: &Arc<RwLock<USBDevice>>, nbr_ports: u8, ttt: u8) -> USBResult<()>;

    fn control_transfer(&self, endpoint: &USBPipe, command: ControlCommand) -> USBResult<()>;

    fn bulk_transfer(&self, endpoint: &USBPipe, buffer: TransferBuffer) -> USBResult<usize>;

    fn allocate_slot(&self) -> USBResult<u8>;

    fn free_slot(&self, slot: u8);
}

