#![feature(allocator_api)]
#![feature(const_in_array_repeat_expressions)]
#![feature(global_asm)]
#![feature(llvm_asm)]
#![allow(dead_code, unused_imports, unused_parens)]
#![cfg_attr(not(test), no_std)]

#[macro_use]
extern crate alloc;
#[macro_use]
extern crate downcast_rs;
#[macro_use]
extern crate log;

use alloc::sync::Arc;

use hashbrown::HashMap;
use spin::{RwLock, Mutex};

use crate::structs::{USBBus, USBDevice};
use crate::traits::{USBHostController, USBPipe};
use crate::items::{EndpointType, ControlCommand, RequestType, TransferBuffer, TypeTriple, TransferDirection, Recipient, Error};
use crate::consts::{REQUEST_GET_DESCRIPTOR, DESCRIPTOR_TYPE_DEVICE, USBSpeed};
use crate::descriptor::USBDeviceDescriptor;
use crate::error::USBError;

#[macro_use]
pub mod macros;

pub mod consts;
pub mod descriptor;
pub mod items;
pub mod structs;
pub mod traits;
pub mod error;

fn as_mut_slice<T>(t: &mut T) -> &mut [u8] {
    unsafe { core::slice::from_raw_parts_mut(t as *mut T as *mut u8, core::mem::size_of::<T>()) }
}

fn as_slice<T>(t: &T) -> &[u8] {
    unsafe { core::slice::from_raw_parts(t as *const T as *const u8, core::mem::size_of::<T>()) }
}


pub struct USBHost {
    count: u32,
    root_hubs: HashMap<u32, (Arc<dyn USBHostController>, Arc<RwLock<USBDevice>>)>,
}


impl USBHost {
    pub fn new() -> Self {
        Self {
            count: 0,
            root_hubs: HashMap::new(),
        }
    }

    pub fn attach_root_hub(&mut self, controller: Arc<dyn USBHostController>) -> Arc<RwLock<USBDevice>> {
        let device = Arc::new(RwLock::new(USBDevice::new(controller.clone())));
        controller.register_root_hub(&device);

        let count = self.count;
        self.root_hubs.insert(count, (controller.clone(), device.clone()));
        self.count = count + 1;

        // open the control "endpoint" on the root hub.
        controller.pipe_open(&device, None);

        device
    }

    /// Called when a new device is put into powered state but not yet addressed.
    /// This function will get the initial descriptor, set address, get full descriptor and attach
    /// a driver (if applicable)
    pub fn new_device(&mut self, parent: Option<Arc<RwLock<USBDevice>>>, bus: Arc<RwLock<USBBus>>,
                      depth: u8, speed: USBSpeed, port: u32) -> Result<Arc<RwLock<USBDevice>>, USBError> {
        // Calculate initial MPS
        let max_packet_size = match speed {
            USBSpeed::Low => 8u16,
            USBSpeed::Full |
            USBSpeed::High => 64u16,
            USBSpeed::Super => 512u16,
            _ => return Err(USBError::InvalidArgument),
        };

        // this should be held until the device is added or failed
        let mut bus_guard = bus.write();

        let mut dev = USBDevice::new(bus.clone(), max_packet_size, 0, speed, depth); // No Addr Yet

        // Setup Default EP Descriptor (This is faked so SW stack is unified)


        // TODO: Temp
        Err(USBError::InvalidArgument)
    }

    fn fetch_descriptor<T>(device: &Arc<RwLock<USBDevice>>, req_type: RequestType, desc_type: u8, desc_index: u8, w_index: u16, buf: &mut T) {
        let control_endpoint = USBPipe { index: 0, endpoint_type: EndpointType::Control };
        let dev_lock = device.read();
        let controller = dev_lock.controller.clone();

        let slice = as_mut_slice(buf);

        controller.control_transfer(&device, &control_endpoint, ControlCommand {
            request_type: TypeTriple(TransferDirection::DeviceToHost, req_type, Recipient::Device),
            request: REQUEST_GET_DESCRIPTOR,
            value: ((desc_type as u16) << 8) | (desc_index as u16),
            index: w_index,
            length: slice.len() as u16,
            buffer: TransferBuffer::Read(slice),
        });
    }

    pub fn enumerate_devices(&self, device: Arc<RwLock<USBDevice>>) {
        let mut device_desc = USBDeviceDescriptor::default();
        Self::fetch_descriptor(&device, RequestType::Standard, DESCRIPTOR_TYPE_DEVICE, 0, 0, &mut device_desc);

        info!("got device descriptor: {:?}", device_desc);
    }

    fn reset_port(&mut self, device: &Arc<RwLock<USBDevice>>) -> Result<(), Error> {

        // self.set_feature(parent.slot_id, port.port_id, FEATURE_PORT_RESET)?;
        //
        // self.wait_until("failed to reset port", PORT_RESET_TIMEOUT, |this| {
        //     if let Ok(status) = this.fetch_port_status(parent.slot_id, port.port_id) {
        //         status.get_change_reset()
        //     } else {
        //         false
        //     }
        // })?;
        //
        // self.clear_feature(parent.slot_id, port.port_id, FEATURE_C_PORT_RESET)?;
        Ok(())
    }
}