#![feature(allocator_api)]
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

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::marker::PhantomData;
use core::time::Duration;

use hashbrown::HashMap;
use spin::{Mutex, RwLock};

use crate::consts::*;
use crate::descriptor::*;
pub use crate::error::{USBError, USBErrorKind, USBResult};
use crate::items::{ControlCommand, EndpointType, PortStatus, Recipient, RequestType, TransferBuffer, TransferDirection, TypeTriple};
use crate::structs::{USBBus, USBDevice, USBPipe, DeviceState};
use crate::traits::USBHostController;
use crate::drivers::mass_storage::MassStorageDriver;
use crate::drivers::keyboard::HIDKeyboard;
use core::sync::atomic::{AtomicU32, Ordering};

#[macro_use]
pub mod macros;

pub mod collection;
pub mod consts;
pub mod descriptor;
pub mod drivers;
mod error;
pub mod items;
pub mod layer;
pub mod structs;
pub mod traits;


pub(crate) fn as_mut_slice<T>(t: &mut T) -> &mut [u8] {
    unsafe { core::slice::from_raw_parts_mut(t as *mut T as *mut u8, core::mem::size_of::<T>()) }
}

pub(crate) fn as_slice<T>(t: &T) -> &[u8] {
    unsafe { core::slice::from_raw_parts(t as *const T as *const u8, core::mem::size_of::<T>()) }
}

pub trait UsbHAL: Sync + Send + 'static {
    fn sleep(dur: Duration);

    fn current_time() -> Duration;

    /// Queue a function to run at a later time, for example
    /// queue work to be done from an interrupt context on a
    /// worker thread
    fn queue_task(func: Box<dyn FnOnce() + Send + 'static>) {
        func()
    }

    // Provided functions

    fn queue_task_fn<F: FnOnce() + Send + 'static>(func: F) {
        Self::queue_task(Box::new(func));
    }

    fn wait_until<E, F: FnMut() -> bool>(e: E, timeout: Duration, mut func: F) -> Result<(), E> {
        let wait_limit = Self::current_time() + timeout;
        loop {
            if func() {
                return Ok(());
            }
            if Self::current_time() > wait_limit {
                return Err(e);
            }
            Self::sleep(Duration::from_millis(1));
        }
    }
}

pub trait HostCallbacks<H: UsbHAL>: Sync + Send {
    fn new_device(&self, _host: &Arc<USBHost<H>>, _device: &Arc<RwLock<USBDevice>>) -> USBResult<()> {
        Ok(())
    }
}


pub struct USBHost<H: UsbHAL> {
    root_hubs: RwLock<HashMap<u32, Arc<USBBus>>>,
    count: AtomicU32,
    __phantom: PhantomData<H>,
    callbacks: Arc<dyn HostCallbacks<H>>,
}


impl<H: UsbHAL> USBHost<H> {
    pub fn new(callbacks: Arc<dyn HostCallbacks<H>>) -> Self {
        Self {
            count: AtomicU32::new(0),
            root_hubs: RwLock::new(HashMap::new()),
            __phantom: PhantomData::default(),
            callbacks,
        }
    }

    pub fn attach_root_hub(&self, controller: Arc<dyn USBHostController>, speed: USBSpeed) -> Arc<RwLock<USBDevice>> {
        let bus = Arc::new(USBBus::new(controller.clone()));
        let device = Self::new_device(None, bus.clone(), speed, 0, 1)
            .unwrap_or_else(|e| panic!("Error: {:?}", e));
        controller.register_root_hub(&device);

        bus.devices.set(0, Some(Arc::downgrade(&device)));

        let count = self.count.fetch_add(1, Ordering::AcqRel);
        {
            let mut r = self.root_hubs.write();
            r.insert(count, bus.clone());
        }

        // open the control "endpoint" on the root hub.
        if let Err(e) = controller.pipe_open(&device, None) {
            error!("failed to open control endpoint on root: {:?}", e);
        }

        device
    }

    /// Called when a new device is put into powered state but not yet addressed.
    /// This function will get the initial descriptor, set address, get full descriptor and attach
    /// a driver (if applicable)
    pub fn new_device(parent: Option<Arc<RwLock<USBDevice>>>, bus: Arc<USBBus>, speed: USBSpeed, addr: u32, port: u8) -> USBResult<Arc<RwLock<USBDevice>>> {
        // Calculate initial MPS
        let max_packet_size = match speed {
            USBSpeed::Low => 8u16,
            USBSpeed::Full |
            USBSpeed::High => 64u16,
            USBSpeed::Super => 512u16,
            _ => return USBErrorKind::InvalidArgument.err("unknown usb speed"),
        };

        let depth = match parent.as_ref() {
            None => 0,
            Some(parent) => {
                let dev_lock = parent.read();
                dev_lock.depth + 1
            }
        };

        let dev = USBDevice::new(parent.clone(), bus.clone(), max_packet_size, addr, port, speed, depth); // No Addr Yet

        // Setup Default EP Descriptor (This is faked so SW stack is unified)


        // TODO: Temp
        Ok(Arc::new(RwLock::new(dev)))
    }

    pub fn device_control_transfer(device: &Arc<RwLock<USBDevice>>, command: ControlCommand) -> USBResult<()> {
        // TODO refactor to store endpoint in the USBDevice then use the existing Control endpoint instead of making a fake one here.
        let cloned_device = Arc::downgrade(&device);

        let dev_lock = device.read();
        let controller = Arc::downgrade(&dev_lock.bus.controller);

        let control_endpoint = USBPipe {
            device: cloned_device,
            controller,
            index: 0,
            endpoint_type: EndpointType::Control,
            max_packet_size: dev_lock.ddesc.get_max_packet_size() as usize,
            is_input: true,
        };

        dev_lock.bus.controller.control_transfer(&control_endpoint, command)
    }

    pub fn fetch_descriptor_slice(device: &Arc<RwLock<USBDevice>>, req_type: RequestType, desc_type: u8, desc_index: u8, w_index: u16, slice: &mut [u8]) -> USBResult<()> {
        Self::device_control_transfer(device, ControlCommand {
            request_type: TypeTriple(TransferDirection::DeviceToHost, req_type, Recipient::Device),
            request: REQUEST_GET_DESCRIPTOR,
            value: ((desc_type as u16) << 8) | (desc_index as u16),
            index: w_index,
            length: slice.len() as u16,
            buffer: TransferBuffer::Read(slice),
        })
    }

    pub fn fetch_descriptor<T>(device: &Arc<RwLock<USBDevice>>, req_type: RequestType, desc_type: u8, desc_index: u8, w_index: u16, buf: &mut T) -> USBResult<()> {
        Self::fetch_descriptor_slice(device, req_type, desc_type, desc_index, w_index, as_mut_slice(buf))
    }

    pub fn fetch_configuration_descriptor(device: &Arc<RwLock<USBDevice>>) -> Result<USBConfigurationDescriptorSet, USBError> {
        let mut config_descriptor = USBConfigurationDescriptor::default();
        Self::fetch_descriptor_slice(device, RequestType::Standard, DESCRIPTOR_TYPE_CONFIGURATION, 0, 0, &mut as_mut_slice(&mut config_descriptor)[..4])?;

        let mut descriptor_buf: Vec<u8> = Vec::new();
        descriptor_buf.resize(config_descriptor.wTotalLength as usize, 0);
        Self::fetch_descriptor_slice(device, RequestType::Standard, DESCRIPTOR_TYPE_CONFIGURATION, 0, 0, descriptor_buf.as_mut_slice())?;

        as_mut_slice(&mut config_descriptor).copy_from_slice(&descriptor_buf[..core::mem::size_of::<USBConfigurationDescriptor>()]);

        let mut current_index = core::mem::size_of::<USBConfigurationDescriptor>();
        let mut interfaces: Vec<USBInterfaceDescriptorSet> = Default::default();
        let mut interface_set: Option<USBInterfaceDescriptorSet> = None;
        loop {
            if current_index + 2 > descriptor_buf.len() {
                if current_index != descriptor_buf.len() {
                    warn!("[USB] Descriptor not fully fetched");
                }
                break;
            }
            let desc_size = descriptor_buf[current_index] as usize;
            if desc_size == 0 {
                break;
            }
            let desc_type = descriptor_buf[current_index + 1];
            match desc_type {
                DESCRIPTOR_TYPE_INTERFACE => {
                    if interface_set.is_some() {
                        interfaces.push(interface_set.take().unwrap());
                    }

                    assert_eq!(desc_size, core::mem::size_of::<USBInterfaceDescriptor>());
                    let mut desc = USBInterfaceDescriptor::default();
                    as_mut_slice(&mut desc).copy_from_slice(&descriptor_buf[current_index..current_index + desc_size]);

                    interface_set = Some(USBInterfaceDescriptorSet::new(desc));
                }
                DESCRIPTOR_TYPE_ENDPOINT => {
                    assert_eq!(desc_size, core::mem::size_of::<USBEndpointDescriptor>());
                    let mut desc = USBEndpointDescriptor::default();
                    as_mut_slice(&mut desc).copy_from_slice(&descriptor_buf[current_index..current_index + desc_size]);

                    match &mut interface_set {
                        Some(ifset) => {
                            ifset.endpoints.push(desc);
                        }
                        _ => {
                            error!("[USB] EP Descriptor without IF");
                        }
                    }
                }
                _ => {
                    debug!("[USB] Unexpected descriptor type: {}", desc_type);
                }
            }
            current_index += desc_size;
        }
        if let Some(ifset) = interface_set {
            interfaces.push(ifset);
        }
        Ok(USBConfigurationDescriptorSet { config: config_descriptor, ifsets: interfaces })
    }

    pub fn fetch_string_descriptor(device: &Arc<RwLock<USBDevice>>, index: u8, lang: u16) -> Result<String, USBError> {
        if index == 0 {
            return USBErrorKind::InvalidArgument.err("invalid descriptor index 0");
        }

        let mut buf = [0u8; 1];
        Self::fetch_descriptor_slice(device, RequestType::Standard, DESCRIPTOR_TYPE_STRING, index, lang, &mut buf)?;
        if buf[0] == 0 {
            return USBErrorKind::InvalidDescriptor.err("descriptor not available")
        }
        let mut buf2: Vec<u8> = Vec::new();
        buf2.resize(buf[0] as usize, 0);
        Self::fetch_descriptor_slice(device, RequestType::Standard, DESCRIPTOR_TYPE_STRING, index, lang, &mut buf2)?;
        if buf2[1] != DESCRIPTOR_TYPE_STRING {
            return Err(USBErrorKind::InvalidDescriptor.msg("got wrong descriptor type value"))
        }

        let buf2: Vec<u16> = buf2.chunks_exact(2).map(|l| { u16::from_ne_bytes([l[0], l[1]]) }).collect();
        Ok(String::from_utf16_lossy(&buf2[1..]))
    }

    pub fn setup_new_device(host: &Arc<Self>, device: Arc<RwLock<USBDevice>>) -> USBResult<()> {
        let mut device_desc = USBDeviceDescriptor::default();
        Self::fetch_descriptor(&device, RequestType::Standard, DESCRIPTOR_TYPE_DEVICE, 0, 0, &mut device_desc)?;

        trace!("got device descriptor: {:?}", device_desc);

        let configuration = Self::fetch_configuration_descriptor(&device).unwrap_or_else(|e| panic!("bad: {:?}", e));
        trace!("configuration: {:#?}", configuration);

        {
            let mut dev_lock = device.write();
            dev_lock.ddesc = device_desc;
            dev_lock.config_desc = Some(configuration.clone());
        }

        Self::device_control_transfer(&device, ControlCommand {
            request_type: request_type!(HostToDevice, Standard, Device),
            request: REQUEST_SET_CONFIGURATION,
            value: configuration.config.bConfigurationValue as u16,
            index: 0,
            length: 0,
            buffer: TransferBuffer::None,
        })?;

        trace!("Applied Config {}", configuration.config.bConfigurationValue);

        {
            let dev_cloned_weak = Arc::downgrade(&device);
            let mut dev_lock = device.write();
            dev_lock.device_state = DeviceState::Idle;
            dev_lock.bus.devices.set(dev_lock.addr as usize, Some(dev_cloned_weak));
        }

        host.callbacks.new_device(host, &device)?;

        Ok(())
    }

}







