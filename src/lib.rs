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
use crate::structs::{USBBus, USBDevice, USBPipe};
use crate::traits::USBHostController;
use crate::drivers::mass_storage::MassStorageDriver;

#[macro_use]
pub mod macros;

pub mod consts;
pub mod descriptor;
mod drivers;
mod error;
pub mod items;
pub mod structs;
pub mod traits;


pub(crate) fn as_mut_slice<T>(t: &mut T) -> &mut [u8] {
    unsafe { core::slice::from_raw_parts_mut(t as *mut T as *mut u8, core::mem::size_of::<T>()) }
}

pub(crate) fn as_slice<T>(t: &T) -> &[u8] {
    unsafe { core::slice::from_raw_parts(t as *const T as *const u8, core::mem::size_of::<T>()) }
}

pub trait HAL2 {
    fn sleep(dur: Duration);

    fn current_time() -> Duration;

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


pub struct USBHost<H: HAL2> {
    count: u32,
    root_hubs: HashMap<u32, Arc<RwLock<USBBus>>>,
    __phantom: PhantomData<H>,
}


impl<H: HAL2> USBHost<H> {
    pub fn new() -> Self {
        Self {
            count: 0,
            root_hubs: HashMap::new(),
            __phantom: PhantomData::default(),
        }
    }

    pub fn attach_root_hub(&mut self, controller: Arc<dyn USBHostController>, speed: USBSpeed) -> Arc<RwLock<USBDevice>> {
        let mut bus = Arc::new(RwLock::new(USBBus::new(controller.clone())));

        let device = Self::new_device(None, bus.clone(), speed, 0, 1)
            .unwrap_or_else(|e| panic!("Error: {:?}", e));
        controller.register_root_hub(&device);

        {
            let mut bus_lock = bus.write();
            bus_lock.devices[0] = Some(device.clone());
        }

        let count = self.count;
        self.root_hubs.insert(count, bus.clone());
        self.count = count + 1;

        // open the control "endpoint" on the root hub.
        controller.pipe_open(&device, None);

        device
    }

    /// Called when a new device is put into powered state but not yet addressed.
    /// This function will get the initial descriptor, set address, get full descriptor and attach
    /// a driver (if applicable)
    pub fn new_device(parent: Option<Arc<RwLock<USBDevice>>>, bus: Arc<RwLock<USBBus>>, speed: USBSpeed, addr: u32, port: u8) -> USBResult<Arc<RwLock<USBDevice>>> {
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

        let mut dev = USBDevice::new(parent.clone(), bus.clone(), max_packet_size, addr, port, speed, depth); // No Addr Yet

        // Setup Default EP Descriptor (This is faked so SW stack is unified)


        // TODO: Temp
        Ok(Arc::new(RwLock::new(dev)))
    }

    fn device_control_transfer(device: &Arc<RwLock<USBDevice>>, command: ControlCommand) -> USBResult<()> {
        // TODO refactor to store endpoint in the USBDevice then use the existing Control endpoint instead of making a fake one here.
        let cloned_device = device.clone();

        let dev_lock = device.read();
        let bus = dev_lock.bus.clone();
        let mut bus_lock = bus.read();

        let control_endpoint = USBPipe {
            device: cloned_device,
            controller: bus_lock.controller.clone(),
            index: 0,
            endpoint_type: EndpointType::Control,
            is_input: true,
        };

        bus_lock.controller.control_transfer(&control_endpoint, command)
    }

    fn fetch_descriptor_slice(device: &Arc<RwLock<USBDevice>>, req_type: RequestType, desc_type: u8, desc_index: u8, w_index: u16, slice: &mut [u8]) -> USBResult<()> {
        Self::device_control_transfer(device, ControlCommand {
            request_type: TypeTriple(TransferDirection::DeviceToHost, req_type, Recipient::Device),
            request: REQUEST_GET_DESCRIPTOR,
            value: ((desc_type as u16) << 8) | (desc_index as u16),
            index: w_index,
            length: slice.len() as u16,
            buffer: TransferBuffer::Read(slice),
        })
    }

    fn fetch_descriptor<T>(device: &Arc<RwLock<USBDevice>>, req_type: RequestType, desc_type: u8, desc_index: u8, w_index: u16, buf: &mut T) -> USBResult<()> {
        Self::fetch_descriptor_slice(device, req_type, desc_type, desc_index, w_index, as_mut_slice(buf))
    }

    fn fetch_configuration_descriptor(device: &Arc<RwLock<USBDevice>>) -> Result<USBConfigurationDescriptorSet, USBError> {
        let mut config_descriptor = USBConfigurationDescriptor::default();
        Self::fetch_descriptor_slice(device, RequestType::Standard, DESCRIPTOR_TYPE_CONFIGURATION, 0, 0, &mut as_mut_slice(&mut config_descriptor)[..4]);

        let mut descriptor_buf: Vec<u8> = Vec::new();
        descriptor_buf.resize(config_descriptor.wTotalLength as usize, 0);
        Self::fetch_descriptor_slice(device, RequestType::Standard, DESCRIPTOR_TYPE_CONFIGURATION, 0, 0, descriptor_buf.as_mut_slice());

        as_mut_slice(&mut config_descriptor).copy_from_slice(&descriptor_buf[..core::mem::size_of::<USBConfigurationDescriptor>()]);

        debug!("Parsing configuration: {:?}", config_descriptor);
        debug!("raw: {:x?}", descriptor_buf.as_slice());

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

    fn fetch_string_descriptor(device: &Arc<RwLock<USBDevice>>, index: u8, lang: u16) -> Result<String, USBError> {
        if index == 0 {
            return USBErrorKind::InvalidArgument.err("invalid descriptor index 0");
        }

        let mut buf = [0u8; 1];
        Self::fetch_descriptor_slice(device, RequestType::Standard, DESCRIPTOR_TYPE_STRING, index, lang, &mut buf);
        if buf[0] == 0 {
            return USBErrorKind::InvalidDescriptor.err("descriptor not available")
        }
        let mut buf2: Vec<u8> = Vec::new();
        buf2.resize(buf[0] as usize, 0);
        Self::fetch_descriptor_slice(device, RequestType::Standard, DESCRIPTOR_TYPE_STRING, index, lang, &mut buf2);
        assert_eq!(buf2[1], DESCRIPTOR_TYPE_STRING);
        let buf2: Vec<u16> = buf2.chunks_exact(2).map(|l| { u16::from_ne_bytes([l[0], l[1]]) }).collect();
        Ok(String::from_utf16_lossy(&buf2[1..]))
    }

    pub fn setup_new_device(device: Arc<RwLock<USBDevice>>) -> USBResult<()> {
        let mut device_desc = USBDeviceDescriptor::default();
        Self::fetch_descriptor(&device, RequestType::Standard, DESCRIPTOR_TYPE_DEVICE, 0, 0, &mut device_desc);

        info!("got device descriptor: {:?}", device_desc);

        let configuration = Self::fetch_configuration_descriptor(&device).unwrap_or_else(|e| panic!("bad: {:?}", e));
        debug!("configuration: {:#?}", configuration);

        {
            let mut dev_lock = device.write();
            dev_lock.config_desc = Some(configuration.clone());
        }

        Self::device_control_transfer(&device, ControlCommand {
            request_type: request_type!(HostToDevice, Standard, Device),
            request: REQUEST_SET_CONFIGURATION,
            value: configuration.config.bConfigurationValue as u16,
            index: 0,
            length: 0,
            buffer: TransferBuffer::None,
        });

        debug!("Applied Config {}", configuration.config.bConfigurationValue);
        H::sleep(Duration::from_millis(10));

        // Fetching language is removed due to various issues and most OS doesn't do it.
        // let mut buf = [0u8; 2];
        // self.fetch_descriptor(port.slot_id, DESCRIPTOR_TYPE_STRING,
        //                       0, 0, &mut buf)?;
        // assert_eq!(buf[1], DESCRIPTOR_TYPE_STRING, "Descriptor is not STRING");
        // assert!(buf[0] >= 4, "has language");
        // let mut buf2: Vec<u8> = Vec::new();
        // buf2.resize(buf[0] as usize, 0);
        // self.fetch_descriptor(port.slot_id, DESCRIPTOR_TYPE_STRING,
        //                       0, 0, &mut buf2)?;
        // let lang = buf2[2] as u16 | ((buf2[3] as u16) << 8);
        //
        // debug!("Language code: {:#x}", lang);

        // Display things
        let mfg = Self::fetch_string_descriptor(&device, device_desc.iManufacturer, 0x409).unwrap_or(String::from("(no manufacturer name)"));
        let prd = Self::fetch_string_descriptor(&device, device_desc.iProduct, 0x409).unwrap_or(String::from("(no product name)"));
        let serial = Self::fetch_string_descriptor(&device, device_desc.iSerialNumber, 0x409).unwrap_or(String::from("(no serial number)"));
        debug!("[XHCI] New device:\n  MFG: {}\n  Prd:{}\n  Serial:{}", mfg, prd, serial);

        for interface in &configuration.ifsets {
            if interface.interface.bAlternateSetting != 0 {
                debug!("Skipping non-default altSetting Interface");
                continue;
            }
            match interface.interface.bInterfaceClass {
                CLASS_CODE_MASS => {
                    if let Err(e) = MassStorageDriver::<H>::probe(&device, interface) {
                        error!("failed to probe msd: {:?}", e);
                    }
                }
                CLASS_CODE_HID => {
                    if let Err(e) = HIDDriver::<H>::probe(&device, interface) {
                        error!("failed to probe hid: {:?}", e);
                    }
                }
                CLASS_CODE_HUB => {
                    if let Err(e) = HubDriver::<H>::probe(&device, interface) {
                        error!("failed to probe hub: {:?}", e);
                    }
                }
                _ => {}
            }
        }

        Ok(())
    }

    fn reset_port(&mut self, device: &Arc<RwLock<USBDevice>>) -> Result<(), USBError> {

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

pub struct HubDriver<H: HAL2> {
    __phantom: PhantomData<H>,
}

impl<H: HAL2> HubDriver<H> {
    pub fn probe(device: &Arc<RwLock<USBDevice>>, interface: &USBInterfaceDescriptorSet) -> USBResult<()> {
        if interface.endpoints.len() == 0 {
            warn!("Hub with no endpoints!");
        }
        if interface.endpoints.len() > 1 {
            warn!("Hub with more than 1 endpoint!");
        }

        let mut hub_descriptor = USBHubDescriptor::default();
        USBHost::<H>::fetch_descriptor(&device, RequestType::Class, DESCRIPTOR_TYPE_HUB, 0, 0, &mut hub_descriptor);
        info!("Hub Descriptor: {:?}", hub_descriptor);

        // Setup EPs
        debug!("Found {} eps on this interface", interface.endpoints.len());

        let controller = device.read().bus.read().controller.clone();
        controller.configure_hub(&device, hub_descriptor.bNbrPorts, hub_descriptor.wHubCharacteristics.get_tt_think_time()).expect("Unable to configure hub");

        // TODO: Potentially Configure the Interrupt EP for Hub (if there is one)

        for num in 1..=hub_descriptor.bNbrPorts {
            if let Err(e) = HubDriver::<H>::probe_port(device, &hub_descriptor, num) {
                error!("failed to probe hub port {}: {:?}", num, e);
            }
        }

        Ok(())
    }

    fn probe_port(device: &Arc<RwLock<USBDevice>>, hub_descriptor: &USBHubDescriptor, port: u8) -> USBResult<()> {
        Self::set_feature(device, port, FEATURE_PORT_POWER)?;
        H::sleep(Duration::from_millis(hub_descriptor.bPwrOn2PwrGood as u64 * 2));

        Self::clear_feature(device, port, FEATURE_C_PORT_CONNECTION)?;
        let mut status = Self::fetch_port_status(device, port)?;

        debug!("Port {}: status={:?}", port, status);

        if !status.get_device_connected() {
            return Ok(());
        }

        debug!("Acquiring locks");

        let child_parent_device = device.clone();

        let dev_lock = device.read();

        let child_bus = dev_lock.bus.clone();
        let bus_lock = dev_lock.bus.read();
        let slot = bus_lock.controller.allocate_slot()?;

        debug!("creating new child device");

        debug!("resetting device");
        Self::reset_port(device, port)?;

        let status = Self::fetch_port_status(device, port)?;

        let child_device = USBHost::<H>::new_device(Some(child_parent_device), child_bus, status.get_speed(), slot as u32, port)?;

        debug!("opening control pipe");
        // open the control "endpoint" on the root hub.
        bus_lock.controller.pipe_open(&child_device, None)?;

        H::sleep(Duration::from_millis(10));
        debug!("Going to fetch descriptor...");

        let mut buf = [0u8; 8];
        USBHost::<H>::fetch_descriptor_slice(&child_device, RequestType::Standard, DESCRIPTOR_TYPE_DEVICE, 0, 0, &mut buf)?;
        debug!("First fetch descriptor: {:?}", buf);

        {
            let mut d = child_device.write();
            d.ddesc.bMaxPacketSize = buf[7];
        }

        debug!("setting address");
        bus_lock.controller.set_address(&child_device, slot as u32)?;

        debug!("recursive setup_new_device()");
        USBHost::<H>::setup_new_device(child_device.clone())?;

        Ok(())
    }

    fn set_feature(device: &Arc<RwLock<USBDevice>>, port: u8, feature: u8) -> USBResult<()> {
        USBHost::<H>::device_control_transfer(&device, ControlCommand {
            request_type: request_type!(HostToDevice, Class, Other),
            request: REQUEST_SET_FEATURE,
            value: feature as u16,
            index: port as u16,
            length: 0,
            buffer: TransferBuffer::None,
        })
    }

    fn clear_feature(device: &Arc<RwLock<USBDevice>>, port: u8, feature: u8) -> USBResult<()> {
        USBHost::<H>::device_control_transfer(&device, ControlCommand {
            request_type: request_type!(HostToDevice, Class, Other),
            request: REQUEST_CLEAR_FEATURE,
            value: feature as u16,
            index: port as u16,
            length: 0,
            buffer: TransferBuffer::None,
        })
    }

    fn fetch_port_status(device: &Arc<RwLock<USBDevice>>, port: u8) -> USBResult<PortStatus> {
        let mut status = PortStatus::default();
        USBHost::<H>::device_control_transfer(&device, ControlCommand {
            request_type: request_type!(DeviceToHost, Class, Other),
            request: REQUEST_GET_STATUS,
            value: 0,
            index: port as u16,
            length: 4,
            buffer: TransferBuffer::Read(as_mut_slice(&mut status)),
        })?;
        Ok(status)
    }

    fn reset_port(device: &Arc<RwLock<USBDevice>>, port: u8) -> USBResult<()> {
        Self::set_feature(device, port, FEATURE_PORT_RESET)?;

        H::wait_until(USBErrorKind::Timeout.msg("failed to reset port"), PORT_RESET_TIMEOUT, || {
            match Self::fetch_port_status(device, port) {
                Ok(s) => s.get_change_reset(),
                Err(_) => false,
            }
        })?;

        Self::clear_feature(device, port, FEATURE_C_PORT_RESET)
    }
}

pub struct HIDDriver<H: HAL2> {
    __phantom: PhantomData<H>,
}

impl<H: HAL2> HIDDriver<H> {
    pub fn probe(device: &Arc<RwLock<USBDevice>>, interface: &USBInterfaceDescriptorSet) -> USBResult<()> {
        if interface.interface.bInterfaceSubClass != 1 {
            debug!("Skipping non bios-mode HID device");
            return Ok(());
        }

        if interface.interface.bInterfaceProtocol != 1 {
            debug!("Skipping non keyboard");
            return Ok(());
        }

        if interface.endpoints.len() == 0 {
            return USBErrorKind::InvalidDescriptor.err("keyboard with no endpoints!");
        }

        let ep_interrupt = interface.endpoints.iter()
            .find(|d| matches!(d.transfer_type(), EndpointType::Interrupt) && d.is_input());
        let ep_interrupt = match ep_interrupt {
            Some(e) => e,
            None => {
                return USBErrorKind::InvalidDescriptor.err("keyboard with no interrupt in endpoint");
            }
        };

        // Enable keyboard

        let dev_lock = device.read();
        let bus_lock = dev_lock.bus.read();

        let pipe = bus_lock.controller.pipe_open(device, Some(ep_interrupt)).unwrap_or_else(|e| panic!("{:?}", e));

        debug!("done keyboard configure endpoint");

        H::sleep(Duration::from_millis(100));

        debug!("set hid idle");
        Self::set_hid_idle(device)?;

        // 0 is Boot Protocol
        debug!("set hid protocol");
        Self::set_hid_protocol(device, 0)?;

        // let pipe = pipe.read();
        //
        // for i in 0..1000 {
        //     debug!("set hid report");
        //     Self::set_hid_report(device, &interface.interface, if i % 2 == 0 { 0 } else { 0xFF })?;
        //
        //     debug!("fetch hid report");
        //     let mut buf = Box::new([0u8; 128]);
        //     match pipe.bulk_read(&mut buf[0..8]) {
        //         Ok(_) => {
        //             info!("got response: {:?}", &buf[0..8]);
        //         }
        //         Err(e) => {
        //             warn!("failed to read: {:?}", e);
        //         }
        //     }
        //
        //     H::sleep(Duration::from_millis(500));
        // }

        Ok(())
    }

    fn set_hid_report(device: &Arc<RwLock<USBDevice>>, interface: &USBInterfaceDescriptor, value: u8) -> USBResult<()> {
        // type == 2 (Output)
        let (desc_type, desc_index) = (2, 0);
        USBHost::<H>::device_control_transfer(device, ControlCommand {
            request_type: request_type!(HostToDevice, Class, Interface),
            request: REQUEST_SET_REPORT,
            value: ((desc_type as u16) << 8) | (desc_index as u16),
            index: interface.bInterfaceNumber as u16,
            length: 1,
            buffer: TransferBuffer::Write(core::slice::from_ref(&value)),
        })
    }

    fn set_hid_idle(device: &Arc<RwLock<USBDevice>>) -> USBResult<()> {
        USBHost::<H>::device_control_transfer(device, ControlCommand {
            request_type: request_type!(HostToDevice, Class, Interface),
            request: REQUEST_SET_IDLE,
            value: 0,
            index: 0,
            length: 0,
            buffer: TransferBuffer::None,
        })
    }

    fn set_hid_protocol(device: &Arc<RwLock<USBDevice>>, value: u8) -> USBResult<()> {
        USBHost::<H>::device_control_transfer(device, ControlCommand {
            request_type: request_type!(HostToDevice, Class, Interface),
            request: REQUEST_SET_PROTOCOL,
            value: value as u16,
            index: 0,
            length: 0,
            buffer: TransferBuffer::None,
        })
    }
}





