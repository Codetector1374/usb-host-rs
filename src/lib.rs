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

use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::marker::PhantomData;
use core::time::Duration;

use hashbrown::HashMap;
use spin::{Mutex, RwLock};

use crate::consts::*;
use crate::descriptor::{USBConfigurationDescriptor, USBConfigurationDescriptorSet, USBDeviceDescriptor, USBEndpointDescriptor, USBHubDescriptor, USBInterfaceDescriptor, USBInterfaceDescriptorSet};
use crate::error::USBError;
use crate::items::{ControlCommand, EndpointType, Error, Recipient, RequestType, TransferBuffer, TransferDirection, TypeTriple};
use crate::structs::{PortStatus, USBBus, USBDevice};
use crate::traits::{USBHostController, USBPipe};

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
    __phantom: PhantomData<H>,
    count: u32,
    root_hubs: HashMap<u32, Arc<RwLock<USBBus>>>,
}


impl<H: HAL2> USBHost<H> {
    pub fn new() -> Self {
        Self {
            __phantom: PhantomData::default(),
            count: 0,
            root_hubs: HashMap::new(),
        }
    }

    pub fn attach_root_hub(&mut self, controller: Arc<dyn USBHostController>) -> Arc<RwLock<USBDevice>> {
        let mut bus = Arc::new(RwLock::new(USBBus::new(controller.clone())));

        let device = Self::new_device(None, bus.clone(), USBSpeed::Super, 0, 1).unwrap_or_else(|e| panic!("Error: {:?}", e));
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
    pub fn new_device(parent: Option<Arc<RwLock<USBDevice>>>, bus: Arc<RwLock<USBBus>>, speed: USBSpeed, addr: u32, port: u8) -> Result<Arc<RwLock<USBDevice>>, USBError> {
        // Calculate initial MPS
        let max_packet_size = match speed {
            USBSpeed::Low => 8u16,
            USBSpeed::Full |
            USBSpeed::High => 64u16,
            USBSpeed::Super => 512u16,
            _ => return Err(USBError::InvalidArgument),
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

    fn device_control_transfer(device: &Arc<RwLock<USBDevice>>, command: ControlCommand) {
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

        bus_lock.controller.control_transfer(&control_endpoint, command);
    }

    fn fetch_descriptor_slice(device: &Arc<RwLock<USBDevice>>, req_type: RequestType, desc_type: u8, desc_index: u8, w_index: u16, slice: &mut [u8]) {
        Self::device_control_transfer(device, ControlCommand {
            request_type: TypeTriple(TransferDirection::DeviceToHost, req_type, Recipient::Device),
            request: REQUEST_GET_DESCRIPTOR,
            value: ((desc_type as u16) << 8) | (desc_index as u16),
            index: w_index,
            length: slice.len() as u16,
            buffer: TransferBuffer::Read(slice),
        });
    }

    fn fetch_descriptor<T>(device: &Arc<RwLock<USBDevice>>, req_type: RequestType, desc_type: u8, desc_index: u8, w_index: u16, buf: &mut T) {
        Self::fetch_descriptor_slice(device, req_type, desc_type, desc_index, w_index, as_mut_slice(buf));
    }

    fn fetch_configuration_descriptor(device: &Arc<RwLock<USBDevice>>) -> Result<USBConfigurationDescriptorSet, Error> {
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

    fn fetch_string_descriptor(device: &Arc<RwLock<USBDevice>>, index: u8, lang: u16) -> Result<String, Error> {
        if index == 0 {
            return Err(Error::Str("invalid descriptor index 0"));
        }

        let mut buf = [0u8; 1];
        Self::fetch_descriptor_slice(device, RequestType::Standard, DESCRIPTOR_TYPE_STRING, index, lang, &mut buf);
        if buf[0] == 0 {
            return Err(Error::Str("USBError::DescriptorNotAvailable"));
        }
        let mut buf2: Vec<u8> = Vec::new();
        buf2.resize(buf[0] as usize, 0);
        Self::fetch_descriptor_slice(device, RequestType::Standard, DESCRIPTOR_TYPE_STRING, index, lang, &mut buf2);
        assert_eq!(buf2[1], DESCRIPTOR_TYPE_STRING);
        let buf2: Vec<u16> = buf2.chunks_exact(2).map(|l| { u16::from_ne_bytes([l[0], l[1]]) }).collect();
        Ok(String::from_utf16_lossy(&buf2[1..]))
    }

    pub fn setup_new_device(device: Arc<RwLock<USBDevice>>) {
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
                    MassStorageDriver::<H>::probe(&device, interface)
                }
                //         CLASS_CODE_MASS => {
                //             if interface.interface.sub_class != 0x6 {
                //                 debug!("Skipping MSD with sub-class other than 0x6 (Transparent SCSI)");
                //                 continue;
                //             }
                //
                //             if interface.interface.protocol != 0x50 {
                //                 debug!("Skipping MSD with protocol other than bulk-only");
                //                 continue;
                //             }
                //
                //             if interface.endpoints.len() < 2 {
                //                 warn!("MSD has not enough endpoints!");
                //                 continue;
                //             }
                //
                //             let mut input_ring: Option<(u8, u8)> = None;
                //             let mut output_ring: Option<(u8, u8)> = None;
                //
                //             self.with_input_context(port, |this, input_ctx| {
                //                 for endpoint in interface.endpoints.iter() {
                //                     if endpoint.bmAttributes != EP_ATTR_BULK {
                //                         continue;
                //                     }
                //
                //                     let typ = if Self::is_ep_input(endpoint.bEndpointAddress) { EP_TYPE_BULK_IN } else { EP_TYPE_BULK_OUT };
                //
                //                     let ring = this.configure_endpoint(port.slot_id, input_ctx,
                //                                                        endpoint.bEndpointAddress,
                //                                                        typ, endpoint.wMaxPacketSize,
                //                                                        endpoint.bInterval, this.get_max_esti_payload(endpoint));
                //
                //                     if Self::is_ep_input(endpoint.bEndpointAddress) {
                //                         input_ring = Some(ring);
                //                     } else {
                //                         output_ring = Some(ring);
                //                     }
                //                 }
                //
                //                 input_ctx.set_configure_ep_meta(configuration.config.config_val,
                //                                                 interface.interface.interface_number, interface.interface.alt_set);
                //
                //                 Ok(())
                //             })?;
                //
                //             let input_ring = input_ring.ok_or(Error::Str("MSD has no bulk input endpoint"))?;
                //             let output_ring = output_ring.ok_or(Error::Str("MSD has no bulk output endpoint"))?;
                //
                //             debug!("MSD endpoints initialized with bulk_in:{:?} bulk_out:{:?}", input_ring, output_ring);
                //
                //             let mut buf: Vec<u8> = Vec::new();
                //             buf.resize(31, 0);
                //
                //             let t = [0x55, 0x53, 0x42, 0x43, 0x13, 0x37, 0x04, 0x20, 0x24, 0x00, 0x00, 0x00, 0x80, 0x00, 6, 0x12, 0x00, 0x00, 0x00, 0x24, 0x00];
                //             (&mut buf[..t.len()]).copy_from_slice(&t);
                //
                //             let r = self.transfer_bulk_ring(output_ring, TransferBuffer::Write(buf.as_ref()));
                //             info!("Transfer Result: {:?}", r);
                //
                //             let mut buf: Vec<u8> = Vec::new();
                //             buf.resize(36, 0);
                //
                //             let r = self.transfer_bulk_ring(input_ring, TransferBuffer::Read(buf.as_mut()));
                //             info!("Transfer Result: {:?}", r);
                //             info!("Got: {:x?}", buf.as_slice());
                //
                //             let mut buf: Vec<u8> = Vec::new();
                //             buf.resize(13, 0);
                //
                //             let r = self.transfer_bulk_ring(input_ring, TransferBuffer::Read(buf.as_mut()));
                //             info!("Transfer Result: {:?}", r);
                //             info!("Got: {:x?}", buf.as_slice());
                //
                //             self.hal.sleep(Duration::from_secs(5));
                //         }
                //         CLASS_CODE_HID => {
                //             if interface.interface.sub_class != 1 {
                //                 debug!("Skipping non bios-mode HID device");
                //                 continue;
                //             }
                //
                //             if interface.interface.protocol != 1 {
                //                 debug!("Skipping non keyboard");
                //                 continue;
                //             }
                //
                //             if interface.endpoints.len() == 0 {
                //                 warn!("keyboard with no endpoints!");
                //                 continue;
                //             }
                //
                //
                //             // Enable keyboard
                //
                //             self.with_input_context(port, |_this, input_ctx| {
                //
                //                 // self.configure_endpoint(port.slot_id, input_ctx.as_mut(),
                //                 //                         interface.endpoints[0].address,
                //                 //                         EP_TYPE_INTERRUPT_IN, interface.endpoints[0].max_packet_size,
                //                 //                         interface.endpoints[0].interval, self.get_max_esti_payload(&interface.endpoints[0]));
                //
                //
                //                 input_ctx.set_configure_ep_meta(configuration.config.config_val,
                //                                                 interface.interface.interface_number, interface.interface.alt_set);
                //
                //                 Ok(())
                //             })?;
                //
                //             debug!("done keyboard configure endpoint");
                //
                //             self.hal.sleep(Duration::from_millis(100));
                //
                //             debug!("set hid idle");
                //             self.set_hid_idle(port.slot_id)?;
                //
                //             // 0 is Boot Protocol
                //             debug!("set hid protocol");
                //             self.set_hid_protocol(port.slot_id, 0)?;
                //
                //
                //         }
                CLASS_CODE_HUB => {
                    HubDriver::<H>::probe(&device, interface)
                }
                _ => {}
            }
        }
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

pub struct HubDriver<H: HAL2> {
    __phantom: PhantomData<H>,
}

impl<H: HAL2> HubDriver<H> {
    pub fn probe(device: &Arc<RwLock<USBDevice>>, interface: &USBInterfaceDescriptorSet) {
        if interface.endpoints.len() == 0 {
            warn!("Hub with no endpoints!");
        }
        if interface.endpoints.len() > 1 {
            warn!("Hub with more than 1 endpoint!");
        }

        let mut hub_descriptor = USBHubDescriptor::default();
        USBHost::<H>::fetch_descriptor(&device, RequestType::Class, DESCRIPTOR_TYPE_HUB, 0, 0, &mut hub_descriptor);
        info!("Hub Descriptor: {:?}", hub_descriptor);

        // // Get Status
        // let mut buf = [0u8; 2];
        // self.send_control_command(port.slot_id,
        //                           0x80,
        //                           0x0, // Get Status
        //                           0x0, 0x0,
        //                           2, None, Some(&mut buf),
        // )?;
        // debug!("Status Read back: {:?}", buf);

        // Setup EPs
        debug!("Found {} eps on this interface", interface.endpoints.len());


        // Reconfigure to hub
        // {
        //     self.with_input_context(port, |_this, input_ctx| {
        //         let mut slot_ctx = input_ctx.get_slot_mut();
        //         slot_ctx.dword1.set_hub(true);
        //         slot_ctx.numbr_ports = hub_descriptor.num_ports;
        //         slot_ctx.slot_state = 0;
        //
        //         slot_ctx.interrupter_ttt = 0;
        //
        //         Ok(())
        //     })?;
        //
        //     // TODO handle hub with no endpoints
        //     self.with_input_context(port, |this, input_ctx| {
        //         this.configure_endpoint(port.slot_id, input_ctx,
        //                                 interface.endpoints[0].bEndpointAddress,
        //                                 EP_TYPE_INTERRUPT_IN, interface.endpoints[0].wMaxPacketSize,
        //                                 interface.endpoints[0].bInterval, this.get_max_esti_payload(&interface.endpoints[0]));
        //
        //         input_ctx.set_configure_ep_meta(configuration.config.config_val,
        //                                         interface.interface.interface_number, interface.interface.alt_set);
        //
        //         Ok(())
        //     })?;
        // }

        // debug!("slot state = {}", self.device_contexts[port.slot_id as usize - 1].as_ref().unwrap().get_slot().slot_state);

        let mut hub_descriptor = USBHubDescriptor::default();
        USBHost::<H>::fetch_descriptor(&device, RequestType::Class, DESCRIPTOR_TYPE_HUB, 0, 0, &mut hub_descriptor);
        info!("Hub Descriptor Pt2: {:?}", hub_descriptor);


        for num in 1..=hub_descriptor.bNbrPorts {
            Self::set_feature(device, num, FEATURE_PORT_POWER);

            H::sleep(Duration::from_millis(hub_descriptor.bPwrOn2PwrGood as u64 * 2));

            Self::clear_feature(device, num, FEATURE_C_PORT_CONNECTION);

            // self.send_control_command(slot_id, request_type!(DeviceToHost, Class, Other), REQUEST_GET_STATUS, 0, port_id as u16, 4, None, Some(as_mut_slice(&mut status)))?;

            let mut status = Self::fetch_port_status(device, num);

            debug!("Port {}: status={:?}", num, status);

            if !status.get_device_connected() {
                continue;
            }

            debug!("Acquiring locks");

            let child_parent_device = device.clone();

            let dev_lock = device.read();

            let child_bus = dev_lock.bus.clone();
            let bus_lock = dev_lock.bus.read();
            let slot = bus_lock.controller.allocate_slot();

            debug!("creating new child device");

            debug!("resetting device");
            Self::reset_port(device, num);

            let status = Self::fetch_port_status(device, num);

            let child_device = USBHost::<H>::new_device(Some(child_parent_device), child_bus, status.get_speed(), slot as u32, num)
                .unwrap_or_else(|e| panic!("{:?}", e));

            debug!("opening control pipe");
            // open the control "endpoint" on the root hub.
            bus_lock.controller.pipe_open(&child_device, None);

            H::sleep(Duration::from_millis(10));
            debug!("Going to fetch descriptor...");

            let mut buf = [0u8; 8];
            USBHost::<H>::fetch_descriptor_slice(&child_device, RequestType::Standard, DESCRIPTOR_TYPE_DEVICE, 0, 0, &mut buf);
            debug!("First fetch descriptor: {:?}", buf);

            {
                let mut d = child_device.write();
                d.ddesc.bMaxPacketSize = buf[7];
            }

            debug!("setting address");
            bus_lock.controller.set_address(&child_device, slot as u32);

            debug!("recursive setup_new_device()");
            USBHost::<H>::setup_new_device(child_device.clone());
        }
    }

    fn set_feature(device: &Arc<RwLock<USBDevice>>, port: u8, feature: u8) {
        USBHost::<H>::device_control_transfer(&device, ControlCommand {
            request_type: request_type!(HostToDevice, Class, Other),
            request: REQUEST_SET_FEATURE,
            value: feature as u16,
            index: port as u16,
            length: 0,
            buffer: TransferBuffer::None,
        });
    }

    fn clear_feature(device: &Arc<RwLock<USBDevice>>, port: u8, feature: u8) {
        USBHost::<H>::device_control_transfer(&device, ControlCommand {
            request_type: request_type!(HostToDevice, Class, Other),
            request: REQUEST_CLEAR_FEATURE,
            value: feature as u16,
            index: port as u16,
            length: 0,
            buffer: TransferBuffer::None,
        });
    }

    fn fetch_port_status(device: &Arc<RwLock<USBDevice>>, port: u8) -> PortStatus {
        let mut status = PortStatus::default();
        USBHost::<H>::device_control_transfer(&device, ControlCommand {
            request_type: request_type!(DeviceToHost, Class, Other),
            request: REQUEST_GET_STATUS,
            value: 0,
            index: port as u16,
            length: 4,
            buffer: TransferBuffer::Read(as_mut_slice(&mut status)),
        });
        status
    }

    fn reset_port(device: &Arc<RwLock<USBDevice>>, port: u8) {
        Self::set_feature(device, port, FEATURE_PORT_RESET);

        H::wait_until("failed to reset port", PORT_RESET_TIMEOUT, || {
            let status = Self::fetch_port_status(device, port);
            status.get_change_reset()
        }).unwrap_or_else(|e| panic!("{:?}", e));

        Self::clear_feature(device, port, FEATURE_C_PORT_RESET);
    }
}

pub struct MassStorageDriver<H: HAL2> {
    __phantom: PhantomData<H>,
}

impl<H: HAL2> MassStorageDriver<H> {
    pub fn probe(device: &Arc<RwLock<USBDevice>>, interface: &USBInterfaceDescriptorSet) {
        if interface.interface.bInterfaceSubClass != 0x6 {
            debug!("Skipping MSD with sub-class other than 0x6 (Transparent SCSI)");
            return;
        }

        if interface.interface.bInterfaceProtocol != 0x50 {
            debug!("Skipping MSD with protocol other than bulk-only");
            return;
        }

        if interface.endpoints.len() < 2 {
            warn!("MSD has not enough endpoints!");
            return;
        }

        debug!("acquiring locks");

        let dev_lock = device.read();
        let bus_lock = dev_lock.bus.read();

        let mut input_ep: Option<Arc<RwLock<USBPipe>>> = None;
        let mut output_ep: Option<Arc<RwLock<USBPipe>>> = None;

        for endpoint in interface.endpoints.iter() {
            if endpoint.bmAttributes != EP_ATTR_BULK {
                continue;
            }

            debug!("opening pipe: {}", endpoint.bEndpointAddress);

            let pipe = bus_lock.controller.pipe_open(device, Some(endpoint)).unwrap_or_else(|e| panic!("failed to open endpoint"));

            if endpoint.is_input() {
                input_ep = Some(pipe);
            } else {
                output_ep = Some(pipe);
            }
        }

        let input_ep = input_ep.ok_or(Error::Str("MSD has no bulk input endpoint")).unwrap_or_else(|e| panic!("{:?}", e));
        let output_ep = output_ep.ok_or(Error::Str("MSD has no bulk output endpoint")).unwrap_or_else(|e| panic!("{:?}", e));

        debug!("locking endpoints");

        let input_lock = input_ep.read();
        let output_lock = output_ep.read();

        let mut buf: Vec<u8> = Vec::new();
        buf.resize(31, 0);

        let t = [0x55, 0x53, 0x42, 0x43, 0x13, 0x37, 0x04, 0x20, 0x24, 0x00, 0x00, 0x00, 0x80, 0x00, 6, 0x12, 0x00, 0x00, 0x00, 0x24, 0x00];
        (&mut buf[..t.len()]).copy_from_slice(&t);

        let r = output_lock.bulk_write(buf.as_slice());
        info!("Transfer Result: {:?}", r);

        let mut buf: Vec<u8> = Vec::new();
        buf.resize(36, 0);

        let r = input_lock.bulk_read(buf.as_mut_slice());
        info!("Transfer Result: {:?}", r);
        info!("Got: {:x?}", buf.as_slice());

        let mut buf: Vec<u8> = Vec::new();
        buf.resize(13, 0);

        let r = input_lock.bulk_read(buf.as_mut_slice());
        info!("Transfer Result: {:?}", r);
        info!("Got: {:x?}", buf.as_slice());

        H::sleep(Duration::from_secs(5));
    }
}




