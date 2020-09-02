use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::marker::PhantomData;
use core::sync::atomic::AtomicU8;
use core::sync::atomic::Ordering;
use core::time::Duration;

use spin::RwLock;

use crate::{as_mut_slice, USBErrorKind, UsbHAL, USBHost, USBResult};
use crate::collection::SyncArray;
use crate::consts::*;
use crate::descriptor::{USBHubDescriptor, USBInterfaceDescriptorSet};
use crate::items::{ControlCommand, EndpointType, PortStatus, RequestType, TransferBuffer};
use crate::layer::PipeAsyncListener;
use crate::structs::{DeviceState, USBDevice, USBDeviceDriver};

pub struct HubDriver<H: UsbHAL> {
    host: Arc<USBHost<H>>,
    device: Arc<RwLock<USBDevice>>,
    num_ports: AtomicU8,
    interface: USBInterfaceDescriptorSet,
    hub_descriptor: RwLock<Option<USBHubDescriptor>>,
    children: SyncArray<Option<Arc<RwLock<USBDevice>>>>,
    __phantom: PhantomData<H>,
}

impl<H: UsbHAL> USBDeviceDriver for HubDriver<H> {
    fn on_attach(&self) -> USBResult<()> {
        let mut hub_descriptor = USBHubDescriptor::default();
        USBHost::<H>::fetch_descriptor(&self.device, RequestType::Class, DESCRIPTOR_TYPE_HUB, 0, 0, &mut hub_descriptor)?;
        info!("Hub Descriptor: {:?}", hub_descriptor);

        *self.hub_descriptor.write() = Some(hub_descriptor.clone());
        self.num_ports.store(hub_descriptor.bNbrPorts, Ordering::Relaxed);

        // Setup EPs
        debug!("Found {} eps on this interface", self.interface.endpoints.len());

        let controller = self.device.read().bus.controller.clone();
        controller.configure_hub(&self.device, hub_descriptor.bNbrPorts, hub_descriptor.wHubCharacteristics.get_tt_think_time()).expect("Unable to configure hub");

        if let Some(ep_interrupt) = self.interface.find_endpoint(EndpointType::Interrupt, true) {
            debug!("has an interrupt endpoint");

            let pipe = controller.pipe_open(&self.device, Some(ep_interrupt))?;

            crate::layer::pipe_async_listener(pipe, 5, ep_interrupt.wMaxPacketSize as usize,
                                              Self::create_interrupt_listener(&self.device)?)?;
        }

        for num in 1..=hub_descriptor.bNbrPorts {
            if let Err(e) = self.probe_port(num) {
                error!("failed to probe hub port {}: {:?}", num, e);
            }
        }

        Ok(())
    }

    fn on_disconnect(&self) {
        debug!("hub disconnected");

        for num in 1..=self.num_ports.load(Ordering::Relaxed) {
            if let Some(child) = self.children.replace(num as usize, None) {
                let mut lock = child.write();
                lock.handle_disconnect(H::current_time());
            }
        }
    }
}

impl<H: UsbHAL> HubDriver<H> {
    pub fn probe(host: &Arc<USBHost<H>>, device: &Arc<RwLock<USBDevice>>, interface: &USBInterfaceDescriptorSet) -> USBResult<()> {
        if interface.interface.bInterfaceClass != CLASS_CODE_HUB {
            return Ok(());
        }

        if interface.endpoints.len() == 0 {
            warn!("Hub with no endpoints!");
        }
        if interface.endpoints.len() > 1 {
            warn!("Hub with more than 1 endpoint!");
        }

        USBDevice::attach_driver(device, Arc::new(HubDriver::<H> {
            host: host.clone(),
            device: device.clone(),
            interface: interface.clone(),
            hub_descriptor: RwLock::new(None),
            num_ports: AtomicU8::new(0),
            children: SyncArray::new(256),
            __phantom: PhantomData::default(),
        }))
    }

    fn create_interrupt_listener(device: &Arc<RwLock<USBDevice>>) -> USBResult<PipeAsyncListener> {
        let driver: Arc<HubDriver<H>> = USBDevice::get_driver(device.read())
            .ok_or(USBErrorKind::InvalidState.msg("driver not attached???"))?
            .downcast_arc().map_err(|_| USBErrorKind::InvalidState.msg("wrong driver???"))?;

        let driver = Arc::downgrade(&driver);

        Ok(Arc::new(move |slice, result| {
            info!("hub status: {:?} res:{:?}", slice, result);

            let arc_driver = match driver.upgrade() {
                Some(a) => a,
                None => return,
            };

            for (byte_i, byte) in slice.iter().cloned().enumerate() {
                for i in 0..8 {
                    // index=0 hub change, index=1-n port 1-n change
                    let index = byte_i * 8 + i;
                    if ((byte >> i) & 0b1) != 0 {
                        if index == 0 {
                            debug!("hub change");
                        } else {
                            Self::interrupt_on_port_change(arc_driver.clone(), index as u8);
                        }
                    }
                }
            }
        }))
    }

    fn interrupt_on_port_change(this: Arc<Self>, index: u8) {
        H::queue_task_fn(move || {
            match Self::fetch_port_status(&this.device, index) {
                Ok(status) => {
                    debug!("port status {}: {:?}", index, &status);

                    if status.get_change_device_connected() {
                        Self::clear_feature(&this.device, index, FEATURE_C_PORT_CONNECTION);
                    }
                    if status.get_change_port_enable() {
                        Self::clear_feature(&this.device, index, FEATURE_C_PORT_ENABLE);
                    }
                    if status.get_change_suspend() {
                        Self::clear_feature(&this.device, index, FEATURE_C_PORT_SUSPEND);
                    }
                    if status.get_change_over_current() {
                        Self::clear_feature(&this.device, index, FEATURE_C_PORT_OVER_CURRENT);
                    }
                    if status.get_change_reset() {
                        Self::clear_feature(&this.device, index, FEATURE_C_PORT_RESET);
                    }

                    if status.get_device_connected() && status.get_change_device_connected() {
                        H::queue_task_fn(move || {
                            if let Err(e) = this.probe_port(index) {
                                warn!("probe error: {:?}", e);
                            }
                        });
                    } else if !status.get_device_connected() && status.get_change_device_connected() {
                        H::queue_task_fn(move || {
                            if let Some(device) = this.children.replace(index as usize, None) {
                                let mut dev_lock = device.write();
                                dev_lock.handle_disconnect(H::current_time());
                                core::mem::drop(dev_lock);
                                info!("device disconnected. Arc {{ strong: {}, weak: {} }}", Arc::strong_count(&device), Arc::weak_count(&device));
                            } else {
                                warn!("got device disconnect for a device never created: port={}", index);
                            }
                        });
                    }
                }
                Err(e) => warn!("port error {}: {:?}", index, e),
            }
        });
    }

    fn probe_port(&self, port: u8) -> USBResult<()> {
        let hub_desc_lock = self.hub_descriptor.read();
        let hub_descriptor = hub_desc_lock.as_ref().unwrap();

        Self::set_feature(&self.device, port, FEATURE_PORT_POWER)?;
        H::sleep(Duration::from_millis(hub_descriptor.bPwrOn2PwrGood as u64 * 2));

        Self::clear_feature(&self.device, port, FEATURE_C_PORT_CONNECTION)?;
        let mut status = Self::fetch_port_status(&self.device, port)?;

        debug!("Port {}: status={:?}", port, status);

        if !status.get_device_connected() {
            return Ok(());
        }

        debug!("Acquiring locks");

        let child_parent_device = self.device.clone();

        let dev_lock = self.device.read();

        let child_bus = dev_lock.bus.clone();
        let controller = dev_lock.bus.controller.clone();
        let slot = controller.allocate_slot()?;

        debug!("creating new child device");

        debug!("resetting device");
        Self::reset_port(&self.device, port)?;

        let status = Self::fetch_port_status(&self.device, port)?;

        let child_device = USBHost::<H>::new_device(Some(child_parent_device), child_bus, status.get_speed(), slot as u32, port)?;

        debug!("opening control pipe");
        // open the control "endpoint" on the root hub.
        controller.pipe_open(&child_device, None)?;

        debug!("Going to fetch descriptor...");

        let mut buf = [0u8; 8];
        USBHost::<H>::fetch_descriptor_slice(&child_device, RequestType::Standard, DESCRIPTOR_TYPE_DEVICE, 0, 0, &mut buf)?;
        debug!("First fetch descriptor: {:?}", buf);

        {
            let mut d = child_device.write();
            d.ddesc.bMaxPacketSize = buf[7];
        }

        debug!("setting address");
        controller.set_address(&child_device, slot as u32)?;

        core::mem::drop(dev_lock);

        debug!("recursive setup_new_device()");
        USBHost::<H>::setup_new_device(&self.host, child_device.clone())?;

        self.children.set(port as usize, Some(child_device));

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

