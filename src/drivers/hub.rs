use alloc::sync::Arc;
use core::marker::PhantomData;
use core::time::Duration;

use spin::RwLock;

use crate::{as_mut_slice, USBErrorKind, UsbHAL, USBHost, USBResult};
use crate::consts::*;
use crate::descriptor::{USBHubDescriptor, USBInterfaceDescriptorSet};
use crate::items::{ControlCommand, PortStatus, RequestType, TransferBuffer};
use crate::structs::{DeviceState, USBDevice, USBDeviceDriver};

pub struct HubDriver<H: UsbHAL> {
    __phantom: PhantomData<H>,
}

impl<H: UsbHAL> USBDeviceDriver for HubDriver<H> {}

impl<H: UsbHAL> HubDriver<H> {
    pub fn probe(host: &Arc<USBHost<H>>, device: &Arc<RwLock<USBDevice>>, interface: &USBInterfaceDescriptorSet) -> USBResult<()> {
        if interface.interface.bInterfaceClass != CLASS_CODE_HUB {
            return Ok(())
        }

        if interface.endpoints.len() == 0 {
            warn!("Hub with no endpoints!");
        }
        if interface.endpoints.len() > 1 {
            warn!("Hub with more than 1 endpoint!");
        }

        {
            let mut d = device.write();
            d.device_state = DeviceState::Owned(Arc::new(HubDriver::<H> { __phantom: PhantomData::default() }))
        }

        let mut hub_descriptor = USBHubDescriptor::default();
        USBHost::<H>::fetch_descriptor(&device, RequestType::Class, DESCRIPTOR_TYPE_HUB, 0, 0, &mut hub_descriptor)?;
        info!("Hub Descriptor: {:?}", hub_descriptor);

        // Setup EPs
        debug!("Found {} eps on this interface", interface.endpoints.len());

        let controller = device.read().bus.read().controller.clone();
        controller.configure_hub(&device, hub_descriptor.bNbrPorts, hub_descriptor.wHubCharacteristics.get_tt_think_time()).expect("Unable to configure hub");

        // TODO: Potentially Configure the Interrupt EP for Hub (if there is one)

        for num in 1..=hub_descriptor.bNbrPorts {
            if let Err(e) = HubDriver::<H>::probe_port(host, device, &hub_descriptor, num) {
                error!("failed to probe hub port {}: {:?}", num, e);
            }
        }

        Ok(())
    }

    fn probe_port(host: &Arc<USBHost<H>>, device: &Arc<RwLock<USBDevice>>, hub_descriptor: &USBHubDescriptor, port: u8) -> USBResult<()> {
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

        core::mem::drop(bus_lock);
        core::mem::drop(dev_lock);

        debug!("recursive setup_new_device()");
        USBHost::<H>::setup_new_device(host, child_device.clone())?;

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

