use alloc::sync::Arc;
use core::marker::PhantomData;
use core::time::Duration;

use spin::RwLock;

use crate::{UsbHAL, USBErrorKind, USBHost, USBResult};
use crate::consts::*;
use crate::descriptor::{USBInterfaceDescriptor, USBInterfaceDescriptorSet};
use crate::items::{ControlCommand, TransferBuffer, EndpointType};
use crate::structs::{USBDevice, USBPipe};

pub struct HIDKeyboard<H: UsbHAL> {
    __phantom: PhantomData<H>,
    device: Arc<RwLock<USBDevice>>,
    pipe: Arc<RwLock<USBPipe>>,
}

impl<H: UsbHAL> HIDKeyboard<H> {
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
