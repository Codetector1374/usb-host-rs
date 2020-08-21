use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::marker::PhantomData;
use core::time::Duration;

use spin::RwLock;

use crate::{USBErrorKind, UsbHAL, USBHost, USBResult};
use crate::consts::*;
use crate::descriptor::{USBInterfaceDescriptor, USBInterfaceDescriptorSet};
use crate::items::{ControlCommand, EndpointType, TransferBuffer};
use crate::structs::{USBDevice, USBDeviceDriver, USBPipe, DeviceState};

pub struct HIDKeyboard<H: UsbHAL> {
    __phantom: PhantomData<H>,
    // device: Arc<RwLock<USBDevice>>,
    // pipe: Arc<RwLock<USBPipe>>,
}

impl<H: UsbHAL> USBDeviceDriver for HIDKeyboard<H> {}

impl<H: UsbHAL> HIDKeyboard<H> {
    pub fn probe(device: &Arc<RwLock<USBDevice>>, interface: &USBInterfaceDescriptorSet) -> USBResult<()> {
        if interface.interface.bInterfaceClass != CLASS_CODE_HID {
            return Ok(());
        }

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

        {
            let mut d = device.write();
            d.device_state = DeviceState::Owned(Arc::new(HIDKeyboard::<H> { __phantom: PhantomData::default() }))
        }

        // Enable keyboard

        let dev_lock = device.read();

        let pipe = dev_lock.bus.controller.pipe_open(device, Some(ep_interrupt)).unwrap_or_else(|e| panic!("{:?}", e));

        debug!("done keyboard configure endpoint");

        H::sleep(Duration::from_millis(100));

        debug!("set hid idle");
        Self::set_hid_idle(device)?;

        // 0 is Boot Protocol
        debug!("set hid protocol");
        Self::set_hid_protocol(device, 0)?;


        for _ in 0..20 {
            let cloned = pipe.clone();
            let mut buf = Vec::new();
            buf.reserve_exact(512);
            buf.resize(8, 0);
            Self::register_callback(cloned, buf);
        }

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

    fn register_callback(pipe: Arc<RwLock<USBPipe>>, buf: Vec<u8>) {
        let cloned_pipe = pipe.clone();
        let pip = pipe.read();
        let res = pip.async_read(buf, Box::new(move |res_buf, result| {
            match result {
                Ok(_) => info!("got interrupt: {:?}", res_buf.as_slice()),
                Err(e) => warn!("failed async read: {:?}", e),
            }

            Self::register_callback(cloned_pipe, res_buf);
        }));
        if let Err(e) = res {
            warn!("failed to queue async read: {:?}", e);
        }
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
