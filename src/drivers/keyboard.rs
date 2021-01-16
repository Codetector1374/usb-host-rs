use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::marker::PhantomData;
use core::time::Duration;

use spin::{RwLock, Mutex};

use crate::{USBErrorKind, UsbHAL, USBHost, USBResult};
use crate::consts::*;
use crate::descriptor::{USBInterfaceDescriptor, USBInterfaceDescriptorSet};
use crate::items::{ControlCommand, EndpointType, TransferBuffer};
use crate::structs::{USBDevice, USBDeviceDriver, USBPipe, DeviceState};
use core::ops::{Index, IndexMut};

struct KeyState {
    data: [u8; 32],
}

impl KeyState {
    pub fn new() -> Self {
        Self {
            data: [0u8; 32]
        }
    }

    /// This sets a key to true
    pub fn set(&mut self, index: u8) {
        let arr_idx = (index / 8) as usize;
        let offset = index % 8;
        self.data[arr_idx] |= (1u8 << offset);
    }

    pub fn get(&self, index: u8) -> bool {
        let arr_idx = (index / 8) as usize;
        let offset = index % 8;
        (self.data[arr_idx] >> offset) & 0x1 != 0
    }

    pub fn clear(&mut self) {
        self.data = [0u8; 32];
    }
}

pub struct HIDKeyboard<H: UsbHAL, C: HIDKeyboardCallback> {
    __phantom: PhantomData<H>,
    __phantom_cb: PhantomData<C>,
    key_state: Mutex<KeyState>,
    // device: Arc<RwLock<USBDevice>>,
    // pipe: Arc<RwLock<USBPipe>>,
}

pub trait HIDKeyboardCallback: Sync + Send + 'static {
    fn key_down(ascii: u8);
}

impl<H: UsbHAL, C: HIDKeyboardCallback> USBDeviceDriver for HIDKeyboard<H, C> {}

impl<H: UsbHAL, C: HIDKeyboardCallback> HIDKeyboard<H, C> {

    fn translate_keycode(code: u8) -> Option<u8> {
        match code {
            // a-z
            0x4..=0x1d => Some('a' as u8 + (code - 0x4)),
            0x1e..=0x27 => Some('1' as u8 + (code - 0x1e)),
            0x28 => Some(0x0A), // Enter -> LF
            0x2a => Some(0x08), // BKSP
            0x2c => Some(0x20), // Space
            0x4c => Some(0x7F), // DEL
            _ => {
                debug!("Unknown HID Code: {:#x}", code);
                None
            }
        }
    }

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
            d.device_state = DeviceState::Owned(Arc::new(
                HIDKeyboard::<H, C> {
                    __phantom: PhantomData::default(),
                    __phantom_cb: PhantomData::default(),
                    key_state: Mutex::new(KeyState::new()),
                }
            ))
        }

        // Enable keyboard

        let dev_lock = device.read();

        let pipe = dev_lock.bus.controller.pipe_open(device, Some(ep_interrupt)).unwrap_or_else(|e| panic!("{:?}", e));

        debug!("done keyboard configure endpoint");

        H::sleep(Duration::from_millis(1));

        debug!("set hid idle");
        Self::set_hid_idle(device)?;

        // 0 is Boot Protocol
        debug!("set hid protocol");
        Self::set_hid_protocol(device, 0)?;

        let driver: Arc<HIDKeyboard<H, C>> = USBDevice::get_driver(device.read())
            .ok_or(USBErrorKind::InvalidState.msg("driver not attached???"))?
            .downcast_arc().map_err(|_| USBErrorKind::InvalidState.msg("wrong driver???"))?;

        let driver = Arc::downgrade(&driver);

        crate::layer::pipe_async_listener(pipe.clone(), 20, 8, Arc::new(move |buf, _| {
            if let Some(arc_driver) = driver.upgrade() {
                let mut state = arc_driver.key_state.lock();
                let mut new_keys = [0u8; 8];
                for (idx, key) in buf.iter().enumerate() {
                    if !state.get(*key) {
                        new_keys[idx] = *key;
                    }
                }

                state.clear();

                for x in buf {
                    state.set(*x);
                }

                for x in new_keys.iter() {
                    if *x != 0 {
                        if let Some(x) = HIDKeyboard::<H, C>::translate_keycode(*x) {
                            C::key_down(x)
                        }
                    }
                }
            }
        }))?;

        // for _ in 0..20 {
        //     let cloned = pipe.clone();
        //     let mut buf = Vec::new();
        //     buf.reserve_exact(512);
        //     buf.resize(8, 0);
        //     Self::register_callback(cloned, buf);
        // }

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
                Ok(_) => trace!("got interrupt: {:?}", res_buf.as_slice()),
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
