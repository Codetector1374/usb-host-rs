#![allow(dead_code)]

use core::time::Duration;

/* --------- TIMEOUT Constants ----------------- */
pub const HALT_TIMEOUT: Duration = Duration::from_millis(16);
pub const RESET_TIMEOUT: Duration = Duration::from_millis(250);
pub const PORT_RESET_TIMEOUT: Duration = Duration::from_millis(1000);

/* --------- Standard Requests ---------------- */
pub const REQUEST_GET_STATUS: u8 = 0;
pub const REQUEST_CLEAR_FEATURE: u8 = 1;
pub const REQUEST_SET_FEATURE: u8 = 3;
pub const REQUEST_SET_ADDRESS: u8 = 5;
pub const REQUEST_GET_DESCRIPTOR: u8 = 6;
pub const REQUEST_SET_DESCRIPTOR: u8 = 7;
pub const REQUEST_GET_CONFIGURATION: u8 = 8;
pub const REQUEST_SET_CONFIGURATION: u8 = 9;
pub const REQUEST_GET_INTERFACE: u8 = 10;
pub const REQUEST_SET_INTERFACE: u8 = 11;

// HID
pub const REQUEST_GET_REPORT: u8 = 1;
pub const REQUEST_SET_REPORT: u8 = 0x9;
pub const REQUEST_SET_IDLE: u8 = 0xA;
pub const REQUEST_SET_PROTOCOL: u8 = 0xB;

/* Hub Requests */
pub const REQUEST_CLEAR_TT_BUFFER: u8 = 8;
pub const REQUEST_RESET_TT: u8 = 9;
pub const REQUEST_RESET_TT_DEFAULT_TT: u16 = 1;
pub const REQUEST_GET_TT_STATE: u8 = 10;
pub const REQUEST_STOP_TT: u8 = 11;

pub const DESCRIPTOR_TYPE_DEVICE: u8 = 1;
pub const DESCRIPTOR_TYPE_CONFIGURATION: u8 = 2;
pub const DESCRIPTOR_TYPE_STRING: u8 = 3;
pub const DESCRIPTOR_TYPE_INTERFACE: u8 = 4;
pub const DESCRIPTOR_TYPE_ENDPOINT: u8 = 5;
pub const DESCRIPTOR_TYPE_DEVICE_QUALIFIER: u8 = 6;
pub const DESCRIPTOR_TYPE_OTHER_SPEED_CONFIGURATION: u8 = 7;
pub const DESCRIPTOR_TYPE_INTERFACE_POWER: u8 = 8;
pub const DESCRIPTOR_TYPE_HUB: u8 = 0x29;
pub const DESCRIPTOR_TYPE_SS_HUB: u8 = 0x2A;

/* ---------- Class Codes ----------- */
pub const CLASS_CODE_HID: u8 = 3;
pub const CLASS_CODE_MASS: u8 = 8;
pub const CLASS_CODE_HUB: u8 = 9;

/* ---------- Feature Selector --------- */
pub const FEATURE_PORT_CONNECTION: u8 = 0x00;
pub const FEATURE_PORT_ENABLE: u8 = 0x01;
pub const FEATURE_PORT_SUSPEND: u8 = 0x02;
pub const FEATURE_PORT_OVER_CURRENT: u8 = 0x03;
pub const FEATURE_PORT_RESET: u8 = 0x04;
pub const FEATURE_PORT_POWER: u8 = 0x08;
pub const FEATURE_PORT_LOW_SPEED: u8 = 0x09;
pub const FEATURE_C_PORT_CONNECTION: u8 = 0x10;
pub const FEATURE_C_PORT_ENABLE: u8 = 0x11;
pub const FEATURE_C_PORT_SUSPEND: u8 = 0x12;
pub const FEATURE_C_PORT_OVER_CURRENT: u8 = 0x13;
pub const FEATURE_C_PORT_RESET: u8 = 0x14;
pub const FEATURE_PORT_TEST: u8 = 0x15;
pub const FEATURE_PORT_INDICATOR: u8 = 0x16;

/* -------- Endpoint values ---------- */
pub const EP_ATTR_CONTROL: u8 = 0;
pub const EP_ATTR_ISOCH: u8 = 1;
pub const EP_ATTR_BULK: u8 = 2;
pub const EP_ATTR_INTERRUPT: u8 = 3;

pub const EP_TYPE_NOT_VALID: u8 = 0;
pub const EP_TYPE_ISOCH_OUT: u8 = 1;
pub const EP_TYPE_BULK_OUT: u8 = 2;
pub const EP_TYPE_INTERRUPT_OUT: u8 = 3;
pub const EP_TYPE_CONTROL_BIDIR: u8 = 4;
pub const EP_TYPE_ISOCH_IN: u8 = 5;
pub const EP_TYPE_BULK_IN: u8 = 6;
pub const EP_TYPE_INTERRUPT_IN: u8 = 7;

#[repr(u8)]
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum USBSpeed {
    Invalid = 0,
    Low = 1,
    Full,
    High,
    Super,
    LastValue,
}

impl USBSpeed {
    pub fn from_raw(speed: u8) -> USBSpeed {
        if speed > USBSpeed::Invalid as u8 && speed < USBSpeed::LastValue as u8 {
            unsafe { core::mem::transmute(speed) }
        } else {
            USBSpeed::Invalid
        }
    }

    pub fn is_low_or_full_speed(&self) -> bool {
        *self == USBSpeed::Low || *self == USBSpeed::Full
    }
}