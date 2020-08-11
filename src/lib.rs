#![feature(allocator_api)]
#![feature(const_in_array_repeat_expressions)]
#![feature(global_asm)]
#![feature(llvm_asm)]
#![allow(dead_code, unused_imports, unused_parens)]
#![cfg_attr(not(test), no_std)]

#[macro_use]
extern crate alloc;
#[macro_use]
extern crate log;

use std::sync::{Arc, RwLock};

use hashbrown::HashMap;

use crate::structs::{USBBus, USBDevice};
use crate::traits::USBHostController;

pub mod structs;
pub mod traits;


pub struct USBHost {
    root_hubs: HashMap<u32, Arc<RwLock<USBDevice>>>,
}


impl USBHost {
    pub fn attach_root_hub(&mut self, controller: Arc<dyn USBHostController>) {

    }

    pub fn new_device(&mut self, parent: Arc<USBDevice>, bus: Arc<USBBus>,
                      depth: u32, speed: u32, port: u8) {

    }
}