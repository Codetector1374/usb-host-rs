use alloc::vec::Vec;
use core::fmt;

use spin::Mutex;

pub struct SyncArray<T>(Vec<Mutex<T>>);

impl<T> SyncArray<T> {
    pub fn set(&self, index: usize, value: T) {
        let mut lock = self.0[index].lock();
        *lock = value;
    }

    pub fn replace(&self, index: usize, value: T) -> T {
        let mut lock = self.0[index].lock();
        core::mem::replace(&mut lock, value)
    }
}

impl<T: Default> SyncArray<T> {
    pub fn new(size: usize) -> Self {
        let mut vec = Vec::new();
        vec.reserve_exact(size);
        for _ in 0..size {
            vec.push(Mutex::new(T::default()));
        }
        Self(vec)
    }
}

impl<T: Clone> SyncArray<T> {
    pub fn new_with_default(size: usize, def: T) -> Self {
        let mut vec = Vec::new();
        vec.reserve_exact(size);
        for _ in 0..size {
            vec.push(Mutex::new(def.clone()));
        }
        Self(vec)
    }

    pub fn get_cloned(&self, index: usize) -> T {
        let lock = self.0[index].lock();
        lock.clone()
    }
}

impl<T: fmt::Debug> fmt::Debug for SyncArray<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "SyncArray({:?})", self.0)
    }
}

