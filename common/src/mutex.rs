use core::cell::RefCell;
use embassy_sync::blocking_mutex::Mutex as EmbassyMutex;

pub struct Mutex<T> {
    mutex: EmbassyMutex<esp_sync::RawMutex, RefCell<T>>,
}

impl<T> Mutex<T> {
    pub const fn new(value: T) -> Self {
        Self {
            mutex: EmbassyMutex::new(RefCell::new(value)),
        }
    }

    pub fn lock<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        self.mutex.lock(|cell| f(&mut *cell.borrow_mut()))
    }

    pub fn get_cloned(&self) -> T
    where
        T: Clone,
    {
        self.mutex.lock(|cell| cell.borrow().clone())
    }

    pub fn set(&self, value: T) {
        self.mutex.lock(|cell| *cell.borrow_mut() = value)
    }
}
