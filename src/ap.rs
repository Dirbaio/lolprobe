use std::rc::Rc;

use crate::dp::Dp;

pub struct Ap {
    dp: Rc<Dp>,
    address: u8,
}

impl Ap {
    pub fn new(dp: Rc<Dp>, address: u8) -> Self {
        Self { dp, address }
    }

    pub fn read(&self, addr: u8) -> Result<u32, anyhow::Error> {
        self.dp.read_ap(self.address, addr)
    }

    pub fn write(&self, addr: u8, val: u32) -> Result<(), anyhow::Error> {
        self.dp.write_ap(self.address, addr, val)
    }
}
