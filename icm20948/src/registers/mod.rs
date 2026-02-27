use core::marker::PhantomData;

pub enum Bank {
    Bank0 = 0,
    Bank1 = 1,
    Bank2 = 2,
    Bank3 = 3
}

pub trait RegisterBank {
    const BANK: Bank;
}

pub struct Bank0;
impl RegisterBank for Bank0 {
    const BANK: Bank = Bank::Bank0;
}

pub struct Bank1;
impl RegisterBank for Bank1 {
    const BANK: Bank = Bank::Bank1;
}

pub struct Bank2;
impl RegisterBank for Bank2 {
    const BANK: Bank = Bank::Bank2;
}

pub struct Bank3;
impl RegisterBank for Bank3 {
    const BANK: Bank = Bank::Bank3;
}

pub struct Register<B: RegisterBank> {
    pub name: &'static str,
    pub addr: u8,
    _bank: PhantomData<B>,
}

impl<B: RegisterBank> Register<B> {
    pub const fn new(name: &'static str, addr: u8) -> Self {
        Self {
            name,
            addr,
            _bank: PhantomData,
        }
    }

    pub const fn get_bank(&self) -> u8 {
        B::BANK as u8
    }
}

pub mod bank0;
pub mod bank1;
pub mod bank2;
pub mod bank3;

pub const BITSHIFT_REG_SELECT:  u8 = 4;
pub const BITSHIFT_SCALE:       u8 = 1;