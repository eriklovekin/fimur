use core::marker::PhantomData;

pub enum Bank {
    Bank0,
    Bank1,
    Bank2,
    Bank3
}

pub trait RegisterBank {
    const bank: Bank;
}

pub struct Bank0;
impl RegisterBank for Bank0 {
    const bank: Bank = Bank::Bank0;
}

pub struct Bank1;
impl RegisterBank for Bank1 {
    const bank: Bank = Bank::Bank1;
}

pub struct Bank2;
impl RegisterBank for Bank2 {
    const bank: Bank = Bank::Bank2;
}

pub struct Bank3;
impl RegisterBank for Bank3 {
    const bank: Bank = Bank::Bank3;
}

struct Register<B: RegisterBank> {
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
}

pub mod bank0;

