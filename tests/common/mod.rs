extern crate tmp1x2;
extern crate embedded_hal_mock as hal;
use hal::i2c::{ Mock as I2cMock, Transaction as I2cTransaction };
use tmp1x2::{ Tmp1x2, SlaveAddr };

pub const DEVICE_ADDRESS: u8 = 0b100_1000;

pub struct Register;

#[allow(unused)]
impl Register {
    pub const TEMPERATURE   : u8 = 0x00;
    pub const CONFIG    : u8 = 0x01;
}

pub struct BitFlagsLow;

#[allow(unused)]
impl BitFlagsLow {
    pub const SHUTDOWN        : u8 = 0b0000_0001;
    pub const RESOLUTION      : u8 = 0b0110_0000;
}

pub struct BitFlagsHigh;

#[allow(unused)]
impl BitFlagsHigh {
    pub const EXTENDED_MODE : u8 = 0b0001_0000;
    pub const ALERT         : u8 = 0b0010_0000;
    pub const CONV_RATE1    : u8 = 0b1000_0000;
}

pub fn setup(expectations: &[I2cTransaction]) -> Tmp1x2<I2cMock> {
    let i2c = I2cMock::new(&expectations);
    Tmp1x2::new(i2c, SlaveAddr::default())
}
