extern crate tmp1x2;
extern crate embedded_hal_mock as hal;
use hal::i2c::{ Transaction as I2cTransaction };

mod common;
use common::{ DEVICE_ADDRESS, setup, Register, BitFlagsLow as BFL, BitFlagsHigh as BFH };

const DEFAULT_CONFIG_MSB: u8 = BFH::CONV_RATE1 | BFH::ALERT;
const DEFAULT_CONFIG_LSB: u8 = BFL::RESOLUTION;

fn get_write_expectation(config_lsb: u8, config_msb: u8) -> [I2cTransaction; 1] {
    [
        I2cTransaction::write(DEVICE_ADDRESS, vec![Register::CONFIG, config_msb, config_lsb])
    ]
}

macro_rules! config_test {
    ($name:ident, $method:ident, $expected_lsb:expr, $expected_msb:expr) => {
        #[test]
        fn $name() {
            let expectations = get_write_expectation($expected_lsb, $expected_msb);
            let mut dev = setup(&expectations);
            dev.$method().unwrap();
            dev.destroy().done();
        }
    };
}

config_test!(can_enable,  enable,  DEFAULT_CONFIG_LSB,     DEFAULT_CONFIG_MSB);
config_test!(can_disable, disable, DEFAULT_CONFIG_LSB | 1, DEFAULT_CONFIG_MSB);

config_test!(can_enable_extended_mode,  enable_extended_mode,  DEFAULT_CONFIG_LSB, DEFAULT_CONFIG_MSB | BFH::EXTENDED_MODE);
config_test!(can_disable_extended_mode, disable_extended_mode, DEFAULT_CONFIG_LSB, DEFAULT_CONFIG_MSB);
