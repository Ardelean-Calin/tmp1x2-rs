//! This is a platform agnostic Rust driver for the TMP102 and TMP112
//! high-accuracy, low-power, digital temperature sensors, based on the
//! [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
//!
//! This driver allows you to:
//! - Enable/disable the device.
//! - Read the temperature.
//!
//! ## The devices
//!
//! TODO
//! 
//! Datasheets:
//! - [TMP102](http://www.ti.com/lit/ds/symlink/tmp102.pdf)
//! - [TMP112x](http://www.ti.com/lit/ds/symlink/tmp112.pdf)
//!
//! This driver is also compatible with the TMP112 family of devices, including
//! TMP112A, TMP112B and TMP112N.
//!
//! ## Usage examples (see also examples folder)
//!
//! ### Read temperature
//!
//! Import this crate and an `embedded_hal` implementation, then instantiate
//! the device:
//!
//! ```no_run
//! extern crate linux_embedded_hal as hal;
//! extern crate tmp1x2;
//!
//! use hal::I2cdev;
//! use tmp1x2::{ Tmp1x2, SlaveAddr };
//!
//! # fn main() {
//! let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! let address = SlaveAddr::default();
//! let mut sensor = Tmp1x2::new(dev, address);
//! let temperature = sensor.read_temperature().unwrap();
//! println!("Temperature: {}", temperature);
//! # }
//! ```
//!
//! ### Provide an alternative address
//!
//! ```no_run
//! extern crate linux_embedded_hal as hal;
//! extern crate tmp1x2;
//!
//! use hal::I2cdev;
//! use tmp1x2::{ Tmp1x2, SlaveAddr };
//!
//! # fn main() {
//! let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! let (a1, a0) = (false, true);
//! let address = SlaveAddr::Alternative(a1, a0);
//! let mut sensor = Tmp1x2::new(dev, address);
//! # }
//! ```
//!
//! ### Enable / disable the sensor
//!
//! ```no_run
//! extern crate linux_embedded_hal as hal;
//! extern crate tmp1x2;
//!
//! use hal::I2cdev;
//! use tmp1x2::{ Tmp1x2, SlaveAddr };
//!
//! # fn main() {
//! let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! let mut sensor = Tmp1x2::new(dev, SlaveAddr::default());
//! sensor.disable().unwrap(); // shutdown
//! sensor.enable().unwrap();
//! # }
//! ```

#![deny(unsafe_code)]
#![deny(missing_docs)]
#![no_std]

extern crate embedded_hal as hal;
use hal::blocking::i2c;

/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    /// I²C bus error
    I2C(E),
}

/// Possible slave addresses
#[derive(Debug, Clone)]
pub enum SlaveAddr {
    /// Default slave address
    Default,
    /// Alternative slave address providing bit values for A1 and A0
    Alternative(bool, bool)
}

impl Default for SlaveAddr {
    /// Default slave address
    fn default() -> Self {
        SlaveAddr::Default
    }
}

impl SlaveAddr {
    fn addr(self, default: u8) -> u8 {
        match self {
            SlaveAddr::Default => default,
            SlaveAddr::Alternative(a1, a0) => default           |
                                              ((a1 as u8) << 1) |
                                                a0 as u8
        }
    }
}


const DEVICE_BASE_ADDRESS: u8 = 0b100_1000;

struct Register;

impl Register {
    const TEMPERATURE : u8 = 0x00;
    const CONFIG      : u8 = 0x01;
}

struct BitFlagsLow;

impl BitFlagsLow {
    const SHUTDOWN        : u8 = 0b0000_0001;
    const RESOLUTION      : u8 = 0b0110_0000;
}

struct BitFlagsHigh;

impl BitFlagsHigh {
    const ALERT         : u8 = 0b0010_0000;
    const CONV_RATE1    : u8 = 0b1000_0000;
}

#[derive(Debug, Clone)]
struct Config {
    lsb: u8,
    msb: u8
}

impl Default for Config {
    fn default() -> Self {
        Config { lsb: BitFlagsLow::RESOLUTION,
                 msb: BitFlagsHigh::ALERT | BitFlagsHigh::CONV_RATE1 }
    }
}

/// TMP1X2 device driver.
#[derive(Debug, Default)]
pub struct Tmp1x2<I2C> {
    /// The concrete I²C device implementation.
    i2c: I2C,
    /// The I²C device address.
    address: u8,
    /// Configuration register status.
    config: Config,
}

impl<I2C, E> Tmp1x2<I2C>
where
    I2C: i2c::Write<Error = E>
{
    /// Create new instance of the TMP1X2 device.
    pub fn new(i2c: I2C, address: SlaveAddr) -> Self {
        Tmp1x2 {
            i2c,
            address: address.addr(DEVICE_BASE_ADDRESS),
            config: Config::default()
        }
    }

    /// Destroy driver instance, return I²C bus instance.
    pub fn destroy(self) -> I2C {
        self.i2c
    }
}

mod configuration;
mod conversion;
mod reading;

#[cfg(test)]
mod tests {
    use super::*;
    extern crate embedded_hal_mock as hal;

    #[test]
    fn can_get_default_address() {
        let addr = SlaveAddr::default();
        assert_eq!(DEVICE_BASE_ADDRESS, addr.addr(DEVICE_BASE_ADDRESS));
    }

    #[test]
    fn can_generate_alternative_addresses() {
        assert_eq!(0b100_1000, SlaveAddr::Alternative(false, false).addr(DEVICE_BASE_ADDRESS));
        assert_eq!(0b100_1001, SlaveAddr::Alternative(false,  true).addr(DEVICE_BASE_ADDRESS));
        assert_eq!(0b100_1010, SlaveAddr::Alternative( true, false).addr(DEVICE_BASE_ADDRESS));
        assert_eq!(0b100_1011, SlaveAddr::Alternative( true,  true).addr(DEVICE_BASE_ADDRESS));
    }

    #[test]
    fn default_config() {
        let dev = Tmp1x2::new(hal::i2c::Mock::new(&[]), SlaveAddr::default());
        assert_eq!(0b0110_0000, dev.config.lsb);
        assert_eq!(0b1010_0000, dev.config.msb);
    }
}