#![deny(unsafe_code)]

extern crate embedded_hal as hal;
use hal::blocking::i2c;
use super::{ Tmp1x2, Register, BitFlagsLow, BitFlagsHigh, Config, Error };


impl<I2C, E> Tmp1x2<I2C>
where
    I2C: i2c::Write<Error = E>
{
    /// Enable the sensor.
    pub fn enable(&mut self) -> Result<(), Error<E>> {
        let Config{ lsb, msb } = self.config;
        self.write_config(lsb & !BitFlagsLow::SHUTDOWN, msb)
    }

    /// Disable the sensor (shutdown).
    pub fn disable(&mut self) -> Result<(), Error<E>> {
        let Config{ lsb, msb } = self.config;
        self.write_config(lsb | BitFlagsLow::SHUTDOWN, msb)
    }

    /// Enable the extended measurement mode.
    ///
    /// This allows measurement of temperatures above 128°C.
    pub fn enable_extended_mode(&mut self) -> Result<(), Error<E>> {
        let Config{ lsb, msb } = self.config;
        self.write_config(lsb, msb | BitFlagsHigh::EXTENDED_MODE)
    }

    /// Disable the extended measurement mode.
    ///
    /// This puts the device in normal measurement mode. It will not measure
    /// temperatures above 128°C.
    pub fn disable_extended_mode(&mut self) -> Result<(), Error<E>> {
        let Config{ lsb, msb } = self.config;
        self.write_config(lsb, msb & !BitFlagsHigh::EXTENDED_MODE)
    }

    fn write_config(&mut self, lsb: u8, msb: u8) -> Result<(), Error<E>> {
        self.i2c
            .write(self.address, &[Register::CONFIG, msb, lsb])
            .map_err(Error::I2C)?;
        self.config = Config { lsb, msb};
        Ok(())
    }
}
