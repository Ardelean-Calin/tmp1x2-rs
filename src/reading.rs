#![deny(unsafe_code)]

extern crate embedded_hal as hal;
use hal::blocking::i2c;
use super::{ Tmp1x2, Register, BitFlagsLow, Error };

use conversion::convert_temp_from_register;

impl<I2C, E> Tmp1x2<I2C>
where
    I2C: i2c::WriteRead<Error = E>
{
    /// Read the temperature from the sensor.
    pub fn read_temperature(&mut self) -> Result<f32, Error<E>> {
        let mut data = [0; 2];
        self.i2c
            .write_read(self.address, &[Register::TEMPERATURE], &mut data)
            .map_err(Error::I2C)?;
        Ok(convert_temp_from_register(data[0], data[1]))
    }

    /// Read whether the one-shot measurement result is ready.
    ///
    /// See also: [trigger_one_shot_measurement()](#method.trigger_one_shot_measurement)
    pub fn is_one_shot_measurement_result_ready(&mut self) -> Result<bool, Error<E>> {
        let mut data = [0; 2];
        self.i2c
            .write_read(self.address, &[Register::CONFIG], &mut data)
            .map_err(Error::I2C)?;
        Ok((data[1] & BitFlagsLow::ONE_SHOT) != 0)
    }
}
