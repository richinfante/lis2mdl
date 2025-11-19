#![no_std]
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{I2c, Operation};

const DEFAULT_DEVICE_ID: u8 = 0x1E;
pub const LIS2MDL_CFG_REG_A: u8 = 0x60;
pub const LIS2MDL_CFG_REG_B: u8 = 0x61;
pub const LIS2MDL_CFG_REG_C: u8 = 0x62;
const LIS2MDL_OUTX_L_REG: u8 = 0x68;
const LIS2MDL_WHO_AM_I_REG: u8 = 0x4F;
const DELAY_TIME: u32 = 125;
const CHIP_ID: u8 = 0x40;
const LIS2MDL_MAG_LSB: f32 = 1.5; // mgauss/LSB
const LIS2MDL_MILLIGAUSS_TO_MICROTESLA: f32 = 0.1; // 1 mgauss = 0.1 microtesla
use micromath::F32Ext;

#[derive(Debug)]
pub struct Lis2mdl<I2C, DELAY> {
    pub(crate) i2c: I2C,
    pub(crate) address: u8,
    pub(crate) delay: DELAY,
    pub mag_x: i16,
    pub mag_y: i16,
    pub mag_z: i16,
    pub x_min: f32,
    pub x_max: f32,
    pub y_min: f32,
    pub y_max: f32,
}

#[derive(Debug)]
pub enum Error<E> {
    // I²C bus error
    I2C(E),
}

impl<I2C, DELAY, E> Lis2mdl<I2C, DELAY>
where
    DELAY: DelayNs,
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    pub fn new<A: Into<Address>>(i2c: I2C, address: A, delay: DELAY) -> Self {
        let a = address.into();
        Lis2mdl {
            i2c,
            address: a.0,
            delay,
            mag_x: 0,
            mag_y: 0,
            mag_z: 0,
            x_min: f32::MAX,
            x_max: f32::MIN,
            y_min: f32::MAX,
            y_max: f32::MIN,
        }
    }

    pub fn start(&mut self) -> Result<(), Error<E>> {
        // self.set_register(LIS2MDL_CFG_REG_A, 0x8C)?;
        // self.delay.delay_ns(10_000);
        // self.set_register(LIS2MDL_CFG_REG_C, 0x11)?;
        // self.delay.delay_ns(10_000);
            // First: ensure I²C mode by explicitly clearing I2C_DISABLE
        self.set_register(LIS2MDL_CFG_REG_C, 0x00)?;   // I²C enabled, auto-inc off
        self.delay.delay_ns(5000);

        // Now enable auto-increment + BDU
        self.set_register(LIS2MDL_CFG_REG_C, 0x11)?;   // IF_ADD_INC = 1, BDU = 1
        self.delay.delay_ns(5000);

        // Finally, set reliable ODR + temp comp
        self.set_register(LIS2MDL_CFG_REG_A, 0x00)?;   // continuous mode, ODR default
        self.delay.delay_ns(5000);

        self.set_register(LIS2MDL_CFG_REG_B, 0x00)?;
        self.delay.delay_ns(10_000);

        Ok(())
    }

    pub fn whoami(&mut self) -> Result<u8, Error<E>> {
        let mut buffer = [0u8; 1];
        let mut operations = [
            Operation::Write(&[LIS2MDL_WHO_AM_I_REG]),
            Operation::Read(&mut buffer),
        ];
        self.i2c
            .transaction(self.address, &mut operations)
            .map_err(Error::I2C)?;

        Ok(buffer[0])
    }

    pub fn get_register(&mut self, reg: u8) -> Result<u8, Error<E>> {
        let mut buffer = [0u8; 1];
        let mut operations = [
            Operation::Write(&[reg]),
            Operation::Read(&mut buffer),
        ];
        self.i2c
            .transaction(self.address, &mut operations)
            .map_err(Error::I2C)?;

        Ok(buffer[0])
    }

    pub fn set_register(&mut self, reg: u8, value: u8) -> Result<(), Error<E>> {
        self.i2c
            .write(self.address, &[reg, value])
            .map_err(Error::I2C)
    }

    pub fn current_xyz(&mut self) -> (f32, f32, f32) {
        let x = self.mag_x as f32 * LIS2MDL_MAG_LSB * LIS2MDL_MILLIGAUSS_TO_MICROTESLA;
        let y = self.mag_y as f32 * LIS2MDL_MAG_LSB * LIS2MDL_MILLIGAUSS_TO_MICROTESLA;
        let z = self.mag_z as f32 * LIS2MDL_MAG_LSB * LIS2MDL_MILLIGAUSS_TO_MICROTESLA;

        (x, y, z)
    }

    pub fn get_heading (&mut self) -> f32 {
        let (x, y, _z) = self.current_xyz();

        // save min/max for calibration
        self.x_max = self.x_max.max(x);
        self.x_min = self.x_min.min(x);
        self.y_max = self.y_max.max(y);
        self.y_min = self.y_min.min(y);

        // hard-iron calibration
        let x_offset = (self.x_max + self.x_min) / 2.0;
        let y_offset = (self.y_max + self.y_min) / 2.0;

        // apply calibration offsets
        let x = x - x_offset;
        let y = y - y_offset;

        let heading = y.atan2(x) * 180.0 / core::f32::consts::PI;
        if heading < 0.0 {
            heading + 360.0
        } else {
            heading
        }
    }

    pub fn read(&mut self) -> Result<(), Error<E>> {
        let mut buffer = [0u8; 6];
        let mut operations = [
            Operation::Write(&[LIS2MDL_OUTX_L_REG]),
            Operation::Read(&mut buffer),
        ];
        self.i2c
            .transaction(self.address, &mut operations)
            .map_err(Error::I2C)?;

        self.mag_x = i16::from_le_bytes([buffer[0], buffer[1]]);
        self.mag_y = i16::from_le_bytes([buffer[2], buffer[3]]);
        self.mag_z = i16::from_le_bytes([buffer[4], buffer[5]]);

        Ok(())
    }
}

// I2C device address
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct Address(pub(crate) u8);

impl Default for Address {
    fn default() -> Self {
        Address(DEFAULT_DEVICE_ID)
    }
}

impl From<u8> for Address {
    fn from(value: u8) -> Self {
        Address(value)
    }
}

impl Address {
    pub fn seq() -> Self {
        Address(DEFAULT_DEVICE_ID)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
}
