use core::ops::Deref;

use bitflags::bitflags;
use esp32_hal::{
    i2c::I2C,
    prelude::{
        _embedded_hal_blocking_delay_DelayMs, _embedded_hal_blocking_i2c_Read,
        _embedded_hal_blocking_i2c_Write,
    },
    Delay,
};
use esp_println::println;

const AHT10_ADDR: u8 = 0x38;

pub struct AHT10<'d, I> {
    pub i2c: I2C<'d, I>,
    pub delay: Delay,
}

bitflags! {
    struct StatusFlags: u8 {
        const BUSY = 0b00000001;
        const MODE = (0b00000010 | 0b00000100);
        const CRC = 0b00001000;
        const CALIBRATION_ENABLE = 0b00010000;
        const FIFO_ENABLE = 0b00100000;
        const FIFO_FULL = 0b01000000;
        const FIFO_EMPTY = 0b10000000;
    }
}

#[allow(dead_code)]
#[derive(Clone, Copy)]
#[repr(u8)]
enum Commands {
    Calibrate = 0xE1,
    GetRaw = 0xA8,
    GetData = 0xAC,
    Reset = 0xBA,
}

#[derive(Debug)]
pub enum Error {
    WriteError,
    ReadError,
    Uncalibrated,
}

impl<'d, I> AHT10<'d, I>
where
    I: esp32_hal::i2c::Instance,
{
    pub fn new(i2c: I2C<'d, I>, delay: Delay) -> Result<Self, self::Error> {
        let mut aht10 = Self { i2c, delay };

        // Soft reset the sensor
        aht10
            .i2c
            .write(AHT10_ADDR, &[Commands::Reset as u8])
            .map_err(|_e| {
                println!("{:?}", _e);
                self::Error::WriteError
            })?;
        aht10.delay.delay_ms(20u32);

        aht10.write_cmd(Commands::Calibrate, 0x0800)?;
        aht10.wait_for_calibration()?;

        Ok(aht10)
    }

    // Waits for the AHT10 to tell us its calibrated
    fn wait_for_calibration(&mut self) -> Result<(), self::Error> {
        loop {
            let status = self.read_status()?;

            if status.contains(StatusFlags::BUSY) {
                self.delay.delay_ms(10u32);
                continue;
            }

            if !status.contains(StatusFlags::CALIBRATION_ENABLE) {
                return Err(self::Error::Uncalibrated);
            }

            return Ok(());
        }
    }

    // Reads the status received from the AHT10
    fn read_status(&mut self) -> Result<StatusFlags, self::Error> {
        let buf: &mut [u8; 1] = &mut [0; 1];
        self.i2c
            .read(AHT10_ADDR, buf)
            .map_err(|_e| self::Error::ReadError)?;

        let status = StatusFlags { bits: buf[0] };
        Ok(status)
    }

    // Writes a command over the i2c interface
    fn write_cmd(&mut self, cmd: Commands, dat: u16) -> Result<(), self::Error> {
        self.i2c
            .write(
                AHT10_ADDR,
                &[cmd as u8, (dat >> 8) as u8, (dat & 0xff) as u8],
            )
            .map_err(|_e| self::Error::WriteError)
    }

    fn send_get_data_cmd(&mut self) -> Result<(), self::Error> {
        self.i2c
            .write(AHT10_ADDR, &[Commands::GetData as u8, 0x33, 0])
            .map_err(|_e| self::Error::WriteError)
    }

    // Soft resets the sensor
    pub fn soft_reset_sensor(&mut self) -> Result<(), self::Error> {
        self.write_cmd(Commands::Reset, 0)?;
        self.delay.delay_ms(0u32);
        Ok(())
    }

    // Reads the humidity
    pub fn get_humidity(&mut self) -> Result<Humidity, self::Error> {
        self.send_get_data_cmd()?;
        self.wait_for_calibration()?;

        let buf: &mut [u8; 6] = &mut [0; 6];
        self.i2c
            .read(AHT10_ADDR, buf)
            .map_err(|_e| self::Error::ReadError)?;

        let hum = (buf[1] as u32) << 12 | ((buf[2] as u32) << 4) | ((buf[3] as u32) >> 4);

        Ok(Humidity::new(hum))
    }

    // Reads the temperature
    pub fn get_temperature(&mut self) -> Result<Temperature, self::Error> {
        self.send_get_data_cmd()?;
        self.wait_for_calibration()?;

        let buf: &mut [u8; 6] = &mut [0; 6];
        self.i2c
            .read(AHT10_ADDR, buf)
            .map_err(|_e| self::Error::ReadError)?;

        let temp = ((buf[3] as u32) & 0x0f << 16) | ((buf[4] as u32) << 8) | (buf[5] as u32);

        Ok(Temperature::new(temp))
    }
}

pub struct Temperature {
    temperature: u32,
}

impl Temperature {
    pub fn new(temp: u32) -> Self {
        Self { temperature: temp }
    }

    pub fn celsius(&self) -> f32 {
        200.0 * (self.temperature as f32) / ((1 << 20) as f32)
    }

    pub fn raw(&self) -> u32 {
        self.temperature
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Humidity {
    humidity: u32,
}

impl Deref for Humidity {
    type Target = u32;
    fn deref(&self) -> &Self::Target {
        &self.humidity
    }
}

impl Humidity {
    pub fn new(hum_bits: u32) -> Self {
        Self { humidity: hum_bits }
    }

    pub fn relative(&self) -> f32 {
        100.0 * (self.humidity as f32) / ((1 << 20) as f32)
    }

    pub fn raw(&self) -> u32 {
        self.humidity
    }
}
