#![allow(dead_code)]

use core::result;
use defmt::{info, Format};
use deku::prelude::*;
use embassy_time::Timer;
use esp_hal::{
    gpio::interconnect::PeripheralOutput,
    i2c::master::{Config, Error as I2cError, I2c, Instance},
    time::Rate,
    Async,
};
use thiserror::Error;

const SENSOR_ADDR: u8 = 0x08;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Command {
    StartWater,
    StartIpa,
    Stop,
    SoftReset,
}

impl Command {
    fn as_bytes(&self) -> [u8; 2] {
        match self {
            Command::StartWater => [0x36, 0x08],
            Command::StartIpa => [0x36, 0x15],
            Command::Stop => [0x3F, 0xF9],
            Command::SoftReset => [0x00, 0x06],
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LiquidType {
    Water,
    Ipa,
}

#[derive(Error, Debug)]
pub enum FlowSensorError {
    #[error("I2C error")]
    I2cError,
    #[error("CRC mismatch")]
    CrcMismatch,
}

impl From<I2cError> for FlowSensorError {
    fn from(err: I2cError) -> Self {
        info!("I2C error: {:?}", err);
        FlowSensorError::I2cError
    }
}

pub type Result<T> = result::Result<T, FlowSensorError>;

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite, Format)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
pub struct FlowSensorInfo {
    pub air_in_line: bool,
    pub high_flow: bool,
    pub exponential_smoothing_active: bool,
    pub ul_per_min: f64,
    pub degrees_c: f64,
}

pub struct FlowSensor<'a> {
    i2c: I2c<'a, Async>,
}

impl<'a> FlowSensor<'a> {
    pub fn new(
        i2c: impl Instance + 'a,
        sda: impl PeripheralOutput<'a>,
        scl: impl PeripheralOutput<'a>,
    ) -> Self {
        let i2c = I2c::new(i2c, Config::default().with_frequency(Rate::from_khz(400)))
            .unwrap()
            .with_sda(sda)
            .with_scl(scl)
            .into_async();
        Self { i2c }
    }

    pub async fn start(&mut self, liquid: LiquidType) -> Result<()> {
        self.stop().await?;
        Timer::after_millis(20).await;
        let cmd = match liquid {
            LiquidType::Water => Command::StartWater,
            LiquidType::Ipa => Command::StartIpa,
        };
        let cmd_bytes = cmd.as_bytes();
        info!("Starting flow sensor with command: {=[?]}", &cmd_bytes[..]);
        self.i2c.write_async(SENSOR_ADDR, &cmd_bytes).await?;
        Timer::after_millis(20).await;
        Ok(())
    }

    pub async fn reset(&mut self) -> Result<()> {
        self.i2c
            .write_async(SENSOR_ADDR, &Command::SoftReset.as_bytes())
            .await?;
        Ok(())
    }

    pub async fn read(&mut self) -> Result<FlowSensorInfo> {
        let mut buf = [0u8; 9];
        self.i2c.read_async(SENSOR_ADDR, &mut buf).await?;

        // Verify CRCs
        if crc8([buf[0], buf[1]]) != buf[2]
            || crc8([buf[3], buf[4]]) != buf[5]
            || crc8([buf[6], buf[7]]) != buf[8]
        {
            return Err(FlowSensorError::CrcMismatch);
        }

        // Parse flow rate (sensor reports in µL/min scaled by 10)
        let flow_raw = i16::from_be_bytes([buf[0], buf[1]]);
        let ul_per_min = flow_raw as f64 / 10.0;

        // Parse temperature
        let temp_raw = i16::from_be_bytes([buf[3], buf[4]]);
        let degrees_c = temp_raw as f64 / 200.0;

        // Parse status word
        let status = u16::from_be_bytes([buf[6], buf[7]]);
        let air_in_line = (status & 0x0001) != 0;
        let high_flow = (status & 0x0002) != 0;
        let exponential_smoothing_active = (status & 0x0004) != 0;

        Ok(FlowSensorInfo {
            air_in_line,
            high_flow,
            exponential_smoothing_active,
            ul_per_min,
            degrees_c,
        })
    }

    pub async fn stop(&mut self) -> Result<()> {
        self.i2c
            .write_async(SENSOR_ADDR, &Command::Stop.as_bytes())
            .await?;
        Ok(())
    }
}

fn crc8(two_bytes: [u8; 2]) -> u8 {
    // CRC-8 polynomial 0x31, init 0xFF, no reflect, xorout 0x00
    // Matches datasheet Table 15.
    let mut crc: u8 = 0xFF;
    for &b in &two_bytes {
        crc ^= b;
        for _ in 0..8 {
            crc = if (crc & 0x80) != 0 {
                (crc << 1) ^ 0x31
            } else {
                crc << 1
            };
        }
    }
    crc
}
