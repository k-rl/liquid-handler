use crate::flow_sensor::{FlowSensor, FlowSensorInfo, LiquidType};
use alloc::{vec, vec::Vec};
use core::{result, slice};
use defmt::{debug, info, Format};
use deku::prelude::*;
use embassy_time::{Duration, TimeoutError};
use embedded_io_async::{Read, Write};
use esp_hal::{peripherals::USB_DEVICE, usb_serial_jtag::UsbSerialJtag, Async};
use thiserror::Error;

#[derive(Error, Debug)]
pub enum Error {
    #[error("Timeout error")]
    Timeout,
    #[error("Received unexpected zero byte")]
    ZeroByte,
}

impl From<TimeoutError> for Error {
    fn from(_: TimeoutError) -> Self {
        Error::Timeout
    }
}

type Result<T> = result::Result<T, Error>;

// Request/response codes.
const INIT: u8 = 0x00;
const FLOW_SENSOR_INFO: u8 = 0x01;
const SET_PUMP_RPM: u8 = 0x02;
const FAIL: u8 = 0xFF;

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite, Format)]
#[deku(id_type = "u8", endian = "big")]
pub enum Request {
    #[deku(id = "INIT")]
    Init,

    #[deku(id = "FLOW_SENSOR_INFO")]
    FlowSensorInfo,

    #[deku(id = "SET_PUMP_RPM")]
    SetPumpRpm(f64),
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite, Format)]
#[deku(id_type = "u8", endian = "big")]
pub enum Response {
    #[deku(id = "INIT")]
    Init,

    #[deku(id = "FLOW_SENSOR_INFO")]
    FlowSensorInfo(FlowSensorInfo),

    #[deku(id = "SET_PUMP_RPM")]
    SetPumpRpm,

    #[deku(id = "FAIL")]
    Fail,
}

pub struct PacketStream<'a> {
    usb: UsbSerialJtag<'a, Async>,
    timeout: Duration,
}

/// Reads and writes packets one at a time, each packet is seperated using
/// [COBS](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing).
impl<'a> PacketStream<'a> {
    pub fn new(device: USB_DEVICE<'a>, timeout: Duration) -> Self {
        PacketStream {
            usb: UsbSerialJtag::new(device).into_async(),
            timeout,
        }
    }

    pub async fn write(&mut self, data: &[u8]) {
        if data.is_empty() {
            return;
        }

        let mut block_len = 1;
        let mut block_start = 0;
        let mut out = vec![0];
        for &b in data {
            if b != 0 {
                out.push(b);
                block_len += 1;
            }

            // Writing a zero or maxing out the block size means we start a new block.
            if b == 0 || block_len == 255 {
                out.push(0);
                out[block_start] = block_len;
                block_len = 1;
                block_start = out.len() - 1;
            }
        }

        // Finish the last block unless we just wrote a 255 length block.
        if (block_start != out.len() - 1) || (*data.last().unwrap() == 0) {
            out[block_start] = block_len;
            out.push(0);
        }

        info!("Sending response: {=[?]}", &out[..]);
        Write::write_all(&mut self.usb, &out).await.unwrap()
    }

    pub async fn read(&mut self) -> Result<Vec<u8>> {
        let mut size = self.read_byte().await;
        let mut vec = Vec::new();
        if size == 0 {
            return Ok(vec);
        }

        loop {
            let l = vec.len();
            vec.resize(l + size as usize - 1, 0);
            info!("Reading {} bytes", size);
            embassy_time::with_timeout(
                self.timeout,
                Read::read_exact(&mut self.usb, &mut vec[l..]),
            )
            .await?
            .unwrap();

            // If we read any zero byte then we're out of frame and should reset.
            if vec[l..].contains(&0) {
                let mut byte = 1;
                // Clear out the buffer until the next restart point.
                while byte != 0 {
                    byte = self.read_byte().await;
                }
                info!("Received unexpected zero byte.");
                break Err(Error::ZeroByte);
            }

            size = self.read_byte().await;
            match size {
                0 => break Ok(vec),
                255 => continue,
                _ => vec.push(0),
            }
        }
    }

    async fn read_byte(&mut self) -> u8 {
        let mut byte = 0u8;
        // Unwrap because we have an infallible error.
        Read::read_exact(&mut self.usb, slice::from_mut(&mut byte))
            .await
            .unwrap();
        byte
    }
}
