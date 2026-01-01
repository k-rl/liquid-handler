#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

mod flow_sensor;
mod tmc2209;
mod usb;

use crate::{
    flow_sensor::{FlowSensor, FlowSensorInfo, LiquidType},
    tmc2209::Tmc2209,
    usb::PacketStream,
};
use defmt::{info, Format};
use deku::prelude::*;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::Duration;
use embassy_time::Timer;
use esp_hal::{self, clock::CpuClock, peripherals::USB_DEVICE, timer::timg::TimerGroup};
use panic_rtt_target as _;

extern crate alloc;

// Request/response codes.
const INIT: u8 = 0x00;
const FLOW_SENSOR_INFO: u8 = 0x01;
const SET_PUMP_RPM: u8 = 0x02;
const GET_PUMP_RPM: u8 = 0x03;
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

    #[deku(id = "GET_PUMP_RPM")]
    GetPumpRpm,
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

    #[deku(id = "GET_PUMP_RPM")]
    GetPumpRpm(f64),

    #[deku(id = "FAIL")]
    Fail,
}

esp_bootloader_esp_idf::esp_app_desc!();

static RPM: Mutex<CriticalSectionRawMutex, f64> = Mutex::new(0.0);

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 73744);
    esp_rtos::start(TimerGroup::new(peripherals.TIMG0).timer0);
    info!("Embassy initialized.");

    // Initialize TMC2209 stepper driver
    let tmc = tmc2209::Tmc2209::new(
        peripherals.UART1,
        peripherals.GPIO13, // TX
        peripherals.GPIO10, // RX
        peripherals.GPIO9,  // STEP
        peripherals.GPIO8,  // DIR
        peripherals.GPIO7,  // ENABLE
    );

    let mut sensor = FlowSensor::new(peripherals.I2C0, peripherals.GPIO2, peripherals.GPIO1);
    info!("Flow sensor initialized.");
    sensor.start(LiquidType::Water).await.unwrap();

    spawner.spawn(run_tmc(tmc)).unwrap();
    Timer::after_secs(4).await;

    run_coordinator(peripherals.USB_DEVICE, sensor).await;
}

pub async fn run_coordinator<'a>(device: USB_DEVICE<'a>, mut sensor: FlowSensor<'a>) -> ! {
    let mut stream = PacketStream::new(device, Duration::from_secs(1));
    loop {
        let Ok(packet) = stream.read().await else {
            info!("Packet read failed.");
            continue;
        };

        info!("Received packet: {=[?]}", &packet[..]);
        let response = if let Ok((_, packet)) = Request::from_bytes((&packet, 0)) {
            match packet {
                Request::Init => Response::Init,
                Request::FlowSensorInfo => Response::FlowSensorInfo(sensor.read().await.unwrap()),
                Request::SetPumpRpm(rpm) => {
                    *RPM.lock().await = rpm;
                    Response::SetPumpRpm
                }
                Request::GetPumpRpm => Response::GetPumpRpm(*RPM.lock().await),
            }
        } else {
            Response::Fail
        };

        info!(
            "Sending response: {=[?]}",
            &response.to_bytes().unwrap()[..]
        );
        stream.write(&response.to_bytes().unwrap()).await;
    }
}

#[embassy_executor::task]
async fn run_tmc(mut tmc: Tmc2209<'static>) -> ! {
    tmc.enable();
    tmc.init().await.unwrap();
    info!("TMC2209 initialized.");
    loop {
        let rpm = *RPM.lock().await;
        info!("RPM: {}", rpm);
        Timer::after_secs(2).await;
        // tmc.step_loop(100.0, 100.0).await.unwrap();
        // Timer::after_millis(500).await;
    }
}
