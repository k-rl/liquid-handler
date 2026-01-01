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
use embassy_time::{Duration, Timer};
use esp_hal::{
    self,
    clock::CpuClock,
    peripherals::{TIMG0, USB_DEVICE},
    time,
    timer::timg::{MwdtStage, TimerGroup, Wdt},
};
use panic_rtt_target as _;

extern crate alloc;

// Request/response codes.
const INIT: u8 = 0x00;
const FLOW_SENSOR_INFO: u8 = 0x01;
const SET_PUMP_RPS: u8 = 0x02;
const GET_PUMP_RPS: u8 = 0x03;
const FAIL: u8 = 0xFF;

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite, Format)]
#[deku(id_type = "u8", endian = "big")]
pub enum Request {
    #[deku(id = "INIT")]
    Init,

    #[deku(id = "FLOW_SENSOR_INFO")]
    FlowSensorInfo,

    #[deku(id = "SET_PUMP_RPS")]
    SetPumpRps(f64),

    #[deku(id = "GET_PUMP_RPS")]
    GetPumpRps,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite, Format)]
#[deku(id_type = "u8", endian = "big")]
pub enum Response {
    #[deku(id = "INIT")]
    Init,

    #[deku(id = "FLOW_SENSOR_INFO")]
    FlowSensorInfo(FlowSensorInfo),

    #[deku(id = "SET_PUMP_RPS")]
    SetPumpRps,

    #[deku(id = "GET_PUMP_RPS")]
    GetPumpRps(f64),

    #[deku(id = "FAIL")]
    Fail,
}

esp_bootloader_esp_idf::esp_app_desc!();

static RPS: Mutex<CriticalSectionRawMutex, f64> = Mutex::new(0.0);

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    rtt_target::rtt_init_defmt!();
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 73744);
    let timers = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timers.timer0);

    info!("Embassy initialized.");
    Timer::after_secs(2).await;

    let mut watchdog = timers.wdt;
    watchdog.set_timeout(MwdtStage::Stage3, time::Duration::from_secs(1));
    watchdog.enable();
    spawner.spawn(run_watchdog_monitor(watchdog)).unwrap();

    // Initialize TMC2209 stepper driver
    let tmc = tmc2209::Tmc2209::new(
        peripherals.UART1,
        peripherals.GPIO13, // TX
        peripherals.GPIO10, // RX
        peripherals.GPIO9,  // STEP
        peripherals.GPIO8,  // DIR
        peripherals.GPIO7,  // ENABLE
    );
    spawner.spawn(run_tmc(tmc)).unwrap();

    // Initialize the sensor and coordinator.
    let mut sensor = FlowSensor::new(peripherals.I2C0, peripherals.GPIO2, peripherals.GPIO1);
    info!("Flow sensor initialized.");
    sensor.start(LiquidType::Water).await.unwrap();
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
                Request::SetPumpRps(rps) => {
                    *RPS.lock().await = rps;
                    Response::SetPumpRps
                }
                Request::GetPumpRps => Response::GetPumpRps(*RPS.lock().await),
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
async fn run_watchdog_monitor(mut watchdog: Wdt<TIMG0<'static>>) -> ! {
    loop {
        watchdog.feed();
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::task]
async fn run_tmc(mut tmc: Tmc2209<'static>) -> ! {
    tmc.enable();
    tmc.init().await.unwrap();
    info!("TMC2209 initialized.");
    let mut v = 0.0;
    let a = 1.0;
    let dx = 1.0 / (tmc.pulses_per_rev() as f64);
    let dv2 = 2.0 * a * dx;
    let mut i = 0;
    loop {
        let prev_v = v;
        let target_v = *RPS.lock().await;
        let v2 = v * v;
        let diff = target_v * target_v - v2;
        if diff.abs() < dv2 {
            v = target_v;
        } else {
            v = libm::sqrt(v2 + diff.signum() * dv2);
        }

        tmc.toggle_step();
        let dt = 2.0 * dx / (v + prev_v + 1e-10);
        if i % 1000 == 0 {
            info!("====Iteration: {}====", i);
            info!("dt: {}", dt);
            info!("dx: {}", dx);
            info!("v2: {}", v2);
            info!("target_v2: {}", target_v * target_v);
            info!("prev_v: {}", prev_v);
            info!("v: {}", v);
            info!("diff: {}", diff);
            info!("target_v: {}", target_v);
        }

        if dt < 1.0 {
            Timer::after_micros((dt * 1e6) as u64).await;
        } else {
            Timer::after_millis(10).await;
        }
        i += 1;
    }
}
