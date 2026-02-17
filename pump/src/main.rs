#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

mod flow_sensor;
mod tmc2209;

use crate::{
    flow_sensor::{FlowSensor, FlowSensorInfo, LiquidType},
    tmc2209::{StopMode, Tmc2209},
};
use alloc::{sync::Arc, vec::Vec};
use common::{mutex::Mutex, usb::PacketStream};
use core::result;
use defmt::{debug, info, Debug2Format, Format};
use deku::prelude::*;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_hal::{
    self,
    clock::CpuClock,
    gpio::{Level::Low, Output, OutputConfig},
    interrupt::software::SoftwareInterruptControl,
    peripherals::{TIMG0, USB_DEVICE},
    system::Stack,
    time,
    timer::{
        timg::{MwdtStage, TimerGroup, Wdt},
        AnyTimer, PeriodicTimer,
    },
};
use heapless::Deque;
use thiserror::Error;

use panic_rtt_target as _;

extern crate alloc;

// Request/response codes.
const INIT: u8 = 0x00;
const FLOW_SENSOR_INFO: u8 = 0x01;
const SET_FLOW_UL_PER_MIN: u8 = 0x02;
const SET_PUMP_RPM: u8 = 0x03;
const GET_PUMP_RPM: u8 = 0x04;
const GET_RMS_AMPS: u8 = 0x05;
const SET_RMS_AMPS: u8 = 0x06;
const GET_STOP_RMS_AMPS: u8 = 0x07;
const SET_STOP_RMS_AMPS: u8 = 0x08;
const GET_STOP_MODE: u8 = 0x09;
const SET_STOP_MODE: u8 = 0x0A;
const GET_MOTOR_LOAD: u8 = 0x3F;
const GET_FLOW_HISTORY: u8 = 0x4A;
const GET_VALVE: u8 = 0x4B;
const SET_VALVE: u8 = 0x4C;
const GET_FLUSH_TIME: u8 = 0x4D;
const SET_FLUSH_TIME: u8 = 0x4E;
const GET_FLUSH_RPM: u8 = 0x4F;
const SET_FLUSH_RPM: u8 = 0x50;
const GET_FLUSHING: u8 = 0x51;
const FAIL: u8 = 0xFF;

const FLOW_HISTORY_LEN: usize = 1000;

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite, Format)]
#[deku(id_type = "u8", endian = "big")]
enum Request {
    #[deku(id = "INIT")]
    Init,

    #[deku(id = "FLOW_SENSOR_INFO")]
    FlowSensorInfo,

    #[deku(id = "SET_FLOW_UL_PER_MIN")]
    SetFlowUlPerMin(f64),

    #[deku(id = "SET_PUMP_RPM")]
    SetPumpRpm(f64),

    #[deku(id = "GET_PUMP_RPM")]
    GetPumpRpm,

    #[deku(id = "GET_RMS_AMPS")]
    GetRmsAmps,

    #[deku(id = "SET_RMS_AMPS")]
    SetRmsAmps(f64),

    #[deku(id = "GET_STOP_RMS_AMPS")]
    GetStoppedRmsAmps,

    #[deku(id = "SET_STOP_RMS_AMPS")]
    SetStoppedRmsAmps(f64),

    #[deku(id = "GET_STOP_MODE")]
    GetStopMode,

    #[deku(id = "SET_STOP_MODE")]
    SetStopMode(#[deku(bits = 8)] StopMode),

    #[deku(id = "GET_MOTOR_LOAD")]
    GetMotorLoad,

    #[deku(id = "GET_FLOW_HISTORY")]
    GetFlowHistory,

    #[deku(id = "GET_VALVE")]
    GetValve,

    #[deku(id = "SET_VALVE")]
    SetValve(bool),

    #[deku(id = "GET_FLUSH_TIME")]
    GetFlushTime,

    #[deku(id = "SET_FLUSH_TIME")]
    SetFlushTime(f64),

    #[deku(id = "GET_FLUSH_RPM")]
    GetFlushRpm,

    #[deku(id = "SET_FLUSH_RPM")]
    SetFlushRpm(f64),

    #[deku(id = "GET_FLUSHING")]
    GetFlushing,
}

#[derive(Debug, Clone, DekuRead, DekuWrite, Format)]
#[deku(id_type = "u8", endian = "big")]
enum Response {
    #[deku(id = "INIT")]
    Init(u8),

    #[deku(id = "FLOW_SENSOR_INFO")]
    FlowSensorInfo(FlowSensorInfo),

    #[deku(id = "SET_FLOW_UL_PER_MIN")]
    SetFlowUlPerMin,

    #[deku(id = "SET_PUMP_RPM")]
    SetPumpRpm,

    #[deku(id = "GET_PUMP_RPM")]
    GetPumpRpm(f64),

    #[deku(id = "GET_RMS_AMPS")]
    GetRmsAmps(f64),

    #[deku(id = "SET_RMS_AMPS")]
    SetRmsAmps,

    #[deku(id = "GET_STOP_RMS_AMPS")]
    GetStoppedRmsAmps(f64),

    #[deku(id = "SET_STOP_RMS_AMPS")]
    SetStoppedRmsAmps,

    #[deku(id = "GET_STOP_MODE")]
    GetStopMode(#[deku(bits = 8)] StopMode),

    #[deku(id = "SET_STOP_MODE")]
    SetStopMode,

    #[deku(id = "GET_MOTOR_LOAD")]
    GetMotorLoad(u16),

    #[deku(id = "GET_FLOW_HISTORY")]
    GetFlowHistory {
        len: u16,
        #[deku(count = "len")]
        samples: Vec<f64>,
    },

    #[deku(id = "GET_VALVE")]
    GetValve(bool),

    #[deku(id = "SET_VALVE")]
    SetValve,

    #[deku(id = "GET_FLUSH_TIME")]
    GetFlushTime(f64),

    #[deku(id = "SET_FLUSH_TIME")]
    SetFlushTime,

    #[deku(id = "GET_FLUSH_RPM")]
    GetFlushRpm(f64),

    #[deku(id = "SET_FLUSH_RPM")]
    SetFlushRpm,

    #[deku(id = "GET_FLUSHING")]
    GetFlushing(bool),

    #[deku(id = "FAIL")]
    Fail,
}

#[derive(Error, Debug)]
pub enum HandleRequestError {
    #[error("TMC error: {error}")]
    Tmc { error: tmc2209::Tmc2209Error },
    #[error("Flow sensor error: {error:?}")]
    FlowSensor { error: flow_sensor::FlowSensorError },
}

impl From<tmc2209::Tmc2209Error> for HandleRequestError {
    fn from(error: tmc2209::Tmc2209Error) -> Self {
        HandleRequestError::Tmc { error }
    }
}

impl From<flow_sensor::FlowSensorError> for HandleRequestError {
    fn from(error: flow_sensor::FlowSensorError) -> Self {
        HandleRequestError::FlowSensor { error }
    }
}

type Result<T> = result::Result<T, HandleRequestError>;

impl Format for HandleRequestError {
    fn format(&self, f: defmt::Formatter) {
        match self {
            HandleRequestError::Tmc { error } => {
                defmt::write!(f, "TMC error: {}", error)
            }
            HandleRequestError::FlowSensor { error } => {
                defmt::write!(f, "Flow sensor error: {:?}", Debug2Format(error))
            }
        }
    }
}

esp_bootloader_esp_idf::esp_app_desc!();

type TmcHandle<'a> = Arc<Tmc2209<'a>>;
type ValveHandle<'a> = Arc<Mutex<Output<'a>>>;

static RPM: Mutex<f64> = Mutex::new(0.0);
static FLUSH_TIME: Mutex<f64> = Mutex::new(0.0);
static FLUSH_RPM: Mutex<f64> = Mutex::new(0.0);
static FLUSH_MODE: Mutex<bool> = Mutex::new(false);
static UL_PER_MIN: Mutex<f64> = Mutex::new(f64::NAN);
static FLOW_HISTORY: Mutex<Deque<f64, FLOW_HISTORY_LEN>> = Mutex::new(Deque::new());
static FLOW_INFO: Mutex<FlowSensorInfo> = Mutex::new(FlowSensorInfo {
    air_in_line: false,
    high_flow: false,
    exponential_smoothing_active: false,
    ul_per_min: 0.0,
    degrees_c: 0.0,
});

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

    let tmc = Arc::new(
        Tmc2209::new(
            peripherals.UART1,
            peripherals.GPIO13, // TX
            peripherals.GPIO10, // RX
            peripherals.GPIO9,  // STEP
            peripherals.GPIO8,  // DIR
            peripherals.GPIO7,  // ENABLE
        )
        .unwrap(),
    );
    tmc.set_sense_ohms(0.110).unwrap();
    let irc = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    static mut STACK: Stack<8192> = Stack::new();
    let tmc_ref = Arc::clone(&tmc);
    #[allow(static_mut_refs)]
    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        irc.software_interrupt0,
        irc.software_interrupt1,
        unsafe { &mut STACK },
        move || run_tmc(tmc_ref, timers.timer1),
    );

    let sensor = FlowSensor::new(peripherals.I2C0, peripherals.GPIO2, peripherals.GPIO1);
    let valve = Arc::new(Mutex::new(Output::new(
        peripherals.GPIO4,
        Low,
        OutputConfig::default(),
    )));
    spawner
        .spawn(run_flow_rate_monitor(sensor, Arc::clone(&valve)))
        .unwrap();
    spawner.spawn(run_reset_monitor(Arc::clone(&tmc))).unwrap();
    run_coordinator(peripherals.USB_DEVICE, tmc, valve).await;
}

async fn run_coordinator<'a>(
    device: USB_DEVICE<'a>,
    tmc: TmcHandle<'a>,
    valve: ValveHandle<'a>,
) -> ! {
    let mut stream = PacketStream::new(device, Duration::from_secs(1));
    loop {
        let Ok(packet) = stream.read().await else {
            info!("Packet read failed.");
            continue;
        };

        debug!("Received packet: {=[?]}", &packet[..]);
        let response = match Request::from_bytes((&packet, 0)) {
            Ok((_, packet)) => match handle_request(packet, &tmc, &valve).await {
                Ok(response) => response,
                Err(err) => {
                    info!("Handle request error: {}", err);
                    Response::Fail
                }
            },
            Err(err) => {
                info!("Failed to decode request: {:?}", Debug2Format(&err));
                Response::Fail
            }
        };

        let response_bytes = response.to_bytes().unwrap();
        debug!("Sending response: {=[?]}", &response_bytes[..]);
        stream.write(&response_bytes).await;
    }
}

async fn handle_request<'a>(
    packet: Request,
    tmc: &TmcHandle<'a>,
    valve: &ValveHandle<'a>,
) -> Result<Response> {
    let response = match packet {
        Request::Init => Response::Init(0),
        Request::FlowSensorInfo => Response::FlowSensorInfo(FLOW_INFO.get_cloned()),
        Request::SetFlowUlPerMin(ul_per_min) => {
            UL_PER_MIN.set(ul_per_min);
            Response::SetFlowUlPerMin
        }
        Request::SetPumpRpm(rpm) => {
            RPM.set(rpm);
            Response::SetPumpRpm
        }
        Request::GetPumpRpm => Response::GetPumpRpm(RPM.get_cloned()),
        Request::GetRmsAmps => Response::GetRmsAmps(tmc.rms_amps()?),
        Request::SetRmsAmps(amps) => {
            tmc.set_rms_amps(amps)?;
            Response::SetRmsAmps
        }
        Request::GetStoppedRmsAmps => Response::GetStoppedRmsAmps(tmc.stopped_rms_amps()?),
        Request::SetStoppedRmsAmps(amps) => {
            tmc.set_stopped_rms_amps(amps)?;
            Response::SetStoppedRmsAmps
        }
        Request::GetStopMode => Response::GetStopMode(tmc.stop_mode()?),
        Request::SetStopMode(mode) => {
            tmc.set_stop_mode(mode)?;
            Response::SetStopMode
        }
        Request::GetMotorLoad => Response::GetMotorLoad(tmc.motor_load()?),
        Request::GetFlowHistory => {
            let samples: Vec<f64> = FLOW_HISTORY.lock(|history| {
                let mut samples = Vec::with_capacity(history.len());
                while let Some(sample) = history.pop_front() {
                    samples.push(sample);
                }
                samples
            });
            Response::GetFlowHistory {
                len: samples.len() as u16,
                samples,
            }
        }
        Request::GetValve => Response::GetValve(valve.lock(|v| v.is_set_high())),
        Request::SetValve(open) => {
            valve.lock(|v| if open { v.set_high() } else { v.set_low() });
            Response::SetValve
        }
        Request::GetFlushTime => Response::GetFlushTime(FLUSH_TIME.get_cloned()),
        Request::SetFlushTime(seconds) => {
            FLUSH_TIME.set(seconds);
            Response::SetFlushTime
        }
        Request::GetFlushRpm => Response::GetFlushRpm(FLUSH_RPM.get_cloned()),
        Request::SetFlushRpm(rpm) => {
            FLUSH_RPM.set(rpm);
            Response::SetFlushRpm
        }
        Request::GetFlushing => Response::GetFlushing(FLUSH_MODE.get_cloned()),
    };
    Ok(response)
}

#[embassy_executor::task]
async fn run_watchdog_monitor(mut watchdog: Wdt<TIMG0<'static>>) -> ! {
    loop {
        watchdog.feed();
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::task]
async fn run_flow_rate_monitor(mut sensor: FlowSensor<'static>, valve: ValveHandle<'static>) -> ! {
    info!("Flow sensor initialized.");
    sensor.start(LiquidType::Water).await.unwrap();
    let mut flush_end = Instant::now();
    let mut seen_liquid = false;
    loop {
        let info = sensor.read().await.unwrap();
        let flush_time = FLUSH_TIME.get_cloned();
        if !info.air_in_line {
            seen_liquid = true;
        } else if seen_liquid {
            FLUSH_MODE.set(true);
            valve.lock(|v| v.set_low());
            flush_end = Instant::now() + Duration::from_millis((flush_time * 1000.0) as u64);
        }

        if Instant::now() > flush_end {
            FLUSH_MODE.set(false);
            valve.lock(|v| v.set_high());
        }
        FLOW_INFO.set(info);
        FLOW_HISTORY.lock(|history| {
            if history.is_full() {
                history.pop_front();
            }
            history.push_back(info.ul_per_min).unwrap();
        });
        Timer::after_millis(1).await;
    }
}

#[embassy_executor::task]
async fn run_reset_monitor(tmc: TmcHandle<'static>) -> ! {
    loop {
        if tmc.is_reset().unwrap() {
            info!("TMC2209 reset detected, re-applying current config.");
            let running = tmc.rms_amps().unwrap();
            let stopped = tmc.stopped_rms_amps().unwrap();
            tmc.set_rms_amps(running).unwrap();
            tmc.set_stopped_rms_amps(stopped).unwrap();
        }
        Timer::after_secs(1).await;
    }
}

fn run_tmc(
    tmc: TmcHandle<'static>,
    timer: impl esp_hal::timer::Timer + Into<AnyTimer<'static>>,
) -> ! {
    let step_us = 10;
    let mut step_timer = PeriodicTimer::new(timer);
    step_timer
        .start(time::Duration::from_micros(step_us))
        .unwrap();
    tmc.enable();
    info!("TMC2209 initialized.");
    let mut v = 0.0;
    let a = 1.0;
    let mut elapsed_us = 0;
    loop {
        let rpm = if FLUSH_MODE.get_cloned() {
            FLUSH_RPM.get_cloned()
        } else {
            RPM.get_cloned()
        };
        let target_v = rpm / 60.0;
        if v < target_v - 0.01 * a {
            v += a * 0.01
        } else if v > target_v + 0.01 * a {
            v -= a * 0.01
        } else {
            v = target_v
        };

        tmc.set_direction(v >= 0.0);
        let dt = (1e6 / (v.abs() + 1e-10)) as u64 / tmc.pulses_per_rev() as u64;
        for _ in 0..1000 {
            step_timer.wait();
            elapsed_us += step_us;
            if elapsed_us >= dt {
                tmc.toggle_step();
                elapsed_us %= dt;
            }
        }
    }
}
