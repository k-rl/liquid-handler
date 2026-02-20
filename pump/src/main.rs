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
    flow_sensor::{FlowSensor, LiquidType},
    tmc2209::Tmc2209,
};
use alloc::{sync::Arc, vec::Vec};
use common::{
    messages::{Request, Response, HEARTBEAT},
    mutex::Mutex,
    usb::PacketStream,
    wifi::{Role, Socket},
};
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
    peripherals::{TIMG0, USB_DEVICE, WIFI},
    system::Stack,
    time,
    timer::{
        timg::{MwdtStage, TimerGroup, Wdt},
        AnyTimer, PeriodicTimer,
    },
};
use heapless::Deque;

use panic_rtt_target as _;

extern crate alloc;

const FLOW_HISTORY_LEN: usize = 1000;

#[derive(Debug)]
pub enum HandleRequestError {
    Tmc(tmc2209::Tmc2209Error),
    FlowSensor(flow_sensor::FlowSensorError),
    Wifi(common::wifi::Error),
    Deku(deku::DekuError),
}

impl From<tmc2209::Tmc2209Error> for HandleRequestError {
    fn from(error: tmc2209::Tmc2209Error) -> Self {
        HandleRequestError::Tmc(error)
    }
}

impl From<flow_sensor::FlowSensorError> for HandleRequestError {
    fn from(error: flow_sensor::FlowSensorError) -> Self {
        HandleRequestError::FlowSensor(error)
    }
}

impl From<common::wifi::Error> for HandleRequestError {
    fn from(error: common::wifi::Error) -> Self {
        HandleRequestError::Wifi(error)
    }
}

impl From<deku::DekuError> for HandleRequestError {
    fn from(error: deku::DekuError) -> Self {
        HandleRequestError::Deku(error)
    }
}

type Result<T> = result::Result<T, HandleRequestError>;

impl Format for HandleRequestError {
    fn format(&self, f: defmt::Formatter) {
        match self {
            HandleRequestError::Tmc(error) => {
                defmt::write!(f, "TMC error: {}", error)
            }
            HandleRequestError::FlowSensor(error) => {
                defmt::write!(f, "Flow sensor error: {:?}", Debug2Format(error))
            }
            HandleRequestError::Wifi(error) => {
                defmt::write!(f, "Wifi error: {:?}", Debug2Format(error))
            }
            HandleRequestError::Deku(error) => {
                defmt::write!(f, "Deku error: {:?}", Debug2Format(error))
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
static AIR_IN_LINE: Mutex<bool> = Mutex::new(false);
static FLOW_RATE: Mutex<f64> = Mutex::new(0.0);

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
    run_coordinator(peripherals.USB_DEVICE, peripherals.WIFI, tmc, valve).await;
}

async fn run_coordinator<'a>(
    device: USB_DEVICE<'a>,
    wifi: WIFI<'a>,
    tmc: TmcHandle<'a>,
    valve: ValveHandle<'a>,
) -> ! {
    let mut stream = PacketStream::new(device, Duration::from_millis(100));
    let mut socket = Socket::new(wifi, Role::Client).await;
    info!("Connected to CNC.");

    loop {
        // Send heartbeat to CNC.
        if socket.write(&[HEARTBEAT]).await.is_err() {
            info!("Heartbeat failed, reconnecting...");
            socket.connect().await;
            info!("Connected to CNC.");
        }

        let Ok(packet) = stream.read().await else {
            continue;
        };

        debug!("Received packet: {=[?]}", &packet[..]);
        let response = if let Ok((_, req)) = Request::from_bytes((&packet, 0)) {
            match handle_request(req, &tmc, &valve, &mut socket).await {
                Ok(response) => response,
                Err(err) => {
                    info!("Handle request error: {}", err);
                    Response::Fail
                }
            }
        } else {
            info!("Failed to decode request: {=[?]}", &packet[..]);
            Response::Fail
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
    socket: &mut Socket<'a>,
) -> Result<Response> {
    let response = match packet {
        // Pump commands.
        Request::Init => Response::Init(0),
        Request::GetAirInLine => Response::GetAirInLine(AIR_IN_LINE.get_cloned()),
        Request::GetFlowRate => Response::GetFlowRate(FLOW_RATE.get_cloned()),
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
        // CNC commands: forward over WiFi.
        Request::Home
        | Request::IsHoming
        | Request::SetPos(..)
        | Request::GetPos
        | Request::SetSpeed(..)
        | Request::GetSpeed
        | Request::SetAccel(..)
        | Request::GetAccel => {
            let bytes = packet.to_bytes().unwrap();
            socket.write(&bytes).await?;
            let resp = socket.read().await?;
            let (_, response) = Response::from_bytes((&resp, 0))?;
            response
        }
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
    let century = Duration::from_secs(100 * 365 * 24 * 3600);
    let mut flush_end = Instant::now();
    // Set air end to be very high so we don't prematurely stop motors before we've seen liquid.
    let mut air_end = Instant::now() + century;
    loop {
        let info = sensor.read().await.unwrap();
        let flush_time = FLUSH_TIME.get_cloned();
        if !info.air_in_line {
            // Reset the air timeout when we see liquid.
            air_end = Instant::now() + Duration::from_secs(5 * 60);
        } else if flush_time > 0.0 {
            let flush_time = FLUSH_TIME.get_cloned();
            FLUSH_MODE.set(true);
            valve.lock(|v| v.set_low());
            flush_end = Instant::now() + Duration::from_millis(1000 * flush_time as u64);
        }

        // Stop flushing if enough time has passed.
        if Instant::now() > flush_end {
            FLUSH_MODE.set(false);
            valve.lock(|v| v.set_high());
        }

        // Stop running the motors if we've only seen air for the last 5 minutes.
        if Instant::now() > air_end {
            air_end = Instant::now() + century;
            FLUSH_RPM.set(0.0);
        }

        AIR_IN_LINE.set(info.air_in_line);
        FLOW_RATE.set(info.ul_per_min);
        FLOW_HISTORY.lock(|history| {
            if history.is_full() {
                history.pop_front();
            }
            history.push_back(info.ul_per_min).unwrap();
        });
        Timer::after_millis(10).await;
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
