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
    tmc2209::{IndexOutput, Tmc2209},
    usb::PacketStream,
};
use alloc::sync::Arc;
use core::{cell::RefCell, result};
use defmt::{info, Format};
use deku::prelude::*;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, blocking_mutex::Mutex};
use embassy_time::{Duration, Timer};
use esp_hal::{
    self,
    clock::CpuClock,
    delay::Delay,
    interrupt::software::SoftwareInterruptControl,
    peripherals::{TIMG0, USB_DEVICE},
    system::Stack,
    time,
    timer::timg::{MwdtStage, TimerGroup, Wdt},
};
use thiserror::Error;

use panic_rtt_target as _;

extern crate alloc;

// Request/response codes.
const INIT: u8 = 0x00;
const FLOW_SENSOR_INFO: u8 = 0x01;
const SET_PUMP_RPM: u8 = 0x02;
const GET_PUMP_RPM: u8 = 0x03;
const SET_TEST_MODE: u8 = 0x04;
const GET_TEST_MODE: u8 = 0x05;
const SET_FILTER_STEP_PULSES: u8 = 0x06;
const GET_FILTER_STEP_PULSES: u8 = 0x07;
const SET_PIN_UART_MODE: u8 = 0x0A;
const GET_PIN_UART_MODE: u8 = 0x0B;
const SET_INDEX_OUTPUT: u8 = 0x0C;
const GET_INDEX_OUTPUT: u8 = 0x0D;
const SET_INVERT_DIRECTION: u8 = 0x0E;
const GET_INVERT_DIRECTION: u8 = 0x0F;
const SET_PWM_ENABLED: u8 = 0x10;
const GET_PWM_ENABLED: u8 = 0x11;
const SET_CURRENT: u8 = 0x12;
const GET_CURRENT: u8 = 0x13;
const SET_EXTERNAL_CURRENT_SCALING: u8 = 0x14;
const GET_EXTERNAL_CURRENT_SCALING: u8 = 0x15;
const SET_MICROSTEPS: u8 = 0x16;
const GET_MICROSTEPS: u8 = 0x17;
const FAIL: u8 = 0xFF;

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite, Format)]
#[deku(id_type = "u8", endian = "big")]
enum Request {
    #[deku(id = "INIT")]
    Init,

    #[deku(id = "FLOW_SENSOR_INFO")]
    FlowSensorInfo,

    #[deku(id = "SET_PUMP_RPM")]
    SetPumpRps(f64),

    #[deku(id = "GET_PUMP_RPM")]
    GetPumpRps,

    #[deku(id = "SET_TEST_MODE")]
    SetTestMode(bool),

    #[deku(id = "GET_TEST_MODE")]
    GetTestMode,

    #[deku(id = "SET_FILTER_STEP_PULSES")]
    SetFilterStepPulses(bool),

    #[deku(id = "GET_FILTER_STEP_PULSES")]
    GetFilterStepPulses,

    #[deku(id = "SET_PIN_UART_MODE")]
    SetPinUartMode(bool),

    #[deku(id = "GET_PIN_UART_MODE")]
    GetPinUartMode,

    #[deku(id = "SET_INDEX_OUTPUT")]
    SetIndexOutput(IndexOutput),

    #[deku(id = "GET_INDEX_OUTPUT")]
    GetIndexOutput,

    #[deku(id = "SET_INVERT_DIRECTION")]
    SetInvertDirection(bool),

    #[deku(id = "GET_INVERT_DIRECTION")]
    GetInvertDirection,

    #[deku(id = "GET_PWM_ENABLED")]
    GetPwmEnabled,

    #[deku(id = "SET_PWM_ENABLED")]
    SetPwmEnabled(bool),

    #[deku(id = "GET_CURRENT")]
    GetCurrent,

    #[deku(id = "SET_CURRENT")]
    SetCurrent {
        running_rms_amps: f64,
        stopped_rms_amps: f64,
        ref_volts: f64,
        sense_ohms: f64,
    },

    #[deku(id = "SET_EXTERNAL_CURRENT_SCALING")]
    SetExternalCurrentScaling(bool),

    #[deku(id = "GET_EXTERNAL_CURRENT_SCALING")]
    GetExternalCurrentScaling,

    #[deku(id = "GET_MICROSTEPS")]
    GetMicrosteps,

    #[deku(id = "SET_MICROSTEPS")]
    SetMicrosteps(u16),
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite, Format)]
#[deku(id_type = "u8", endian = "big")]
enum Response {
    #[deku(id = "INIT")]
    Init,

    #[deku(id = "FLOW_SENSOR_INFO")]
    FlowSensorInfo(FlowSensorInfo),

    #[deku(id = "SET_PUMP_RPM")]
    SetPumpRps,

    #[deku(id = "GET_PUMP_RPM")]
    GetPumpRps(f64),

    #[deku(id = "SET_TEST_MODE")]
    SetTestMode,

    #[deku(id = "GET_TEST_MODE")]
    GetTestMode(bool),

    #[deku(id = "SET_FILTER_STEP_PULSES")]
    SetFilterStepPulses,

    #[deku(id = "GET_FILTER_STEP_PULSES")]
    GetFilterStepPulses(bool),

    #[deku(id = "SET_PIN_UART_MODE")]
    SetPinUartMode,

    #[deku(id = "GET_PIN_UART_MODE")]
    GetPinUartMode(bool),

    #[deku(id = "SET_INDEX_OUTPUT")]
    SetIndexOutput,

    #[deku(id = "GET_INDEX_OUTPUT")]
    GetIndexOutput(IndexOutput),

    #[deku(id = "SET_INVERT_DIRECTION")]
    SetInvertDirection,

    #[deku(id = "GET_INVERT_DIRECTION")]
    GetInvertDirection(bool),

    #[deku(id = "SET_PWM_ENABLED")]
    SetPwmEnabled,

    #[deku(id = "GET_PWM_ENABLED")]
    GetPwmEnabled(bool),

    #[deku(id = "SET_CURRENT")]
    SetCurrent,

    #[deku(id = "GET_CURRENT")]
    GetCurrent(f64, f64),

    #[deku(id = "SET_EXTERNAL_CURRENT_SCALING")]
    SetExternalCurrentScaling,

    #[deku(id = "GET_EXTERNAL_CURRENT_SCALING")]
    GetExternalCurrentScaling(bool),

    #[deku(id = "GET_MICROSTEPS")]
    GetMicrosteps(u16),

    #[deku(id = "SET_MICROSTEPS")]
    SetMicrosteps,

    #[deku(id = "FAIL")]
    Fail,
}

#[derive(Error, Debug)]
pub enum HandleRequestError {
    #[error("TMC error")]
    TmcError,
    #[error("Flow sensor error")]
    FlowSensorError,
}

impl From<tmc2209::Tmc2209Error> for HandleRequestError {
    fn from(_: tmc2209::Tmc2209Error) -> Self {
        HandleRequestError::TmcError
    }
}

impl From<flow_sensor::FlowSensorError> for HandleRequestError {
    fn from(_: flow_sensor::FlowSensorError) -> Self {
        HandleRequestError::FlowSensorError
    }
}

type Result<T> = result::Result<T, HandleRequestError>;

esp_bootloader_esp_idf::esp_app_desc!();

type TmcMutex<'a> = Arc<Mutex<CriticalSectionRawMutex, RefCell<Tmc2209<'a>>>>;

static RPM: Mutex<CriticalSectionRawMutex, RefCell<f64>> = Mutex::new(RefCell::new(0.0));

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

    let tmc: TmcMutex = Arc::new(Mutex::new(RefCell::new(Tmc2209::new(
        peripherals.UART1,
        peripherals.GPIO13, // TX
        peripherals.GPIO10, // RX
        peripherals.GPIO9,  // STEP
        peripherals.GPIO8,  // DIR
        peripherals.GPIO7,  // ENABLE
    ))));
    let irc = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    static mut STACK: Stack<8192> = Stack::new();
    let tmc_ref = Arc::clone(&tmc);
    #[allow(static_mut_refs)]
    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        irc.software_interrupt0,
        irc.software_interrupt1,
        unsafe { &mut STACK },
        || run_tmc(tmc_ref),
    );

    // Initialize the sensor and coordinator.
    let mut sensor = FlowSensor::new(peripherals.I2C0, peripherals.GPIO2, peripherals.GPIO1);
    info!("Flow sensor initialized.");
    sensor.start(LiquidType::Water).await.unwrap();
    run_coordinator(peripherals.USB_DEVICE, sensor, tmc).await;
}

async fn run_coordinator<'a>(
    device: USB_DEVICE<'a>,
    mut sensor: FlowSensor<'a>,
    tmc: TmcMutex<'a>,
) -> ! {
    let mut stream = PacketStream::new(device, Duration::from_secs(1));
    loop {
        let Ok(packet) = stream.read().await else {
            info!("Packet read failed.");
            continue;
        };

        info!("Received packet: {=[?]}", &packet[..]);
        let response = if let Ok((_, packet)) = Request::from_bytes((&packet, 0)) {
            handle_request(packet, &mut sensor, &tmc)
                .await
                .unwrap_or(Response::Fail)
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

async fn handle_request<'a>(
    packet: Request,
    sensor: &mut FlowSensor<'a>,
    tmc: &TmcMutex<'a>,
) -> Result<Response> {
    let response = match packet {
        Request::Init => Response::Init,
        Request::FlowSensorInfo => Response::FlowSensorInfo(sensor.read().await?),
        Request::SetPumpRps(rpm) => {
            RPM.lock(|x| x.replace(rpm));
            Response::SetPumpRps
        }
        Request::GetPumpRps => Response::GetPumpRps(RPM.lock(|x| *x.borrow())),
        Request::GetTestMode => Response::GetTestMode(tmc.lock(|x| x.borrow_mut().test_mode())?),
        Request::SetTestMode(enable) => {
            tmc.lock(|x| x.borrow_mut().set_test_mode(enable))?;
            Response::SetTestMode
        }
        Request::GetFilterStepPulses => {
            Response::GetFilterStepPulses(tmc.lock(|x| x.borrow_mut().filter_step_pulses())?)
        }
        Request::SetFilterStepPulses(enable) => {
            tmc.lock(|x| x.borrow_mut().set_filter_step_pulses(enable))?;
            Response::SetFilterStepPulses
        }
        Request::GetPinUartMode => {
            Response::GetPinUartMode(tmc.lock(|x| x.borrow_mut().pin_uart_mode())?)
        }
        Request::SetPinUartMode(enable) => {
            tmc.lock(|x| x.borrow_mut().set_pin_uart_mode(enable))?;
            Response::SetPinUartMode
        }
        Request::GetIndexOutput => {
            Response::GetIndexOutput(tmc.lock(|x| x.borrow_mut().index_output())?)
        }
        Request::SetIndexOutput(output) => {
            tmc.lock(|x| x.borrow_mut().set_index_output(output))?;
            Response::SetIndexOutput
        }
        Request::GetInvertDirection => {
            Response::GetInvertDirection(tmc.lock(|x| x.borrow_mut().invert_direction())?)
        }
        Request::SetInvertDirection(enable) => {
            tmc.lock(|x| x.borrow_mut().set_invert_direction(enable))?;
            Response::SetInvertDirection
        }
        Request::GetPwmEnabled => {
            Response::GetPwmEnabled(tmc.lock(|x| x.borrow_mut().pwm_enabled())?)
        }
        Request::SetPwmEnabled(enable) => {
            tmc.lock(|x| x.borrow_mut().set_pwm_enabled(enable))?;
            Response::SetPwmEnabled
        }
        Request::GetCurrent => {
            let (run, stop) = tmc.lock(|x| x.borrow_mut().current());
            Response::GetCurrent(run, stop)
        }
        Request::SetCurrent {
            running_rms_amps,
            stopped_rms_amps,
            ref_volts,
            sense_ohms,
        } => {
            tmc.lock(|x| {
                x.borrow_mut().set_current(
                    running_rms_amps,
                    stopped_rms_amps,
                    ref_volts,
                    sense_ohms,
                )
            })?;
            Response::SetCurrent
        }
        Request::GetExternalCurrentScaling => Response::GetExternalCurrentScaling(
            tmc.lock(|x| x.borrow_mut().external_current_scaling())?,
        ),
        Request::SetExternalCurrentScaling(enable) => {
            tmc.lock(|x| x.borrow_mut().set_external_current_scaling(enable))?;
            Response::SetExternalCurrentScaling
        }
        Request::GetMicrosteps => {
            Response::GetMicrosteps(tmc.lock(|x| x.borrow_mut().microsteps())?)
        }
        Request::SetMicrosteps(n) => {
            tmc.lock(|x| x.borrow_mut().set_microsteps(n))?;
            Response::SetMicrosteps
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

fn run_tmc(tmc: TmcMutex<'static>) -> ! {
    let delay = Delay::new();
    tmc.lock(|x| x.borrow_mut().enable());
    info!("TMC2209 initialized.");
    let mut v = 0.0;
    let a = 1.0;
    let dx = 1.0 / (tmc.lock(|x| x.borrow_mut().pulses_per_rev().unwrap()) as f64);
    let dv2 = 2.0 * a * dx;
    let mut i = 0;
    loop {
        let prev_v = v;
        let target_v = RPM.lock(|x| *x.borrow());
        let v2 = v * v;
        let diff = target_v * target_v - v2;
        if diff.abs() < dv2 {
            v = target_v;
        } else {
            v = libm::sqrt(v2 + diff.signum() * dv2);
        }

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
            tmc.lock(|x| x.borrow_mut().toggle_step());
            delay.delay_micros((dt * 1e6) as u32);
        } else {
            delay.delay_millis(10);
        }
        i += 1;
    }
}
