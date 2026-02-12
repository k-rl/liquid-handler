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
    tmc2209::{
        BlankTime, OverTemperatureStatus, PwmFrequency, StopMode, TemperatureThreshold, Tmc2209,
    },
};
use common::{mutex::Mutex, usb::PacketStream};
use alloc::{sync::Arc, vec::Vec};
use core::result;
use defmt::{debug, info, Debug2Format, Format};
use deku::prelude::*;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
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
const GET_MICROSTEPS: u8 = 0x0F;
const SET_MICROSTEPS: u8 = 0x10;
const GET_BLANK_TIME: u8 = 0x1B;
const SET_BLANK_TIME: u8 = 0x1C;
const GET_HYSTERESIS_END: u8 = 0x1D;
const SET_HYSTERESIS_END: u8 = 0x1E;
const GET_HYSTERESIS_START: u8 = 0x1F;
const SET_HYSTERESIS_START: u8 = 0x20;
const GET_DECAY_TIME: u8 = 0x21;
const SET_DECAY_TIME: u8 = 0x22;
const GET_PWM_MAX_RPM: u8 = 0x23;
const SET_PWM_MAX_RPM: u8 = 0x24;
const GET_DRIVER_SWITCH_AUTOSCALE_LIMIT: u8 = 0x25;
const SET_DRIVER_SWITCH_AUTOSCALE_LIMIT: u8 = 0x26;
const GET_MAX_AMPLITUDE_CHANGE: u8 = 0x27;
const SET_MAX_AMPLITUDE_CHANGE: u8 = 0x28;
const GET_PWM_FREQUENCY: u8 = 0x2D;
const SET_PWM_FREQUENCY: u8 = 0x2E;
const GET_PWM_GRADIENT: u8 = 0x2F;
const SET_PWM_GRADIENT: u8 = 0x30;
const GET_PWM_OFFSET: u8 = 0x31;
const SET_PWM_OFFSET: u8 = 0x32;
const GET_MICROSTEP_TIME: u8 = 0x3E;
const GET_MOTOR_LOAD: u8 = 0x3F;
const GET_MICROSTEP_POSITION: u8 = 0x40;
const GET_MICROSTEP_CURRENT: u8 = 0x41;
const GET_CURRENT_SCALE: u8 = 0x44;
const GET_TEMPERATURE: u8 = 0x45;
const GET_OVERTEMPERATURE: u8 = 0x49;
const GET_FLOW_HISTORY: u8 = 0x4A;
const GET_VALVE: u8 = 0x4B;
const SET_VALVE: u8 = 0x4C;
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

    #[deku(id = "GET_MICROSTEPS")]
    GetMicrosteps,

    #[deku(id = "SET_MICROSTEPS")]
    SetMicrosteps(u16),

    #[deku(id = "GET_BLANK_TIME")]
    GetBlankTime,

    #[deku(id = "SET_BLANK_TIME")]
    SetBlankTime(#[deku(bits = 8)] BlankTime),

    #[deku(id = "GET_HYSTERESIS_END")]
    GetHysteresisEnd,

    #[deku(id = "SET_HYSTERESIS_END")]
    SetHysteresisEnd(i8),

    #[deku(id = "GET_HYSTERESIS_START")]
    GetHysteresisStart,

    #[deku(id = "SET_HYSTERESIS_START")]
    SetHysteresisStart(u8),

    #[deku(id = "GET_DECAY_TIME")]
    GetDecayTime,

    #[deku(id = "SET_DECAY_TIME")]
    SetDecayTime(u8),

    #[deku(id = "GET_PWM_MAX_RPM")]
    GetPwmMaxRpm,

    #[deku(id = "SET_PWM_MAX_RPM")]
    SetPwmMaxRpm(f64),

    #[deku(id = "GET_DRIVER_SWITCH_AUTOSCALE_LIMIT")]
    GetDriverSwitchAutoscaleLimit,

    #[deku(id = "SET_DRIVER_SWITCH_AUTOSCALE_LIMIT")]
    SetDriverSwitchAutoscaleLimit(u8),

    #[deku(id = "GET_MAX_AMPLITUDE_CHANGE")]
    GetMaxAmplitudeChange,

    #[deku(id = "SET_MAX_AMPLITUDE_CHANGE")]
    SetMaxAmplitudeChange(u8),

    #[deku(id = "GET_PWM_FREQUENCY")]
    GetPwmFrequency,

    #[deku(id = "SET_PWM_FREQUENCY")]
    SetPwmFrequency(#[deku(bits = 8)] PwmFrequency),

    #[deku(id = "GET_PWM_GRADIENT")]
    GetPwmGradient,

    #[deku(id = "SET_PWM_GRADIENT")]
    SetPwmGradient(u8),

    #[deku(id = "GET_PWM_OFFSET")]
    GetPwmOffset,

    #[deku(id = "SET_PWM_OFFSET")]
    SetPwmOffset(u8),

    #[deku(id = "GET_MICROSTEP_TIME")]
    GetMicrostepTime,

    #[deku(id = "GET_MOTOR_LOAD")]
    GetMotorLoad,

    #[deku(id = "GET_MICROSTEP_POSITION")]
    GetMicrostepPosition,

    #[deku(id = "GET_MICROSTEP_CURRENT")]
    GetMicrostepCurrent,

    #[deku(id = "GET_CURRENT_SCALE")]
    GetCurrentScale,

    #[deku(id = "GET_TEMPERATURE")]
    GetTemperature,

    #[deku(id = "GET_OVERTEMPERATURE")]
    GetOverTemperature,

    #[deku(id = "GET_FLOW_HISTORY")]
    GetFlowHistory,

    #[deku(id = "GET_VALVE")]
    GetValve,

    #[deku(id = "SET_VALVE")]
    SetValve(bool),
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

    #[deku(id = "GET_MICROSTEPS")]
    GetMicrosteps(u16),

    #[deku(id = "SET_MICROSTEPS")]
    SetMicrosteps,

    #[deku(id = "GET_BLANK_TIME")]
    GetBlankTime(#[deku(bits = 8)] BlankTime),

    #[deku(id = "SET_BLANK_TIME")]
    SetBlankTime,

    #[deku(id = "GET_HYSTERESIS_END")]
    GetHysteresisEnd(i8),

    #[deku(id = "SET_HYSTERESIS_END")]
    SetHysteresisEnd,

    #[deku(id = "GET_HYSTERESIS_START")]
    GetHysteresisStart(u8),

    #[deku(id = "SET_HYSTERESIS_START")]
    SetHysteresisStart,

    #[deku(id = "GET_DECAY_TIME")]
    GetDecayTime(u8),

    #[deku(id = "SET_DECAY_TIME")]
    SetDecayTime,

    #[deku(id = "GET_PWM_MAX_RPM")]
    GetPwmMaxRpm(f64),

    #[deku(id = "SET_PWM_MAX_RPM")]
    SetPwmMaxRpm,

    #[deku(id = "GET_DRIVER_SWITCH_AUTOSCALE_LIMIT")]
    GetDriverSwitchAutoscaleLimit(u8),

    #[deku(id = "SET_DRIVER_SWITCH_AUTOSCALE_LIMIT")]
    SetDriverSwitchAutoscaleLimit,

    #[deku(id = "GET_MAX_AMPLITUDE_CHANGE")]
    GetMaxAmplitudeChange(u8),

    #[deku(id = "SET_MAX_AMPLITUDE_CHANGE")]
    SetMaxAmplitudeChange,

    #[deku(id = "GET_PWM_FREQUENCY")]
    GetPwmFrequency(#[deku(bits = 8)] PwmFrequency),

    #[deku(id = "SET_PWM_FREQUENCY")]
    SetPwmFrequency,

    #[deku(id = "GET_PWM_GRADIENT")]
    GetPwmGradient(u8),

    #[deku(id = "SET_PWM_GRADIENT")]
    SetPwmGradient,

    #[deku(id = "GET_PWM_OFFSET")]
    GetPwmOffset(u8),

    #[deku(id = "SET_PWM_OFFSET")]
    SetPwmOffset,

    #[deku(id = "GET_MICROSTEP_TIME")]
    GetMicrostepTime(u32),

    #[deku(id = "GET_MOTOR_LOAD")]
    GetMotorLoad(u16),

    #[deku(id = "GET_MICROSTEP_POSITION")]
    GetMicrostepPosition(u16),

    #[deku(id = "GET_MICROSTEP_CURRENT")]
    GetMicrostepCurrent(i16, i16),

    #[deku(id = "GET_CURRENT_SCALE")]
    GetCurrentScale(u8),

    #[deku(id = "GET_TEMPERATURE")]
    GetTemperature(#[deku(bits = 8)] TemperatureThreshold),

    #[deku(id = "GET_OVERTEMPERATURE")]
    GetOverTemperature(#[deku(bits = 8)] OverTemperatureStatus),

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

static RPM: Mutex<f64> = Mutex::new(0.0);
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
    spawner.spawn(run_flow_rate_monitor(sensor)).unwrap();
    let valve = Output::new(peripherals.GPIO4, Low, OutputConfig::default());
    run_coordinator(peripherals.USB_DEVICE, tmc, valve).await;
}

async fn run_coordinator<'a>(
    device: USB_DEVICE<'a>,
    tmc: TmcHandle<'a>,
    mut valve: Output<'a>,
) -> ! {
    let mut stream = PacketStream::new(device, Duration::from_secs(1));
    loop {
        let Ok(packet) = stream.read().await else {
            info!("Packet read failed.");
            continue;
        };

        debug!("Received packet: {=[?]}", &packet[..]);
        let response = match Request::from_bytes((&packet, 0)) {
            Ok((_, packet)) => match handle_request(packet, &tmc, &mut valve).await {
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
        stream.write(&response.to_bytes().unwrap()).await;
    }
}

async fn handle_request<'a>(
    packet: Request,
    tmc: &TmcHandle<'a>,
    valve: &mut Output<'a>,
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
        Request::GetMicrosteps => Response::GetMicrosteps(tmc.microsteps()),
        Request::SetMicrosteps(n) => {
            tmc.set_microsteps(n)?;
            Response::SetMicrosteps
        }
        Request::GetBlankTime => Response::GetBlankTime(tmc.blank_time()?),
        Request::SetBlankTime(time) => {
            tmc.set_blank_time(time)?;
            Response::SetBlankTime
        }
        Request::GetHysteresisEnd => Response::GetHysteresisEnd(tmc.hysteresis_end()?),
        Request::SetHysteresisEnd(end) => {
            tmc.set_hysteresis_end(end)?;
            Response::SetHysteresisEnd
        }
        Request::GetHysteresisStart => Response::GetHysteresisStart(tmc.hysteresis_start()?),
        Request::SetHysteresisStart(start) => {
            tmc.set_hysteresis_start(start)?;
            Response::SetHysteresisStart
        }
        Request::GetDecayTime => Response::GetDecayTime(tmc.decay_time()?),
        Request::SetDecayTime(time) => {
            tmc.set_decay_time(time)?;
            Response::SetDecayTime
        }
        Request::GetPwmMaxRpm => Response::GetPwmMaxRpm(tmc.pwm_max_rpm()?),
        Request::SetPwmMaxRpm(rpm) => {
            tmc.set_pwm_max_rpm(rpm)?;
            Response::SetPwmMaxRpm
        }
        Request::GetDriverSwitchAutoscaleLimit => {
            Response::GetDriverSwitchAutoscaleLimit(tmc.driver_switch_autoscale_limit()?)
        }
        Request::SetDriverSwitchAutoscaleLimit(limit) => {
            tmc.set_driver_switch_autoscale_limit(limit)?;
            Response::SetDriverSwitchAutoscaleLimit
        }
        Request::GetMaxAmplitudeChange => {
            Response::GetMaxAmplitudeChange(tmc.max_amplitude_change()?)
        }
        Request::SetMaxAmplitudeChange(change) => {
            tmc.set_max_amplitude_change(change)?;
            Response::SetMaxAmplitudeChange
        }
        Request::GetPwmFrequency => Response::GetPwmFrequency(tmc.pwm_frequency()?),
        Request::SetPwmFrequency(frequency) => {
            tmc.set_pwm_frequency(frequency)?;
            Response::SetPwmFrequency
        }
        Request::GetPwmGradient => Response::GetPwmGradient(tmc.pwm_gradient()?),
        Request::SetPwmGradient(gradient) => {
            tmc.set_pwm_gradient(gradient)?;
            Response::SetPwmGradient
        }
        Request::GetPwmOffset => Response::GetPwmOffset(tmc.pwm_offset()?),
        Request::SetPwmOffset(offset) => {
            tmc.set_pwm_offset(offset)?;
            Response::SetPwmOffset
        }
        Request::GetMicrostepTime => Response::GetMicrostepTime(tmc.microstep_time()?),
        Request::GetMotorLoad => Response::GetMotorLoad(tmc.motor_load()?),
        Request::GetMicrostepPosition => Response::GetMicrostepPosition(tmc.microstep_position()?),
        Request::GetMicrostepCurrent => {
            let (a, b) = tmc.microstep_current()?;
            Response::GetMicrostepCurrent(a, b)
        }
        Request::GetCurrentScale => Response::GetCurrentScale(tmc.current_scale()?),
        Request::GetTemperature => Response::GetTemperature(tmc.temperature()?),
        Request::GetOverTemperature => Response::GetOverTemperature(tmc.over_temperature()?),
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
        Request::GetValve => Response::GetValve(valve.is_set_high()),
        Request::SetValve(open) => {
            if open {
                valve.set_high();
            } else {
                valve.set_low();
            }
            Response::SetValve
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
async fn run_flow_rate_monitor(mut sensor: FlowSensor<'static>) -> ! {
    info!("Flow sensor initialized.");
    sensor.start(LiquidType::Water).await.unwrap();
    loop {
        match sensor.read().await {
            Ok(info) => {
                FLOW_INFO.set(info);
                FLOW_HISTORY.lock(|history| {
                    if history.is_full() {
                        history.pop_front();
                    }
                    history.push_back(info.ul_per_min).unwrap();
                });
            }
            Err(err) => {
                info!("Flow sensor error: {:?}", Debug2Format(&err));
            }
        }
        Timer::after_millis(50).await;
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
        let target_v = RPM.get_cloned() / 60.0;
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
