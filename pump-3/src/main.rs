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
    tmc2209::{
        BlankTime, OverTemperatureStatus, PhaseStatus, PwmFrequency, StopMode,
        TemperatureThreshold, Tmc2209,
    },
    usb::PacketStream,
};
use alloc::sync::Arc;
use core::{cell::RefCell, result};
use defmt::{info, Debug2Format, Format};
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
const GET_RMS_AMPS: u8 = 0x0C;
const SET_RMS_AMPS: u8 = 0x0D;
const GET_STOPPED_RMS_AMPS: u8 = 0x70;
const SET_STOPPED_RMS_AMPS: u8 = 0x71;
const GET_STOP_MODE: u8 = 0x0E;
const SET_STOP_MODE: u8 = 0x0F;
const GET_POWERDOWN_DURATION_S: u8 = 0x10;
const SET_POWERDOWN_DURATION_S: u8 = 0x11;
const GET_POWERDOWN_DELAY_S: u8 = 0x12;
const SET_POWERDOWN_DELAY_S: u8 = 0x13;
const GET_MICROSTEPS: u8 = 0x14;
const SET_MICROSTEPS: u8 = 0x15;
const GET_FILTER_STEP_PULSES: u8 = 0x18;
const SET_FILTER_STEP_PULSES: u8 = 0x19;
const GET_DOUBLE_EDGE_STEP: u8 = 0x1A;
const SET_DOUBLE_EDGE_STEP: u8 = 0x1B;
const GET_INTERPOLATE_MICROSTEPS: u8 = 0x1C;
const SET_INTERPOLATE_MICROSTEPS: u8 = 0x1D;
const GET_SHORT_SUPPLY_PROTECT: u8 = 0x2C;
const SET_SHORT_SUPPLY_PROTECT: u8 = 0x2D;
const GET_SHORT_GROUND_PROTECT: u8 = 0x2E;
const SET_SHORT_GROUND_PROTECT: u8 = 0x2F;
const GET_BLANK_TIME: u8 = 0x30;
const SET_BLANK_TIME: u8 = 0x31;
const GET_HYSTERESIS_END: u8 = 0x32;
const SET_HYSTERESIS_END: u8 = 0x33;
const GET_HYSTERESIS_START: u8 = 0x34;
const SET_HYSTERESIS_START: u8 = 0x35;
const GET_DECAY_TIME: u8 = 0x36;
const SET_DECAY_TIME: u8 = 0x37;
const GET_PWM_MAX_RPM: u8 = 0x38;
const SET_PWM_MAX_RPM: u8 = 0x39;
const GET_DRIVER_SWITCH_AUTOSCALE_LIMIT: u8 = 0x3A;
const SET_DRIVER_SWITCH_AUTOSCALE_LIMIT: u8 = 0x3B;
const GET_MAX_AMPLITUDE_CHANGE: u8 = 0x3C;
const SET_MAX_AMPLITUDE_CHANGE: u8 = 0x3D;
const GET_PWM_AUTOGRADIENT: u8 = 0x3E;
const SET_PWM_AUTOGRADIENT: u8 = 0x3F;
const GET_PWN_AUTOSCALE: u8 = 0x40;
const SET_PWN_AUTOSCALE: u8 = 0x41;
const GET_PWM_FREQUENCY: u8 = 0x42;
const SET_PWM_FREQUENCY: u8 = 0x43;
const GET_PWM_GRADIENT: u8 = 0x44;
const SET_PWM_GRADIENT: u8 = 0x45;
const GET_PWM_OFFSET: u8 = 0x46;
const SET_PWM_OFFSET: u8 = 0x47;
const GET_CHARGE_PUMP_UNDERVOLTAGE: u8 = 0x4C;
const GET_DRIVER_ERROR: u8 = 0x4D;
const GET_IS_RESET: u8 = 0x4E;
const GET_DIRECTION_PIN: u8 = 0x51;
const GET_DISABLE_PWM_PIN: u8 = 0x52;
const GET_STEP_PIN: u8 = 0x53;
const GET_POWERDOWN_UART_PIN: u8 = 0x54;
const GET_DIAGNOSTIC_PIN: u8 = 0x55;
const GET_MICROSTEP2_PIN: u8 = 0x56;
const GET_MICROSTEP1_PIN: u8 = 0x57;
const GET_DISABLE_PIN: u8 = 0x58;
const GET_MICROSTEP_TIME: u8 = 0x59;
const GET_MOTOR_LOAD: u8 = 0x5A;
const GET_MICROSTEP_POSITION: u8 = 0x5B;
const GET_MICROSTEP_CURRENT: u8 = 0x5C;
const GET_STOPPED: u8 = 0x5D;
const GET_PWM_MODE: u8 = 0x5E;
const GET_CURRENT_SCALE: u8 = 0x5F;
const GET_TEMPERATURE: u8 = 0x60;
const GET_OPEN_LOAD: u8 = 0x61;
const GET_LOW_SIDE_SHORT: u8 = 0x62;
const GET_GROUND_SHORT: u8 = 0x63;
const GET_OVER_TEMPERATURE: u8 = 0x64;
const FAIL: u8 = 0xFF;

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite, Format)]
#[deku(id_type = "u8", endian = "big")]
enum Request {
    #[deku(id = "INIT")]
    Init,

    #[deku(id = "FLOW_SENSOR_INFO")]
    FlowSensorInfo,

    #[deku(id = "SET_PUMP_RPM")]
    SetPumpRpm(f64),

    #[deku(id = "GET_PUMP_RPM")]
    GetPumpRpm,

    #[deku(id = "GET_RMS_AMPS")]
    GetRmsAmps,

    #[deku(id = "SET_RMS_AMPS")]
    SetRmsAmps(f64),

    #[deku(id = "GET_STOPPED_RMS_AMPS")]
    GetStoppedRmsAmps,

    #[deku(id = "SET_STOPPED_RMS_AMPS")]
    SetStoppedRmsAmps(f64),

    #[deku(id = "GET_STOP_MODE")]
    GetStopMode,

    #[deku(id = "SET_STOP_MODE")]
    SetStopMode(#[deku(bits = 8)] StopMode),

    #[deku(id = "GET_POWERDOWN_DURATION_S")]
    GetPowerdownDurationS,

    #[deku(id = "SET_POWERDOWN_DURATION_S")]
    SetPowerdownDurationS(f64),

    #[deku(id = "GET_POWERDOWN_DELAY_S")]
    GetPowerdownDelayS,

    #[deku(id = "SET_POWERDOWN_DELAY_S")]
    SetPowerdownDelayS(f64),

    #[deku(id = "GET_MICROSTEPS")]
    GetMicrosteps,

    #[deku(id = "SET_MICROSTEPS")]
    SetMicrosteps(u16),

    #[deku(id = "GET_FILTER_STEP_PULSES")]
    GetFilterStepPulses,

    #[deku(id = "SET_FILTER_STEP_PULSES")]
    SetFilterStepPulses(bool),

    #[deku(id = "GET_DOUBLE_EDGE_STEP")]
    GetDoubleEdgeStep,

    #[deku(id = "SET_DOUBLE_EDGE_STEP")]
    SetDoubleEdgeStep(bool),

    #[deku(id = "GET_INTERPOLATE_MICROSTEPS")]
    GetInterpolateMicrosteps,

    #[deku(id = "SET_INTERPOLATE_MICROSTEPS")]
    SetInterpolateMicrosteps(bool),

    #[deku(id = "GET_SHORT_SUPPLY_PROTECT")]
    GetShortSupplyProtect,

    #[deku(id = "SET_SHORT_SUPPLY_PROTECT")]
    SetShortSupplyProtect(bool),

    #[deku(id = "GET_SHORT_GROUND_PROTECT")]
    GetShortGroundProtect,

    #[deku(id = "SET_SHORT_GROUND_PROTECT")]
    SetShortGroundProtect(bool),

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

    #[deku(id = "GET_PWM_AUTOGRADIENT")]
    GetPwmAutogradient,

    #[deku(id = "SET_PWM_AUTOGRADIENT")]
    SetPwmAutogradient(bool),

    #[deku(id = "GET_PWN_AUTOSCALE")]
    GetPwnAutoscale,

    #[deku(id = "SET_PWN_AUTOSCALE")]
    SetPwnAutoscale(bool),

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

    #[deku(id = "GET_CHARGE_PUMP_UNDERVOLTAGE")]
    GetChargePumpUndervoltage,

    #[deku(id = "GET_DRIVER_ERROR")]
    GetDriverError,

    #[deku(id = "GET_IS_RESET")]
    GetIsReset,

    #[deku(id = "GET_DIRECTION_PIN")]
    GetDirectionPin,

    #[deku(id = "GET_DISABLE_PWM_PIN")]
    GetDisablePwmPin,

    #[deku(id = "GET_STEP_PIN")]
    GetStepPin,

    #[deku(id = "GET_POWERDOWN_UART_PIN")]
    GetPowerdownUartPin,

    #[deku(id = "GET_DIAGNOSTIC_PIN")]
    GetDiagnosticPin,

    #[deku(id = "GET_MICROSTEP2_PIN")]
    GetMicrostep2Pin,

    #[deku(id = "GET_MICROSTEP1_PIN")]
    GetMicrostep1Pin,

    #[deku(id = "GET_DISABLE_PIN")]
    GetDisablePin,

    #[deku(id = "GET_MICROSTEP_TIME")]
    GetMicrostepTime,

    #[deku(id = "GET_MOTOR_LOAD")]
    GetMotorLoad,

    #[deku(id = "GET_MICROSTEP_POSITION")]
    GetMicrostepPosition,

    #[deku(id = "GET_MICROSTEP_CURRENT")]
    GetMicrostepCurrent,

    #[deku(id = "GET_STOPPED")]
    GetStopped,

    #[deku(id = "GET_PWM_MODE")]
    GetPwmMode,

    #[deku(id = "GET_CURRENT_SCALE")]
    GetCurrentScale,

    #[deku(id = "GET_TEMPERATURE")]
    GetTemperature,

    #[deku(id = "GET_OPEN_LOAD")]
    GetOpenLoad,

    #[deku(id = "GET_LOW_SIDE_SHORT")]
    GetLowSideShort,

    #[deku(id = "GET_GROUND_SHORT")]
    GetGroundShort,

    #[deku(id = "GET_OVER_TEMPERATURE")]
    GetOverTemperature,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite, Format)]
#[deku(id_type = "u8", endian = "big")]
enum Response {
    #[deku(id = "INIT")]
    Init,

    #[deku(id = "FLOW_SENSOR_INFO")]
    FlowSensorInfo(FlowSensorInfo),

    #[deku(id = "SET_PUMP_RPM")]
    SetPumpRpm,

    #[deku(id = "GET_PUMP_RPM")]
    GetPumpRpm(f64),

    #[deku(id = "GET_RMS_AMPS")]
    GetRmsAmps(f64),

    #[deku(id = "SET_RMS_AMPS")]
    SetRmsAmps,

    #[deku(id = "GET_STOPPED_RMS_AMPS")]
    GetStoppedRmsAmps(f64),

    #[deku(id = "SET_STOPPED_RMS_AMPS")]
    SetStoppedRmsAmps,

    #[deku(id = "GET_STOP_MODE")]
    GetStopMode(#[deku(bits = 8)] StopMode),

    #[deku(id = "SET_STOP_MODE")]
    SetStopMode,

    #[deku(id = "GET_POWERDOWN_DURATION_S")]
    GetPowerdownDurationS(f64),

    #[deku(id = "SET_POWERDOWN_DURATION_S")]
    SetPowerdownDurationS,

    #[deku(id = "GET_POWERDOWN_DELAY_S")]
    GetPowerdownDelayS(f64),

    #[deku(id = "SET_POWERDOWN_DELAY_S")]
    SetPowerdownDelayS,

    #[deku(id = "GET_MICROSTEPS")]
    GetMicrosteps(u16),

    #[deku(id = "SET_MICROSTEPS")]
    SetMicrosteps,

    #[deku(id = "GET_FILTER_STEP_PULSES")]
    GetFilterStepPulses(bool),

    #[deku(id = "SET_FILTER_STEP_PULSES")]
    SetFilterStepPulses,

    #[deku(id = "GET_DOUBLE_EDGE_STEP")]
    GetDoubleEdgeStep(bool),

    #[deku(id = "SET_DOUBLE_EDGE_STEP")]
    SetDoubleEdgeStep,

    #[deku(id = "GET_INTERPOLATE_MICROSTEPS")]
    GetInterpolateMicrosteps(bool),

    #[deku(id = "SET_INTERPOLATE_MICROSTEPS")]
    SetInterpolateMicrosteps,

    #[deku(id = "GET_SHORT_SUPPLY_PROTECT")]
    GetShortSupplyProtect(bool),

    #[deku(id = "SET_SHORT_SUPPLY_PROTECT")]
    SetShortSupplyProtect,

    #[deku(id = "GET_SHORT_GROUND_PROTECT")]
    GetShortGroundProtect(bool),

    #[deku(id = "SET_SHORT_GROUND_PROTECT")]
    SetShortGroundProtect,

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

    #[deku(id = "GET_PWM_AUTOGRADIENT")]
    GetPwmAutogradient(bool),

    #[deku(id = "SET_PWM_AUTOGRADIENT")]
    SetPwmAutogradient,

    #[deku(id = "GET_PWN_AUTOSCALE")]
    GetPwnAutoscale(bool),

    #[deku(id = "SET_PWN_AUTOSCALE")]
    SetPwnAutoscale,

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

    #[deku(id = "GET_CHARGE_PUMP_UNDERVOLTAGE")]
    GetChargePumpUndervoltage(bool),

    #[deku(id = "GET_DRIVER_ERROR")]
    GetDriverError(bool),

    #[deku(id = "GET_IS_RESET")]
    GetIsReset(bool),

    #[deku(id = "GET_DIRECTION_PIN")]
    GetDirectionPin(bool),

    #[deku(id = "GET_DISABLE_PWM_PIN")]
    GetDisablePwmPin(bool),

    #[deku(id = "GET_STEP_PIN")]
    GetStepPin(bool),

    #[deku(id = "GET_POWERDOWN_UART_PIN")]
    GetPowerdownUartPin(bool),

    #[deku(id = "GET_DIAGNOSTIC_PIN")]
    GetDiagnosticPin(bool),

    #[deku(id = "GET_MICROSTEP2_PIN")]
    GetMicrostep2Pin(bool),

    #[deku(id = "GET_MICROSTEP1_PIN")]
    GetMicrostep1Pin(bool),

    #[deku(id = "GET_DISABLE_PIN")]
    GetDisablePin(bool),

    #[deku(id = "GET_MICROSTEP_TIME")]
    GetMicrostepTime(u32),

    #[deku(id = "GET_MOTOR_LOAD")]
    GetMotorLoad(u16),

    #[deku(id = "GET_MICROSTEP_POSITION")]
    GetMicrostepPosition(u16),

    #[deku(id = "GET_MICROSTEP_CURRENT")]
    GetMicrostepCurrent(i16, i16),

    #[deku(id = "GET_STOPPED")]
    GetStopped(bool),

    #[deku(id = "GET_PWM_MODE")]
    GetPwmMode(bool),

    #[deku(id = "GET_CURRENT_SCALE")]
    GetCurrentScale(u8),

    #[deku(id = "GET_TEMPERATURE")]
    GetTemperature(#[deku(bits = 8)] TemperatureThreshold),

    #[deku(id = "GET_OPEN_LOAD")]
    GetOpenLoad(#[deku(bits = 8)] PhaseStatus),

    #[deku(id = "GET_LOW_SIDE_SHORT")]
    GetLowSideShort(#[deku(bits = 8)] PhaseStatus),

    #[deku(id = "GET_GROUND_SHORT")]
    GetGroundShort(#[deku(bits = 8)] PhaseStatus),

    #[deku(id = "GET_OVER_TEMPERATURE")]
    GetOverTemperature(#[deku(bits = 8)] OverTemperatureStatus),

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

    let mut tmc = Tmc2209::new(
        peripherals.UART1,
        peripherals.GPIO13, // TX
        peripherals.GPIO10, // RX
        peripherals.GPIO9,  // STEP
        peripherals.GPIO8,  // DIR
        peripherals.GPIO7,  // ENABLE
    )
    .unwrap();
    tmc.set_sense_ohms(0.110).unwrap();
    let tmc: TmcMutex = Arc::new(Mutex::new(RefCell::new(tmc)));
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
        let response = match Request::from_bytes((&packet, 0)) {
            Ok((_, packet)) => match handle_request(packet, &mut sensor, &tmc).await {
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
        info!("Sending response: {=[?]}", &response_bytes[..]);
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
        Request::SetPumpRpm(rpm) => {
            RPM.lock(|x| x.replace(rpm));
            Response::SetPumpRpm
        }
        Request::GetPumpRpm => Response::GetPumpRpm(RPM.lock(|x| *x.borrow())),
        Request::GetRmsAmps => Response::GetRmsAmps(tmc.lock(|x| x.borrow_mut().rms_amps())?),
        Request::SetRmsAmps(amps) => {
            tmc.lock(|x| x.borrow_mut().set_rms_amps(amps))?;
            Response::SetRmsAmps
        }
        Request::GetStoppedRmsAmps => {
            Response::GetStoppedRmsAmps(tmc.lock(|x| x.borrow_mut().stopped_rms_amps())?)
        }
        Request::SetStoppedRmsAmps(amps) => {
            tmc.lock(|x| x.borrow_mut().set_stopped_rms_amps(amps))?;
            Response::SetStoppedRmsAmps
        }
        Request::GetStopMode => Response::GetStopMode(tmc.lock(|x| x.borrow_mut().stop_mode())?),
        Request::SetStopMode(mode) => {
            info!("{}", mode);
            tmc.lock(|x| x.borrow_mut().set_stop_mode(mode))?;
            Response::SetStopMode
        }
        Request::GetPowerdownDurationS => {
            Response::GetPowerdownDurationS(tmc.lock(|x| x.borrow_mut().powerdown_duration_s())?)
        }
        Request::SetPowerdownDurationS(duration) => {
            tmc.lock(|x| x.borrow_mut().set_powerdown_duration_s(duration))?;
            Response::SetPowerdownDurationS
        }
        Request::GetPowerdownDelayS => {
            Response::GetPowerdownDelayS(tmc.lock(|x| x.borrow_mut().powerdown_delay_s())?)
        }
        Request::SetPowerdownDelayS(delay) => {
            tmc.lock(|x| x.borrow_mut().set_powerdown_delay_s(delay))?;
            Response::SetPowerdownDelayS
        }
        Request::GetMicrosteps => {
            Response::GetMicrosteps(tmc.lock(|x| x.borrow_mut().microsteps())?)
        }
        Request::SetMicrosteps(n) => {
            tmc.lock(|x| x.borrow_mut().set_microsteps(n))?;
            Response::SetMicrosteps
        }
        Request::GetFilterStepPulses => {
            Response::GetFilterStepPulses(tmc.lock(|x| x.borrow_mut().filter_step_pulses())?)
        }
        Request::SetFilterStepPulses(enable) => {
            tmc.lock(|x| x.borrow_mut().set_filter_step_pulses(enable))?;
            Response::SetFilterStepPulses
        }
        Request::GetDoubleEdgeStep => {
            Response::GetDoubleEdgeStep(tmc.lock(|x| x.borrow_mut().double_edge_step())?)
        }
        Request::SetDoubleEdgeStep(enable) => {
            tmc.lock(|x| x.borrow_mut().set_double_edge_step(enable))?;
            Response::SetDoubleEdgeStep
        }
        Request::GetInterpolateMicrosteps => Response::GetInterpolateMicrosteps(
            tmc.lock(|x| x.borrow_mut().interpolate_microsteps())?,
        ),
        Request::SetInterpolateMicrosteps(enable) => {
            tmc.lock(|x| x.borrow_mut().set_interpolate_microsteps(enable))?;
            Response::SetInterpolateMicrosteps
        }
        Request::GetShortSupplyProtect => {
            Response::GetShortSupplyProtect(tmc.lock(|x| x.borrow_mut().short_supply_protect())?)
        }
        Request::SetShortSupplyProtect(enable) => {
            tmc.lock(|x| x.borrow_mut().set_short_supply_protect(enable))?;
            Response::SetShortSupplyProtect
        }
        Request::GetShortGroundProtect => {
            Response::GetShortGroundProtect(tmc.lock(|x| x.borrow_mut().short_ground_protect())?)
        }
        Request::SetShortGroundProtect(enable) => {
            tmc.lock(|x| x.borrow_mut().set_short_ground_protect(enable))?;
            Response::SetShortGroundProtect
        }
        Request::GetBlankTime => Response::GetBlankTime(tmc.lock(|x| x.borrow_mut().blank_time())?),
        Request::SetBlankTime(time) => {
            tmc.lock(|x| x.borrow_mut().set_blank_time(time))?;
            Response::SetBlankTime
        }
        Request::GetHysteresisEnd => {
            Response::GetHysteresisEnd(tmc.lock(|x| x.borrow_mut().hysteresis_end())?)
        }
        Request::SetHysteresisEnd(end) => {
            tmc.lock(|x| x.borrow_mut().set_hysteresis_end(end))?;
            Response::SetHysteresisEnd
        }
        Request::GetHysteresisStart => {
            Response::GetHysteresisStart(tmc.lock(|x| x.borrow_mut().hysteresis_start())?)
        }
        Request::SetHysteresisStart(start) => {
            tmc.lock(|x| x.borrow_mut().set_hysteresis_start(start))?;
            Response::SetHysteresisStart
        }
        Request::GetDecayTime => Response::GetDecayTime(tmc.lock(|x| x.borrow_mut().decay_time())?),
        Request::SetDecayTime(time) => {
            tmc.lock(|x| x.borrow_mut().set_decay_time(time))?;
            Response::SetDecayTime
        }
        Request::GetPwmMaxRpm => {
            Response::GetPwmMaxRpm(tmc.lock(|x| x.borrow_mut().pwm_max_rpm())?)
        }
        Request::SetPwmMaxRpm(rpm) => {
            tmc.lock(|x| x.borrow_mut().set_pwm_max_rpm(rpm))?;
            Response::SetPwmMaxRpm
        }
        Request::GetDriverSwitchAutoscaleLimit => Response::GetDriverSwitchAutoscaleLimit(
            tmc.lock(|x| x.borrow_mut().driver_switch_autoscale_limit())?,
        ),
        Request::SetDriverSwitchAutoscaleLimit(limit) => {
            tmc.lock(|x| x.borrow_mut().set_driver_switch_autoscale_limit(limit))?;
            Response::SetDriverSwitchAutoscaleLimit
        }
        Request::GetMaxAmplitudeChange => {
            Response::GetMaxAmplitudeChange(tmc.lock(|x| x.borrow_mut().max_amplitude_change())?)
        }
        Request::SetMaxAmplitudeChange(change) => {
            tmc.lock(|x| x.borrow_mut().set_max_amplitude_change(change))?;
            Response::SetMaxAmplitudeChange
        }
        Request::GetPwmAutogradient => {
            Response::GetPwmAutogradient(tmc.lock(|x| x.borrow_mut().pwm_autograd())?)
        }
        Request::SetPwmAutogradient(enable) => {
            tmc.lock(|x| x.borrow_mut().set_pwm_autograd(enable))?;
            Response::SetPwmAutogradient
        }
        Request::GetPwnAutoscale => {
            Response::GetPwnAutoscale(tmc.lock(|x| x.borrow_mut().pwm_autoscale())?)
        }
        Request::SetPwnAutoscale(enable) => {
            tmc.lock(|x| x.borrow_mut().set_pwm_autoscale(enable))?;
            Response::SetPwnAutoscale
        }
        Request::GetPwmFrequency => {
            Response::GetPwmFrequency(tmc.lock(|x| x.borrow_mut().pwm_frequency())?)
        }
        Request::SetPwmFrequency(frequency) => {
            tmc.lock(|x| x.borrow_mut().set_pwm_frequency(frequency))?;
            Response::SetPwmFrequency
        }
        Request::GetPwmGradient => {
            Response::GetPwmGradient(tmc.lock(|x| x.borrow_mut().pwm_gradient())?)
        }
        Request::SetPwmGradient(gradient) => {
            tmc.lock(|x| x.borrow_mut().set_pwm_gradient(gradient))?;
            Response::SetPwmGradient
        }
        Request::GetPwmOffset => Response::GetPwmOffset(tmc.lock(|x| x.borrow_mut().pwm_offset())?),
        Request::SetPwmOffset(offset) => {
            tmc.lock(|x| x.borrow_mut().set_pwm_offset(offset))?;
            Response::SetPwmOffset
        }
        Request::GetChargePumpUndervoltage => Response::GetChargePumpUndervoltage(
            tmc.lock(|x| x.borrow_mut().charge_pump_undervoltage())?,
        ),
        Request::GetDriverError => {
            Response::GetDriverError(tmc.lock(|x| x.borrow_mut().driver_error())?)
        }
        Request::GetIsReset => Response::GetIsReset(tmc.lock(|x| x.borrow_mut().is_reset())?),
        Request::GetDirectionPin => {
            Response::GetDirectionPin(tmc.lock(|x| x.borrow_mut().direction_pin())?)
        }
        Request::GetDisablePwmPin => {
            Response::GetDisablePwmPin(tmc.lock(|x| x.borrow_mut().disable_pwm_pin())?)
        }
        Request::GetStepPin => Response::GetStepPin(tmc.lock(|x| x.borrow_mut().step_pin())?),
        Request::GetPowerdownUartPin => {
            Response::GetPowerdownUartPin(tmc.lock(|x| x.borrow_mut().powerdown_uart_pin())?)
        }
        Request::GetDiagnosticPin => {
            Response::GetDiagnosticPin(tmc.lock(|x| x.borrow_mut().diagnostic_pin())?)
        }
        Request::GetMicrostep2Pin => {
            Response::GetMicrostep2Pin(tmc.lock(|x| x.borrow_mut().microstep2_pin())?)
        }
        Request::GetMicrostep1Pin => {
            Response::GetMicrostep1Pin(tmc.lock(|x| x.borrow_mut().microstep1_pin())?)
        }
        Request::GetDisablePin => {
            Response::GetDisablePin(tmc.lock(|x| x.borrow_mut().disable_pin())?)
        }
        Request::GetMicrostepTime => {
            Response::GetMicrostepTime(tmc.lock(|x| x.borrow_mut().microstep_time())?)
        }
        Request::GetMotorLoad => Response::GetMotorLoad(tmc.lock(|x| x.borrow_mut().motor_load())?),
        Request::GetMicrostepPosition => {
            Response::GetMicrostepPosition(tmc.lock(|x| x.borrow_mut().microstep_position())?)
        }
        Request::GetMicrostepCurrent => {
            let (a, b) = tmc.lock(|x| x.borrow_mut().microstep_current())?;
            Response::GetMicrostepCurrent(a, b)
        }
        Request::GetStopped => Response::GetStopped(tmc.lock(|x| x.borrow_mut().stopped())?),
        Request::GetPwmMode => Response::GetPwmMode(tmc.lock(|x| x.borrow_mut().pwm_mode())?),
        Request::GetCurrentScale => {
            Response::GetCurrentScale(tmc.lock(|x| x.borrow_mut().current_scale())?)
        }
        Request::GetTemperature => {
            Response::GetTemperature(tmc.lock(|x| x.borrow_mut().temperature())?)
        }
        Request::GetOpenLoad => Response::GetOpenLoad(tmc.lock(|x| x.borrow_mut().open_load())?),
        Request::GetLowSideShort => {
            Response::GetLowSideShort(tmc.lock(|x| x.borrow_mut().low_side_short())?)
        }
        Request::GetGroundShort => {
            Response::GetGroundShort(tmc.lock(|x| x.borrow_mut().ground_short())?)
        }
        Request::GetOverTemperature => {
            Response::GetOverTemperature(tmc.lock(|x| x.borrow_mut().over_temperature())?)
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
        let target_v = RPM.lock(|x| *x.borrow()) / 60.0;
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
