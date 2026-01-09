#![allow(dead_code)]

use core::result;
use defmt::{debug, info, Format};
use deku::ctx::BitSize;
use deku::prelude::*;
use embedded_io::{Read, ReadExactError, Write};
use esp_hal::{
    delay::Delay,
    gpio::{
        interconnect::{PeripheralInput, PeripheralOutput},
        Level::Low,
        Output, OutputConfig, OutputPin,
    },
    uart::{self, Instance, IoError, Uart},
    Blocking,
};
use thiserror::Error;

// General configuration registers.
const GLOBAL_CONFIG_REG: u8 = 0x00;
const GLOBAL_STATUS_REG: u8 = 0x01;
const TRANSMISSION_COUNT_REG: u8 = 0x02;
const RESPONSE_DELAY_REG: u8 = 0x03;
const DEFAULTS_WRITE_REG: u8 = 0x04;
const DEFAULTS_READ_REG: u8 = 0x05;
const PIN_STATES_REG: u8 = 0x06;
const FACTORY_CONFIG_REG: u8 = 0x07;
// Velocity dependent control.
const CURRENT_CONFIG_REG: u8 = 0x10;
const POWER_DOWN_DELAY_REG: u8 = 0x11;
const MICROSTEP_TIME_REG: u8 = 0x12;
const PWM_THRESHOLD_REG: u8 = 0x13;
const VELOCITY_REG: u8 = 0x22;
const COOLSTEP_THRESHOLD_REG: u8 = 0x14;
const STALLGUARD_THRESHOLD_REG: u8 = 0x40;
const MOTOR_LOAD_REG: u8 = 0x41;
const COOLSTEP_CONFIG_REG: u8 = 0x42;
// Microstepping control registers.
const MICROSTEP_POSITION_REG: u8 = 0x6A;
const MICROSTEP_CURRENT_REG: u8 = 0x6B;
// Driver control registers.
const DRIVER_CONFIG_REG: u8 = 0x6C;
const DRIVER_STATUS_REG: u8 = 0x6F;
const PWM_CONFIG_REG: u8 = 0x70;
const PWM_SCALE_REG: u8 = 0x71;
const PWM_AUTO_REG: u8 = 0x72;

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(endian = "big", magic = b"\x05")]
struct ReadRequestFrame {
    address: u8,
    #[deku(pad_bits_before = "1")]
    #[deku(bits = 7)]
    register: u8,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(endian = "big", magic = b"\x05\xff")]
struct ReadResponseFrame {
    #[deku(pad_bits_before = "1")]
    #[deku(bits = 7)]
    register: u8,
    #[deku(ctx = "*register")]
    data: FrameData,
    crc: u8,
}

#[deku_derive(DekuRead, DekuWrite)]
#[derive(Debug, Clone, Copy)]
#[deku(endian = "big", magic = b"\x05")]
struct WriteFrame {
    address: u8,
    #[deku(bits = 1, temp, temp_value = "true")]
    write_bit: bool,
    #[deku(bits = 7)]
    register: u8,
    #[deku(ctx = "*register")]
    data: FrameData,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(
    ctx = "endian: deku::ctx::Endian, register: u8",
    id = "register",
    endian = "endian"
)]
enum FrameData {
    // General configuration registers.
    #[deku(id = "GLOBAL_CONFIG_REG")]
    GlobalConfig(GlobalConfig),

    #[deku(id = "GLOBAL_STATUS_REG")]
    GlobalStatus(GlobalStatus),

    #[deku(id = "TRANSMISSION_COUNT_REG")]
    TransmissionCount(#[deku(pad_bits_before = "24")] u8),

    #[deku(id = "RESPONSE_DELAY_REG")]
    ResponseDelay(
        #[deku(pad_bits_before = "20")]
        #[deku(bits = "4")]
        #[deku(pad_bits_after = "8")]
        u8,
    ),

    #[deku(id = "DEFAULTS_WRITE_REG")]
    DefaultsWrite(DefaultsWrite),

    // TODO: Turn this into a struct.
    #[deku(id = "DEFAULTS_READ_REG")]
    DefaultsRead(u32),

    #[deku(id = "PIN_STATES_REG")]
    PinStates(PinStates),

    #[deku(id = "FACTORY_CONFIG_REG")]
    FactoryConfig(FactoryConfig),

    // Velocity dependent control.
    #[deku(id = "CURRENT_CONFIG_REG")]
    CurrentConfig(CurrentConfig),

    #[deku(id = "POWER_DOWN_DELAY_REG")]
    PowerDownDelay(#[deku(pad_bits_before = "24")] u8),

    #[deku(id = "MICROSTEP_TIME_REG")]
    MicrostepTime(
        #[deku(pad_bits_before = "12")]
        #[deku(bits = 20)]
        u32,
    ),

    #[deku(id = "PWM_THRESHOLD_REG")]
    PwmThreshold(
        #[deku(pad_bits_before = "12")]
        #[deku(bits = 20)]
        u32,
    ),

    #[deku(id = "VELOCITY_REG")]
    Velocity(
        #[deku(pad_bits_before = "8")]
        #[deku(bits = 24)]
        i32,
    ),

    #[deku(id = "COOLSTEP_THRESHOLD_REG")]
    CoolstepThreshold(
        #[deku(pad_bits_before = "12")]
        #[deku(bits = 20)]
        u32,
    ),

    #[deku(id = "STALLGUARD_THRESHOLD_REG")]
    StallguardThreshold(#[deku(pad_bits_before = "24")] u8),

    #[deku(id = "MOTOR_LOAD_REG")]
    MotorLoad(
        #[deku(pad_bits_before = "22")]
        #[deku(bits = 10)]
        u16,
    ),

    #[deku(id = "COOLSTEP_CONFIG_REG")]
    CoolstepConfig(CoolstepConfig),

    // Microstepping control registers.
    #[deku(id = "MICROSTEP_POSITION_REG")]
    MicrostepPosition(
        #[deku(pad_bits_before = "22")]
        #[deku(bits = 10)]
        u16,
    ),

    #[deku(id = "MICROSTEP_CURRENT_REG")]
    MicrostepCurrent(
        #[deku(pad_bits_before = "7")]
        #[deku(bits = 9)]
        i16,
        #[deku(pad_bits_before = "7")]
        #[deku(bits = 9)]
        i16,
    ),

    // Driver control registers.
    #[deku(id = "DRIVER_CONFIG_REG")]
    DriverConfig(DriverConfig),

    #[deku(id = "DRIVER_STATUS_REG")]
    DriverStatus(DriverStatus),

    #[deku(id = "PWM_CONFIG_REG")]
    PwmConfig(PwmConfig),

    #[deku(id = "PWM_SCALE_REG")]
    PwmScale(PwmScale),

    #[deku(id = "PWM_AUTO_REG")]
    PwmAuto(PwmAuto),
}

// General configuration registers.
#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct GlobalConfig {
    #[deku(pad_bits_before = "22")]
    #[deku(bits = 1)]
    test_mode: bool,
    #[deku(bits = 1)]
    filter_step_pulses: bool,
    #[deku(bits = 1)]
    uart_selects_microsteps: bool,
    #[deku(bits = 1)]
    pin_uart_mode: bool,
    index_output: IndexOutput,
    #[deku(bits = 1)]
    invert_direction: bool,
    #[deku(bits = 1)]
    disable_pwm: bool,
    #[deku(bits = 1)]
    internal_sense_resistor: bool,
    #[deku(bits = 1)]
    external_current_scaling: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, DekuRead, DekuWrite, Format)]
#[deku(
    id_type = "u8",
    bits = 2,
    ctx = "endian: deku::ctx::Endian",
    endian = "endian"
)]
pub enum IndexOutput {
    #[deku(id = 0b00)]
    Period,
    #[deku(id = 0b01)]
    Overtemperature,
    #[deku(id = 0b10)]
    Microstep,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct GlobalStatus {
    #[deku(pad_bits_before = "29")]
    #[deku(bits = 1)]
    charge_pump_undervoltage: bool,
    #[deku(bits = 1)]
    driver_error: bool,
    #[deku(bits = 1)]
    reset: bool,
}

#[deku_derive(DekuRead, DekuWrite)]
#[derive(Debug, Clone, Copy)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct DefaultsWrite {
    #[deku(pad_bits_before = "16")]
    #[deku(temp, temp_value = "0xBD")]
    magic: u8,
    #[deku(pad_bits_before = "2")]
    #[deku(bits = 2)]
    byte_address: u8,
    #[deku(pad_bits_before = "1")]
    #[deku(bits = 3)]
    bit_address: u8,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct PinStates {
    version: u8,
    #[deku(pad_bits_before = "14")]
    #[deku(bits = 1)]
    direction: bool,
    #[deku(bits = 1)]
    disable_pwm: bool,
    #[deku(bits = 1)]
    step: bool,
    #[deku(bits = 1)]
    powerdown_uart: bool,
    #[deku(pad_bits_before = "1")]
    #[deku(bits = 1)]
    diagnostic: bool,
    #[deku(bits = 1)]
    microstep2: bool,
    #[deku(bits = 1)]
    microstep1: bool,
    #[deku(pad_bits_before = "1")]
    #[deku(bits = 1)]
    disable: bool,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct FactoryConfig {
    #[deku(pad_bits_before = "22")]
    #[deku(bits = 2)]
    overtemperature_threshold: u8,
    #[deku(pad_bits_before = "3")]
    #[deku(bits = 5)]
    clock_frequency: u8,
}

// Velocity dependent control.
#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct CurrentConfig {
    #[deku(pad_bits_before = "12")]
    #[deku(bits = 4)]
    powerdown_time: u8,
    #[deku(pad_bits_before = "3")]
    #[deku(bits = 5)]
    running_current_scale: u8,
    #[deku(pad_bits_before = "3")]
    #[deku(bits = 5)]
    stopped_current_scale: u8,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct CoolstepConfig {
    #[deku(pad_bits_before = "16")]
    #[deku(bits = 1)]
    lower_min_current: bool,
    #[deku(bits = 2)]
    current_downstep_rate: u8,
    #[deku(pad_bits_before = "1")]
    #[deku(bits = 4)]
    stallguard_hysteresis: u8,
    #[deku(pad_bits_before = "1")]
    #[deku(bits = 2)]
    current_upstep: u8,
    #[deku(pad_bits_before = "1")]
    #[deku(bits = 4)]
    stallguard_threshold: u8,
}

// Driver control registers.
#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct DriverConfig {
    #[deku(bits = 1)]
    disable_short_supply_protect: bool,
    #[deku(bits = 1)]
    disable_short_ground_protect: bool,
    #[deku(bits = 1)]
    double_edge_step: bool,
    #[deku(bits = 1)]
    interpolate_microsteps: bool,
    microstep_resolution: MicrostepResolution,
    #[deku(pad_bits_before = "6")]
    #[deku(bits = 1)]
    low_sense_resistor_voltage: bool,
    blank_time: BlankTime,
    #[deku(pad_bits_before = "4")]
    #[deku(map = "|x: i8| -> result::Result<_, DekuError> { Ok(((x & 0x0F) as u8 as i8) - 3)}")]
    #[deku(writer = "(self.hysteresis_end + 3).to_writer(deku::writer, BitSize(4))")]
    #[deku(bits = 4)]
    hysteresis_end: i8,
    #[deku(map = "|x: u8| -> result::Result<_, DekuError> { Ok(x + 1)}")]
    #[deku(writer = "(self.hysteresis_start - 1).to_writer(deku::writer, BitSize(3))")]
    #[deku(bits = 3)]
    hysteresis_start: u8,
    #[deku(bits = 4)]
    decay_time: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, DekuRead, DekuWrite)]
#[deku(
    id_type = "u8",
    bits = 4,
    ctx = "endian: deku::ctx::Endian",
    endian = "endian"
)]
pub enum MicrostepResolution {
    #[deku(id = 0b1000)]
    M1,
    #[deku(id = 0b0111)]
    M2,
    #[deku(id = 0b0110)]
    M4,
    #[deku(id = 0b0101)]
    M8,
    #[deku(id = 0b0100)]
    M16,
    #[deku(id = 0b0011)]
    M32,
    #[deku(id = 0b0010)]
    M64,
    #[deku(id = 0b0001)]
    M128,
    #[deku(id = 0b0000)]
    M256,
}

impl From<u16> for MicrostepResolution {
    fn from(mut x: u16) -> Self {
        x = x.clamp(1, 256);
        match x.ilog2() {
            0 => MicrostepResolution::M1,
            1 => MicrostepResolution::M2,
            2 => MicrostepResolution::M4,
            3 => MicrostepResolution::M8,
            4 => MicrostepResolution::M16,
            5 => MicrostepResolution::M32,
            6 => MicrostepResolution::M64,
            7 => MicrostepResolution::M128,
            8 => MicrostepResolution::M256,
            _ => unreachable!(),
        }
    }
}

impl From<MicrostepResolution> for u16 {
    fn from(x: MicrostepResolution) -> Self {
        match x {
            MicrostepResolution::M1 => 1,
            MicrostepResolution::M2 => 2,
            MicrostepResolution::M4 => 4,
            MicrostepResolution::M8 => 8,
            MicrostepResolution::M16 => 16,
            MicrostepResolution::M32 => 32,
            MicrostepResolution::M64 => 64,
            MicrostepResolution::M128 => 128,
            MicrostepResolution::M256 => 256,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, DekuRead, DekuWrite)]
#[deku(
    id_type = "u8",
    bits = 2,
    ctx = "endian: deku::ctx::Endian",
    endian = "endian"
)]
pub enum BlankTime {
    #[deku(id = 0b00)]
    B16,
    #[deku(id = 0b01)]
    B24,
    #[deku(id = 0b10)]
    B36,
    #[deku(id = 0b11)]
    B54,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct DriverStatus {
    #[deku(bits = 1)]
    stopped: bool,
    #[deku(bits = 1)]
    pwm_mode: bool,
    #[deku(pad_bits_before = "9")]
    #[deku(bits = 5)]
    current_scale: u8,
    #[deku(pad_bits_before = "4")]
    temperature: TemperatureThreshold,
    open_load: PhaseStatus,
    low_side_short: PhaseStatus,
    ground_short: PhaseStatus,
    overtemperature: OvertemperatureStatus,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, DekuRead, DekuWrite)]
#[deku(
    id_type = "u8",
    bits = 2,
    ctx = "endian: deku::ctx::Endian",
    endian = "endian"
)]
pub enum PhaseStatus {
    #[deku(id = 0b00)]
    None,
    #[deku(id = 0b01)]
    PhaseA,
    #[deku(id = 0b10)]
    PhaseB,
    #[deku(id = 0b11)]
    BothPhases,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, DekuRead, DekuWrite)]
#[deku(
    id_type = "u8",
    bits = 4,
    ctx = "endian: deku::ctx::Endian",
    endian = "endian"
)]
pub enum TemperatureThreshold {
    #[deku(id = 0b0000)]
    Normal,
    #[deku(id = 0b0001)]
    Temp120C,
    #[deku(id = 0b0011)]
    Temp143C,
    #[deku(id = 0b0111)]
    Temp150C,
    #[deku(id = 0b1111)]
    Temp157C,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, DekuRead, DekuWrite)]
#[deku(
    id_type = "u8",
    bits = 2,
    ctx = "endian: deku::ctx::Endian",
    endian = "endian"
)]
pub enum OvertemperatureStatus {
    #[deku(id = 0b00)]
    Normal,
    #[deku(id = 0b01)]
    Warning,
    #[deku(id = 0b11)]
    Shutdown,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct PwmConfig {
    #[deku(bits = 4)]
    driver_switch_autoscale_limit: u8,
    #[deku(bits = 4)]
    max_amplitude_change: u8,
    #[deku(pad_bits_before = "2")]
    freewheel_mode: FreewheelMode,
    #[deku(bits = 1)]
    autogradient: bool,
    #[deku(bits = 1)]
    autoscale: bool,
    frequency: PwmFrequency,
    gradient: u8,
    offset: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, DekuRead, DekuWrite)]
#[deku(
    id_type = "u8",
    bits = 2,
    ctx = "endian: deku::ctx::Endian",
    endian = "endian"
)]
pub enum PwmFrequency {
    #[deku(id = 0b00)]
    Div1024,
    #[deku(id = 0b01)]
    Div683,
    #[deku(id = 0b10)]
    Div512,
    #[deku(id = 0b11)]
    Div410,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, DekuRead, DekuWrite)]
#[deku(
    id_type = "u8",
    bits = 2,
    ctx = "endian: deku::ctx::Endian",
    endian = "endian"
)]
pub enum FreewheelMode {
    #[deku(id = 0b00)]
    Normal,
    #[deku(id = 0b01)]
    Freewheeling,
    #[deku(id = 0b10)]
    ShortLowSide,
    #[deku(id = 0b11)]
    ShortHighSide,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct PwmScale {
    #[deku(pad_bits_before = "7")]
    #[deku(bits = 9)]
    offset: i16,
    #[deku(pad_bits_before = "8")]
    scale: u8,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct PwmAuto {
    #[deku(pad_bits_before = "8")]
    gradient: u8,
    #[deku(pad_bits_before = "8")]
    offset: u8,
}

#[derive(Debug, Clone, Copy)]
pub struct PwmState {
    pub auto_grad: u8,
    pub auto_offset: u8,
    pub scale: u8,
    pub offset: i16,
}

#[derive(Error, Debug)]
pub enum Tmc2209Error {
    #[error("UART read error")]
    UartReadError,
    #[error("UART write error")]
    UartWriteError,
    #[error("CRC mismatch")]
    CrcMismatch,
    #[error("Invalid response")]
    InvalidResponse,
}

impl From<deku::DekuError> for Tmc2209Error {
    fn from(_: deku::DekuError) -> Self {
        Tmc2209Error::InvalidResponse
    }
}

impl From<ReadExactError<IoError>> for Tmc2209Error {
    fn from(_: ReadExactError<IoError>) -> Self {
        Tmc2209Error::UartReadError
    }
}

impl From<IoError> for Tmc2209Error {
    fn from(_: IoError) -> Self {
        Tmc2209Error::UartWriteError
    }
}

pub type Result<T> = result::Result<T, Tmc2209Error>;

pub struct Tmc2209<'a> {
    uart: Uart<'a, Blocking>,
    step_pin: Output<'a>,
    dir_pin: Output<'a>,
    disable_pin: Output<'a>,
    address: u8,
    sense_ohms: f64,
    steps_per_rev: u32,
}

impl<'a> Tmc2209<'a> {
    pub fn new(
        uart: impl Instance + 'a,
        tx: impl PeripheralOutput<'a>,
        rx: impl PeripheralInput<'a>,
        step: impl OutputPin + 'a,
        dir: impl OutputPin + 'a,
        disable: impl OutputPin + 'a,
    ) -> Self {
        let config = uart::Config::default().with_baudrate(115_200);
        let uart = Uart::new(uart, config).unwrap().with_tx(tx).with_rx(rx);

        let step_pin = Output::new(step, Low, OutputConfig::default());
        let dir_pin = Output::new(dir, Low, OutputConfig::default());
        let disable_pin = Output::new(disable, Low, OutputConfig::default());

        let mut tmc = Self {
            uart,
            step_pin,
            dir_pin,
            disable_pin,
            address: 0,
            sense_ohms: 0.110,
            steps_per_rev: 200,
            /*
            global_config: GlobalConfig {
                test_mode: false,
                filter_step_pulses: true,
                uart_selects_microsteps: true,
                pin_uart_mode: true,
                index_output: IndexOutput::Period,
                invert_direction: false,
                disable_pwm: false,
                internal_sense_resistor: false,
                external_current_scaling: false,
            },
            response_delay: 3,
            current_config: CurrentConfig {
                stopped_current_scale: 10,
                running_current_scale: 0,
                powerdown_time: 8,
            },
            powerdown_delay: 20,
            pwm_threshold: 999999,
            velocity: 0,
            coolstep_threshold: 0,
            stallguard_threshold: 0,
            coolstep_config: CoolstepConfig {
                lower_min_current: false,
                current_downstep_rate: 0,
                stallguard_hysteresis: 0,
                current_upstep: 0,
                stallguard_threshold: 0,
            },
            driver_config: DriverConfig {
                disable_short_supply_protect: false,
                disable_short_ground_protect: false,
                double_edge_step: false,
                interpolate_microsteps: true,
                microstep_resolution: MicrostepResolution::M16,
                low_sense_resistor_voltage: false,
                blank_time: BlankTime::B36,
                hysteresis_end: -3,
                hysteresis_start: 4,
                decay_time: 5,
            },
            pwm_config: PwmConfig {
                driver_switch_autoscale_limit: 12,
                max_amplitude_change: 8,
                freewheel_mode: FreewheelMode::Normal,
                autogradient: true,
                autoscale: true,
                frequency: PwmFrequency::Div683,
                gradient: 14,
                offset: 36,
            },
            */
        };
        tmc.set_running_amps(0.150).unwrap();
        tmc
    }

    pub fn enable(&mut self) {
        self.disable_pin.set_low();
    }

    pub fn disable(&mut self) {
        self.disable_pin.set_high();
    }

    // =============================
    // ====Configuration options====
    // =============================

    // Global Configs
    pub fn steps_per_rev(&self) -> u32 {
        self.steps_per_rev
    }

    pub fn set_steps_per_rev(&mut self, steps: u32) {
        self.steps_per_rev = steps;
    }

    pub fn test_mode(&mut self) -> Result<bool> {
        Ok(self.global_config()?.test_mode)
    }

    pub fn set_test_mode(&mut self, enable: bool) -> Result<()> {
        let mut config = self.global_config()?;
        config.test_mode = enable;
        self.write_register(GLOBAL_CONFIG_REG, FrameData::GlobalConfig(config))?;
        Ok(())
    }

    pub fn filter_step_pulses(&mut self) -> Result<bool> {
        Ok(self.global_config()?.filter_step_pulses)
    }

    pub fn set_filter_step_pulses(&mut self, enable: bool) -> Result<()> {
        let mut config = self.global_config()?;
        config.filter_step_pulses = enable;
        self.write_register(GLOBAL_CONFIG_REG, FrameData::GlobalConfig(config))?;
        Ok(())
    }

    pub fn pin_uart_mode(&mut self) -> Result<bool> {
        Ok(self.global_config()?.pin_uart_mode)
    }

    pub fn set_pin_uart_mode(&mut self, enable: bool) -> Result<()> {
        let mut config = self.global_config()?;
        config.pin_uart_mode = enable;
        self.write_register(GLOBAL_CONFIG_REG, FrameData::GlobalConfig(config))?;
        Ok(())
    }

    pub fn index_output(&mut self) -> Result<IndexOutput> {
        Ok(self.global_config()?.index_output)
    }

    pub fn set_index_output(&mut self, output: IndexOutput) -> Result<()> {
        let mut config = self.global_config()?;
        config.index_output = output;
        self.write_register(GLOBAL_CONFIG_REG, FrameData::GlobalConfig(config))?;
        Ok(())
    }

    pub fn invert_direction(&mut self) -> Result<bool> {
        Ok(self.global_config()?.invert_direction)
    }

    pub fn set_invert_direction(&mut self, enable: bool) -> Result<()> {
        let mut config = self.global_config()?;
        config.invert_direction = enable;
        self.write_register(GLOBAL_CONFIG_REG, FrameData::GlobalConfig(config))?;
        Ok(())
    }

    pub fn pwm_enabled(&mut self) -> Result<bool> {
        Ok(!self.global_config()?.disable_pwm)
    }

    pub fn set_pwm_enabled(&mut self, enable: bool) -> Result<()> {
        let mut config = self.global_config()?;
        config.disable_pwm = !enable;
        self.write_register(GLOBAL_CONFIG_REG, FrameData::GlobalConfig(config))?;
        Ok(())
    }

    pub fn internal_sense_resistor(&mut self) -> Result<bool> {
        Ok(self.global_config()?.internal_sense_resistor)
    }

    pub fn set_internal_sense_resistor(&mut self, enable: bool) -> Result<()> {
        let mut config = self.global_config()?;
        config.internal_sense_resistor = enable;
        self.write_register(GLOBAL_CONFIG_REG, FrameData::GlobalConfig(config))?;
        Ok(())
    }

    pub fn external_current_scaling(&mut self) -> Result<bool> {
        Ok(self.global_config()?.external_current_scaling)
    }

    pub fn set_external_current_scaling(&mut self, enable: bool) -> Result<()> {
        let mut config = self.global_config()?;
        config.external_current_scaling = enable;
        self.write_register(GLOBAL_CONFIG_REG, FrameData::GlobalConfig(config))?;
        Ok(())
    }

    pub fn response_delay(&mut self) -> Result<u8> {
        if let FrameData::ResponseDelay(delay) = self.read_register(RESPONSE_DELAY_REG)? {
            Ok(delay)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn set_response_delay(&mut self, delay: u8) -> Result<()> {
        assert!(
            (0..=15).contains(&delay),
            "Response delay must be between 0 and 15."
        );
        self.write_register(RESPONSE_DELAY_REG, FrameData::ResponseDelay(delay))?;
        Ok(())
    }

    // Current Configs
    pub fn stopped_amps(&mut self) -> Result<f64> {
        let config = self.current_config()?;
        Ok(self.current_scale_to_amps(config.stopped_current_scale)?)
    }

    pub fn set_stopped_amps(&mut self, rms_amps: f64) -> Result<()> {
        let mut config = self.current_config()?;
        config.stopped_current_scale = self.get_current_scale(rms_amps)?;
        self.write_register(CURRENT_CONFIG_REG, FrameData::CurrentConfig(config))?;
        Ok(())
    }

    pub fn running_amps(&mut self) -> Result<f64> {
        let config = self.current_config()?;
        Ok(self.current_scale_to_amps(config.running_current_scale)?)
    }

    pub fn set_running_amps(&mut self, rms_amps: f64) -> Result<()> {
        let mut config = self.current_config()?;
        config.running_current_scale = self.get_current_scale(rms_amps)?;
        self.write_register(CURRENT_CONFIG_REG, FrameData::CurrentConfig(config))?;
        Ok(())
    }

    pub fn powerdown_time(&mut self) -> Result<u8> {
        Ok(self.current_config()?.powerdown_time)
    }

    pub fn set_powerdown_time(&mut self, time: u8) -> Result<()> {
        let mut config = self.current_config()?;
        config.powerdown_time = time;
        self.write_register(CURRENT_CONFIG_REG, FrameData::CurrentConfig(config))?;
        Ok(())
    }

    pub fn powerdown_delay(&mut self) -> Result<u8> {
        if let FrameData::PowerDownDelay(delay) = self.read_register(POWER_DOWN_DELAY_REG)? {
            Ok(delay)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn set_powerdown_delay(&mut self, delay: u8) -> Result<()> {
        self.write_register(POWER_DOWN_DELAY_REG, FrameData::PowerDownDelay(delay))?;
        Ok(())
    }

    pub fn pwm_threshold(&mut self) -> Result<u32> {
        if let FrameData::PwmThreshold(threshold) = self.read_register(PWM_THRESHOLD_REG)? {
            Ok(threshold)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn set_pwm_threshold(&mut self, threshold: u32) -> Result<()> {
        self.write_register(PWM_THRESHOLD_REG, FrameData::PwmThreshold(threshold))?;
        Ok(())
    }

    pub fn velocity(&mut self) -> Result<f64> {
        let velocity = if let FrameData::Velocity(value) = self.read_register(VELOCITY_REG)? {
            value
        } else {
            return Err(Tmc2209Error::InvalidResponse);
        };
        let pps = (velocity as f64) * 0.715;
        Ok(pps * 60.0 / (self.pulses_per_rev()? as f64))
    }

    pub fn set_velocity(&mut self, rpm: f64) -> Result<()> {
        let pps = (self.pulses_per_rev()? as f64) * rpm / 60.0;
        let velocity = (pps / 0.715) as i32;
        self.write_register(VELOCITY_REG, FrameData::Velocity(velocity))?;
        Ok(())
    }

    // Stallguard and coolstep control
    pub fn coolstep_threshold(&mut self) -> Result<u32> {
        if let FrameData::CoolstepThreshold(threshold) =
            self.read_register(COOLSTEP_THRESHOLD_REG)?
        {
            Ok(threshold)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn set_coolstep_threshold(&mut self, threshold: u32) -> Result<()> {
        self.write_register(
            COOLSTEP_THRESHOLD_REG,
            FrameData::CoolstepThreshold(threshold),
        )?;
        Ok(())
    }

    pub fn stallguard_threshold(&mut self) -> Result<u8> {
        if let FrameData::StallguardThreshold(threshold) =
            self.read_register(STALLGUARD_THRESHOLD_REG)?
        {
            Ok(threshold)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn set_stallguard_threshold(&mut self, threshold: u8) -> Result<()> {
        self.write_register(
            STALLGUARD_THRESHOLD_REG,
            FrameData::StallguardThreshold(threshold),
        )?;
        Ok(())
    }

    pub fn coolstep_lower_min_current(&mut self) -> Result<bool> {
        Ok(self.coolstep_config()?.lower_min_current)
    }

    pub fn set_coolstep_lower_min_current(&mut self, enable: bool) -> Result<()> {
        let mut config = self.coolstep_config()?;
        config.lower_min_current = enable;
        self.write_register(COOLSTEP_CONFIG_REG, FrameData::CoolstepConfig(config))?;
        Ok(())
    }

    pub fn coolstep_current_downstep_rate(&mut self) -> Result<u8> {
        Ok(self.coolstep_config()?.current_downstep_rate)
    }

    pub fn set_coolstep_current_downstep_rate(&mut self, rate: u8) -> Result<()> {
        let mut config = self.coolstep_config()?;
        config.current_downstep_rate = rate;
        self.write_register(COOLSTEP_CONFIG_REG, FrameData::CoolstepConfig(config))?;
        Ok(())
    }

    pub fn stallguard_hysteresis(&mut self) -> Result<u8> {
        Ok(self.coolstep_config()?.stallguard_hysteresis)
    }

    pub fn set_stallguard_hysteresis(&mut self, hysteresis: u8) -> Result<()> {
        let mut config = self.coolstep_config()?;
        config.stallguard_hysteresis = hysteresis;
        self.write_register(COOLSTEP_CONFIG_REG, FrameData::CoolstepConfig(config))?;
        Ok(())
    }

    pub fn current_upstep(&mut self) -> Result<u8> {
        Ok(self.coolstep_config()?.current_upstep)
    }

    pub fn set_current_upstep(&mut self, upstep: u8) -> Result<()> {
        let mut config = self.coolstep_config()?;
        config.current_upstep = upstep;
        self.write_register(COOLSTEP_CONFIG_REG, FrameData::CoolstepConfig(config))?;
        Ok(())
    }

    pub fn coolstep_stallguard_threshold(&mut self) -> Result<u8> {
        Ok(self.coolstep_config()?.stallguard_threshold)
    }

    pub fn set_coolstep_stallguard_threshold(&mut self, threshold: u8) -> Result<()> {
        let mut config = self.coolstep_config()?;
        config.stallguard_threshold = threshold;
        self.write_register(COOLSTEP_CONFIG_REG, FrameData::CoolstepConfig(config))?;
        Ok(())
    }

    // Driver Configs
    pub fn short_supply_protect(&mut self) -> Result<bool> {
        Ok(!self.driver_config()?.disable_short_supply_protect)
    }

    pub fn set_short_supply_protect(&mut self, enable: bool) -> Result<()> {
        let mut config = self.driver_config()?;
        config.disable_short_supply_protect = !enable;
        self.write_register(DRIVER_CONFIG_REG, FrameData::DriverConfig(config))?;
        Ok(())
    }

    pub fn short_ground_protect(&mut self) -> Result<bool> {
        Ok(!self.driver_config()?.disable_short_ground_protect)
    }

    pub fn set_short_ground_protect(&mut self, enable: bool) -> Result<()> {
        let mut config = self.driver_config()?;
        config.disable_short_ground_protect = !enable;
        self.write_register(DRIVER_CONFIG_REG, FrameData::DriverConfig(config))?;
        Ok(())
    }

    pub fn double_edge_step(&mut self) -> Result<bool> {
        Ok(self.driver_config()?.double_edge_step)
    }

    pub fn set_double_edge_step(&mut self, enable: bool) -> Result<()> {
        let mut config = self.driver_config()?;
        config.double_edge_step = enable;
        self.write_register(DRIVER_CONFIG_REG, FrameData::DriverConfig(config))?;
        Ok(())
    }

    pub fn interpolate_microsteps(&mut self) -> Result<bool> {
        Ok(self.driver_config()?.interpolate_microsteps)
    }

    pub fn set_interpolate_microsteps(&mut self, enable: bool) -> Result<()> {
        let mut config = self.driver_config()?;
        config.interpolate_microsteps = enable;
        self.write_register(DRIVER_CONFIG_REG, FrameData::DriverConfig(config))?;
        Ok(())
    }

    pub fn microsteps(&mut self) -> Result<u16> {
        Ok(self.driver_config()?.microstep_resolution.into())
    }

    pub fn set_microsteps(&mut self, num_microsteps: u16) -> Result<()> {
        let mut global_config = self.global_config()?;
        if num_microsteps == 0 {
            global_config.uart_selects_microsteps = false;
        } else {
            global_config.uart_selects_microsteps = true;
            let mut driver_config = self.driver_config()?;
            driver_config.microstep_resolution = num_microsteps.into();
            self.write_register(DRIVER_CONFIG_REG, FrameData::DriverConfig(driver_config))?;
        }
        self.write_register(GLOBAL_CONFIG_REG, FrameData::GlobalConfig(global_config))?;
        Ok(())
    }

    pub fn low_sense_resistor_voltage(&mut self) -> Result<bool> {
        Ok(self.driver_config()?.low_sense_resistor_voltage)
    }

    pub fn set_low_sense_resistor_voltage(&mut self, enable: bool) -> Result<()> {
        let mut config = self.driver_config()?;
        config.low_sense_resistor_voltage = enable;
        self.write_register(DRIVER_CONFIG_REG, FrameData::DriverConfig(config))?;
        Ok(())
    }

    pub fn blank_time(&mut self) -> Result<BlankTime> {
        Ok(self.driver_config()?.blank_time)
    }

    pub fn set_blank_time(&mut self, time: BlankTime) -> Result<()> {
        let mut config = self.driver_config()?;
        config.blank_time = time;
        self.write_register(DRIVER_CONFIG_REG, FrameData::DriverConfig(config))?;
        Ok(())
    }

    pub fn hysteresis_end(&mut self) -> Result<i8> {
        Ok(self.driver_config()?.hysteresis_end)
    }

    pub fn set_hysteresis_end(&mut self, end: i8) -> Result<()> {
        assert!(
            (-3..=12).contains(&end),
            "Hysteresis end must be between -3 and 12."
        );
        let mut config = self.driver_config()?;
        config.hysteresis_end = end;
        self.write_register(DRIVER_CONFIG_REG, FrameData::DriverConfig(config))?;
        Ok(())
    }

    pub fn hysteresis_start(&mut self) -> Result<u8> {
        Ok(self.driver_config()?.hysteresis_start)
    }

    pub fn set_hysteresis_start(&mut self, start: u8) -> Result<()> {
        assert!(
            (0..=7).contains(&start),
            "Hysteresis start must be between 0 and 7."
        );
        let mut config = self.driver_config()?;
        config.hysteresis_start = start;
        self.write_register(DRIVER_CONFIG_REG, FrameData::DriverConfig(config))?;
        Ok(())
    }

    pub fn decay_time(&mut self) -> Result<u8> {
        Ok(self.driver_config()?.decay_time)
    }

    pub fn set_decay_time(&mut self, time: u8) -> Result<()> {
        assert!(
            (0..=15).contains(&time),
            "Decay time must be between 0 and 15."
        );
        let mut config = self.driver_config()?;
        config.decay_time = time;
        self.write_register(DRIVER_CONFIG_REG, FrameData::DriverConfig(config))?;
        Ok(())
    }

    // PWM Configs
    pub fn driver_switch_autoscale_limit(&mut self) -> Result<u8> {
        Ok(self.pwm_config()?.driver_switch_autoscale_limit)
    }

    pub fn set_driver_switch_autoscale_limit(&mut self, limit: u8) -> Result<()> {
        assert!(
            (0..=15).contains(&limit),
            "Autoscale limit mut be between 0 and 15."
        );
        let mut config = self.pwm_config()?;
        config.driver_switch_autoscale_limit = limit;
        self.write_register(PWM_CONFIG_REG, FrameData::PwmConfig(config))?;
        Ok(())
    }

    pub fn max_amplitude_change(&mut self) -> Result<u8> {
        Ok(self.pwm_config()?.max_amplitude_change)
    }

    pub fn set_max_amplitude_change(&mut self, change: u8) -> Result<()> {
        assert!(
            (0..=15).contains(&change),
            "Max amplitude change must be between 0 and 15."
        );
        let mut config = self.pwm_config()?;
        config.max_amplitude_change = change;
        self.write_register(PWM_CONFIG_REG, FrameData::PwmConfig(config))?;
        Ok(())
    }

    pub fn freewheel_mode(&mut self) -> Result<FreewheelMode> {
        Ok(self.pwm_config()?.freewheel_mode)
    }

    pub fn set_freewheel_mode(&mut self, mode: FreewheelMode) -> Result<()> {
        let mut config = self.pwm_config()?;
        config.freewheel_mode = mode;
        self.write_register(PWM_CONFIG_REG, FrameData::PwmConfig(config))?;
        Ok(())
    }

    pub fn pwm_autogradient(&mut self) -> Result<bool> {
        Ok(self.pwm_config()?.autogradient)
    }

    pub fn set_pwm_autogradient(&mut self, enable: bool) -> Result<()> {
        let mut config = self.pwm_config()?;
        config.autogradient = enable;
        self.write_register(PWM_CONFIG_REG, FrameData::PwmConfig(config))?;
        Ok(())
    }

    pub fn pwn_autoscale(&mut self) -> Result<bool> {
        Ok(self.pwm_config()?.autoscale)
    }

    pub fn set_pwn_autoscale(&mut self, enable: bool) -> Result<()> {
        let mut config = self.pwm_config()?;
        config.autoscale = enable;
        self.write_register(PWM_CONFIG_REG, FrameData::PwmConfig(config))?;
        Ok(())
    }

    pub fn pwm_frequency(&mut self) -> Result<PwmFrequency> {
        Ok(self.pwm_config()?.frequency)
    }

    pub fn set_pwm_frequency(&mut self, frequency: PwmFrequency) -> Result<()> {
        let mut config = self.pwm_config()?;
        config.frequency = frequency;
        self.write_register(PWM_CONFIG_REG, FrameData::PwmConfig(config))?;
        Ok(())
    }

    pub fn pwm_gradient(&mut self) -> Result<u8> {
        Ok(self.pwm_config()?.gradient)
    }

    pub fn set_pwm_gradient(&mut self, gradient: u8) -> Result<()> {
        let mut config = self.pwm_config()?;
        config.gradient = gradient;
        self.write_register(PWM_CONFIG_REG, FrameData::PwmConfig(config))?;
        Ok(())
    }

    pub fn pwm_offset(&mut self) -> Result<u8> {
        Ok(self.pwm_config()?.offset)
    }

    pub fn set_pwm_offset(&mut self, offset: u8) -> Result<()> {
        let mut config = self.pwm_config()?;
        config.offset = offset;
        self.write_register(PWM_CONFIG_REG, FrameData::PwmConfig(config))?;
        Ok(())
    }

    // ========================
    // ====Motion Control======
    // ========================
    pub fn toggle_step(&mut self) {
        self.step_pin.toggle();
    }

    // ========================
    // ====Status Functions====
    // ========================
    // Global Status
    pub fn charge_pump_undervoltage(&mut self) -> Result<bool> {
        if let FrameData::GlobalStatus(data) = self.read_register(GLOBAL_STATUS_REG)? {
            Ok(data.charge_pump_undervoltage)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn driver_error(&mut self) -> Result<bool> {
        if let FrameData::GlobalStatus(data) = self.read_register(GLOBAL_STATUS_REG)? {
            Ok(data.driver_error)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn is_reset(&mut self) -> Result<bool> {
        if let FrameData::GlobalStatus(data) = self.read_register(GLOBAL_STATUS_REG)? {
            Ok(data.reset)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn transmission_count(&mut self) -> Result<u8> {
        if let FrameData::TransmissionCount(x) = self.read_register(TRANSMISSION_COUNT_REG)? {
            Ok(x)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn version(&mut self) -> Result<u8> {
        if let FrameData::PinStates(data) = self.read_register(PIN_STATES_REG)? {
            Ok(data.version)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn direction_pin(&mut self) -> Result<bool> {
        if let FrameData::PinStates(data) = self.read_register(PIN_STATES_REG)? {
            Ok(data.direction)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn disable_pwm_pin(&mut self) -> Result<bool> {
        if let FrameData::PinStates(data) = self.read_register(PIN_STATES_REG)? {
            Ok(data.disable_pwm)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }
    pub fn step_pin(&mut self) -> Result<bool> {
        if let FrameData::PinStates(data) = self.read_register(PIN_STATES_REG)? {
            Ok(data.step)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn powerdown_uart_pin(&mut self) -> Result<bool> {
        if let FrameData::PinStates(data) = self.read_register(PIN_STATES_REG)? {
            Ok(data.powerdown_uart)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn diagnostic_pin(&mut self) -> Result<bool> {
        if let FrameData::PinStates(data) = self.read_register(PIN_STATES_REG)? {
            Ok(data.diagnostic)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn microstep2_pin(&mut self) -> Result<bool> {
        if let FrameData::PinStates(data) = self.read_register(PIN_STATES_REG)? {
            Ok(data.microstep2)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn microstep1_pin(&mut self) -> Result<bool> {
        if let FrameData::PinStates(data) = self.read_register(PIN_STATES_REG)? {
            Ok(data.microstep1)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn disable_pin(&mut self) -> Result<bool> {
        if let FrameData::PinStates(data) = self.read_register(PIN_STATES_REG)? {
            Ok(data.disable)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn microstep_time(&mut self) -> Result<u32> {
        if let FrameData::MicrostepTime(x) = self.read_register(MICROSTEP_TIME_REG)? {
            Ok(x)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn motor_load(&mut self) -> Result<u16> {
        if let FrameData::MotorLoad(x) = self.read_register(MOTOR_LOAD_REG)? {
            Ok(x)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn microstep_position(&mut self) -> Result<u16> {
        if let FrameData::MicrostepPosition(x) = self.read_register(MICROSTEP_POSITION_REG)? {
            Ok(x)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn microstep_current(&mut self) -> Result<(i16, i16)> {
        if let FrameData::MicrostepCurrent(a, b) = self.read_register(MICROSTEP_CURRENT_REG)? {
            Ok((a, b))
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn stopped(&mut self) -> Result<bool> {
        if let FrameData::DriverStatus(data) = self.read_register(DRIVER_STATUS_REG)? {
            Ok(data.stopped)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn pwm_mode(&mut self) -> Result<bool> {
        if let FrameData::DriverStatus(data) = self.read_register(DRIVER_STATUS_REG)? {
            Ok(data.pwm_mode)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn current_scale(&mut self) -> Result<u8> {
        if let FrameData::DriverStatus(data) = self.read_register(DRIVER_STATUS_REG)? {
            Ok(data.current_scale)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn temperature(&mut self) -> Result<TemperatureThreshold> {
        if let FrameData::DriverStatus(data) = self.read_register(DRIVER_STATUS_REG)? {
            Ok(data.temperature)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn open_load(&mut self) -> Result<PhaseStatus> {
        if let FrameData::DriverStatus(data) = self.read_register(DRIVER_STATUS_REG)? {
            Ok(data.open_load)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn low_side_short(&mut self) -> Result<PhaseStatus> {
        if let FrameData::DriverStatus(data) = self.read_register(DRIVER_STATUS_REG)? {
            Ok(data.low_side_short)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn ground_short(&mut self) -> Result<PhaseStatus> {
        if let FrameData::DriverStatus(data) = self.read_register(DRIVER_STATUS_REG)? {
            Ok(data.ground_short)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn overtemperature(&mut self) -> Result<OvertemperatureStatus> {
        if let FrameData::DriverStatus(data) = self.read_register(DRIVER_STATUS_REG)? {
            Ok(data.overtemperature)
        } else {
            Err(Tmc2209Error::InvalidResponse)
        }
    }

    pub fn pwm_state(&mut self) -> Result<PwmState> {
        let mut state = PwmState {
            auto_grad: 0,
            auto_offset: 0,
            scale: 0,
            offset: 0,
        };
        let FrameData::PwmAuto(data) = self.read_register(PWM_AUTO_REG)? else {
            return Err(Tmc2209Error::InvalidResponse);
        };
        state.auto_grad = data.gradient;
        state.auto_offset = data.offset;
        let FrameData::PwmScale(data) = self.read_register(PWM_SCALE_REG)? else {
            return Err(Tmc2209Error::InvalidResponse);
        };
        state.scale = data.scale;
        state.offset = data.offset;
        Ok(state)
    }

    pub fn pulses_per_rev(&mut self) -> Result<u32> {
        let msteps_per_step = u16::from(self.microsteps()?) as u32;
        let pulses_per_mstep = if self.double_edge_step()? { 1 } else { 2 };
        Ok(pulses_per_mstep * msteps_per_step * self.steps_per_rev)
    }

    // ===========================
    // ==== Utility Functions ====
    // ===========================
    fn global_config(&mut self) -> Result<GlobalConfig> {
        match self.read_register(GLOBAL_CONFIG_REG)? {
            FrameData::GlobalConfig(config) => Ok(config),
            _ => Err(Tmc2209Error::InvalidResponse),
        }
    }

    fn current_config(&mut self) -> Result<CurrentConfig> {
        match self.read_register(CURRENT_CONFIG_REG)? {
            FrameData::CurrentConfig(config) => Ok(config),
            _ => Err(Tmc2209Error::InvalidResponse),
        }
    }

    fn coolstep_config(&mut self) -> Result<CoolstepConfig> {
        match self.read_register(COOLSTEP_CONFIG_REG)? {
            FrameData::CoolstepConfig(config) => Ok(config),
            _ => Err(Tmc2209Error::InvalidResponse),
        }
    }

    fn driver_config(&mut self) -> Result<DriverConfig> {
        match self.read_register(DRIVER_CONFIG_REG)? {
            FrameData::DriverConfig(config) => Ok(config),
            _ => Err(Tmc2209Error::InvalidResponse),
        }
    }

    fn pwm_config(&mut self) -> Result<PwmConfig> {
        match self.read_register(PWM_CONFIG_REG)? {
            FrameData::PwmConfig(config) => Ok(config),
            _ => Err(Tmc2209Error::InvalidResponse),
        }
    }

    fn write_register(&mut self, register: u8, data: FrameData) -> Result<()> {
        // Read transmission count before write
        let count_before = self.transmission_count()?;

        let frame = WriteFrame {
            address: self.address,
            register,
            data,
        };

        // Write frame to buffer with extra byte for CRC.
        let mut buf = [0u8; 8];
        let bytes = frame.to_bytes()?;
        buf[..7].copy_from_slice(&bytes[..7]);
        buf[7] = crc8(&buf[0..7]);

        info!(
            "Writing register {:#04x}, frame: {=[u8]:#02x}",
            register, buf
        );

        self.uart.write_all(&buf)?;

        let delay = Delay::new();
        delay.delay_millis(5);

        // Read back the echo since TX and RX are on the same line
        let mut echo = [0u8; 8];
        self.uart.read_exact(&mut echo)?;

        // Verify echo matches what we sent
        if echo != buf {
            return Err(Tmc2209Error::InvalidResponse);
        }

        // Verify transmission count incremented
        let count_after = self.transmission_count()?;
        if count_after != count_before.wrapping_add(1) {
            return Err(Tmc2209Error::InvalidResponse);
        }

        delay.delay_millis(10);
        Ok(())
    }

    fn read_register(&mut self, register: u8) -> Result<FrameData> {
        // Create read request frame
        let request = ReadRequestFrame {
            address: self.address,
            register,
        };

        // Write request to buffer with extra byte for CRC
        let mut request_buf = [0u8; 4];
        let bytes = request.to_bytes()?;
        request_buf[..3].copy_from_slice(&bytes[..3]);
        request_buf[3] = crc8(&request_buf[0..3]);

        debug!(
            "Reading register {:#04x}, request: {=[u8]:#02x}",
            register, &request_buf
        );

        self.uart.write_all(&request_buf)?;
        let delay = Delay::new();
        delay.delay_millis(10);

        // Read back the echo since TX and RX are on the same line
        let mut echo = [0u8; 4];
        self.uart.read_exact(&mut echo)?;

        // Verify echo matches what we sent
        if echo != request_buf {
            return Err(Tmc2209Error::InvalidResponse);
        }

        // Wait for response
        delay.delay_millis(10);

        // Read response (8 bytes)
        let mut buf = [0u8; 8];
        self.uart.read_exact(&mut buf)?;

        debug!("Read response: {=[u8]:#02x}", &buf);

        // Verify CRC before parsing
        let received_crc = buf[7];
        let calculated_crc = crc8(&buf[0..7]);
        if received_crc != calculated_crc {
            return Err(Tmc2209Error::CrcMismatch);
        }

        // Parse response frame
        let (_rest, response) = ReadResponseFrame::from_bytes((&buf, 0))?;

        Ok(response.data)
    }

    fn get_current_scale(&mut self, rms_amps: f64) -> Result<u8> {
        let ohms = self.sense_ohms + 0.02;
        let volts = if self.low_sense_resistor_voltage()? {
            0.180
        } else {
            0.325
        };
        let scale = libm::sqrt(2.0) * 32.0 * rms_amps * ohms / volts - 1.0;
        info!("Current scale: {}", scale);
        Ok(libm::round(libm::fmax(0.0, libm::fmin(31.0, scale))) as u8)
    }

    fn current_scale_to_amps(&mut self, scale: u8) -> Result<f64> {
        let ohms = self.sense_ohms + 0.02;
        let volts = if self.low_sense_resistor_voltage()? {
            0.180
        } else {
            0.325
        };
        Ok(((scale as f64) + 1.0) * volts / (libm::sqrt(2.0) * 32.0 * ohms))
    }
}

fn crc8(data: &[u8]) -> u8 {
    // CRC-8 polynomial 0x07, init 0x00, no reflect, xorout 0x00
    // Matches TMC2209 datasheet
    let mut crc: u8 = 0x00;
    for &byte in data {
        let mut byte = byte;
        for _ in 0..8 {
            crc = if ((crc >> 7) ^ (byte & 0x01)) != 0 {
                (crc << 1) ^ 0x07
            } else {
                crc << 1
            };
            byte >>= 1;
        }
    }
    crc
}
