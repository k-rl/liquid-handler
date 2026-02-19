use alloc::vec::Vec;
use deku::prelude::*;

// Pump codes..
pub const INIT: u8 = 0x00;
pub const GET_AIR_IN_LINE: u8 = 0x01;
pub const GET_FLOW_RATE: u8 = 0x02;
pub const SET_FLOW_UL_PER_MIN: u8 = 0x03;
pub const SET_PUMP_RPM: u8 = 0x04;
pub const GET_PUMP_RPM: u8 = 0x05;
pub const GET_RMS_AMPS: u8 = 0x06;
pub const SET_RMS_AMPS: u8 = 0x07;
pub const GET_STOP_RMS_AMPS: u8 = 0x08;
pub const SET_STOP_RMS_AMPS: u8 = 0x09;
pub const GET_MOTOR_LOAD: u8 = 0x0A;
pub const GET_FLOW_HISTORY: u8 = 0x0B;
pub const GET_VALVE: u8 = 0x0C;
pub const SET_VALVE: u8 = 0x0D;
pub const GET_FLUSH_TIME: u8 = 0x0E;
pub const SET_FLUSH_TIME: u8 = 0x0F;
pub const GET_FLUSH_RPM: u8 = 0x10;
pub const SET_FLUSH_RPM: u8 = 0x11;
pub const GET_FLUSHING: u8 = 0x12;

// CNC codes.
pub const HOME: u8 = 0x80;
pub const IS_HOMING: u8 = 0x81;
pub const SET_POS: u8 = 0x82;
pub const GET_POS: u8 = 0x83;
pub const SET_SPEED: u8 = 0x84;
pub const GET_SPEED: u8 = 0x85;
pub const SET_ACCEL: u8 = 0x86;
pub const GET_ACCEL: u8 = 0x87;

// Shared codes.
pub const HEARTBEAT: u8 = 0xFE;
pub const FAIL: u8 = 0xFF;

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(id_type = "u8", endian = "big")]
pub enum Request {
    // Pump variants.
    #[deku(id = "INIT")]
    Init,

    #[deku(id = "GET_AIR_IN_LINE")]
    GetAirInLine,

    #[deku(id = "GET_FLOW_RATE")]
    GetFlowRate,

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

    // CNC variants.
    #[deku(id = "HOME")]
    Home,

    #[deku(id = "IS_HOMING")]
    IsHoming,

    #[deku(id = "SET_POS")]
    SetPos(i64, i64, i64),

    #[deku(id = "GET_POS")]
    GetPos,

    #[deku(id = "SET_SPEED")]
    SetSpeed(f64),

    #[deku(id = "GET_SPEED")]
    GetSpeed,

    #[deku(id = "SET_ACCEL")]
    SetAccel(f64),

    #[deku(id = "GET_ACCEL")]
    GetAccel,
}

#[derive(Debug, Clone, DekuRead, DekuWrite)]
#[deku(id_type = "u8", endian = "big")]
pub enum Response {
    // Pump variants.
    #[deku(id = "INIT")]
    Init(u8),

    #[deku(id = "GET_AIR_IN_LINE")]
    GetAirInLine(bool),

    #[deku(id = "GET_FLOW_RATE")]
    GetFlowRate(f64),

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

    // CNC variants.
    #[deku(id = "HOME")]
    Home,

    #[deku(id = "IS_HOMING")]
    IsHoming(bool),

    #[deku(id = "SET_POS")]
    SetPos,

    #[deku(id = "GET_POS")]
    GetPos(i64, i64, i64),

    #[deku(id = "SET_SPEED")]
    SetSpeed,

    #[deku(id = "GET_SPEED")]
    GetSpeed(f64),

    #[deku(id = "SET_ACCEL")]
    SetAccel,

    #[deku(id = "GET_ACCEL")]
    GetAccel(f64),

    // Shared.
    #[deku(id = "FAIL")]
    Fail,
}
