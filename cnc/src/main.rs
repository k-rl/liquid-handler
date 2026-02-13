#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

mod stepper;

use alloc::sync::Arc;
use crate::stepper::Stepper;
use common::{mutex::Mutex, usb::PacketStream};
use deku::prelude::*;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::{
    self,
    clock::CpuClock,
    interrupt::software::SoftwareInterruptControl,
    peripherals::{TIMG0, USB_DEVICE},
    system::Stack,
    time,
    timer::{
        timg::{MwdtStage, TimerGroup, Wdt},
        AnyTimer, PeriodicTimer,
    },
};

use panic_halt as _;

extern crate alloc;

type Motor = Arc<Mutex<Stepper<'static>>>;

// Request/response codes.
const INIT: u8 = 0x00;
const HOME: u8 = 0x01;
const IS_HOMING: u8 = 0x02;
const SET_POS: u8 = 0x03;
const GET_POS: u8 = 0x04;
const SET_SPEED: u8 = 0x05;
const GET_SPEED: u8 = 0x06;
const SET_ACCEL: u8 = 0x07;
const GET_ACCEL: u8 = 0x08;
const FAIL: u8 = 0xFF;

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(id_type = "u8", endian = "big")]
enum Request {
    #[deku(id = "INIT")]
    Init,

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
enum Response {
    #[deku(id = "INIT")]
    Init(u8),

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

    #[deku(id = "FAIL")]
    Fail,
}

esp_bootloader_esp_idf::esp_app_desc!();

static HOME_MODE: Mutex<bool> = Mutex::new(true);

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 73744);
    let timers = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timers.timer0);

    Timer::after_secs(2).await;

    let mut watchdog = timers.wdt;
    watchdog.set_timeout(MwdtStage::Stage3, time::Duration::from_secs(1));
    watchdog.enable();
    spawner.spawn(run_watchdog_monitor(watchdog)).unwrap();

    // Pin mapping: Arduino Nano ESP32 -> ESP32-S3 GPIO
    // X: D4=GPIO7, D3=GPIO6, D2=GPIO5, A1=GPIO2
    // Y: A2=GPIO3, A3=GPIO4, A4=GPIO11, A5=GPIO12
    // Z: D8=GPIO17, D7=GPIO10, D6=GPIO9, D5=GPIO8
    // Limit switch pins: X=D10(GPIO21), Y=D11(GPIO38), Z=D9(GPIO18)
    let x_motor: Motor = Arc::new(Mutex::new(Stepper::new(
        peripherals.GPIO7, peripherals.GPIO6,
        peripherals.GPIO5, peripherals.GPIO2,
        peripherals.GPIO21,
    )));
    let y_motor: Motor = Arc::new(Mutex::new(Stepper::new(
        peripherals.GPIO3, peripherals.GPIO4,
        peripherals.GPIO11, peripherals.GPIO12,
        peripherals.GPIO38,
    )));
    let z_motor: Motor = Arc::new(Mutex::new(Stepper::new(
        peripherals.GPIO17, peripherals.GPIO10,
        peripherals.GPIO9, peripherals.GPIO8,
        peripherals.GPIO18,
    )));

    let x_clone = x_motor.clone();
    let y_clone = y_motor.clone();
    let z_clone = z_motor.clone();

    let irc = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    static mut STACK: Stack<8192> = Stack::new();
    #[allow(static_mut_refs)]
    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        irc.software_interrupt0,
        irc.software_interrupt1,
        unsafe { &mut STACK },
        move || run_steppers(x_clone, y_clone, z_clone, timers.timer1),
    );

    run_coordinator(peripherals.USB_DEVICE, x_motor, y_motor, z_motor).await;
}

async fn run_coordinator<'a>(device: USB_DEVICE<'a>, x: Motor, y: Motor, z: Motor) -> ! {
    let mut stream = PacketStream::new(device, Duration::from_secs(1));
    loop {
        let Ok(packet) = stream.read().await else {
            continue;
        };

        let response = match Request::from_bytes((&packet, 0)) {
            Ok((_, packet)) => handle_request(packet, &x, &y, &z),
            Err(_) => Response::Fail,
        };

        let response_bytes = response.to_bytes().unwrap();
        stream.write(&response_bytes).await;
    }
}

fn handle_request(packet: Request, x: &Motor, y: &Motor, z: &Motor) -> Response {
    match packet {
        Request::Init => Response::Init(1),
        Request::Home => {
            HOME_MODE.set(true);
            Response::Home
        }
        Request::SetPos(px, py, pz) => {
            x.lock(|m| m.set_target_pos(px));
            y.lock(|m| m.set_target_pos(py));
            z.lock(|m| m.set_target_pos(pz));
            Response::SetPos
        }
        Request::GetPos => {
            let px = x.lock(|m| m.pos());
            let py = y.lock(|m| m.pos());
            let pz = z.lock(|m| m.pos());
            Response::GetPos(px, py, pz)
        }
        Request::SetSpeed(s) => {
            x.lock(|m| m.set_max_speed(s));
            y.lock(|m| m.set_max_speed(s));
            z.lock(|m| m.set_max_speed(s));
            Response::SetSpeed
        }
        Request::GetSpeed => {
            Response::GetSpeed(x.lock(|m| m.max_speed()))
        }
        Request::SetAccel(a) => {
            x.lock(|m| m.set_accel(a));
            y.lock(|m| m.set_accel(a));
            z.lock(|m| m.set_accel(a));
            Response::SetAccel
        }
        Request::GetAccel => {
            Response::GetAccel(x.lock(|m| m.accel()))
        }
        Request::IsHoming => {
            Response::IsHoming(HOME_MODE.get_cloned())
        }
    }
}

#[embassy_executor::task]
async fn run_watchdog_monitor(mut watchdog: Wdt<TIMG0<'static>>) -> ! {
    loop {
        watchdog.feed();
        Timer::after_millis(100).await;
    }
}

fn run_steppers(
    x: Motor,
    y: Motor,
    z: Motor,
    timer: impl esp_hal::timer::Timer + Into<AnyTimer<'static>>,
) -> ! {
    let step_us = 10;
    let mut step_timer = PeriodicTimer::new(timer);
    step_timer
        .start(time::Duration::from_micros(step_us))
        .unwrap();

    loop {
        if HOME_MODE.get_cloned() {
            z.lock(|m| m.home());
            x.lock(|m| m.home());
            y.lock(|m| m.home());
            HOME_MODE.set(false);
        }

        x.lock(|m| m.step());
        y.lock(|m| m.step());
        z.lock(|m| m.step());

        step_timer.wait();
    }
}
