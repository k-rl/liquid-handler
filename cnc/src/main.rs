#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

mod stepper;

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

// Request/response codes.
const INIT: u8 = 0x00;
const HOME: u8 = 0x01;
const SET_POS: u8 = 0x02;
const GET_POS: u8 = 0x03;
const FAIL: u8 = 0xFF;

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(id_type = "u8", endian = "big")]
enum Request {
    #[deku(id = "INIT")]
    Init,

    #[deku(id = "HOME")]
    Home,

    #[deku(id = "SET_POS")]
    SetPos(i64, i64, i64),

    #[deku(id = "GET_POS")]
    GetPos,
}

#[derive(Debug, Clone, DekuRead, DekuWrite)]
#[deku(id_type = "u8", endian = "big")]
enum Response {
    #[deku(id = "INIT")]
    Init(u8),

    #[deku(id = "HOME")]
    Home,

    #[deku(id = "SET_POS")]
    SetPos,

    #[deku(id = "GET_POS")]
    GetPos(i64, i64, i64),

    #[deku(id = "FAIL")]
    Fail,
}

esp_bootloader_esp_idf::esp_app_desc!();

static TARGET_POS: Mutex<(i64, i64, i64)> = Mutex::new((0, 0, 0));
static POS: Mutex<(i64, i64, i64)> = Mutex::new((0, 0, 0));
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
    let x_motor = Stepper::new(
        peripherals.GPIO7, peripherals.GPIO6,
        peripherals.GPIO5, peripherals.GPIO2,
        peripherals.GPIO21,
    );
    let y_motor = Stepper::new(
        peripherals.GPIO3, peripherals.GPIO4,
        peripherals.GPIO11, peripherals.GPIO12,
        peripherals.GPIO38,
    );
    let z_motor = Stepper::new(
        peripherals.GPIO17, peripherals.GPIO10,
        peripherals.GPIO9, peripherals.GPIO8,
        peripherals.GPIO18,
    );

    let irc = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    static mut STACK: Stack<8192> = Stack::new();
    #[allow(static_mut_refs)]
    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        irc.software_interrupt0,
        irc.software_interrupt1,
        unsafe { &mut STACK },
        move || run_steppers(x_motor, y_motor, z_motor, timers.timer1),
    );

    run_coordinator(peripherals.USB_DEVICE).await;
}

async fn run_coordinator<'a>(device: USB_DEVICE<'a>) -> ! {
    let mut stream = PacketStream::new(device, Duration::from_secs(1));
    loop {
        let Ok(packet) = stream.read().await else {
            continue;
        };

        let response = match Request::from_bytes((&packet, 0)) {
            Ok((_, packet)) => handle_request(packet),
            Err(_) => Response::Fail,
        };

        let response_bytes = response.to_bytes().unwrap();
        stream.write(&response_bytes).await;
    }
}

fn handle_request(packet: Request) -> Response {
    match packet {
        Request::Init => Response::Init(1),
        Request::Home => {
            HOME_MODE.set(true);
            Response::Home
        }
        Request::SetPos(x, y, z) => {
            TARGET_POS.set((x, y, z));
            Response::SetPos
        }
        Request::GetPos => {
            let (x, y, z) = POS.get_cloned();
            Response::GetPos(x, y, z)
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
    mut x_motor: Stepper<'static>,
    mut y_motor: Stepper<'static>,
    mut z_motor: Stepper<'static>,
    timer: impl esp_hal::timer::Timer + Into<AnyTimer<'static>>,
) -> ! {
    let step_us = 10;
    let mut step_timer = PeriodicTimer::new(timer);
    step_timer
        .start(time::Duration::from_micros(step_us))
        .unwrap();

    x_motor.set_max_speed(1000.0);
    x_motor.set_accel(1000.0);
    y_motor.set_max_speed(1000.0);
    y_motor.set_accel(1000.0);
    z_motor.set_max_speed(1000.0);
    z_motor.set_accel(1000.0);

    loop {
        if HOME_MODE.get_cloned() {
            z_motor.home();
            x_motor.home();
            y_motor.home();
            TARGET_POS.set((0, 0, 0));
            POS.set((0, 0, 0));
            HOME_MODE.set(false);
        }

        let (tx, ty, tz) = TARGET_POS.get_cloned();
        x_motor.set_target_pos(tx);
        y_motor.set_target_pos(ty);
        z_motor.set_target_pos(tz);

        x_motor.step();
        y_motor.step();
        z_motor.step();

        POS.set((
            x_motor.pos(),
            y_motor.pos(),
            z_motor.pos(),
        ));

        step_timer.wait();
    }
}
