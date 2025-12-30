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

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_hal::{self, clock::CpuClock, timer::timg::TimerGroup};
use panic_rtt_target as _;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 73744);
    esp_rtos::start(TimerGroup::new(peripherals.TIMG0).timer0);
    info!("Embassy initialized!");

    // Initialize TMC2209 stepper driver
    let mut tmc = tmc2209::Tmc2209::new(
        peripherals.UART1,
        peripherals.GPIO17, // TX
        peripherals.GPIO18, // RX
        peripherals.GPIO4,  // STEP
        peripherals.GPIO5,  // DIR
        peripherals.GPIO6,  // ENABLE
    );
    tmc.enable();
    info!("TMC2209 initialized");

    let mut sensor =
        flow_sensor::FlowSensor::new(peripherals.I2C0, peripherals.GPIO2, peripherals.GPIO1);
    info!("Flow sensor initialized");

    spawner.spawn(server()).unwrap();
    Timer::after_secs(4).await;

    usb::protocol_task(peripherals.USB_DEVICE, sensor).await;
}

#[embassy_executor::task]
async fn server() -> ! {
    loop {
        info!("Hello from second task!");
        Timer::after_millis(500).await;
    }
}
