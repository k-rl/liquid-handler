#![allow(dead_code)]

use esp_hal::{
    gpio::{Input, InputConfig, InputPin, Level::Low, Output, OutputConfig, OutputPin},
    time::Instant,
};

pub struct Stepper<'a> {
    pins: [Output<'a>; 4],
    limit_switch: Input<'a>,
    v: f64,
    a: f64,
    elapsed_us: u64,
    last_time: Instant,
    pos: i64,
    target_pos: i64,
    max_speed: f64,
    phase: u8,
}

impl<'a> Stepper<'a> {
    pub fn new(
        pin0: impl OutputPin + 'a,
        pin1: impl OutputPin + 'a,
        pin2: impl OutputPin + 'a,
        pin3: impl OutputPin + 'a,
        limit_switch: impl InputPin + 'a,
    ) -> Self {
        let pins = [
            Output::new(pin0, Low, OutputConfig::default()),
            Output::new(pin1, Low, OutputConfig::default()),
            Output::new(pin2, Low, OutputConfig::default()),
            Output::new(pin3, Low, OutputConfig::default()),
        ];
        Self {
            pins,
            limit_switch: Input::new(limit_switch, InputConfig::default()),
            v: 0.0,
            a: 0.0,
            elapsed_us: 0,
            last_time: Instant::now(),
            pos: 0,
            target_pos: 0,
            max_speed: 0.0,
            phase: 0,
        }
    }

    pub fn step(&mut self) {
        let now = Instant::now();
        let delta_us = (now - self.last_time).as_micros();
        self.last_time = now;

        if self.pos == self.target_pos {
            self.v = 0.0;
            return;
        }

        let dist = (self.target_pos - self.pos) as f64;
        let a = if dist * self.v <= 0.0 || dist.abs() > self.v * self.v / (2.0 * self.a) {
            dist.signum() * self.a
        } else {
            -dist.signum() * self.a
        };

        self.v += a * (delta_us as f64 / 1e6);
        if self.v > self.max_speed {
            self.v = self.max_speed;
        } else if self.v < -self.max_speed {
            self.v = -self.max_speed;
        }

        let dt = (1e6 / (self.v.abs() + 1e-10)) as u64;
        self.elapsed_us += delta_us;
        if self.elapsed_us >= dt {
            self.elapsed_us -= dt;

            if self.v >= 0.0 {
                self.phase = (self.phase + 1) % 8;
                self.pos += 1;
            } else {
                self.phase = (self.phase + 7) % 8;
                self.pos -= 1;
            }

            self.set_phase();
        }
    }

    fn set_phase(&mut self) {
        let phase = self.phase as usize;
        for i in 0..4 {
            if i == phase / 2 || (phase % 2 == 1 && i == (phase / 2 + 1) % 4) {
                self.pins[i].set_high();
            } else {
                self.pins[i].set_low();
            }
        }
    }

    pub fn home(&mut self) {
        self.pos = 0;
        while self.limit_switch.is_low() {
            self.phase = (self.phase + 7) % 8;
            self.set_phase();
            let start = Instant::now();
            while (Instant::now() - start).as_micros() < 1000 {}
        }
        self.pos = 0;
        self.v = 0.0;
        self.elapsed_us = 0;
        self.last_time = Instant::now();
    }

    pub fn pos(&self) -> i64 {
        self.pos
    }

    pub fn set_max_speed(&mut self, speed: f64) {
        self.max_speed = speed;
    }

    pub fn set_target_pos(&mut self, pos: i64) {
        self.target_pos = pos;
    }

    pub fn set_accel(&mut self, accel: f64) {
        self.a = accel;
    }
}
