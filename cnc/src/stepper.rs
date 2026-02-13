#![allow(dead_code)]

use esp_hal::{
    delay::Delay,
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
    phase: i8,
    backlash: i64,
    backlash_remaining: i64,
    forward: bool,
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
            a: 1000.0,
            elapsed_us: 0,
            last_time: Instant::now(),
            pos: 0,
            target_pos: 0,
            max_speed: 1000.0,
            phase: 0,
            backlash: 0,
            backlash_remaining: 0,
            forward: true,
        }
    }

    pub fn step(&mut self) {
        let now = Instant::now();
        let delta_us = (now - self.last_time).as_micros();
        self.last_time = now;

        if self.pos == self.target_pos {
            self.v = 0.0;
            self.backlash_remaining = 0;
            return;
        }

        // Accelerate toward target, or decelerate if we'd overshoot.
        let dist = (self.target_pos - self.pos) as f64;
        let effective_dist = dist.abs() + self.backlash_remaining as f64;
        let a = if dist * self.v <= 0.0 || effective_dist > self.v * self.v / (2.0 * self.a) {
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

        // Emit a step when enough time has passed for the current velocity.
        let dt = (1e6 / (self.v.abs() + 1e-10)) as u64;
        self.elapsed_us += delta_us;
        if self.elapsed_us >= dt {
            self.elapsed_us -= dt;

            // Compensate for backlash on direction change.
            if (self.v > 0.0) ^ self.forward {
                self.backlash_remaining = self.backlash - self.backlash_remaining;
                self.forward = !self.forward;
            }

            self.step_motor(self.forward);
            if self.backlash_remaining > 0 {
                self.backlash_remaining -= 1;
            } else if self.forward {
                self.pos += 1;
            } else {
                self.pos -= 1;
            }
        }
    }

    fn step_motor(&mut self, forward: bool) {
        if forward {
            self.phase = (self.phase + 1) % 8;
        } else {
            self.phase = (self.phase - 1).rem_euclid(8);
        }

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
        // Drive into limit switch.
        while self.limit_switch.is_low() {
            self.step_motor(false);
            Delay::new().delay_millis(1);
        }

        // Measure backlash: 5 cycles of back-off then re-approach.
        let mut total: i64 = 0;
        for _ in 0..5 {
            // Back off until switch releases.
            let mut count: i64 = 0;
            while self.limit_switch.is_high() {
                self.step_motor(true);
                Delay::new().delay_millis(1);
                count += 1;
            }

            // Re-approach until switch triggers.
            while self.limit_switch.is_low() {
                self.step_motor(false);
                Delay::new().delay_millis(1);
            }
            total += count;
        }
        // Round instead of truncate.
        self.backlash = (total + 2) / 5;

        self.pos = 0;
        self.v = 0.0;
        self.elapsed_us = 0;
        self.last_time = Instant::now();
        self.forward = false;
        self.backlash_remaining = 0;
    }

    pub fn pos(&self) -> i64 {
        self.pos
    }

    pub fn max_speed(&self) -> f64 {
        self.max_speed
    }

    pub fn set_max_speed(&mut self, speed: f64) {
        self.max_speed = speed;
    }

    pub fn set_target_pos(&mut self, pos: i64) {
        self.target_pos = pos;
    }

    pub fn accel(&self) -> f64 {
        self.a
    }

    pub fn set_accel(&mut self, accel: f64) {
        self.a = accel;
    }
}
