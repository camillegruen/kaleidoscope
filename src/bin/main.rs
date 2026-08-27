#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]


use esp_hal::{
    delay::Delay,
    gpio::DriveMode,
    ledc::{
        channel::{self, ChannelHW, ChannelIFace},
        timer::{self, TimerIFace},
        LSGlobalClkSource, Ledc, LowSpeed,
    },
    main,
    time::{Instant, Rate},
};


#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]


const DRIVE_BASE_FREQ_KHZ: u32 = 20;

fn now_ms() -> u64 {
    Instant::now().duration_since_epoch().as_millis()
}

pub struct DriveCell<'a> {
    in1: channel::Channel<'a, LowSpeed>,
    in2: channel::Channel<'a, LowSpeed>,
    state: bool,
    /// Stands in for `_ctimer[_tnum]`, which the C++ increments from a 1 ms ISR.
    ctimer_base_ms: u64,
}

impl<'a> DriveCell<'a> {
    pub fn new(in1: channel::Channel<'a, LowSpeed>, in2: channel::Channel<'a, LowSpeed>) -> Self {
        Self { in1, in2, state: false, ctimer_base_ms: now_ms() }
    }

    /// Raw 8-bit duty, matching ledcWrite() with an 8-bit resolution.
    fn write(&mut self, a: u8, b: u8) {
        let _ = self.in1.set_duty_hw(a as u32);
        let _ = self.in2.set_duty_hw(b as u32);
    }

    fn ctimer(&self) -> u32 {
        now_ms().saturating_sub(self.ctimer_base_ms) as u32
    }

    /// Flip polarity every `flip_speed_ms` at duty `power_percent`.
    /// `smooth` ramps the transition instead of hard-switching — intended for
    /// motions slower than about 2 Hz. Note that in smooth mode the C++
    /// ignores `power_percent` entirely, and so does this.
    pub fn run(&mut self, smooth: bool, power_percent: u8, flip_speed_ms: u16) {
        let period = flip_speed_ms.max(1) as u32;
        let half = (period / 2).max(1);

        let mut ct = self.ctimer();
        if ct >= period {
            self.state = !self.state;
            self.ctimer_base_ms = now_ms();
            ct = 0;
        }

        if !smooth {
            let dc = ((power_percent as u32 * 255) / 100) as u8;
            if self.state {
                self.write(0, dc);
            } else {
                self.write(dc, 0);
            }
        } else {
            // Reproduces the C++ unsigned arithmetic exactly, including the
            // wrap in the `else` branch just past the half-period, which the
            // >= 255 clamp turns into a brief full-scale spike. Faithful to
            // the original; see the note below if you'd rather smooth it out.
            let raw: u32 = if self.state {
                if ct <= half {
                    (ct * 128) / half + 128
                } else {
                    383u32.wrapping_sub((ct * 255) / period)
                }
            } else {
                if ct <= half {
                    128u32.wrapping_sub((ct * 128) / half)
                } else {
                    ((ct * 255) / period).wrapping_sub(128)
                }
            };

            let dc16 = raw as u16; // truncation to uint16_t, as in the original
            let dc = if dc16 >= 255 { 255u8 } else { dc16 as u8 };
            self.write(dc, 255 - dc);
        }
    }

    /// Buzzing sound; `us_buzz` sets the pitch (40-120 us works well).
    pub fn buzz(&mut self, us_buzz: u16, delay: &Delay) {
        delay.delay_micros(us_buzz as u32);
        self.state = !self.state;
        if self.state {
            self.write(0, 255);
        } else {
            self.write(255, 0);
        }
    }

    /// For motors: direction + speed. For coils: polarity + field strength.
    pub fn drive(&mut self, direction: bool, power_percent: u8) {
        let dc = ((power_percent as u32 * 255) / 100) as u8;
        if direction {
            self.write(0, dc);
        } else {
            self.write(dc, 0);
        }
    }

    /// Short full-power pulse, then release that side.
    pub fn pulse(&mut self, direction: bool, ms_duration: u8, delay: &Delay) {
        if direction {
            self.write(0, 255);
            delay.delay_millis(ms_duration as u32);
            let _ = self.in2.set_duty_hw(0);
        } else {
            self.write(255, 0);
            delay.delay_millis(ms_duration as u32);
            let _ = self.in1.set_duty_hw(0);
        }
    }

    /// The startup jingle: 1000 buzzes at a fixed pitch, then a 3000-step sweep.
    pub fn tone(&mut self, delay: &Delay) {
        for _ in 0..1000u16 {
            self.buzz(100, delay);
        }
        for d in 0..3000u16 {
            self.buzz(30 + d / 50, delay);
        }
    }

    /// Flip polarity once, at the given duty.
    pub fn toggle(&mut self, power_percent: u8) {
        let dc = ((power_percent as u32 * 255) / 100) as u8;
        self.state = !self.state;
        if self.state {
            self.write(0, dc);
        } else {
            self.write(dc, 0);
        }
    }
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty8Bit, // matches the 0-255 duty values
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(DRIVE_BASE_FREQ_KHZ),
        })
        .unwrap();

    let cfg = || channel::config::Config {
        timer: &lstimer0,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    };

    // IN1_pin1 = 2, IN1_pin2 = 3, IN2_pin1 = 5, IN2_pin2 = 6
    let mut ch0 = ledc.channel(channel::Number::Channel0, peripherals.GPIO2);
    let mut ch1 = ledc.channel(channel::Number::Channel1, peripherals.GPIO3);
    let mut ch2 = ledc.channel(channel::Number::Channel2, peripherals.GPIO5);
    let mut ch3 = ledc.channel(channel::Number::Channel3, peripherals.GPIO6);
    ch0.configure(cfg()).unwrap();
    ch1.configure(cfg()).unwrap();
    ch2.configure(cfg()).unwrap();
    ch3.configure(cfg()).unwrap();

    let mut flat_flap1 = DriveCell::new(ch0, ch1);
    let mut flat_flap2 = DriveCell::new(ch2, ch3);

    flat_flap1.tone(&delay);
    flat_flap2.tone(&delay);

    let mut flap_counter: u16 = 0;

    loop {
        delay.delay_millis(1);
        flap_counter = flap_counter.wrapping_add(1);

        if flap_counter < 2000 {
            flat_flap1.run(false, 100, 100);
            flat_flap2.run(false, 100, 100);
        } else if flap_counter < 8000 {
            flat_flap1.run(true, 100, 1000);
            flat_flap2.run(true, 100, 1000);
        } else {
            flap_counter = 0;

            let poses: [(bool, bool); 6] = [
                (false, true),
                (true, true),
                (true, false),
                (true, true),
                (false, false),
                (true, true),
            ];
            for (d1, d2) in poses {
                flat_flap1.drive(d1, 100);
                flat_flap2.drive(d2, 100);
                delay.delay_millis(500);
            }

            flat_flap1.tone(&delay);
            flat_flap2.tone(&delay);
        }
    }
}