//! The pwm part of the VM's periph interrupt, see `virtmach::interrupts::periph`
//! (pwm_*); `call` gets the function number counted from `periph::PWM`.
//!
//! Outputs are the LEDC channels in low speed mode, one per pin of
//! `int_gpio::ALLOWED`. They all share timer 0, so setting the frequency of
//! one pin changes it for the others; their duty cycles are kept as fractions
//! and applied again for the new resolution. The resolution is the highest the
//! frequency allows, up to 14 bits.

use esp_idf_svc::sys::{
    ledc_channel_config, ledc_channel_config_t, soc_periph_ledc_clk_src_legacy_t_LEDC_AUTO_CLK, ledc_get_freq,
    ledc_mode_t_LEDC_LOW_SPEED_MODE, ledc_set_duty, ledc_stop, ledc_timer_config, ledc_timer_config_t,
    ledc_timer_t_LEDC_TIMER_0, ledc_update_duty, ESP_OK,
};
use virtmach::{RuntimeError, Storage, VMAtom, VirtMach};

use crate::int_gpio::Gpio;

const CHANNELS: usize = 6;
const MODE: u32 = ledc_mode_t_LEDC_LOW_SPEED_MODE;
const TIMER_CLOCK: u64 = 80_000_000;
const MAX_BITS: u32 = 14;
/// Duty fractions are kept in 1/FULL
const FULL: u64 = 1 << 16;

#[derive(Default)]
pub struct Pwm {
    /// The pin of each LEDC channel in use.
    pins: [Option<u32>; CHANNELS],
    /// The duty of each channel as a fraction of FULL.
    duty: [u64; CHANNELS],
    frequency: u32,
    bits: u32,
}

impl Pwm {
    fn channel(&self, pin: u32) -> Option<usize> {
        self.pins.iter().position(|p| *p == Some(pin))
    }

    fn apply(&self, channel: usize) {
        let duty = (self.duty[channel] * (1 << self.bits) / FULL) as u32;
        unsafe {
            ledc_set_duty(MODE, channel as u32, duty);
            ledc_update_duty(MODE, channel as u32);
        }
    }

    /// Runs the timer at the frequency, returns what it reached or 0.
    fn set_frequency(&mut self, frequency: u32) -> u32 {
        let mut bits = (TIMER_CLOCK / frequency as u64).max(1).ilog2().min(MAX_BITS);
        while bits >= 1 {
            let mut config: ledc_timer_config_t = unsafe { core::mem::zeroed() };
            config.speed_mode = MODE;
            config.duty_resolution = bits;
            config.timer_num = ledc_timer_t_LEDC_TIMER_0;
            config.freq_hz = frequency;
            config.clk_cfg = soc_periph_ledc_clk_src_legacy_t_LEDC_AUTO_CLK;
            if unsafe { ledc_timer_config(&config) } == ESP_OK {
                self.bits = bits;
                self.frequency = unsafe { ledc_get_freq(MODE, ledc_timer_t_LEDC_TIMER_0) };
                for channel in (0..CHANNELS).filter(|c| self.pins[*c].is_some()) {
                    self.apply(channel);
                }
                return self.frequency;
            }
            bits -= 1;
        }
        0
    }

    pub fn call<S: Storage>(&mut self, vm: &mut VirtMach<S>, op: VMAtom) {
        let pin = vm.stack_pop();
        let returns = (op == 0) as usize;
        let valid = Gpio::allowed(pin) && matches!(op, 0..=3);
        if !valid {
            // the arguments of an unknown function are unknown too
            if matches!(op, 0..=3) {
                for _ in 0..[1, 2, 1, 0][op as usize] { vm.stack_pop(); }
            }
            for _ in 0..returns { vm.stack_push(0); }
            vm.error = if matches!(op, 0..=3) { RuntimeError::InterruptError } else { RuntimeError::UnimplementedInterruptFunc };
            return;
        }
        let pin = pin as u32;
        match op {
            0 => {
                let frequency = vm.stack_pop();
                if frequency <= 0 {
                    vm.stack_push(0);
                    vm.error = RuntimeError::InterruptError;
                    return;
                }
                let reached = self.set_frequency(frequency as u32);
                if reached == 0 {
                    vm.stack_push(0);
                    return;
                }
                let Some(channel) = self.channel(pin).or_else(|| self.pins.iter().position(Option::is_none)) else {
                    vm.stack_push(0);
                    vm.error = RuntimeError::InterruptError;
                    return;
                };
                let first = self.pins[channel].is_none();
                if first { self.duty[channel] = 0; }
                let mut config: ledc_channel_config_t = unsafe { core::mem::zeroed() };
                config.gpio_num = pin as i32;
                config.speed_mode = MODE;
                config.channel = channel as u32;
                config.timer_sel = ledc_timer_t_LEDC_TIMER_0;
                config.duty = (self.duty[channel] * (1 << self.bits) / FULL) as u32;
                if unsafe { ledc_channel_config(&config) } == ESP_OK {
                    self.pins[channel] = Some(pin);
                    vm.stack_push(reached.min(VMAtom::MAX as u32) as VMAtom);
                } else {
                    vm.stack_push(0);
                }
            }
            1 | 2 => {
                let value = vm.stack_pop();
                let max = if op == 1 { vm.stack_pop() } else { 0 };
                let channel = self.channel(pin);
                let (Some(channel), true) = (channel, value >= 0 && (op == 2 || max > 0)) else {
                    vm.error = RuntimeError::InterruptError;
                    return;
                };
                self.duty[channel] = if op == 1 {
                    (value.min(max) as u64 * FULL) / max as u64
                } else {
                    // microseconds of the period 1/frequency
                    (value as u64 * self.frequency as u64 * FULL / 1_000_000).min(FULL)
                };
                self.apply(channel);
            }
            _ => match self.channel(pin) {
                Some(channel) => {
                    unsafe { ledc_stop(MODE, channel as u32, 0); }
                    self.pins[channel] = None;
                }
                None => vm.error = RuntimeError::InterruptError,
            },
        }
    }
}
