//! The VM's gpio interrupt on the ESP32-C3, see `virtmach::interrupts::gpio`
//! for the functions. Pins are the chip's GPIO numbers, port p holds the pins
//! p * VMAtom::BITS up to p * VMAtom::BITS + VMAtom::BITS - 1.
//!
//! Only the pins the firmware doesn't use itself are open to programs, see
//! `ALLOWED`. Any other pin, an invalid mode or pull and a port without any
//! of them set RuntimeError::InterruptError; the masked functions skip the
//! mask bits of pins that aren't allowed. Outputs are set up with the input
//! enabled too, so read returns the level on the pad for them as well.

use esp_idf_svc::sys::{
    gpio_get_level, gpio_mode_t, gpio_mode_t_GPIO_MODE_INPUT, gpio_mode_t_GPIO_MODE_INPUT_OUTPUT,
    gpio_mode_t_GPIO_MODE_INPUT_OUTPUT_OD, gpio_pull_mode_t_GPIO_FLOATING, gpio_pull_mode_t_GPIO_PULLDOWN_ONLY,
    gpio_pull_mode_t_GPIO_PULLUP_ONLY, gpio_reset_pin, gpio_set_direction, gpio_set_level, gpio_set_pull_mode,
};
use virtmach::{interrupts::{gpio, SoftInterrupt}, RuntimeError, Storage, VMAtom, VirtMach};

/// GPIO0, 4-7 and 10. Taken: 1-3 by the I2S LED chain (`iled`), 8/9 by the
/// OLED's I2C, 11-17 by the SPI flash, 18/19 by USB and 20/21 by UART0.
const ALLOWED: u32 = 1 << 0 | 1 << 4 | 1 << 5 | 1 << 6 | 1 << 7 | 1 << 10;

#[derive(Default)]
pub struct Gpio {
    /// Pins that were switched to the GPIO function by a setup.
    configured: u32,
    /// The output level last written to each pin, for toggle.
    levels: u32,
}

impl Gpio {
    fn allowed(pin: VMAtom) -> bool {
        (0..32).contains(&pin) && ALLOWED & (1 << pin) != 0
    }

    /// The allowed pins of a port as (first pin, bits), None if it has none.
    fn port(port: VMAtom) -> Option<(u32, u32)> {
        if port < 0 {
            return None;
        }
        let first = port as u32 * VMAtom::BITS;
        let pins = if first < 32 { ALLOWED >> first } else { 0 };
        let pins = if VMAtom::BITS < 32 { pins & ((1 << VMAtom::BITS) - 1) } else { pins };
        if pins == 0 { None } else { Some((first, pins)) }
    }

    fn setup(&mut self, pin: u32, mode: gpio_mode_t) {
        if self.configured & (1 << pin) == 0 {
            // selects the GPIO function, but also enables the pull-up - the
            // pull is set_pull's business, so it starts out as none
            unsafe { gpio_reset_pin(pin as i32); }
            unsafe { gpio_set_pull_mode(pin as i32, gpio_pull_mode_t_GPIO_FLOATING); }
            self.configured |= 1 << pin;
        }
        unsafe { gpio_set_direction(pin as i32, mode); }
    }

    fn write(&mut self, pin: u32, high: bool) {
        if high { self.levels |= 1 << pin; } else { self.levels &= !(1 << pin); }
        unsafe { gpio_set_level(pin as i32, high as u32); }
    }

    fn read(pin: u32) -> bool {
        unsafe { gpio_get_level(pin as i32) != 0 }
    }
}

impl<S: Storage> SoftInterrupt<S> for Gpio {
    fn name(&self) -> &str {
        return gpio::NAME;
    }

    fn call(&mut self, vm: &mut VirtMach<S>) {
        let op = vm.stack_pop();
        match op {
            0..=6 => {
                let pin = vm.stack_pop();
                let value = if op <= 2 { vm.stack_pop() } else { 0 };
                let valid = Self::allowed(pin) && match op {
                    0 | 1 => (0..=2).contains(&value),
                    _ => true,
                };
                if !valid {
                    if op == 6 { vm.stack_push(0); }
                    vm.error = RuntimeError::InterruptError;
                    return;
                }
                let pin = pin as u32;
                match op {
                    0 => self.setup(pin, match value {
                        0 => gpio_mode_t_GPIO_MODE_INPUT,
                        1 => gpio_mode_t_GPIO_MODE_INPUT_OUTPUT,
                        _ => gpio_mode_t_GPIO_MODE_INPUT_OUTPUT_OD,
                    }),
                    1 => unsafe {
                        gpio_set_pull_mode(pin as i32, match value {
                            0 => gpio_pull_mode_t_GPIO_FLOATING,
                            1 => gpio_pull_mode_t_GPIO_PULLUP_ONLY,
                            _ => gpio_pull_mode_t_GPIO_PULLDOWN_ONLY,
                        });
                    },
                    2 => self.write(pin, value != 0),
                    3 => self.write(pin, true),
                    4 => self.write(pin, false),
                    5 => self.write(pin, self.levels & (1 << pin) == 0),
                    _ => vm.stack_push(Self::read(pin) as VMAtom),
                }
            }
            10 | 11 => {
                let (port, mask) = (vm.stack_pop(), vm.stack_pop());
                let values = if op == 10 { vm.stack_pop() } else { 0 };
                let Some((first, pins)) = Self::port(port) else {
                    if op == 11 { vm.stack_push(0); }
                    vm.error = RuntimeError::InterruptError;
                    return;
                };
                // the mask as unsigned bits of the atom, without sign extension
                let bits = mask as u32 & pins & if VMAtom::BITS < 32 { (1 << VMAtom::BITS) - 1 } else { u32::MAX };
                let mut read: u32 = 0;
                for bit in (0..VMAtom::BITS).filter(|bit| bits & (1 << bit) != 0) {
                    if op == 10 {
                        self.write(first + bit, values as u32 & (1 << bit) != 0);
                    } else if Self::read(first + bit) {
                        read |= 1 << bit;
                    }
                }
                if op == 11 { vm.stack_push(read as VMAtom); }
            }
            _ => { vm.error = RuntimeError::UnimplementedInterruptFunc; }
        }
    }
}
