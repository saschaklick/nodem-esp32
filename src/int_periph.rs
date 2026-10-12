//! The VM's periph interrupt on the ESP32-C3, see `virtmach::interrupts::periph`
//! for the functions. Each function goes to the peripheral it belongs to,
//! numbered from the first function of that peripheral.

use virtmach::{interrupts::{periph, SoftInterrupt}, RuntimeError, Storage, VirtMach};

use crate::int_gpio::Gpio;
use crate::int_i2c::I2c;
use crate::int_pwm::Pwm;
use crate::int_uart::Uart;

#[derive(Default)]
pub struct Periph {
    gpio: Gpio,
    pwm: Pwm,
    uart: Uart,
    i2c: I2c,
}

impl<S: Storage> SoftInterrupt<S> for Periph {
    fn name(&self) -> &str {
        return periph::NAME;
    }

    fn call(&mut self, vm: &mut VirtMach<S>) {
        let op = vm.stack_pop();
        match op {
            periph::GPIO..periph::PWM => self.gpio.call(vm, op - periph::GPIO),
            periph::PWM..periph::UART => self.pwm.call(vm, op - periph::PWM),
            periph::UART..periph::I2C => self.uart.call(vm, op - periph::UART),
            periph::I2C.. => self.i2c.call(vm, op - periph::I2C),
            _ => { vm.error = RuntimeError::UnimplementedInterruptFunc; }
        }
    }
}

/// Reads the bytes of a string into a vec, sets the error and returns None if
/// the string doesn't exist or `start`/`length` don't fit into it.
pub fn string_bytes<S: Storage>(vm: &mut VirtMach<S>, index: i32, start: i32, length: i32) -> Option<Vec<u8>> {
    if !(0..=255).contains(&index) || start < 0 || length < 0 {
        vm.error = RuntimeError::InterruptError;
        return None;
    }
    let total = vm.copy_bin(index as _, &mut []);
    if vm.error != RuntimeError::NoError {
        return None;
    }
    if start as usize + length as usize > total {
        vm.error = RuntimeError::InterruptError;
        return None;
    }
    let mut buf = vec![0u8; total];
    vm.copy_bin(index as _, &mut buf);
    Some(buf[start as usize..(start + length) as usize].to_vec())
}

/// The memory cells address..address + length, None (and an error) if outside
pub fn mem_range<S: Storage>(vm: &mut VirtMach<S>, address: i32, length: i32) -> Option<core::ops::Range<usize>> {
    if address < 0 || length < 0 || address as usize + length as usize > vm.memory.len() {
        vm.error = RuntimeError::InterruptError;
        return None;
    }
    Some(address as usize..(address + length) as usize)
}
