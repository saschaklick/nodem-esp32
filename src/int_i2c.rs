//! The i2c part of the VM's periph interrupt, see `virtmach::interrupts::periph`
//! (i2c_*); `call` gets the function number counted from `periph::I2C`.
//!
//! The ESP32-C3 has a single controller, port 0, and the firmware owns it: it
//! drives the OLED on GPIO8/9 and installed the driver at 1 MHz. Programs use
//! the same bus, transfers are blocking and so never interleave with the
//! display's. i2c_setup only checks the arguments - the clock stays as it is.
//! Any other port or device is an RuntimeError::InterruptError.
//!
//! The driver can't tell a missing address acknowledge from a missing data
//! acknowledge, both are -1; other failures are -4 for a timeout and -3.

use esp_idf_svc::hal::delay::TickType;
use esp_idf_svc::sys::{
    i2c_master_read_from_device, i2c_master_write_read_device, i2c_master_write_to_device, ESP_ERR_TIMEOUT,
    ESP_FAIL, ESP_OK,
};
use virtmach::{RuntimeError, Storage, VMAtom, VirtMach};

use crate::int_periph::{mem_range, string_bytes};

const PORT: u32 = 0;
const TIMEOUT_MS: u64 = 50;

#[derive(Default)]
pub struct I2c;

fn status(err: i32, count: usize) -> VMAtom {
    match err {
        ESP_OK => count.min(VMAtom::MAX as usize) as VMAtom,
        ESP_FAIL => -1,
        ESP_ERR_TIMEOUT => -4,
        _ => -3,
    }
}

fn ticks() -> u32 {
    TickType::new_millis(TIMEOUT_MS).ticks()
}

fn write(device: i32, bytes: &[u8]) -> VMAtom {
    status(unsafe { i2c_master_write_to_device(PORT, device as u8, bytes.as_ptr(), bytes.len(), ticks()) }, bytes.len())
}

fn read(device: i32, buf: &mut [u8]) -> VMAtom {
    status(unsafe { i2c_master_read_from_device(PORT, device as u8, buf.as_mut_ptr(), buf.len(), ticks()) }, buf.len())
}

impl I2c {
    pub fn call<S: Storage>(&mut self, vm: &mut VirtMach<S>, op: VMAtom) {
        // arguments after the port
        let args = match op {
            0 => 1, 1 => 1, 2 => 2, 3 => 1, 4 => 3, 5 => 2,
            10 => 3, 11 => 3, 12 => 4, 13 => 3, 14 => 5,
            _ => { vm.error = RuntimeError::UnimplementedInterruptFunc; return; }
        };
        let port = vm.stack_pop();
        let mut a = [0i32; 5];
        for slot in a.iter_mut().take(args) { *slot = vm.stack_pop() as i32; }
        let returns = (op != 0) as usize;
        let fail = |vm: &mut VirtMach<S>| {
            for _ in 0..returns { vm.stack_push(0); }
            vm.error = RuntimeError::InterruptError;
        };
        // a[0] is the speed for setup, otherwise the 7 bit device
        let bad_device = op != 0 && !(0..=0x7f).contains(&a[0]);
        if port as u32 != PORT || bad_device || op == 0 && a[0] <= 0 {
            return fail(vm);
        }
        let device = a[0];
        match op {
            0 => {}
            1 => vm.stack_push(write(device, &[])),
            2 => vm.stack_push(write(device, &[a[1] as u8])),
            3 => {
                let mut byte = [0u8; 1];
                let result = read(device, &mut byte);
                vm.stack_push(if result > 0 { byte[0] as VMAtom } else { result });
            }
            4 => {
                let result = write(device, &[a[1] as u8, a[2] as u8]);
                vm.stack_push(if result > 0 { result - 1 } else { result });
            }
            5 => {
                let mut byte = [0u8; 1];
                let result = status(unsafe {
                    i2c_master_write_read_device(PORT, device as u8, [a[1] as u8].as_ptr(), 1, byte.as_mut_ptr(), 1, ticks())
                }, 1);
                vm.stack_push(if result > 0 { byte[0] as VMAtom } else { result });
            }
            // write_mem(device, address, length)
            10 => {
                let Some(range) = mem_range(vm, a[1], a[2]) else { return fail(vm); };
                let bytes: Vec<u8> = vm.memory[range].iter().map(|cell| *cell as u8).collect();
                vm.stack_push(write(device, &bytes));
            }
            // read_mem(device, address, length)
            11 => {
                let Some(range) = mem_range(vm, a[1], a[2]) else { return fail(vm); };
                let mut buf = vec![0u8; range.len()];
                let result = read(device, &mut buf);
                if result >= 0 {
                    for (cell, byte) in vm.memory[range].iter_mut().zip(&buf) { *cell = *byte as VMAtom; }
                }
                vm.stack_push(result);
            }
            // write_str(device, src_index, start, length)
            12 => {
                let Some(bytes) = string_bytes(vm, a[1], a[2], a[3]) else { return fail(vm); };
                vm.stack_push(write(device, &bytes));
            }
            // read_str(device, dest_index, length)
            13 => {
                if !(0..=255).contains(&a[1]) || a[2] < 0 { return fail(vm); }
                let mut buf = vec![0u8; a[2] as usize];
                let result = read(device, &mut buf);
                if result >= 0 { vm.set_bin(a[1] as _, &buf); }
                vm.stack_push(result);
            }
            // write_read_mem(device, src_address, src_length, dest_address, dest_length)
            _ => {
                let (Some(src), Some(dest)) = (mem_range(vm, a[1], a[2]), mem_range(vm, a[3], a[4])) else { return fail(vm); };
                let bytes: Vec<u8> = vm.memory[src].iter().map(|cell| *cell as u8).collect();
                let mut buf = vec![0u8; dest.len()];
                let result = status(unsafe {
                    i2c_master_write_read_device(PORT, device as u8, bytes.as_ptr(), bytes.len(), buf.as_mut_ptr(), buf.len(), ticks())
                }, buf.len());
                if result >= 0 {
                    for (cell, byte) in vm.memory[dest].iter_mut().zip(&buf) { *cell = *byte as VMAtom; }
                }
                vm.stack_push(result);
            }
        }
    }
}
