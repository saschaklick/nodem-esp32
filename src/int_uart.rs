//! The uart part of the VM's periph interrupt, see `virtmach::interrupts::periph`
//! (uart_*); `call` gets the function number counted from `periph::UART`.
//!
//! Only port 1 is open to programs, port 0 is the console and the "#..."
//! protocol. It runs on `TX`/`RX`, which setup routes there - so GPIO6 and 7
//! belong to the uart from then on. Nothing waits: reads return what was
//! received, writes send as much as fits into the transmit buffer and return
//! how many bytes that was. Any other port or a port that wasn't set up is an
//! RuntimeError::InterruptError.

use esp_idf_svc::sys::{
    uart_config_t, uart_driver_delete, uart_driver_install, uart_get_buffered_data_len,
    uart_get_tx_buffer_free_size, uart_is_driver_installed, uart_param_config, uart_read_bytes, uart_set_pin,
    uart_write_bytes, ESP_OK,
};
use virtmach::{RuntimeError, Storage, VMAtom, VirtMach};

use crate::int_periph::{mem_range, string_bytes};

const PORT: u32 = 1;
const TX: i32 = 6;
const RX: i32 = 7;
const BUFFER: i32 = 256;

#[derive(Default)]
pub struct Uart;

impl Uart {
    fn ready(port: VMAtom) -> bool {
        port as u32 == PORT && unsafe { uart_is_driver_installed(PORT) }
    }

    fn available() -> usize {
        let mut size = 0usize;
        unsafe { uart_get_buffered_data_len(PORT, &mut size) };
        size
    }

    fn free() -> usize {
        let mut size = 0usize;
        unsafe { uart_get_tx_buffer_free_size(PORT, &mut size) };
        size
    }

    fn read(buf: &mut [u8]) -> usize {
        let n = unsafe { uart_read_bytes(PORT, buf.as_mut_ptr().cast(), buf.len() as u32, 0) };
        n.max(0) as usize
    }

    /// Sends what fits into the transmit buffer, returns how many bytes.
    fn write(bytes: &[u8]) -> usize {
        let n = bytes.len().min(Self::free());
        if n == 0 { return 0; }
        unsafe { uart_write_bytes(PORT, bytes.as_ptr().cast(), n) }.max(0) as usize
    }

    fn setup(baud: VMAtom, config: VMAtom) -> bool {
        let parity = match (config >> 2) & 3 { 0 => 0, 1 => 2, 2 => 3, _ => return false };
        if baud <= 0 { return false; }
        let mut uart: uart_config_t = unsafe { core::mem::zeroed() };
        uart.baud_rate = baud as i32 * 10;
        uart.data_bits = 3 - (config & 3) as u32;
        uart.parity = parity;
        uart.stop_bits = if config & 16 != 0 { 3 } else { 1 };
        uart.__bindgen_anon_1.source_clk = 0; // default clock
        unsafe {
            if uart_is_driver_installed(PORT) { uart_driver_delete(PORT); }
            uart_driver_install(PORT, BUFFER, BUFFER, 0, core::ptr::null_mut(), 0) == ESP_OK
                && uart_param_config(PORT, &uart) == ESP_OK
                && uart_set_pin(PORT, TX, RX, -1, -1) == ESP_OK
        }
    }

    pub fn call<S: Storage>(&mut self, vm: &mut VirtMach<S>, op: VMAtom) {
        let port = vm.stack_pop();
        // (arguments after the port, returns) of each function
        let (args, returns) = match op {
            0 => (2, 0), 1 => (0, 1), 2 => (0, 1), 3 => (1, 0), 4 => (0, 1),
            10 => (2, 1), 11 => (2, 1), 12 => (3, 1), 13 => (2, 1),
            _ => { vm.error = RuntimeError::UnimplementedInterruptFunc; return; }
        };
        let mut a = [0i32; 3];
        for slot in a.iter_mut().take(args) { *slot = vm.stack_pop() as i32; }
        let fail = |vm: &mut VirtMach<S>| {
            for _ in 0..returns { vm.stack_push(0); }
            vm.error = RuntimeError::InterruptError;
        };
        if op != 0 && !Self::ready(port) || op == 0 && port as u32 != PORT {
            return fail(vm);
        }
        match op {
            0 => if !Self::setup(a[0] as VMAtom, a[1] as VMAtom) { vm.error = RuntimeError::InterruptError; },
            1 => vm.stack_push(Self::available().min(VMAtom::MAX as usize) as VMAtom),
            2 => {
                let mut byte = [0u8; 1];
                vm.stack_push(if Self::read(&mut byte) == 1 { byte[0] as VMAtom } else { -1 });
            }
            3 => { Self::write(&[a[0] as u8]); }
            4 => vm.stack_push(Self::free().min(VMAtom::MAX as usize) as VMAtom),
            // read_str(dest_index, max), read_mem(address, max)
            10 | 11 => {
                if a[1] < 0 || (op == 10 && !(0..=255).contains(&a[0])) { return fail(vm); }
                let max = (a[1] as usize).min(Self::available());
                let mut buf = vec![0u8; max];
                if op == 11 {
                    let Some(range) = mem_range(vm, a[0], max as i32) else { return fail(vm); };
                    let n = Self::read(&mut buf);
                    for (cell, byte) in vm.memory[range].iter_mut().zip(&buf[..n]) { *cell = *byte as VMAtom; }
                    vm.stack_push(n as VMAtom);
                } else {
                    let n = Self::read(&mut buf);
                    vm.set_bin(a[0] as _, &buf[..n]);
                    vm.stack_push(n as VMAtom);
                }
            }
            // write_str(src_index, start, length)
            12 => {
                let Some(bytes) = string_bytes(vm, a[0], a[1], a[2]) else { return fail(vm); };
                vm.stack_push(Self::write(&bytes) as VMAtom);
            }
            // write_mem(address, length)
            _ => {
                let Some(range) = mem_range(vm, a[0], a[1]) else { return fail(vm); };
                let bytes: Vec<u8> = vm.memory[range].iter().map(|cell| *cell as u8).collect();
                vm.stack_push(Self::write(&bytes) as VMAtom);
            }
        }
    }
}
