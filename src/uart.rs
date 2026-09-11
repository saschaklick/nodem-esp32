use std::cell::RefCell;
use std::rc::Rc;
use esp_idf_svc::hal::reset::restart;
use esp_idf_svc::hal::uart::{AsyncUartDriver, UartDriver};
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use embassy_time::Timer;

use nodem_rs::runtime::Runtime;

use crate::command_listener::CommandListener;
use crate::global::Global;

/// Watches UART0 for incoming bytes.
///
/// `command_buf`/`command_buf_pos` (staging the partial command as bytes
/// trickle in) and `runtime` (the `nodem_rs` DOM, which `process_command`
/// parses complete commands against) all live on `Global` now, borrowed
/// together for each drain below so `runtime.process_command` can read
/// `command_buf` while it's mutating `runtime` in the same call.
///
/// NOTE: UART0 is also this device's system console
/// (`CONFIG_ESP_CONSOLE_UART_NUM=0`) - `log::` output is mirrored to
/// USB-Serial-JTAG too (`CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG=y`), so
/// logging should keep working, but installing our own driver on UART0
/// alongside the console's is untested here (no hardware to verify against).
/// If you see a UART driver-install error or garbled console output at boot,
/// move this task to `uart1`/`uart2` instead. Precisely because UART0 is
/// shared this way, the very first byte ever received here turns all
/// `log::` output off for the rest of the process's life
/// (`log::set_max_level(LevelFilter::Off)` - checked by every `log::info!`/
/// `error!`/... call site before it does any work, so this is a global,
/// one-line kill switch rather than something each call site needs to
/// check) - once a real client is actually talking the "#..." protocol over
/// this wire, any further asynchronous log line landing mid-stream would
/// corrupt whatever it's trying to read, so logging loses out to that
/// unconditionally rather than staying on until something proves it's a
/// problem. A release build already starts from that same silenced state
/// before this task (or anything else) even runs - not via this same
/// runtime mechanism, but compiled out entirely (`Cargo.toml`'s
/// `log = { features = ["release_max_level_off"] }` - see `main`'s doc
/// comment) - so this first-byte switch only ever has visible work left to
/// do in a debug build.
pub async fn uart_task(
    mut uart: AsyncUartDriver<'static, UartDriver<'static>>,
    global: Rc<RefCell<Global>>,
    nvs: EspDefaultNvsPartition,
) {
    let mut interrupt = CommandListener::new(nvs);
    let mut logging_disabled = false;

    loop {
        // `AsyncUartDriver` only exposes async `read`/`write` (no `remaining_read`,
        // no timeout param, no `wait_tx_done`) - those blocking calls all live on
        // the wrapped `UartDriver`, reached via `.driver()`/`.driver_mut()`.
        while uart.driver().remaining_read().unwrap_or(0) > 0 {
            // A plain `&mut Global`, not the `RefMut` guard itself: letting two
            // different fields (`command_buf` and `runtime`/`command_buf_pos`) be
            // borrowed at once in the same expression below - e.g.
            // `g.runtime.process_command(&g.command_buf[..], ...)` - only type-checks
            // as disjoint field borrows when `g` isn't a smart pointer needing its
            // own repeated `Deref`/`DerefMut` calls.
            let mut guard = global.borrow_mut();
            let g: &mut Global = &mut guard;

            let pos = g.command_buf_pos;
            let read_res = uart.driver().read(&mut g.command_buf[pos..], 0);
            if read_res.is_ok(){
                let n = read_res.unwrap();
                if n > 0 && !logging_disabled {
                    log::set_max_level(log::LevelFilter::Off);
                    logging_disabled = true;
                }
                log_uart_recv(&g.command_buf[pos..pos + n]);
                g.command_buf_pos += n;
                let pos = g.command_buf_pos;
                interrupt.update_status(&g.wifi_status, &g.cloud_status);
                let ret = g.runtime.process_command(&g.command_buf[..pos], uart.driver_mut(), &mut interrupt);

                // `interrupt` can't reach into `global` itself (see its doc
                // comment) - `g` is that borrow, already held here, so carry
                // over whatever `process_line` flagged on `interrupt` now.
                if interrupt.take_wifi_reconnect() {
                    g.wifi_reconnect = true;
                }
                if interrupt.take_reregister() {
                    g.reregister = true;
                }
                if let Some(device_name) = interrupt.take_device_name() {
                    g.cloud_status.device_name = Some(device_name);
                }
                if let Some(iled_config) = interrupt.take_iled_config() {
                    g.iled_config = iled_config;
                }
                if let Some(pkg_reload) = interrupt.take_pkg_reload() {
                    g.pkg_reload = Some(pkg_reload);
                }

                match ret.1 {
                    Ok(()) => {
                        for i in 0..ret.0 {
                            g.command_buf[i] = g.command_buf[ret.0 + i];
                        }
                        g.command_buf_pos -= ret.0;
                    }
                    Err(_err) => {
                        let _ = uart.driver().write(b"uart.1\r\n");
                    }
                };
                let _ = uart.driver().wait_tx_done(0);

                // Checked last, once the "#reset" ack above has actually gone
                // out over the wire - `restart()` never returns.
                if interrupt.take_restart() {
                    log::info!("Reset requested, restarting...");
                    restart();
                }
            }else{
                log::error!("UART read error");
                g.command_buf_pos = 0;
            }
            if g.command_buf_pos >= g.command_buf.len() {
                log::error!("UART read overflow");
                let _ = uart.driver().write(b"uart.2\r\n");
                g.command_buf_pos = 0;
            }
        }
        Timer::after_millis(1).await;
    }
}

/// Mirrors `websocket::log_websocket_event`'s `Text`/`Binary` split: UART has
/// no framing to say which one a given chunk is, so this just checks whether
/// it happens to be valid UTF-8 (true for every real "#..." command).
fn log_uart_recv(data: &[u8]) {
    match std::str::from_utf8(data) {
        Ok(text) => log::info!("UART recv text: {text}"),
        Err(_) => log::info!("UART recv binary ({} bytes)", data.len()),
    }
}
