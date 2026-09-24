use std::cell::RefCell;
use std::rc::Rc;

use embassy_time::Timer;
use esp_idf_svc::hal::i2c::I2cDriver;
use ssd1306::prelude::*;
use ssd1306::{I2CDisplayInterface, Ssd1306};

use crate::global::{Global, OledConnectionStatus, DISPLAY_BUFFER_OLED};

/// Flushes `Global::display_buffer` to the SSD1306 over I2C (SDA=GPIO8, SCL=GPIO9)
/// whenever `nodem_task` marks it dirty. The panel is always 128x64; a
/// framebuffer of any other `NodemConfig` size is shown from its top-left
/// corner, cropped or padded with blank pixels (see `Global::display_pixel`).
/// Only clears its own
/// `DISPLAY_BUFFER_OLED` slot of `display_buffer_dirty` - see that field's
/// doc comment for why this can't share a single flag with `iled_task`.
pub async fn oled_task(i2c: I2cDriver<'static>, global: Rc<RefCell<Global>>) {
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0);

    if let Err(e) = display.init() {
        log::error!("SSD1306 init failed: {e:?}");
        global.borrow_mut().oled_status.status = OledConnectionStatus::Failed(format!("init: {e:?}"));
        return;
    }
    global.borrow_mut().oled_status.status = OledConnectionStatus::Connected;

    loop {
        if !global.borrow().display_buffer_dirty[DISPLAY_BUFFER_OLED] {
            // Poll frequently rather than busy-spin: this executor is
            // single-threaded and cooperative, so a loop with no `.await` at
            // all would starve every other task (heartbeat, wifi, websocket).
            Timer::after_millis(1).await;
            continue;
        }

        let mut page = [0b0000000u8; 128];

        const MAX_ATTEMPTS: u32 = 3;
        let mut result = Ok(());

        for attempt in 1..=MAX_ATTEMPTS {
            for row in 0 .. 8 {
                {
                    let g = global.borrow();
                    for col in 0 .. 128 {
                        let mut col_buf = 0u8;
                        for b in 0 .. 8 {
                            col_buf |= (g.display_pixel(col, row * 8 + b) as u8) << b;
                        }
                        page[col] = col_buf;
                    }
                }

                match &result { Ok(()) => { result = display.set_row((row * 8) as u8); } Err(_) => { break; } }
                match &result { Ok(()) => { result = display.set_column(0); } Err(_) => { break; } }
                match &result { Ok(()) => { result = display.draw(&page); } Err(_) => { break; } }
            }

            match &result {
                Ok(()) => break,
                Err(e) => {
                    global.borrow_mut().oled_status.errors += 1;
                    log::warn!("OLED connection check attempt {attempt}/{MAX_ATTEMPTS} failed: {e:?}");
                    Timer::after_millis(10).await;
                }
            }
        }

        let mut g = global.borrow_mut();
        match &result {
            Ok(()) => g.oled_status.status = OledConnectionStatus::Connected,
            Err(e) => {
                log::error!("OLED connection check failed {MAX_ATTEMPTS} times in a row, reinitializing display");
                g.oled_status.reinits += 1;
                g.oled_status.status = match display.init() {
                    Ok(()) => OledConnectionStatus::Connected,
                    Err(init_e) => {
                        log::error!("OLED reinitialization failed: {init_e:?}");
                        OledConnectionStatus::Failed(format!("{e:?}, reinit: {init_e:?}"))
                    }
                };
            }
        }

        g.display_buffer_dirty[DISPLAY_BUFFER_OLED] = false;
    }
}
