use std::cell::RefCell;
use std::rc::Rc;

use embassy_time::Timer;
use esp_idf_svc::hal::i2c::I2cDriver;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};
use ssd1306::prelude::*;
use ssd1306::size::{
    DisplaySize128x32, DisplaySize128x64, DisplaySize64x32, DisplaySize64x48, DisplaySize72x40, DisplaySize96x16,
};
use ssd1306::{I2CDisplayInterface, Ssd1306};

use crate::driver::{DriverKind, DriverView};
use crate::global::{Global, OledConnectionStatus, DISPLAY_BUFFER_OLED};

// pub(crate): `command_listener::CommandListener` writes `NVS_KEY_CONFIG`
// on "#oled"/"#factory" and shows it in "#cfg" - see `OledConfig`.
// Shared with `iled` - both keep their config in the "drivers" namespace.
pub(crate) const NVS_NAMESPACE: &str = "drivers";
pub(crate) const NVS_KEY_CONFIG: &str = "oled";
pub(crate) const NVS_VALUE_MAX_LEN: usize = 48;

/// The panel sizes the ssd1306 crate has a `DisplaySize` for - anything else
/// is rejected by `OledConfig::parse`.
const SSD1306_SIZES: [(u8, u8); 6] = [(128, 64), (128, 32), (96, 16), (72, 40), (64, 48), (64, 32)];

/// Largest `SSD1306_SIZES` entry's frame, in bytes (1 bit/pixel).
const MAX_FRAME_LEN: usize = 128 * 64 / 8;

/// The I2C OLED, persisted as `NVS_KEY_CONFIG` in the form
/// "<protocol>:<width>:<height>:<x>:<y>[:<rotation>]", e.g. the default
/// "ssd1306:128:64:0:0". `ssd1306` is the only protocol; `width`/`height` is
/// the panel's own size (one of `SSD1306_SIZES`), `rotation` (0/90/180/270,
/// clockwise, default 0) how the output is turned on the panel, and `x`/`y`
/// a hardware offset into the SSD1306's own display RAM, on top of the
/// crate's per-size `OFFSETX`/`OFFSETY` - where the draw area starts, for
/// glass that doesn't sit where the crate assumes. `y` is in pixels but the
/// controller addresses whole 8-pixel pages, so it must be a multiple of 8.
/// Whatever would land past the controller's 128x64 RAM is cut off. Which
/// part of the nodem framebuffer the output shows is a separate, software
/// mapping: the "oled" device entry in "#nodem" - see `DriverView`. An empty value or any
/// other protocol disables the display: it is neither initialized nor sent
/// any data (`Global::oled_config` is `None`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct OledConfig {
    pub width: u8,
    pub height: u8,
    pub x: u16,
    pub y: u16,
    pub rotation: u16,
}

impl Default for OledConfig {
    fn default() -> Self {
        Self { width: 128, height: 64, x: 0, y: 0, rotation: 0 }
    }
}

impl OledConfig {
    /// `Ok(None)` for a disabled display (empty, or a protocol other than
    /// "ssd1306"); `Err(())` for an "ssd1306" value with missing/extra
    /// fields, a size not in `SSD1306_SIZES`, an `x`/`y` outside the
    /// controller's 128x64 RAM or a `y` that isn't a multiple of 8, or a
    /// rotation other than 0/90/180/270.
    pub(crate) fn parse(s: &str) -> Result<Option<Self>, ()> {
        let mut fields = s.trim().split(':').map(str::trim);
        if fields.next() != Some("ssd1306") {
            return Ok(None);
        }

        let width: u8 = fields.next().ok_or(())?.parse().map_err(|_| ())?;
        let height: u8 = fields.next().ok_or(())?.parse().map_err(|_| ())?;
        let x: u16 = fields.next().ok_or(())?.parse().map_err(|_| ())?;
        let y: u16 = fields.next().ok_or(())?.parse().map_err(|_| ())?;
        let rotation: u16 = match fields.next() {
            Some(r) => r.parse().map_err(|_| ())?,
            None => 0,
        };
        if fields.next().is_some() {
            return Err(());
        }

        if !SSD1306_SIZES.contains(&(width, height))
            || x >= 128
            || y >= 64
            || y % 8 != 0
            || ![0, 90, 180, 270].contains(&rotation)
        {
            return Err(());
        }

        Ok(Some(Self { width, height, x, y, rotation }))
    }

    /// Rotation is only written when non-zero, so the default round-trips
    /// to exactly "ssd1306:128:64:0:0".
    pub(crate) fn to_nvs_string(&self) -> String {
        let mut s = format!("ssd1306:{}:{}:{}:{}", self.width, self.height, self.x, self.y);
        if self.rotation != 0 {
            s.push_str(&format!(":{}", self.rotation));
        }
        s
    }

    /// The panel's size as seen after `rotation` - width and height swap
    /// for 90/270.
    fn rotated_size(&self) -> (usize, usize) {
        let (w, h) = (self.width as usize, self.height as usize);
        if self.rotation % 180 == 90 { (h, w) } else { (w, h) }
    }

    /// This frame's output, mapped into nodem as the "oled" device.
    fn view<'a>(&self, g: &'a Global) -> DriverView<'a> {
        let (w, h) = self.rotated_size();
        DriverView::new(g, DriverKind::Oled, w, h)
    }

    /// Whether panel pixel `(px, py)` is lit: turned by `rotation` into
    /// `view`'s coordinates.
    fn pixel(&self, view: &DriverView, px: usize, py: usize) -> bool {
        let (w, h) = (self.width as usize, self.height as usize);
        let (rx, ry) = match self.rotation {
            90 => (py, w - 1 - px),
            180 => (w - 1 - px, h - 1 - py),
            270 => (h - 1 - py, px),
            _ => (px, py),
        };
        view.pixel(rx, ry)
    }
}

/// Reads `NVS_KEY_CONFIG` back, the same way `nodem::read_nodem_config`
/// does its own: a missing value (first boot) or a malformed "ssd1306" one
/// is replaced in NVS by `OledConfig::default()`. A deliberately disabled
/// value (empty/other protocol) is kept as it is.
fn read_oled_config(nvs: &EspNvs<NvsDefault>) -> Option<OledConfig> {
    let mut buf = [0u8; NVS_VALUE_MAX_LEN + 1];
    let raw = nvs.get_str(NVS_KEY_CONFIG, &mut buf).ok().flatten();

    if let Some(raw) = raw {
        match OledConfig::parse(raw) {
            Ok(config) => return config,
            Err(()) => log::warn!("Malformed '{NVS_KEY_CONFIG}' in NVS ('{raw}'), falling back to default"),
        }
    }

    let config = OledConfig::default();

    if let Err(e) = nvs.set_str(NVS_KEY_CONFIG, &config.to_nvs_string()) {
        log::error!("Failed to persist default '{NVS_KEY_CONFIG}' to NVS: {e:?}");
    }

    Some(config)
}

/// Drives the I2C OLED (SDA=GPIO8, SCL=GPIO9) as `Global::oled_config` says:
/// loaded from `NVS_KEY_CONFIG` at startup, then followed live - "#oled"/
/// "#factory" set it, and any change tears the current display down and
/// sets it up anew (or leaves the bus idle, if now disabled).
pub async fn oled_task(i2c: I2cDriver<'static>, global: Rc<RefCell<Global>>, nvs: EspDefaultNvsPartition) {
    let config = match EspNvs::new(nvs, NVS_NAMESPACE, true) {
        Ok(nvs) => read_oled_config(&nvs),
        Err(e) => {
            log::error!("Failed to open '{NVS_NAMESPACE}' NVS namespace: {e:?}");
            Some(OledConfig::default())
        }
    };
    global.borrow_mut().oled_config = config;

    let mut i2c = i2c;

    loop {
        let config = global.borrow().oled_config;
        let Some(config) = config else {
            global.borrow_mut().oled_status.status = OledConnectionStatus::Disabled;
            while global.borrow().oled_config.is_none() {
                Timer::after_millis(50).await;
            }
            continue;
        };

        i2c = match (config.width, config.height) {
            (128, 32) => run_ssd1306(i2c, DisplaySize128x32, config, &global).await,
            (96, 16) => run_ssd1306(i2c, DisplaySize96x16, config, &global).await,
            (72, 40) => run_ssd1306(i2c, DisplaySize72x40, config, &global).await,
            (64, 48) => run_ssd1306(i2c, DisplaySize64x48, config, &global).await,
            (64, 32) => run_ssd1306(i2c, DisplaySize64x32, config, &global).await,
            _ => run_ssd1306(i2c, DisplaySize128x64, config, &global).await,
        };
    }
}

/// Flushes `Global::display_buffer` (through `config`'s offset/rotation) to
/// an SSD1306 of `size` whenever `nodem_task` marks it dirty, until
/// `Global::oled_config` stops being `config` - then switches the panel off
/// and hands the bus back to `oled_task`. Only clears its own
/// `DISPLAY_BUFFER_OLED` slot of `display_buffer_dirty` - see that field's
/// doc comment for why this can't share a single flag with `iled_task`. A
/// failed initial `init()` leaves the display `Failed` until the config
/// changes; a failed flush reinitializes and the next frame tries again.
async fn run_ssd1306<SIZE: DisplaySize>(
    i2c: I2cDriver<'static>,
    size: SIZE,
    config: OledConfig,
    global: &Rc<RefCell<Global>>,
) -> I2cDriver<'static> {
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, size, DisplayRotation::Rotate0);

    let initialized = match display.init() {
        Ok(()) => {
            global.borrow_mut().oled_status.status = OledConnectionStatus::Connected;
            true
        }
        Err(e) => {
            log::error!("SSD1306 init failed: {e:?}");
            global.borrow_mut().oled_status.status = OledConnectionStatus::Failed(format!("init: {e:?}"));
            false
        }
    };

    // First frame goes out right away, dirty or not - the panel has just
    // been (re)initialized and shows nothing yet.
    let mut force = true;
    // The draw area in display RAM: the crate's own offset for this panel
    // size plus the configured hardware offset, cut off at the controller's
    // RAM edge (`config.y` and every `height` are whole pages, so `pages`
    // is too).
    let start = (SIZE::OFFSETX.saturating_add(config.x as u8), SIZE::OFFSETY.saturating_add(config.y as u8));
    let cols = (SIZE::WIDTH as usize).min((SIZE::DRIVER_COLS as usize).saturating_sub(start.0 as usize));
    let pages = (SIZE::HEIGHT as usize).min((SIZE::DRIVER_ROWS as usize).saturating_sub(start.1 as usize)) / 8;
    let end = (start.0 + cols as u8, start.1 + (pages * 8) as u8);
    let frame_len = cols * pages;
    let mut frame = [0u8; MAX_FRAME_LEN];

    loop {
        let (dirty, current) = {
            let g = global.borrow();
            (g.display_buffer_dirty[DISPLAY_BUFFER_OLED], g.oled_config)
        };
        if current != Some(config) {
            break;
        }
        if !initialized || !(dirty || force) {
            // Poll frequently rather than busy-spin: this executor is
            // single-threaded and cooperative, so a loop with no `.await` at
            // all would starve every other task (heartbeat, wifi, websocket).
            Timer::after_millis(1).await;
            continue;
        }
        force = false;

        // Page-major, as the SSD1306 takes it in horizontal addressing mode
        // (what `init()` sets up): one byte per column per 8-pixel page, LSB
        // at the top.
        {
            let g = global.borrow();
            let view = config.view(&g);
            for page in 0..pages {
                for col in 0..cols {
                    let mut byte = 0u8;
                    for b in 0..8 {
                        byte |= (config.pixel(&view, col, page * 8 + b) as u8) << b;
                    }
                    frame[page * cols + col] = byte;
                }
            }
        }

        const MAX_ATTEMPTS: u32 = 3;
        let mut result = Ok(());

        for attempt in 1..=MAX_ATTEMPTS {
            result = if frame_len == 0 {
                Ok(())
            } else {
                display.set_draw_area(start, end).and_then(|()| display.draw(&frame[..frame_len]))
            };

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

    // Blank the panel rather than leave it frozen on the last frame - the
    // next config may be a different size/offset, or disabled altogether.
    if initialized {
        if let Err(e) = display.set_display_on(false) {
            log::warn!("OLED switch-off failed: {e:?}");
        }
    }

    display.release().release()
}
