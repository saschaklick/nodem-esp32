use std::cell::RefCell;
use std::rc::Rc;
use std::sync::{mpsc, Arc, Mutex, PoisonError};
use std::task::{Poll, Waker};

use embassy_time::Timer;
use esp_idf_svc::hal::delay::TickType;
use esp_idf_svc::hal::i2c::I2cDriver;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};
use esp_idf_svc::sys::EspError;

use crate::driver::{DriverKind, DriverView};
use crate::global::{Global, OledConnectionStatus, DISPLAY_BUFFER_OLED};

// pub(crate): `command_listener::CommandListener` writes `NVS_KEY_CONFIG`
// on "#oled"/"#factory" and shows it in "#cfg" - see `OledConfig`.
// Shared with `iled` - both keep their config in the "drivers" namespace.
pub(crate) const NVS_NAMESPACE: &str = "drivers";
pub(crate) const NVS_KEY_CONFIG: &str = "oled";
pub(crate) const NVS_VALUE_MAX_LEN: usize = 48;

/// 7-bit I2C address of the panel - 0x3C is what nearly every module ships
/// with (0x3D is the usual alternative strap).
const I2C_ADDRESS: u8 = 0x3C;

/// Per-transfer I2C timeout. A full 128x64 frame is ~1KB, i.e. ~10ms at the
/// bus' 1MHz - this only has to catch a hung bus, not pace the writes.
const I2C_TIMEOUT: TickType = TickType::new_millis(100);

/// The largest display RAM of any `OledChip`: columns, and 8-pixel pages -
/// which is what a segment is (see `OledChip::segment_header`).
const MAX_COLS: usize = 132;
const MAX_SEGMENTS: usize = 64 / 8;

/// Capacity of each of the two segment buffers: a segment's addressing
/// header (at most `MAX_HEADER_LEN`) plus one page of `MAX_COLS` bytes. Also
/// holds the init/off command sequences, which are shorter.
const MAX_HEADER_LEN: usize = 16;
const SEGMENT_BUF_LEN: usize = MAX_HEADER_LEN + MAX_COLS;

/// Stack of the I2C worker thread - it only runs `I2cDriver::write` and
/// hands the buffer back.
const I2C_THREAD_STACK: usize = 4096;

/// I2C control bytes, as the SSD1306-family controllers take them in front
/// of every byte (Co=1) or run of bytes (Co=0): D/C#=0 for commands, 1 for
/// display RAM data.
const CONTROL_COMMANDS: u8 = 0x00;
const CONTROL_COMMAND_SINGLE: u8 = 0x80;
const CONTROL_DATA: u8 = 0x40;

/// The controller chips an `OledConfig` can drive - its "<protocol>" field.
/// Each one knows its display RAM layout, the panel sizes it supports, and
/// the bytes that init it, switch it off, and address one segment (8-pixel
/// page) of a frame - every one of those a single I2C write, so a chip only
/// builds buffers and never touches the bus itself. A new chip gets a
/// variant here, an entry in `ALL`, and its arm in every `match` below.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum OledChip {
    Ssd1306,
    Sh1106,
}

impl OledChip {
    const ALL: [OledChip; 2] = [OledChip::Ssd1306, OledChip::Sh1106];

    /// The "<protocol>" name in `NVS_KEY_CONFIG`.
    fn name(self) -> &'static str {
        match self {
            Self::Ssd1306 => "ssd1306",
            Self::Sh1106 => "sh1106",
        }
    }

    fn from_name(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|chip| chip.name() == name)
    }

    /// The panel sizes (width, height) this chip is known to drive -
    /// anything else is rejected by `OledConfig::parse`.
    fn sizes(self) -> &'static [(u8, u8)] {
        match self {
            Self::Ssd1306 => &ssd1306::SIZES,
            Self::Sh1106 => &sh1106::SIZES,
        }
    }

    /// The controller's display RAM, in pixels (columns, rows) - at most
    /// `MAX_COLS` x `MAX_SEGMENTS * 8`.
    fn ram_size(self) -> (u8, u8) {
        match self {
            Self::Ssd1306 => ssd1306::RAM_SIZE,
            Self::Sh1106 => sh1106::RAM_SIZE,
        }
    }

    /// Where a `width`x`height` panel's glass usually starts in display RAM
    /// (columns, rows) - `OledConfig::x`/`y` come on top of this.
    fn ram_offset(self, width: u8, height: u8) -> (u8, u8) {
        match self {
            Self::Ssd1306 => ssd1306::ram_offset(width, height),
            Self::Sh1106 => sh1106::ram_offset(width, height),
        }
    }

    /// Appends the init sequence, as one I2C write, to `buf`.
    fn init_commands(self, width: u8, height: u8, buf: &mut Vec<u8>) {
        match self {
            Self::Ssd1306 => ssd1306::init_commands(width, height, buf),
            Self::Sh1106 => sh1106::init_commands(width, height, buf),
        }
    }

    /// Appends the switch-off command, as one I2C write, to `buf`.
    fn off_commands(self, buf: &mut Vec<u8>) {
        match self {
            Self::Ssd1306 => ssd1306::off_commands(buf),
            Self::Sh1106 => sh1106::off_commands(buf),
        }
    }

    /// Appends what points display RAM at page `page` of `area` and
    /// switches to data, to `buf` - the page's `area.cols` bytes (one per
    /// column, LSB at the top) follow it in the same I2C write. At most
    /// `MAX_HEADER_LEN` bytes.
    fn segment_header(self, area: &DrawArea, page: u8, buf: &mut Vec<u8>) {
        match self {
            Self::Ssd1306 => ssd1306::segment_header(area, page, buf),
            Self::Sh1106 => sh1106::segment_header(area, page, buf),
        }
    }
}

/// A rectangle of display RAM, in columns and whole 8-pixel pages.
#[derive(Clone, Copy, Debug)]
struct DrawArea {
    col: u8,
    page: u8,
    cols: u8,
    pages: u8,
}

/// SSD1306: 128x64 display RAM. The init sequence (and per-size
/// settings/offsets) is the one the `ssd1306` crate used before, at its
/// `DisplayRotation::Rotate0` - rotation is done in software, by
/// `OledConfig::pixel`.
mod ssd1306 {
    use super::{DrawArea, CONTROL_COMMANDS, CONTROL_COMMAND_SINGLE, CONTROL_DATA};

    pub(super) const SIZES: [(u8, u8); 6] = [(128, 64), (128, 32), (96, 16), (72, 40), (64, 48), (64, 32)];
    pub(super) const RAM_SIZE: (u8, u8) = (128, 64);

    const DISPLAY_OFF: u8 = 0xAE;
    const DISPLAY_ON: u8 = 0xAF;
    const CLOCK_DIV: u8 = 0xD5;
    const MULTIPLEX: u8 = 0xA8;
    const DISPLAY_OFFSET: u8 = 0xD3;
    const START_LINE_0: u8 = 0x40;
    const CHARGE_PUMP: u8 = 0x8D;
    const ADDRESS_MODE: u8 = 0x20;
    const ADDRESS_MODE_HORIZONTAL: u8 = 0x00;
    const COM_PINS: u8 = 0xDA;
    const INTERNAL_IREF: u8 = 0xAD;
    const SEGMENT_REMAP_ON: u8 = 0xA1;
    const COM_SCAN_REVERSED: u8 = 0xC8;
    const PRECHARGE: u8 = 0xD9;
    const CONTRAST: u8 = 0x81;
    const VCOMH_DESELECT: u8 = 0xDB;
    const ALL_ON_OFF: u8 = 0xA4;
    const INVERT_OFF: u8 = 0xA6;
    const SCROLL_OFF: u8 = 0x2E;
    const COLUMN_ADDRESS: u8 = 0x21;
    const PAGE_ADDRESS: u8 = 0x22;

    pub(super) fn ram_offset(width: u8, height: u8) -> (u8, u8) {
        match (width, height) {
            (72, 40) => (28, 0),
            (64, 48) | (64, 32) => (32, 0),
            _ => (0, 0),
        }
    }

    pub(super) fn init_commands(width: u8, height: u8, buf: &mut Vec<u8>) {
        // Alternative COM pin layout for the 64-row-wired panels,
        // sequential for the 32/16-row ones.
        let com_pins = match (width, height) {
            (128, 32) | (96, 16) => 0x02,
            _ => 0x12,
        };

        buf.push(CONTROL_COMMANDS);
        buf.extend_from_slice(&[DISPLAY_OFF]);
        buf.extend_from_slice(&[CLOCK_DIV, 0x80]);
        buf.extend_from_slice(&[MULTIPLEX, height - 1]);
        buf.extend_from_slice(&[DISPLAY_OFFSET, 0]);
        buf.extend_from_slice(&[START_LINE_0]);
        buf.extend_from_slice(&[CHARGE_PUMP, 0x14]);
        buf.extend_from_slice(&[ADDRESS_MODE, ADDRESS_MODE_HORIZONTAL]);
        buf.extend_from_slice(&[COM_PINS, com_pins]);
        if (width, height) == (72, 40) {
            // Internal current reference on, at 240uA.
            buf.extend_from_slice(&[INTERNAL_IREF, 0x30]);
        }
        buf.extend_from_slice(&[SEGMENT_REMAP_ON, COM_SCAN_REVERSED]);
        buf.extend_from_slice(&[PRECHARGE, 0x21]);
        buf.extend_from_slice(&[CONTRAST, 0x5F]);
        buf.extend_from_slice(&[VCOMH_DESELECT, 0x40]);
        buf.extend_from_slice(&[ALL_ON_OFF, INVERT_OFF, SCROLL_OFF, DISPLAY_ON]);
    }

    pub(super) fn off_commands(buf: &mut Vec<u8>) {
        buf.extend_from_slice(&[CONTROL_COMMANDS, DISPLAY_OFF]);
    }

    /// A one-page column/page window, each command byte behind its own
    /// Co=1 control byte, then the data control byte - 13 bytes.
    pub(super) fn segment_header(area: &DrawArea, page: u8, buf: &mut Vec<u8>) {
        let page = area.page + page;
        for byte in [COLUMN_ADDRESS, area.col, area.col + area.cols - 1, PAGE_ADDRESS, page, page] {
            buf.extend_from_slice(&[CONTROL_COMMAND_SINGLE, byte]);
        }
        buf.push(CONTROL_DATA);
    }
}

/// SH1106: 132x64 display RAM, of which a 128x64 panel shows columns 2..130.
/// Page addressing only - no column/page window and no auto-advance to the
/// next page - which a segment per page, each with its own page/column
/// address, suits as it is. The init sequence is the SSD1306's minus what
/// the SH1106 lacks (addressing mode, `0x8D` charge pump, scroll), with its
/// DC-DC converter (`0xAD`) and pump voltage instead, and the datasheet's
/// reset values for precharge/VCOMH.
mod sh1106 {
    use super::{DrawArea, CONTROL_COMMANDS, CONTROL_COMMAND_SINGLE, CONTROL_DATA};

    pub(super) const SIZES: [(u8, u8); 1] = [(128, 64)];
    pub(super) const RAM_SIZE: (u8, u8) = (132, 64);

    const DISPLAY_OFF: u8 = 0xAE;
    const DISPLAY_ON: u8 = 0xAF;
    const CLOCK_DIV: u8 = 0xD5;
    const MULTIPLEX: u8 = 0xA8;
    const DISPLAY_OFFSET: u8 = 0xD3;
    const START_LINE_0: u8 = 0x40;
    const DC_DC: u8 = 0xAD;
    const DC_DC_ON: u8 = 0x8B;
    const PUMP_VOLTAGE_8V0: u8 = 0x32;
    const COM_PINS: u8 = 0xDA;
    const SEGMENT_REMAP_ON: u8 = 0xA1;
    const COM_SCAN_REVERSED: u8 = 0xC8;
    const PRECHARGE: u8 = 0xD9;
    const CONTRAST: u8 = 0x81;
    const VCOMH_DESELECT: u8 = 0xDB;
    const ALL_ON_OFF: u8 = 0xA4;
    const INVERT_OFF: u8 = 0xA6;
    const PAGE_ADDRESS: u8 = 0xB0;
    const COLUMN_LOW: u8 = 0x00;
    const COLUMN_HIGH: u8 = 0x10;

    pub(super) fn ram_offset(width: u8, _height: u8) -> (u8, u8) {
        ((RAM_SIZE.0 - width) / 2, 0)
    }

    pub(super) fn init_commands(_width: u8, height: u8, buf: &mut Vec<u8>) {
        buf.push(CONTROL_COMMANDS);
        buf.extend_from_slice(&[DISPLAY_OFF]);
        buf.extend_from_slice(&[CLOCK_DIV, 0x80]);
        buf.extend_from_slice(&[MULTIPLEX, height - 1]);
        buf.extend_from_slice(&[DISPLAY_OFFSET, 0]);
        buf.extend_from_slice(&[START_LINE_0]);
        buf.extend_from_slice(&[DC_DC, DC_DC_ON, PUMP_VOLTAGE_8V0]);
        buf.extend_from_slice(&[COM_PINS, 0x12]);
        buf.extend_from_slice(&[SEGMENT_REMAP_ON, COM_SCAN_REVERSED]);
        buf.extend_from_slice(&[PRECHARGE, 0x22]);
        buf.extend_from_slice(&[CONTRAST, 0x80]);
        buf.extend_from_slice(&[VCOMH_DESELECT, 0x35]);
        buf.extend_from_slice(&[ALL_ON_OFF, INVERT_OFF, DISPLAY_ON]);
    }

    pub(super) fn off_commands(buf: &mut Vec<u8>) {
        buf.extend_from_slice(&[CONTROL_COMMANDS, DISPLAY_OFF]);
    }

    /// Page address plus the start column's low/high nibble, each behind its
    /// own Co=1 control byte, then the data control byte - 7 bytes.
    pub(super) fn segment_header(area: &DrawArea, page: u8, buf: &mut Vec<u8>) {
        let page = area.page + page;
        for byte in [PAGE_ADDRESS | page, COLUMN_LOW | (area.col & 0x0F), COLUMN_HIGH | (area.col >> 4)] {
            buf.extend_from_slice(&[CONTROL_COMMAND_SINGLE, byte]);
        }
        buf.push(CONTROL_DATA);
    }
}

/// Where the I2C worker thread hands a finished write back: the buffer
/// (to be reused) and how the write went, plus the waker of whoever awaits
/// it in `I2cWorker::complete`.
#[derive(Default)]
struct Completion {
    state: Mutex<CompletionState>,
}

#[derive(Default)]
struct CompletionState {
    done: Option<(Vec<u8>, Result<(), EspError>)>,
    waker: Option<Waker>,
}

/// Owns the I2C bus on a thread of its own, so the (blocking) `I2cDriver`
/// writes run alongside the executor instead of stalling it: `submit` hands
/// it a buffer to write as one I2C transaction and returns right away,
/// `complete` awaits that write and gets the buffer back. At most one write
/// is outstanding at a time - `submit` is only called once the previous
/// one has been `complete`d.
struct I2cWorker {
    jobs: mpsc::Sender<Vec<u8>>,
    completion: Arc<Completion>,
}

impl I2cWorker {
    fn spawn(mut i2c: I2cDriver<'static>) -> std::io::Result<Self> {
        let (jobs, rx) = mpsc::channel::<Vec<u8>>();
        let completion = Arc::new(Completion::default());
        let thread_completion = completion.clone();

        std::thread::Builder::new().name("oled-i2c".into()).stack_size(I2C_THREAD_STACK).spawn(move || {
            for buf in rx {
                let result = i2c.write(I2C_ADDRESS, &buf, I2C_TIMEOUT.ticks());
                let mut state = thread_completion.state.lock().unwrap_or_else(PoisonError::into_inner);
                state.done = Some((buf, result));
                if let Some(waker) = state.waker.take() {
                    waker.wake();
                }
            }
        })?;

        Ok(Self { jobs, completion })
    }

    fn submit(&self, buf: Vec<u8>) {
        // The thread only ends with the channel, i.e. never while `self`
        // is around.
        let _ = self.jobs.send(buf);
    }

    async fn complete(&self) -> (Vec<u8>, Result<(), EspError>) {
        core::future::poll_fn(|cx| {
            let mut state = self.completion.state.lock().unwrap_or_else(PoisonError::into_inner);
            match state.done.take() {
                Some(done) => Poll::Ready(done),
                None => {
                    state.waker = Some(cx.waker().clone());
                    Poll::Pending
                }
            }
        })
        .await
    }
}

/// FNV-1a, 64 bit - just to tell whether a segment changed since it was
/// last sent, not against anything adversarial.
fn hash(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf2_9ce4_8422_2325, |h, &b| (h ^ b as u64).wrapping_mul(0x0100_0000_01b3))
}

/// One panel as `config` describes it, flushed segment by segment through
/// two buffers: while one is being written by the `I2cWorker`, the next
/// segment is prepared in the other (`prep`); once the write is complete,
/// `prep` goes out and the returned buffer becomes the next `prep`. Each
/// sent segment's hash is kept in `sent`, and a segment whose prepared
/// buffer hashes the same as last time is not sent again - `prep` is simply
/// reused for the next one.
struct OledDisplay<'w> {
    worker: &'w I2cWorker,
    config: OledConfig,
    area: DrawArea,
    prep: Vec<u8>,
    /// The other buffer - `None` while the worker has it.
    spare: Option<Vec<u8>>,
    /// Hash of what each segment's display RAM was last sent - `None` when
    /// unknown (not sent since init, or its write failed).
    sent: [Option<u64>; MAX_SEGMENTS],
}

impl<'w> OledDisplay<'w> {
    fn new(worker: &'w I2cWorker, config: OledConfig) -> Self {
        Self {
            worker,
            config,
            area: config.draw_area(),
            prep: Vec::with_capacity(SEGMENT_BUF_LEN),
            spare: Some(Vec::with_capacity(SEGMENT_BUF_LEN)),
            sent: [None; MAX_SEGMENTS],
        }
    }

    /// (Re)initializes the panel. Its display RAM is unknown afterwards, so
    /// the next frame sends every segment.
    async fn init(&mut self) -> Result<(), EspError> {
        self.sent = [None; MAX_SEGMENTS];
        self.prep.clear();
        self.config.chip.init_commands(self.config.width, self.config.height, &mut self.prep);
        self.send_prep().await
    }

    async fn off(&mut self) -> Result<(), EspError> {
        self.prep.clear();
        self.config.chip.off_commands(&mut self.prep);
        self.send_prep().await
    }

    /// Writes `prep` on its own - only while no segment is outstanding.
    async fn send_prep(&mut self) -> Result<(), EspError> {
        let spare = self.spare.take().expect("send_prep with a write outstanding");
        self.worker.submit(core::mem::replace(&mut self.prep, spare));
        let (buf, result) = self.worker.complete().await;
        self.spare = Some(buf);
        result
    }

    /// Fills `prep` with segment `page`: the chip's header, then one byte
    /// per column of that 8-pixel page (LSB at the top), taken from the
    /// "oled" device's view of `display_buffer` through `config`'s rotation.
    fn prepare(&mut self, page: u8, global: &Rc<RefCell<Global>>) {
        self.prep.clear();
        self.config.chip.segment_header(&self.area, page, &mut self.prep);

        let g = global.borrow();
        let view = self.config.view(&g);
        let row = page as usize * 8;
        for col in 0..self.area.cols as usize {
            let mut byte = 0u8;
            for b in 0..8 {
                byte |= (self.config.pixel(&view, col, row + b) as u8) << b;
            }
            self.prep.push(byte);
        }
    }

    /// Awaits the outstanding write of segment `page`, forgetting its hash
    /// if it failed so it is sent again.
    async fn complete(&mut self, page: u8) -> Result<(), EspError> {
        let (buf, result) = self.worker.complete().await;
        self.spare = Some(buf);
        if result.is_err() {
            self.sent[page as usize] = None;
        }
        result
    }

    /// Sends every segment whose content changed since it was last sent.
    /// Stops at the first failed write: the segments after it keep their
    /// hash of what the panel still shows, so the next attempt re-renders
    /// and sends exactly what is still missing.
    async fn flush(&mut self, global: &Rc<RefCell<Global>>) -> Result<(), EspError> {
        let mut outstanding: Option<u8> = None;

        for page in 0..self.area.pages {
            self.prepare(page, global);
            let hash = hash(&self.prep);
            if self.sent[page as usize] == Some(hash) {
                continue;
            }

            if let Some(prev) = outstanding.take() {
                self.complete(prev).await?;
            }

            self.sent[page as usize] = Some(hash);
            let spare = self.spare.take().expect("segment submitted with a write outstanding");
            self.worker.submit(core::mem::replace(&mut self.prep, spare));
            outstanding = Some(page);
        }

        match outstanding {
            Some(prev) => self.complete(prev).await,
            None => Ok(()),
        }
    }
}

/// The I2C OLED, persisted as `NVS_KEY_CONFIG` in the form
/// "<protocol>:<width>:<height>:<x>:<y>[:<rotation>]", e.g. the default
/// "ssd1306:128:64:0:0". `protocol` is an `OledChip` name ("ssd1306" or
/// "sh1106"); `width`/`height` is the panel's own size (one of
/// that chip's `OledChip::sizes`), `rotation` (0/90/180/270, clockwise,
/// default 0) how the output is turned on the panel, and `x`/`y` a hardware
/// offset into the controller's display RAM, on top of the chip's per-size
/// `OledChip::ram_offset` - where the draw area starts, for glass that
/// doesn't sit where that assumes. `y` is in pixels but the controller
/// addresses whole 8-pixel pages, so it must be a multiple of 8. Whatever
/// would land past the controller's RAM is cut off. Which
/// part of the nodem framebuffer the output shows is a separate, software
/// mapping: the "oled" device entry in "#nodem" - see `DriverView`. An empty value or an
/// unknown protocol disables the display: it is neither initialized nor sent
/// any data (`Global::oled_config` is `None`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct OledConfig {
    pub chip: OledChip,
    pub width: u8,
    pub height: u8,
    pub x: u16,
    pub y: u16,
    pub rotation: u16,
}

impl Default for OledConfig {
    fn default() -> Self {
        Self { chip: OledChip::Ssd1306, width: 128, height: 64, x: 0, y: 0, rotation: 0 }
    }
}

impl OledConfig {
    /// `Ok(None)` for a disabled display (empty, or not an `OledChip`
    /// name); `Err(())` for a known chip's value with missing/extra fields,
    /// a size not in its `OledChip::sizes`, an `x`/`y` outside its display
    /// RAM or a `y` that isn't a multiple of 8, or a rotation other than
    /// 0/90/180/270.
    pub(crate) fn parse(s: &str) -> Result<Option<Self>, ()> {
        let mut fields = s.trim().split(':').map(str::trim);
        let Some(chip) = fields.next().and_then(OledChip::from_name) else {
            return Ok(None);
        };

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

        let (ram_cols, ram_rows) = chip.ram_size();
        if !chip.sizes().contains(&(width, height))
            || x >= ram_cols as u16
            || y >= ram_rows as u16
            || y % 8 != 0
            || ![0, 90, 180, 270].contains(&rotation)
        {
            return Err(());
        }

        Ok(Some(Self { chip, width, height, x, y, rotation }))
    }

    /// Rotation is only written when non-zero, so the default round-trips
    /// to exactly "ssd1306:128:64:0:0".
    pub(crate) fn to_nvs_string(&self) -> String {
        let mut s = format!("{}:{}:{}:{}:{}", self.chip.name(), self.width, self.height, self.x, self.y);
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

    /// The draw area in display RAM: the chip's own offset for this panel
    /// size plus the configured hardware offset, cut off at the RAM's edge
    /// (`y` and every supported `height` are whole pages, so `pages` is too).
    fn draw_area(&self) -> DrawArea {
        let (ram_cols, ram_rows) = self.chip.ram_size();
        let (off_x, off_y) = self.chip.ram_offset(self.width, self.height);
        let col = off_x.saturating_add(self.x as u8);
        let row = off_y.saturating_add(self.y as u8);
        DrawArea {
            col,
            page: row / 8,
            cols: self.width.min(ram_cols.saturating_sub(col)),
            pages: self.height.min(ram_rows.saturating_sub(row)) / 8,
        }
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
/// does its own: a missing value (first boot) or a malformed one for a known chip
/// is replaced in NVS by `OledConfig::default()`. A deliberately disabled
/// value (empty/unknown protocol) is kept as it is.
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
/// sets it up anew (or leaves the bus idle, if now disabled). The bus
/// itself is driven from an `I2cWorker` thread.
pub async fn oled_task(i2c: I2cDriver<'static>, global: Rc<RefCell<Global>>, nvs: EspDefaultNvsPartition) {
    let config = match EspNvs::new(nvs, NVS_NAMESPACE, true) {
        Ok(nvs) => read_oled_config(&nvs),
        Err(e) => {
            log::error!("Failed to open '{NVS_NAMESPACE}' NVS namespace: {e:?}");
            Some(OledConfig::default())
        }
    };
    global.borrow_mut().oled_config = config;

    let worker = match I2cWorker::spawn(i2c) {
        Ok(worker) => worker,
        Err(e) => {
            log::error!("Failed to start OLED I2C thread: {e:?}");
            global.borrow_mut().oled_status.status = OledConnectionStatus::Failed(format!("thread: {e:?}"));
            return;
        }
    };

    loop {
        let config = global.borrow().oled_config;
        let Some(config) = config else {
            global.borrow_mut().oled_status.status = OledConnectionStatus::Disabled;
            while global.borrow().oled_config.is_none() {
                Timer::after_millis(50).await;
            }
            continue;
        };

        run_display(OledDisplay::new(&worker, config), &global).await;
    }
}

/// Flushes `Global::display_buffer` (through `config`'s offset/rotation) to
/// the panel whenever `nodem_task` marks it dirty, until
/// `Global::oled_config` stops being its config - then switches the panel
/// off and returns to `oled_task`. Only clears its own `DISPLAY_BUFFER_OLED`
/// slot of `display_buffer_dirty` - see that field's doc comment for why
/// this can't share a single flag with `iled_task` - and does so as a frame
/// starts, since other tasks (`nodem_task` included) keep running while its
/// segments are on the bus: a change mid-frame gets a frame of its own. A
/// failed initial init leaves the display `Failed` until the config
/// changes; a failed flush reinitializes and the next frame tries again.
async fn run_display(mut display: OledDisplay<'_>, global: &Rc<RefCell<Global>>) {
    let config = display.config;

    let initialized = match display.init().await {
        Ok(()) => {
            global.borrow_mut().oled_status.status = OledConnectionStatus::Connected;
            true
        }
        Err(e) => {
            log::error!("{} init failed: {e:?}", config.chip.name());
            global.borrow_mut().oled_status.status = OledConnectionStatus::Failed(format!("init: {e:?}"));
            false
        }
    };

    // First frame goes out right away, dirty or not - the panel has just
    // been (re)initialized and shows nothing yet.
    let mut force = true;

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
        global.borrow_mut().display_buffer_dirty[DISPLAY_BUFFER_OLED] = false;

        const MAX_ATTEMPTS: u32 = 3;
        let mut result = Ok(());

        for attempt in 1..=MAX_ATTEMPTS {
            result = display.flush(global).await;

            match &result {
                Ok(()) => break,
                Err(e) => {
                    global.borrow_mut().oled_status.errors += 1;
                    log::warn!("OLED connection check attempt {attempt}/{MAX_ATTEMPTS} failed: {e:?}");
                    Timer::after_millis(10).await;
                }
            }
        }

        let status = match &result {
            Ok(()) => OledConnectionStatus::Connected,
            Err(e) => {
                log::error!("OLED connection check failed {MAX_ATTEMPTS} times in a row, reinitializing display");
                global.borrow_mut().oled_status.reinits += 1;
                match display.init().await {
                    Ok(()) => OledConnectionStatus::Connected,
                    Err(init_e) => {
                        log::error!("OLED reinitialization failed: {init_e:?}");
                        OledConnectionStatus::Failed(format!("{e:?}, reinit: {init_e:?}"))
                    }
                }
            }
        };
        global.borrow_mut().oled_status.status = status;
    }

    // Blank the panel rather than leave it frozen on the last frame - the
    // next config may be a different size/offset, or disabled altogether.
    if initialized {
        if let Err(e) = display.off().await {
            log::warn!("OLED switch-off failed: {e:?}");
        }
    }
}
