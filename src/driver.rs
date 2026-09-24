use embassy_time::Instant;

use crate::global::Global;

/// How many `DeviceMapping`s `NodemConfig` holds at most.
pub const MAX_DEVICES: usize = 4;

/// The output drivers a `DeviceMapping` can name.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DriverKind {
    Iled,
    Oled,
}

impl DriverKind {
    fn parse(s: &str) -> Option<Self> {
        match s {
            "iled" => Some(Self::Iled),
            "oled" => Some(Self::Oled),
            _ => None,
        }
    }

    fn name(self) -> &'static str {
        match self {
            Self::Iled => "iled",
            Self::Oled => "oled",
        }
    }
}

/// Where a driver's output sits in the nodem framebuffer - one of
/// `NodemConfig`'s device entries, "<driver>:<x>:<y>:<scale_x>:<scale_y>",
/// e.g. "oled:0:0:2:2". A (scaled) framebuffer smaller than the driver's
/// output is centered on it - see `DriverView::pixel` - and `x`/`y` shift which
/// part of it shows there, in framebuffer pixels (negative is fine; whatever
/// ends up outside the framebuffer just reads as off). `scale_x`/`scale_y`
/// is how many driver pixels one framebuffer pixel covers (1 = pixel for
/// pixel, 2 = each framebuffer pixel two driver pixels wide).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct DeviceMapping {
    pub driver: DriverKind,
    pub x: i16,
    pub y: i16,
    pub scale_x: u8,
    pub scale_y: u8,
}

impl DeviceMapping {
    /// `None` for anything but exactly five colon-separated fields, an
    /// unknown driver, or a scale of 0.
    pub(crate) fn parse(s: &str) -> Option<Self> {
        let mut fields = s.split(':').map(str::trim);
        let driver = DriverKind::parse(fields.next()?)?;
        let x: i16 = fields.next()?.parse().ok()?;
        let y: i16 = fields.next()?.parse().ok()?;
        let scale_x: u8 = fields.next()?.parse().ok()?;
        let scale_y: u8 = fields.next()?.parse().ok()?;
        if fields.next().is_some() || scale_x == 0 || scale_y == 0 {
            return None;
        }
        Some(Self { driver, x, y, scale_x, scale_y })
    }

    pub(crate) fn to_nvs_string(&self) -> String {
        format!("{}:{}:{}:{}:{}", self.driver.name(), self.x, self.y, self.scale_x, self.scale_y)
    }
}

/// One frame's worth of what a driver shows, `width`x`height` of its own
/// pixels: built fresh (`new`) by `iled_task`/`oled_task` each time they
/// sample, then queried pixel by pixel (`pixel`). Resolves the driver's
/// `DeviceMapping` from `Global::nodem_config` once up front - so a "#nodem"
/// change shows up on the very next frame - and maps each driver pixel into
/// the nodem framebuffer through it. A driver with no mapping gets nothing
/// from nodem; it shows `fallback_pixel`'s blinking dotted frame instead.
pub struct DriverView<'a> {
    g: &'a Global,
    mapping: Option<DeviceMapping>,
    width: usize,
    height: usize,
    // 0/1, flipping once a second - which half of the fallback frame's dots
    // is lit.
    phase: usize,
}

impl<'a> DriverView<'a> {
    pub fn new(g: &'a Global, driver: DriverKind, width: usize, height: usize) -> Self {
        Self {
            g,
            mapping: g.nodem_config.mapping(driver),
            width,
            height,
            phase: (Instant::now().as_secs() % 2) as usize,
        }
    }

    /// Whether driver pixel `(x, y)` is lit. Anything outside
    /// `width`x`height` is off. Along each axis where the framebuffer, scaled
    /// up by the mapping, is smaller than the driver's output, it's centered
    /// on it (an equal margin either side); where it's as big or bigger, it
    /// starts at the edge. Then it's shifted by the mapping's `x`/`y` - e.g.
    /// a 96x32 framebuffer at scale 1 on a 128x64 OLED starts 16 pixels in
    /// from the left and top, but on a 64x32 one at the top-left corner.
    pub fn pixel(&self, x: usize, y: usize) -> bool {
        if x >= self.width || y >= self.height {
            return false;
        }
        let Some(m) = self.mapping else {
            return self.fallback_pixel(x, y);
        };
        let axis = |d: usize, size: usize, fb_size: u16, scale: u8, shift: i16| {
            let scale = scale as i32;
            let margin = ((size as i32 - fb_size as i32 * scale) / 2).max(0);
            shift as i32 + (d as i32 - margin).div_euclid(scale)
        };
        self.g.display_pixel(
            axis(x, self.width, self.g.nodem_config.width, m.scale_x, m.x),
            axis(y, self.height, self.g.nodem_config.height, m.scale_y, m.y),
        )
    }

    /// A dotted frame around the driver's whole `width`x`height`, every
    /// other pixel along the edge lit - which ones alternates once a second.
    fn fallback_pixel(&self, x: usize, y: usize) -> bool {
        let edge = x == 0 || y == 0 || x == self.width - 1 || y == self.height - 1;
        edge && (x + y + self.phase) % 2 == 0
    }
}
