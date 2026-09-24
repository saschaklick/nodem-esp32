use std::cell::RefCell;
use std::rc::Rc;

use embassy_time::Timer;
use esp_idf_svc::hal::gpio::{AnyIOPin, AnyOutputPin};
use esp_idf_svc::hal::i2s::config::{
    Config as I2sChannelConfig, DataBitWidth, SlotMode, StdClkConfig, StdConfig, StdGpioConfig,
    StdSlotConfig,
};
use esp_idf_svc::hal::i2s::{I2sDriver, I2sTx, I2S0};
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};

use crate::global::{Global, DISPLAY_BUFFER_ILED};

// pub(crate): `command_listener::CommandListener` writes
// `NVS_KEY_CONFIG` when "#iled" successfully parses - see
// `IledConfig::parse`/`to_csv`.
// Shared with `oled` - both keep their config in the "drivers" namespace.
pub(crate) const NVS_NAMESPACE: &str = "drivers";
pub(crate) const NVS_KEY_CONFIG: &str = "iled";

/// Hard cap on `IledConfig::width * height`. `FRAME_LEN`/the I2S channel's
/// DMA buffer are sized for this worst case once (in `build_i2s`), so a
/// "#iled" command (`crate::command_listener`) changing the chain's shape at
/// runtime never needs to resize them - it only ever changes how much of
/// that fixed-size frame is actually LED data versus trailing reset/off.
/// Comfortably above the default 1x1; a config over this is rejected rather
/// than silently truncated.
pub(crate) const MAX_LEDS: usize = 128;

/// Bounds on `TimingConfig`'s fields - together with `MAX_LEDS` above and
/// `MAX_COLOR_CHANNELS`/`MAX_CHANNEL_BITS` below, these fix `FRAME_LEN` (and
/// thus the I2S channel's DMA buffer) at a single worst case sized once at
/// startup, the same "size for the worst case, not the current config"
/// strategy `MAX_LEDS` uses for chain length. A faster bit rate needs more
/// bytes to hold a given nanosecond duration, so the worst case for buffer
/// sizing is the *fastest* allowed rate (`MAX_RESOLUTION` / `MIN_PATTERN_NS`)
/// combined with the *longest* allowed reset (`MAX_RESET_NS`) - any actual
/// `TimingConfig` within these bounds needs no more than `FRAME_LEN` bytes.
/// Same real-world floor/ceiling as when these fields were in whole
/// microseconds (1us / 1000us) - only the unit (and so the achievable
/// granularity above the floor) changed, not the worst case this sizes for.
const MIN_RESOLUTION: u8 = 2;
const MAX_RESOLUTION: u8 = 8;
const MIN_PATTERN_NS: u32 = 1_000;
const MAX_RESET_NS: u32 = 1_000_000;

/// Which of the 4 possible color channel letters a `ColorPattern` entry
/// names - see that struct's doc comment.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum Channel {
    Red,
    Green,
    Blue,
    WarmWhite,
    ColdWhite,
}

impl Channel {
    fn letter(self) -> char {
        match self {
            Self::Red => 'r',
            Self::Green => 'g',
            Self::Blue => 'b',
            Self::WarmWhite => 'w',
            Self::ColdWhite => 'c',
        }
    }

    /// This channel's value out of `IledConfig::on_color` - always the full,
    /// stored 8-bit value; `Framebuffer::render` is what shifts it down to
    /// whatever bit depth a `ColorPattern` entry actually asks for.
    fn raw_value(self, (r, g, b, w, c): (u8, u8, u8, u8, u8)) -> u8 {
        match self {
            Self::Red => r,
            Self::Green => g,
            Self::Blue => b,
            Self::WarmWhite => w,
            Self::ColdWhite => c,
        }
    }
}

/// At most one entry per known `Channel` letter, so `MAX_COLOR_CHANNELS`
/// channels is the most a `ColorPattern` can ever need to hold - `parse`
/// doesn't actually enforce "at most one of each letter" (a repeated letter
/// just means that channel's value gets sent more than once), but rejects
/// anything past this many entries regardless.
pub(crate) const MAX_COLOR_CHANNELS: usize = 5;

/// A channel's bit depth can't exceed this: `IledConfig::on_color`'s values
/// are always plain 8-bit numbers, so there's no extra precision past 8 bits
/// for `Framebuffer::render` to send.
const MAX_CHANNEL_BITS: u8 = 8;

/// The wire layout of one LED's color packet, set via the last of the
/// colon-separated parts of the "#iled" command's protocol field
/// (`ColorPattern::parse`, called from `IledConfig::parse`): an ordered
/// sequence of `<channel-letter><bit-depth>` pairs with no separators -
/// "g8b8r8w8" is a
/// 32-bit packet, green then blue then red then warmwhite, 8 bits each;
/// "c4" is a single 4-bit coldwhite-only channel. Letters: r(ed), g(reen),
/// b(lue), w(armwhite), c(oldwhite). A channel's bit depth can be less than
/// 8 - `Framebuffer::render` then only sends that channel's most-significant
/// `bits` bits of its value, shifting the rest off (see its doc comment).
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct ColorPattern {
    channels: [(Channel, u8); MAX_COLOR_CHANNELS],
    count: u8,
}

impl Default for ColorPattern {
    /// "g8r8b8w8" - the traditional fixed GRBW, 8 bits each, wire layout
    /// this replaced, made explicit; also the color-pattern part of what an
    /// entirely empty protocol field falls back to (see `IledConfig::parse`).
    fn default() -> Self {
        Self::parse("g8r8b8w8").expect("\"g8r8b8w8\" is a valid ColorPattern")
    }
}

impl ColorPattern {
    fn channels(&self) -> &[(Channel, u8)] {
        &self.channels[..self.count as usize]
    }

    /// Total wire bits one LED's color packet takes - the sum of every
    /// entry's bit depth.
    fn total_bits(&self) -> usize {
        self.channels().iter().map(|&(_, bits)| bits as usize).sum()
    }

    /// Parses a color-pattern string (see this struct's doc comment) -
    /// `<letter><digits>` pairs, back to back, no separators, at most
    /// `MAX_COLOR_CHANNELS` of them. `None` on an unrecognized letter, a
    /// letter with no digits after it (or digits outside `1..=MAX_CHANNEL_BITS`),
    /// too many entries, or an empty string (empty is `IledConfig::parse`'s
    /// job to default, not this).
    pub(crate) fn parse(s: &str) -> Option<Self> {
        let bytes = s.as_bytes();
        let mut channels = [(Channel::Red, 0u8); MAX_COLOR_CHANNELS];
        let mut count = 0usize;
        let mut i = 0;

        while i < bytes.len() {
            let channel = match bytes[i] {
                b'r' => Channel::Red,
                b'g' => Channel::Green,
                b'b' => Channel::Blue,
                b'w' => Channel::WarmWhite,
                b'c' => Channel::ColdWhite,
                _ => return None,
            };
            i += 1;

            let digits_start = i;
            while i < bytes.len() && bytes[i].is_ascii_digit() {
                i += 1;
            }
            if i == digits_start || count >= MAX_COLOR_CHANNELS {
                return None;
            }

            let bits: u8 = s[digits_start..i].parse().ok()?;
            if bits == 0 || bits > MAX_CHANNEL_BITS {
                return None;
            }

            channels[count] = (channel, bits);
            count += 1;
        }

        if count == 0 {
            return None;
        }

        Some(Self { channels, count: count as u8 })
    }

    /// The inverse of `parse`.
    pub(crate) fn to_csv(&self) -> String {
        self.channels().iter().map(|&(c, bits)| format!("{}{bits}", c.letter())).collect()
    }
}

/// Parses one of the "#iled" command's color fields (`color`/`off_color` -
/// they're both this same format, just with different empty-field defaults,
/// hence `empty_default` rather than a hardcoded one): `#` followed by up to
/// 10 hex digits, 2 per channel, in a fixed r, g, b, warmwhite, coldwhite
/// order - "#0f000001" sets r=0x0f, g=0x00, b=0x00, warmwhite=0x01, and
/// (since the string ends there) coldwhite=0: channels past however many
/// digits are actually given default to 0 rather than needing to be spelled
/// out. The string can end at any digit, not just on a 2-digit boundary -
/// "#0f0" is r=0x0f, g=0x0 (a lone trailing digit, parsed same as any other
/// hex digit), b/warmwhite/coldwhite=0 - so a caller can always just trim
/// trailing zero digits instead of needing to spell out whole channels.  An
/// empty field is `empty_default` as-is (not run through that same
/// "trailing channels default to 0" logic - it's already a complete value).
fn parse_color(s: &str, empty_default: (u8, u8, u8, u8, u8)) -> Option<(u8, u8, u8, u8, u8)> {
    if s.is_empty() {
        return Some(empty_default);
    }

    let hex = s.strip_prefix('#')?;
    if hex.len() > 2 * 5 {
        return None;
    }

    let mut values = [0u8; 5];
    for (i, chunk) in hex.as_bytes().chunks(2).enumerate() {
        values[i] = u8::from_str_radix(std::str::from_utf8(chunk).ok()?, 16).ok()?;
    }

    Some((values[0], values[1], values[2], values[3], values[4]))
}

/// The inverse of `parse_color`: always the full 10 hex digits (r, g, b,
/// warmwhite, coldwhite), even if the value that produced them came from a
/// shorter string that left some channels defaulted to 0 - a canonical form
/// is simpler than trying to reproduce however many pairs the original
/// input happened to specify.
fn color_to_csv((r, g, b, w, c): (u8, u8, u8, u8, u8)) -> String {
    format!("#{r:02x}{g:02x}{b:02x}{w:02x}{c:02x}")
}

/// Which horizontal edge the chain enters the framebuffer from - the `r`/`l`
/// character of the "#iled" command's `<layout>` sub-field (`parse_layout`).
/// Only meaningful on its own for a `Axis::Row` chain (it's the base
/// direction each row runs, see `ChainLayout::line_forward`); for
/// `Axis::Column` it instead picks which column is physical line 0 (see
/// `ChainLayout::led_index`).
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum HEdge {
    Left,
    Right,
}

/// Which vertical edge the chain enters the framebuffer from - the `t`/`b`
/// character of the `<layout>` sub-field. Symmetric with `HEdge`: the base
/// direction each column runs for `Axis::Column`, or which row is physical
/// line 0 for `Axis::Row`.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum VEdge {
    Top,
    Bottom,
}

/// Which way the chain's "lines" run - the optional `x`/`y` character of the
/// `<layout>` sub-field (default `Row`): `Row` means a line is one row (`width`
/// pixels, scanned along X, lines stacked along Y); `Column` means a line is
/// one column (`height` pixels, scanned along Y, lines stacked along X).
/// This is what lets a chain be wired either for a wide/short panel (`Row`)
/// or one built from vertical strips (`Column`) - see `ChainLayout::led_index`.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum Axis {
    Row,
    Column,
}

/// How a configured chain's lines (rows or columns - see `Axis`) are wired
/// into one physical chain: which corner the chain starts at (`h_edge` x
/// `v_edge`), which axis is scanned line-by-line (`axis`), and whether
/// successive lines all run the same direction or alternate. This is
/// exactly how real LED matrix panels vary in practice: `serpentine: true`
/// is a boustrophedon layout - line 0 runs the base direction, line 1 the
/// opposite, line 2 back to the base direction, and so on - which is the
/// common wiring choice since it avoids a long return wire from the end of
/// one line back to the start of the next. `serpentine: false` instead
/// wires every line the same direction, for panels/chains that really are
/// wired straight through.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) struct ChainLayout {
    pub(crate) h_edge: HEdge,
    pub(crate) v_edge: VEdge,
    pub(crate) axis: Axis,
    pub(crate) serpentine: bool,
}

impl ChainLayout {
    /// Whether physical line `line` (a row if `axis` is `Row`, a column if
    /// `Column`) runs in its base direction ("forward") or the opposite -
    /// flips every other line when `serpentine` is set. The base direction
    /// comes from `h_edge` for a `Row` axis (each row runs along X) and from
    /// `v_edge` for a `Column` axis (each column runs along Y) - see
    /// `led_index` for how the *other* edge picks which line is line 0.
    fn line_forward(&self, line: usize) -> bool {
        let base_forward = match self.axis {
            Axis::Row => self.h_edge == HEdge::Left,
            Axis::Column => self.v_edge == VEdge::Top,
        };
        if self.serpentine && line % 2 == 1 { !base_forward } else { base_forward }
    }

    /// Maps framebuffer pixel `(x, y)` (within a `width`x`height`
    /// framebuffer) to its index in the physical chain.
    fn led_index(&self, x: usize, y: usize, width: usize, height: usize) -> usize {
        match self.axis {
            Axis::Row => {
                let line = if self.v_edge == VEdge::Top { y } else { height - 1 - y };
                let pos = if self.line_forward(line) { x } else { width - 1 - x };
                line * width + pos
            }
            Axis::Column => {
                let line = if self.h_edge == HEdge::Left { x } else { width - 1 - x };
                let pos = if self.line_forward(line) { y } else { height - 1 - y };
                line * height + pos
            }
        }
    }
}

/// User-configurable timing for the bit encoding, set via the first four of
/// the five colon-separated parts of the "#iled" command's protocol field
/// (see `TimingConfig::parse` and, for how that field also carries
/// `ColorPattern`'s part, `IledConfig::parse`). `pattern_high`/`pattern_low`
/// hold the raw bit patterns (their low
/// `resolution` bits, MSB first) shifted out for a data "1"/"0"
/// respectively - independently configurable, *not* one derived as the
/// other's complement.
///
/// That independence is deliberate, not just flexibility for its own sake:
/// two real-hardware findings (from systematically sending single-byte
/// color values and comparing which came through intact) turned out to be
/// impossible to satisfy *both* of at once as long as "0" was forced to be
/// "1"'s complement:
///
/// 1. Each pattern should be *monotonic* - a run of one bit then a run of
///    the other, never alternating back and forth (a pattern like `1101` -
///    high, high, low, high - was tried, and reintroduces a low dip mid
///    period). WS2812-style chips find the start of the *next* bit by
///    watching for the next rising edge, so a pattern with more than one
///    rising edge in its own period can make the chip think the next bit
///    started early - desyncing every bit after it, not just the one with
///    the bad shape. Systemic, not narrow.
/// 2. A "1" data-bit immediately followed by a "0" data-bit (or vice versa)
///    must not merge into an out-of-spec run at the boundary: with
///    `pattern_high` "0111" (ends in 1) and `pattern_low` as its complement
///    "1000" (starts with 1) - the complement-derived setup this replaced -
///    a "1" bit followed by a "0" bit emitted "...0111 1000...": "0111"'s
///    trailing three 1s merged with "1000"'s leading 1 into a run of *four*
///    consecutive high sub-bits, a full extra sub-bit longer than either
///    symbol's own high time.
///
/// With `pattern_high`/`pattern_low` independent, both constraints hold at
/// once: pick two *monotonic* patterns that both start with the same bit
/// (e.g. both starting - and, for a clean boundary in the other direction
/// too, both ending - on "1" vs "0" isn't required, just a shared leading
/// bit) - then every symbol boundary, in either direction, always meets two
/// *different* bits (one pattern's trailing bit against the other's
/// leading bit), so no run can ever extend past what already exists inside
/// one symbol. That's exactly what `Default`'s "1110"/"1000" does: both
/// monotonic, both start with 1.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct TimingConfig {
    pub pattern_high: u8,
    pub pattern_low: u8,
    pub resolution: u8,
    pub pattern_ns: u32,
    pub reset_ns: u32,
}

impl Default for TimingConfig {
    /// "1110"/"1000" (resolution 4) at 1250ns/pattern, 600ns reset - the
    /// timing part of what an entirely empty protocol field falls back to
    /// (see `IledConfig::parse`); there's no per-part empty-field tolerance
    /// within a *non-empty* protocol field ("none of these subvalues might
    /// be left out"), so these are just this struct's own fixed defaults,
    /// used only when the whole protocol field is blank. Both monotonic and
    /// both starting with 1 - see this struct's own doc comment for why
    /// that combination is what real hardware testing showed actually
    /// matters, and why plain complementary patterns (which this replaced)
    /// couldn't satisfy it.
    fn default() -> Self {
        Self { pattern_high: 0b1110, pattern_low: 0b1000, resolution: 4, pattern_ns: 1_250, reset_ns: 600 }
    }
}

impl TimingConfig {
    /// Sub-bit clock rate the encoder runs at: `resolution` sub-bits every
    /// `pattern_ns` nanoseconds.
    pub(crate) fn bit_rate_hz(&self) -> u32 {
        (self.resolution as u64 * 1_000_000_000 / self.pattern_ns as u64) as u32
    }

    /// The I2S sample rate that gets `bit_rate_hz` out of `build_i2s`'s I2S
    /// setup - `sample_rate_hz * 2 * 8 == bit_rate_hz`; see `build_i2s`'s doc
    /// comment for why that `* 2` is there even in Mono mode.
    pub(crate) fn sample_rate_hz(&self) -> u32 {
        self.bit_rate_hz() / 16
    }

    /// Parses the first four of the "#iled" command's protocol field's five
    /// colon-separated parts (`IledConfig::parse` splits the whole field and
    /// hands each part here/to `ColorPattern::parse` - see its own doc
    /// comment for why none of the five can be left out individually).
    /// `high`/`low` must be strings of only "0"/"1" characters, the same
    /// length, `MIN_RESOLUTION..=MAX_RESOLUTION` long, one for the data "1"
    /// symbol and one for "0" (see this struct's doc comment for why
    /// they're independent rather than one being the other's complement).
    /// Neither pattern being non-monotonic is rejected here, even though
    /// this struct's own doc comment explains why that matters on real
    /// WS2812-style hardware - deliberately left unvalidated so "#iled" can
    /// be used to experiment with the timing itself, not just the parts of
    /// it already known to work. `pattern_ns` and `reset_ns` are plain
    /// nanosecond counts (nanosecond, not microsecond, precision is what
    /// real WS2812 sub-bit widths, ~300-900ns, actually need to be tunable
    /// at); the latter is capped at `MAX_RESET_NS` (see its doc comment for
    /// why - it, and `MIN_PATTERN_NS` implicitly via `pattern_ns`'s lower
    /// bound, are what keep `FRAME_LEN`'s worst-case sizing valid). `None`
    /// on anything out of those bounds, including either string being empty
    /// (an empty `high`/`low` fails the resolution-length check below, and
    /// an empty `pattern_ns`/`reset_ns` fails to parse as a number).
    pub(crate) fn parse(high: &str, low: &str, pattern_ns: &str, reset_ns: &str) -> Option<Self> {
        let resolution = high.len();
        if low.len() != resolution || !(MIN_RESOLUTION as usize..=MAX_RESOLUTION as usize).contains(&resolution) {
            return None;
        }
        if !high.bytes().chain(low.bytes()).all(|b| b == b'0' || b == b'1') {
            return None;
        }

        let to_bits = |s: &str| s.bytes().fold(0u8, |acc, b| (acc << 1) | (b - b'0'));
        let pattern_high = to_bits(high);
        let pattern_low = to_bits(low);

        let pattern_ns: u32 = pattern_ns.parse().ok()?;
        let reset_ns: u32 = reset_ns.parse().ok()?;

        if pattern_ns < MIN_PATTERN_NS || reset_ns > MAX_RESET_NS {
            return None;
        }

        Some(Self { pattern_high, pattern_low, resolution: resolution as u8, pattern_ns, reset_ns })
    }

    /// The inverse of `parse`: renders this timing back to the same
    /// "<high>:<low>:<pattern_ns>:<reset_ns>" colon-separated parts
    /// `IledConfig::to_csv` assembles the protocol field's first four parts
    /// from.
    pub(crate) fn to_csv(&self) -> String {
        let render = |pattern: u8| -> String {
            (0..self.resolution).rev().map(|i| if (pattern >> i) & 1 == 1 { '1' } else { '0' }).collect()
        };

        format!("{}:{}:{}:{}", render(self.pattern_high), render(self.pattern_low), self.pattern_ns, self.reset_ns)
    }
}

/// The chain's runtime-configurable shape, layout, color and bit
/// timing/wire-packet layout, set via the "#iled" command
/// (`crate::command_listener`, via `IledConfig::parse`) and read fresh by
/// `iled_task` every refresh from `Global::iled_config` - so a config change
/// takes effect on the very next frame, no restart needed (`iled_task` does
/// have to rebuild its I2S driver when `timing` in particular changes,
/// though - see its doc comment). `width * height` must not exceed
/// `MAX_LEDS`; `parse` rejects configs that would. `PartialEq` is what lets
/// `iled_task` tell a freshly-received config apart from the one it's
/// already showing - see its doc comment.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct IledConfig {
    pub width: usize,
    pub height: usize,
    pub layout: ChainLayout,
    pub on_color: (u8, u8, u8, u8, u8),
    pub off_color: (u8, u8, u8, u8, u8),
    pub timing: TimingConfig,
    pub color_pattern: ColorPattern,
}

impl Default for IledConfig {
    /// Matches "#iled,1:1:i,,,": every field at its own empty-input default
    /// (1x1, `HEdge::Left`/`VEdge::Top`/`Axis::Row`, all-off off) except
    /// serpentine wiring, which is on ("i" - an empty layout leaves it off),
    /// plus `DEFAULT_PROTOCOL` for the entirely-blank protocol field,
    /// which is `TimingConfig::default()`/`ColorPattern::default()`'s
    /// "1110"/"1000" pattern, 1250ns/600ns timing and "g8r8b8w8" wire layout,
    /// and rgb(16, 0, 0) on. 1x1 rather than some larger
    /// chain size is deliberate: this is also what `read_iled_config` falls
    /// back to (and persists) when nothing valid is in NVS yet, and assuming
    /// a specific chain shape/size before anyone has confirmed one is
    /// actually connected risks driving hardware that isn't there.
    fn default() -> Self {
        Self {
            width: 1,
            height: 1,
            layout: ChainLayout { h_edge: HEdge::Left, v_edge: VEdge::Top, axis: Axis::Row, serpentine: true },
            on_color: (16, 0, 0, 0, 0),
            off_color: (0, 0, 0, 0, 0),
            timing: TimingConfig::default(),
            color_pattern: ColorPattern::default(),
        }
    }
}

/// Parses the "#iled" command's geometry field's `<layout>` sub-field (the
/// third of its 3 colon-separated parts, after `<width>`/`<height>` - see
/// `IledConfig::parse`) - the merged replacement for what used to be two
/// separate top-level fields (orientation and invert). Up to 4 characters,
/// order-independent: exactly one of `r`/`l` (which horizontal edge the
/// chain enters from - `HEdge`), exactly one of `t`/`b` (which vertical edge
/// - `VEdge`), at most one of `x`/`y` (which axis is scanned line-by-line,
/// default `x`/`Row` if omitted - `Axis`), and at most one `i` (serpentine
/// wiring, default off if omitted - `ChainLayout::serpentine`). An entirely
/// empty sub-field means every part at its own default: `HEdge::Left`,
/// `VEdge::Top`, `Axis::Row`, not serpentine. `None` on an unrecognized
/// character or a second character from a category that only allows one
/// (e.g. both `r` and `l`, or both `x` and `y`).
fn parse_layout(s: &str) -> Option<ChainLayout> {
    let mut h_edge = None;
    let mut v_edge = None;
    let mut axis = None;
    let mut serpentine = false;

    for c in s.chars() {
        match c {
            'r' if h_edge.is_none() => h_edge = Some(HEdge::Right),
            'l' if h_edge.is_none() => h_edge = Some(HEdge::Left),
            't' if v_edge.is_none() => v_edge = Some(VEdge::Top),
            'b' if v_edge.is_none() => v_edge = Some(VEdge::Bottom),
            'x' if axis.is_none() => axis = Some(Axis::Row),
            'y' if axis.is_none() => axis = Some(Axis::Column),
            'i' if !serpentine => serpentine = true,
            _ => return None,
        }
    }

    Some(ChainLayout {
        h_edge: h_edge.unwrap_or(HEdge::Left),
        v_edge: v_edge.unwrap_or(VEdge::Top),
        axis: axis.unwrap_or(Axis::Row),
        serpentine,
    })
}

/// The inverse of `parse_layout`: always renders all 4 parts explicitly
/// (rather than omitting whichever are at their default), so this is a
/// stable canonical form for `NVS_KEY_CONFIG` and "#cfg" to round-trip.
fn layout_to_csv(layout: &ChainLayout) -> String {
    let mut s = String::new();
    s.push(match layout.h_edge { HEdge::Right => 'r', HEdge::Left => 'l' });
    s.push(match layout.v_edge { VEdge::Top => 't', VEdge::Bottom => 'b' });
    s.push(match layout.axis { Axis::Row => 'x', Axis::Column => 'y' });
    if layout.serpentine { s.push('i'); }
    s
}

/// Parses `pixels_per_row`/`rows`: empty or "0" both mean "1" - a chain
/// can't usefully be 0 pixels wide/tall, and treating that as a request for
/// the smallest real size (rather than an error) means leaving either field
/// blank "just works" instead of needing an explicit "1".
fn parse_dimension(s: &str) -> Option<usize> {
    if s.is_empty() {
        return Some(1);
    }

    let n: usize = s.parse().ok()?;
    Some(if n == 0 { 1 } else { n })
}

/// A library of known iLED chip names, each mapped to the "#iled" protocol
/// string (the whole "<pattern_high>:<pattern_low>:<pattern_ns>:<reset_ns>:
/// <color_pattern>" value `IledConfig::parse`'s protocol field otherwise
/// takes literally - see its doc comment) that drives that chip correctly.
/// Add an entry here for a new chip once its timing/wire-layout is actually
/// known/verified, rather than expecting every caller to memorize or
/// re-derive its five colon-separated parts by hand. Matched
/// case-sensitively, in order, by `lookup_chip_protocol`.
const CHIP_PROTOCOLS: &[(&str, &str)] = &[
    ("ws2816", "110:100:1250:100:g8r8b8"),
    ("ws2816b", "1110:1000:1250:600:g8r8b8"),
    ("ws2816b_rgbw", "1110:1000:1250:600:g8r8b8w8"),    
];

/// Which `CHIP_PROTOCOLS` entry an entirely blank protocol field resolves
/// to (see `IledConfig::parse`) - named rather than duplicating its protocol
/// string literally here, so the two can never drift apart.
const DEFAULT_CHIP: &str = "ws2816b_rgbw";

/// Resolves a chip name to its `CHIP_PROTOCOLS` protocol string, if it names
/// a known one.
fn lookup_chip_protocol(name: &str) -> Option<&'static str> {
    CHIP_PROTOCOLS.iter().find(|&&(chip, _)| chip == name).map(|&(_, protocol)| protocol)
}

impl IledConfig {
    /// Parses a "#iled,<geometry>,<protocol>,<color>,<off_color>" command's
    /// args - everything after "iled," as a single comma-separated `line` -
    /// into a config. This is `crate::command_listener`'s entire involvement
    /// with "#iled" beyond storing whatever this returns; the splitting into
    /// 4 comma-separated fields (missing trailing fields default to "", same
    /// as leaving them blank) - the first of which, `geometry`, is itself
    /// further split into 3 colon-separated sub-fields,
    /// `<width>:<height>:<layout>` - as well as every field's own parsing,
    /// defaulting and validation lives here (or in `parse_layout`/
    /// `TimingConfig::parse`/`ColorPattern::parse`/`parse_color` for their
    /// own fields) instead.
    ///
    /// `protocol` bundles what used to be four separate fields (the bit
    /// pattern, `pattern_ns`, `reset_ns` and `color_pattern`) into one
    /// colon-separated string - or, instead of spelling that out, the name
    /// of a `CHIP_PROTOCOLS` entry (e.g. "ws2816b_rgbw") that expands to one.
    /// An entirely blank field resolves `DEFAULT_CHIP` the same way. Once
    /// resolved to an actual colon-separated string (whether typed out
    /// directly or looked up by name), all five parts
    /// (`<pattern_high>:<pattern_low>:<pattern_ns>:<reset_ns>:<color_pattern>`)
    /// must be present; none of them individually falls back to a default
    /// the way every other field here does. `None` means malformed input:
    /// `width`/`height` failing to parse (though not being empty or "0" -
    /// see `parse_dimension`), an unrecognized `layout` character (see
    /// `parse_layout`), `protocol` (after any name lookup) not splitting
    /// into exactly five parts, a `width * height` outside what `FRAME_LEN`
    /// can actually handle (see `MAX_LEDS`), or a protocol
    /// part/color/off_color outside those fields' own parsers' bounds.
    pub(crate) fn parse(line: &str) -> Option<Self> {
        let mut fields = line.split(',').map(str::trim);
        let geometry = fields.next().unwrap_or("");
        let protocol = fields.next().unwrap_or("");
        let color = fields.next().unwrap_or("");
        let off_color = fields.next().unwrap_or("");

        let mut geometry = geometry.split(':').map(str::trim);
        let width = geometry.next().unwrap_or("");
        let height = geometry.next().unwrap_or("");
        let layout = geometry.next().unwrap_or("");

        let width = parse_dimension(width)?;
        let height = parse_dimension(height)?;
        let layout = parse_layout(layout)?;

        let protocol = if protocol.is_empty() {
            lookup_chip_protocol(DEFAULT_CHIP).expect("DEFAULT_CHIP must name a real CHIP_PROTOCOLS entry")
        } else {
            lookup_chip_protocol(protocol).unwrap_or(protocol)
        };
        let mut protocol_parts = protocol.split(':');
        let pattern_high = protocol_parts.next()?;
        let pattern_low = protocol_parts.next()?;
        let pattern_ns = protocol_parts.next()?;
        let reset_ns = protocol_parts.next()?;
        let color_pattern = protocol_parts.next()?;
        if protocol_parts.next().is_some() {
            return None;
        }

        let timing = TimingConfig::parse(pattern_high, pattern_low, pattern_ns, reset_ns)?;
        let color_pattern = ColorPattern::parse(color_pattern)?;
        let on_color = parse_color(color, (16, 0, 0, 0, 0))?;
        let off_color = parse_color(off_color, (0, 0, 0, 0, 0))?;

        if width * height > MAX_LEDS {
            return None;
        }

        // `Framebuffer::render`/`render_led` assume one LED's encoded packet
        // (`color_pattern.total_bits() * timing.resolution` bits) is a whole
        // number of bytes, so they can build it byte-at-a-time instead of
        // tracking an arbitrary bit position - reject any combination that
        // wouldn't be, rather than let them silently truncate a trailing
        // partial byte.
        if (color_pattern.total_bits() * timing.resolution as usize) % 8 != 0 {
            return None;
        }

        Some(Self { width, height, layout, on_color, off_color, timing, color_pattern })
    }

    /// The inverse of `parse`: renders this config back to the same 4
    /// comma-separated fields its command takes (`width`/`height`/`layout`
    /// assembled back into one colon-separated `geometry` field, and
    /// `timing`/`color_pattern` into one colon-separated protocol field),
    /// for `NVS_KEY_CONFIG` (`parse_csv` reads it back) and for "#cfg" to
    /// display.
    pub(crate) fn to_csv(&self) -> String {
        format!(
            "{}:{}:{},{}:{},{},{}",
            self.width,
            self.height,
            layout_to_csv(&self.layout),
            self.timing.to_csv(),
            self.color_pattern.to_csv(),
            color_to_csv(self.on_color),
            color_to_csv(self.off_color),
        )
    }

    /// Parses `to_csv`'s output (as read back from `NVS_KEY_CONFIG`) into a
    /// config - `to_csv` always emits exactly the same comma-separated line
    /// shape `parse` takes, so this is just `parse` under another name.
    pub(crate) fn parse_csv(csv: &str) -> Option<Self> {
        Self::parse(csv)
    }
}

/// Worst-case wire bits one LED's color packet can take: every possible
/// `ColorPattern` entry at its widest bit depth.
const MAX_TOTAL_CHANNEL_BITS: usize = MAX_COLOR_CHANNELS * MAX_CHANNEL_BITS as usize;

/// Worst-case bytes one LED's data can take: `MAX_TOTAL_CHANNEL_BITS` wire
/// bits, each becoming `MAX_RESOLUTION` output bits (see `TimingConfig`),
/// rounded up to a whole byte - instead of whatever the current config's
/// `color_pattern`/`timing.resolution` actually need.
const MAX_BYTES_PER_LED: usize = (MAX_TOTAL_CHANNEL_BITS * MAX_RESOLUTION as usize + 7) / 8;

const MAX_LED_DATA_BYTES: usize = MAX_LEDS * MAX_BYTES_PER_LED;

/// Worst-case bit rate `TimingConfig::bit_rate_hz` can produce - fastest
/// resolution over the shortest pattern - used only to size `FRAME_LEN`'s
/// reset tail for the worst case; see `MIN_RESOLUTION`/`MAX_RESOLUTION`'s
/// doc comment.
const MAX_BIT_RATE_HZ: u64 = MAX_RESOLUTION as u64 * 1_000_000_000 / MIN_PATTERN_NS as u64;

/// Bytes needed to hold `MAX_RESET_NS` of held-low output at `MAX_BIT_RATE_HZ`
/// - see `MIN_RESOLUTION`/`MAX_RESOLUTION`'s doc comment for why this (and
/// not the current config's actual reset) is what sizes `FRAME_LEN`.
const MAX_RESET_BYTES: usize = (MAX_BIT_RATE_HZ * MAX_RESET_NS as u64 / 1_000_000_000 / 8) as usize;

// pub(crate): `build_i2s` needs this to size the I2S channel's DMA buffer -
// see its comment on `frames_per_buffer`. Sized for `MAX_LEDS` LEDs at
// `TimingConfig`/`ColorPattern`'s worst-case bit rate/reset/packet size, not
// the currently-configured `IledConfig`'s actual values - see `MAX_LEDS`'s
// doc comment for why. Whatever's beyond the actually-rendered LEDs and
// their actually-needed reset for a smaller/slower/narrower config just
// stays zero, extending the reset/off tail well past what's needed -
// harmless, WS2812 doesn't mind a longer-than-needed low period.
pub(crate) const FRAME_LEN: usize = MAX_LED_DATA_BYTES + MAX_RESET_BYTES;

/// Encodes one LED's color into `buf` (exactly `bytes_per_led` - see
/// `Framebuffer::render` - long, and assumed already zeroed) per
/// `color_pattern` (each channel entry takes only its most-significant
/// `bits` bits of `color`'s full 8-bit value, shifting the rest off - a
/// `bits < 8` channel loses precision, not range) and `timing`'s bit
/// encoding. `IledConfig::parse` rejects any `color_pattern`/`timing`
/// combination whose total encoded bit count isn't a whole number of bytes,
/// so this can assume that and write whole bytes at a time instead of
/// tracking an arbitrary bit position across the buffer.
fn render_led(buf: &mut [u8], color: (u8, u8, u8, u8, u8), color_pattern: &ColorPattern, timing: &TimingConfig) {
    let mut byte = 0u8;
    let mut filled = 0u8;
    let mut pos = 0usize;

    for &(channel, bits) in color_pattern.channels() {
        let shifted = channel.raw_value(color) >> (8 - bits);
        for i in (0..bits).rev() {
            let symbol = if (shifted >> i) & 1 == 1 { timing.pattern_high } else { timing.pattern_low };
            for j in (0..timing.resolution).rev() {
                byte = (byte << 1) | ((symbol >> j) & 1);
                filled += 1;
                pos += 1;
                if filled == 8 {
                    buf[pos / 8 - 1] = byte;
                    byte = 0;
                    filled = 0;
                }
            }
        }
    }
}

/// A tiny software framebuffer: `config.height` rows of `config.width`
/// on/off pixels - monochrome, so there's nothing to store per pixel beyond
/// that single bit - backed by a fixed `MAX_LEDS`-sized array (only the
/// first `width * height` entries are ever read or written) so this never
/// needs to allocate regardless of what `IledConfig` it's built with.
/// `render` is what turns "on" into `config.on_color` (and "off" into
/// `config.off_color`) at the right LED (via `ChainLayout::led_index`) and
/// flattens everything into the wire bitstream.
struct Framebuffer {
    pixels: [bool; MAX_LEDS],
    config: IledConfig,
}

impl Framebuffer {
    fn blank(config: IledConfig) -> Self {
        Self { pixels: [false; MAX_LEDS], config }
    }

    fn set(&mut self, x: usize, y: usize, on: bool) {
        self.pixels[y * self.config.width + x] = on;
    }

    /// Flattens the framebuffer into one full chain update, followed by the
    /// reset/off tail - see `FRAME_LEN`'s doc comment. `config.on_color`/
    /// `config.off_color` are the same for every "on"/"off" pixel
    /// respectively, so each is encoded into a byte pattern exactly once
    /// (via `render_led`) up front, and every pixel then just copies the
    /// matching one of those two patterns into place - no per-pixel bit
    /// encoding. Writes into a caller-provided buffer rather than returning
    /// `[u8; FRAME_LEN]` by value: `FRAME_LEN` is a few KB (worst-case sized
    /// - see its doc comment), and a stack-to-stack copy of a buffer that
    /// size on every refresh is exactly the kind of thing that can push the
    /// shared, cooperatively-scheduled executor stack (`main`'s,
    /// `CONFIG_ESP_MAIN_TASK_STACK_SIZE` in `sdkconfig.defaults`) past its
    /// limit - this crate has already hit a "Stack protection fault" from
    /// stack-resident buffers growing once before (see that file's own
    /// comments). `iled_task` keeps one `frame` buffer alive for the task's
    /// whole lifetime and reuses it every refresh instead.
    fn render(&self, frame: &mut [u8; FRAME_LEN]) {
        frame.fill(0);

        let bits_per_led = self.config.color_pattern.total_bits() * self.config.timing.resolution as usize;
        let bytes_per_led = bits_per_led / 8;

        let mut on = [0u8; MAX_BYTES_PER_LED];
        let mut off = [0u8; MAX_BYTES_PER_LED];
        render_led(&mut on[..bytes_per_led], self.config.on_color, &self.config.color_pattern, &self.config.timing);
        render_led(&mut off[..bytes_per_led], self.config.off_color, &self.config.color_pattern, &self.config.timing);

        for y in 0..self.config.height {
            for x in 0..self.config.width {
                let on_pixel = self.pixels[y * self.config.width + x];
                let pattern = if on_pixel { &on[..bytes_per_led] } else { &off[..bytes_per_led] };
                let led = self.config.layout.led_index(x, y, self.config.width, self.config.height);
                frame[led * bytes_per_led..][..bytes_per_led].copy_from_slice(pattern);
            }
        }
    }
}

/// Crops the top-left `config.width`x`config.height` corner of the shared
/// nodem DOM framebuffer (`Global::display_buffer`, read through
/// `Global::display_pixel`) into the framebuffer, pixel for pixel: LED
/// framebuffer pixel `(x, y)` is exactly DOM pixel `(x, y)`, no scaling or
/// sampling. Both sides are already monochrome (on/off), so this is a direct
/// copy of that one bit, no thresholding or color conversion needed. An LED
/// beyond the DOM's own `NodemConfig` size just stays off.
fn sample_from_dom(fb: &mut Framebuffer, config: &IledConfig, g: &Global) {
    for y in 0..config.height {
        for x in 0..config.width {
            fb.set(x, y, g.display_pixel(x, y));
        }
    }
}

/// Reads `NVS_KEY_CONFIG` back into an `IledConfig`. If there isn't one yet
/// (first boot, or NVS wiped by "#factory") or it fails to parse (shouldn't
/// happen, since it's never written except via `IledConfig::to_csv`, but NVS
/// content can outlive the firmware that wrote it - a stored value from an
/// older firmware with a different field layout is the realistic way this
/// happens), this doesn't just return a default in memory - it *removes* the
/// unparseable value (so it doesn't confuse a future firmware version that
/// might parse a similar-but-different string into nonsense instead of
/// falling back cleanly) and then *writes* `IledConfig::default()` to NVS,
/// via the same `to_csv`/`parse_csv` round-trip a "#iled" command uses, so
/// the next boot reads back the same thing instead of re-deriving it, and
/// "#cfg" reflects reality rather than showing nothing.
fn read_iled_config(nvs: &EspNvs<NvsDefault>) -> IledConfig {
    let mut buf = [0u8; 80];

    let raw = nvs.get_str(NVS_KEY_CONFIG, &mut buf).ok().flatten();
    let stored = raw.and_then(IledConfig::parse_csv);

    if let Some(config) = stored {
        return config;
    }

    if raw.is_some() {
        if let Err(e) = nvs.remove(NVS_KEY_CONFIG) {
            log::error!("Failed to remove malformed '{NVS_KEY_CONFIG}' from NVS: {e:?}");
        }
    }

    let config = IledConfig::default();

    if let Err(e) = nvs.set_str(NVS_KEY_CONFIG, &config.to_csv()) {
        log::error!("Failed to persist default '{NVS_KEY_CONFIG}' to NVS: {e:?}");
    }

    config
}

/// Fixed IO_MUX pins for GPSPI2 on the ESP32-C3 aren't in play here - these
/// are just plain GPIOs, picked (and previously wired up once in `main.rs`,
/// before the I2S driver needed rebuilding at runtime moved that here) so
/// only `DOUT` needs to actually be connected to the chain's `Din`.
const BCLK_PIN: u8 = 1;
const DOUT_PIN: u8 = 2;
const WS_PIN: u8 = 3;

/// (Re)builds the I2S TX driver for the LED chain at a given sample rate,
/// tx-enabled and ready to write to. Every part of the driver config other
/// than the sample rate is fixed - see `iled_task`'s doc comment for the
/// reasoning behind `auto_clear`, `frames_per_buffer(FRAME_LEN)`, Mono +
/// `msb_slot_default`, and why the sample rate has to be
/// `timing.sample_rate_hz()` rather than something simpler.
///
/// Uses `I2S0::steal`/`AnyOutputPin::steal` rather than peripherals handed
/// in from `main.rs`, because this needs to be callable more than once: a
/// "#iled" command changing `TimingConfig` (and thus the required bit rate)
/// can't be applied by reconfiguring a live `I2sDriver` - the safe wrapper
/// this crate is built on doesn't expose ESP-IDF's
/// `i2s_channel_reconfig_std_clock`, or even the raw channel handle it'd
/// need - so `iled_task` instead drops the old driver (freeing GPIO1-3 and
/// I2S0) and calls this again to build a fresh one. `steal`ing only after
/// the previous driver is dropped (never while one is still alive) is
/// exactly the safe use `AnyOutputPin::steal`'s own docs describe: "keep
/// working with the peripheral after you dropped the driver that consumes
/// this".
fn build_i2s(sample_rate_hz: u32) -> anyhow::Result<I2sDriver<'static, I2sTx>> {
    let config = StdConfig::new(
        I2sChannelConfig::default().auto_clear(true).frames_per_buffer(FRAME_LEN as u32),
        StdClkConfig::from_sample_rate_hz(sample_rate_hz),
        StdSlotConfig::msb_slot_default(DataBitWidth::Bits8, SlotMode::Mono),
        StdGpioConfig::default(),
    );

    let mut i2s = I2sDriver::<I2sTx>::new_std_tx(
        unsafe { I2S0::steal() },
        &config,
        unsafe { AnyIOPin::steal(BCLK_PIN) },     // BCLK (unused) - bclk/ws need InputPin + OutputPin
        unsafe { AnyOutputPin::steal(DOUT_PIN) }, // DOUT - the WS2812 chain
        Option::<AnyIOPin>::None,                 // MCLK (unused)
        unsafe { AnyIOPin::steal(WS_PIN) },       // WS (unused)
    )?;
    i2s.tx_enable()?;

    Ok(i2s)
}

/// Fixed choices `build_i2s` makes (or that ESP-IDF defaults to for them)
/// that determine how a `TimingConfig`'s `sample_rate_hz()` actually turns
/// into hardware clock signals - kept alongside `log_timing_report` since
/// that's the only place they're used, purely to compute what the hardware
/// will actually do rather than to configure anything (`StdClkConfig::
/// from_sample_rate_hz`/`msb_slot_default` bake in the real ones).
const I2S_MCLK_MULTIPLE: u32 = 256; // `StdClkConfig::from_sample_rate_hz`'s default `MclkMultiple::M256`
const I2S_SOURCE_CLOCK_HZ: u32 = 160_000_000; // PLL_F160M - `ClockSource::default()` on ESP32-C3

/// Logs a report of what a `TimingConfig` actually asks the I2S hardware to
/// do, and how big a DMA buffer that takes - called by `iled_task` whenever
/// it (re)builds the I2S driver (at startup, and after any "#iled" command
/// that changes `timing`), so the log has a record of the real hardware
/// numbers behind whatever pattern/pattern_ns/reset_ns a "#iled" command
/// just asked for, without having to re-derive them by hand from this
/// module's doc comments.
///
/// `bclk_hz`/`mclk_hz` are exact - `i2s_std_calculate_clock` (ESP-IDF)
/// computes both as plain integer multiples of `sample_rate_hz` (`* 16` for
/// 2 slots of 8 bits, `* I2S_MCLK_MULTIPLE`), so there's no rounding to
/// report there. The one place real rounding happens is deriving `mclk_hz`
/// from the fixed `I2S_SOURCE_CLOCK_HZ` PLL: ESP-IDF programs an 8-bit
/// integer + 9-bit/9-bit fractional divider (`I2S_LL_CLK_FRAC_DIV_N_MAX`/
/// `_AB_MAX`, both 256/512 on ESP32-C3) to approximate `mclk_div` as closely
/// as that divider can - reported here as the ideal (unrounded) ratio, since
/// that divider's own achievable error is on the order of parts-per-million
/// (see the "resolution of the timing values" discussion this followed) and
/// not worth reproducing ESP-IDF's own fractional-search algorithm just to
/// log.
fn log_timing_report(timing: &TimingConfig) {
    let render = |pattern: u8| -> String {
        (0..timing.resolution).rev().map(|i| if (pattern >> i) & 1 == 1 { '1' } else { '0' }).collect()
    };

    let bit_rate_hz = timing.bit_rate_hz();
    let sample_rate_hz = timing.sample_rate_hz();
    let bclk_hz = sample_rate_hz * 16;
    let mclk_hz = sample_rate_hz * I2S_MCLK_MULTIPLE;
    let mclk_div = I2S_SOURCE_CLOCK_HZ as f64 / mclk_hz as f64;

    log::info!(
        "iLED timing: pattern \"1\"={} \"0\"={} (resolution {}), {}ns/pattern, {}ns reset -> bit rate {}Hz, I2S sample rate {}Hz -> BCLK {}Hz, MCLK {}Hz (= {}MHz source / {mclk_div:.4}); DMA buffer {FRAME_LEN} bytes/write",
        render(timing.pattern_high),
        render(timing.pattern_low),
        timing.resolution,
        timing.pattern_ns,
        timing.reset_ns,
        bit_rate_hz,
        sample_rate_hz,
        bclk_hz,
        mclk_hz,
        I2S_SOURCE_CLOCK_HZ / 1_000_000,
    );
}

/// Drives a chain of WS2812-compatible LEDs on GPIO2 through the I2S TX
/// peripheral's DMA engine instead of bit-banging: each refresh's whole
/// bit-encoded frame (see `render_led`/`Framebuffer::render`) goes out in one
/// `write_all_async` call, timed entirely by I2S's own DMA-fed shift clock.
/// The driver (built by `build_i2s`) is `msb_slot_default(Bits8,
/// SlotMode::Mono)` (msb, no bit-shift, so the byte stream serializes out
/// unmodified) at a sample rate that makes `TimingConfig::bit_rate_hz` come
/// out right - `sample_rate_hz * 2 * 8 == bit_rate_hz`. That `* 2` is there
/// even though this is Mono: ESP-IDF's std driver always clocks BCLK for 2
/// slots' worth of bits per WS period regardless of `slot_mode` (only the
/// *DMA buffer packing* actually shrinks for Mono), so assuming
/// `sample_rate_hz * 8 * 1` (this was tried, back when the bit rate was a
/// fixed constant) doubles the real bit rate, squeezing every pulse into
/// half its intended width - dim, mostly-unlit colors on real hardware,
/// since a too-short "1" pulse mostly reads back as "0". Mono specifically,
/// not stereo: with 2 active slots the DMA buffer interleaves our stream
/// with a second slot we never intended (this was also tried, separately,
/// and corrupted colors worse down the chain until it desynced badly enough
/// to look like an early reset). Mono's single active slot means the DMA
/// buffer really is our byte stream verbatim. `auto_clear`: I2S TX's DMA is
/// continuous - it doesn't idle between `write_all_async` calls, it keeps
/// shifting out whatever's in its DMA buffers; without this, any bytes in a
/// buffer not just overwritten keep whatever an *earlier* write left there
/// instead of reading as reset/idle. `frames_per_buffer(FRAME_LEN)`: sizing
/// each DMA buffer to exactly one frame makes every `write_all_async` call
/// start and end precisely on a buffer boundary, so no frame's data can ever
/// straddle one and corrupt (or skip) a LED's update - both of these were
/// real, previously-diagnosed bugs on real hardware, not just theoretical
/// concerns. BCLK and WS are real pins that mode requires wiring up in the
/// driver config, but carry nothing this protocol cares about - only DOUT
/// (GPIO2, wired to the chain's `Din`) matters electrically.
///
/// Mirrors the shared nodem DOM framebuffer (see `sample_from_dom`) onto the
/// LED chain, checking every 100ms whether there's anything new to show -
/// unlike `nodem_task`'s 60Hz redraw of that same buffer for the OLED,
/// there's no need to match that rate here: even the largest `IledConfig`
/// this supports (`MAX_LEDS`) is far coarser than the OLED's real
/// resolution, so anything actually worth showing on this chain changes far
/// slower than the DOM itself gets redrawn. Two independent things can make
/// a check into an actual redraw: `Global::display_buffer_dirty[DISPLAY_BUFFER_ILED]`
/// (cleared here once a frame's actually been sent - this task's *own* slot,
/// not shared with `oled_task`'s `DISPLAY_BUFFER_OLED` one, since a single
/// shared flag meant whichever of the two cleared it first starved the
/// other; see `DISPLAY_BUFFER_CONSUMERS`'s doc comment) says `nodem_task`
/// has re-rendered since the last check; comparing the freshly-read
/// `Global::iled_config` against the last one this used says a "#iled"
/// command has changed the chain's own shape/color/timing since then.
/// Either on its own is reason enough - a config change should show up
/// immediately, not wait on the DOM's own redraw cadence, and vice versa. If
/// specifically `timing` changed, the driver also gets rebuilt (see
/// `build_i2s`) at the new bit rate before that redraw goes out, so the very
/// next frame is already sent at the right speed. `Global::iled_config`
/// itself starts out as `Some(IledConfig::default())` (set in
/// `Global::new()`, before this task or `nvs` even exist yet), so this loads
/// whatever's actually persisted in `NVS_KEY_CONFIG` over that default
/// before the main loop starts, the same way `wifi_task`/`websocket_task`
/// restore their own settings from NVS at startup - `read_iled_config` never
/// itself produces `None`, only a bare "#iled" command does that later (see
/// `command_listener`).
///
/// `Global::iled_config` going from `Some` to `None` (feature disabled) is
/// handled specially: rather than just stop writing and leave the chain
/// frozen showing whatever it last displayed, one further frame goes out
/// with every pixel off (using the *last* known `config`'s shape/off_color/
/// timing/color_pattern, since the new state carries none of that) so the
/// chain actually goes dark, and only then does the loop stop touching I2S
/// until re-enabled. `enabled` (not `config` itself) tracks this - `config`
/// keeps holding the last known real config throughout, both so that "off"
/// frame can be built and so that a later `Some` re-enabling the feature can
/// still tell whether `timing` actually changed since the driver was last
/// (re)built, exactly as it would across any other config change.
pub async fn iled_task(global: Rc<RefCell<Global>>, nvs: EspDefaultNvsPartition) {
    let mut config = match EspNvs::new(nvs, NVS_NAMESPACE, true) {
        Ok(device_nvs) => read_iled_config(&device_nvs),
        Err(e) => {
            log::error!("Failed to open '{NVS_NAMESPACE}' NVS namespace: {e:?}");
            IledConfig::default()
        }
    };
    global.borrow_mut().iled_config = Some(config);

    let mut i2s = match build_i2s(config.timing.sample_rate_hz()) {
        Ok(i2s) => i2s,
        Err(e) => {
            log::error!("Failed to build iLED I2S driver: {e:?}");
            return;
        }
    };
    log_timing_report(&config.timing);

    // Lives for this task's whole lifetime and gets reused every refresh
    // (`Framebuffer::render` writes into it rather than returning a new one)
    // - see `render`'s doc comment for why a few-KB buffer like this
    // shouldn't be copied around by value on every 100ms tick.
    let mut frame = [0u8; FRAME_LEN];
    let mut enabled = true;

    loop {
        Timer::after_millis(1000 / 30).await;

        let (dirty, new_config) = {
            let g = global.borrow();
            (g.display_buffer_dirty[DISPLAY_BUFFER_ILED], g.iled_config)
        };

        let Some(new_config) = new_config else {
            if enabled {
                let fb = Framebuffer::blank(config);
                fb.render(&mut frame);
                if let Err(e) = i2s.write_all_async(&frame).await {
                    log::error!("iLED I2S write failed while disabling: {e:?}");
                }
                enabled = false;
            }
            global.borrow_mut().display_buffer_dirty[DISPLAY_BUFFER_ILED] = false;
            continue;
        };

        let config_changed = !enabled || new_config != config;
        let timing_changed = new_config.timing != config.timing;
        config = new_config;
        enabled = true;

        if !dirty && !config_changed {
            continue;
        }

        if timing_changed {
            drop(i2s);
            i2s = match build_i2s(config.timing.sample_rate_hz()) {
                Ok(i2s) => i2s,
                Err(e) => {
                    log::error!("Failed to rebuild iLED I2S driver after a timing change: {e:?}");
                    return;
                }
            };
            log_timing_report(&config.timing);
        }

        let mut fb = Framebuffer::blank(config);

        {
            let g = global.borrow();
            sample_from_dom(&mut fb, &config, &g);
        }

        fb.render(&mut frame);
        if let Err(e) = i2s.write_all_async(&frame).await {
            log::error!("iLED I2S write failed: {e:?}");
        }

        global.borrow_mut().display_buffer_dirty[DISPLAY_BUFFER_ILED] = false;
    }
}
