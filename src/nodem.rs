use std::cell::RefCell;
use std::rc::Rc;

use embassy_time::{Duration, Instant, Timer};
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs};
use esp_idf_svc::sys::{self, esp_partition_mmap_handle_t};

use nodem_rs::media::Ret;
use nodem_rs::runtime::Runtime;

use crate::command_listener::{CommandListener, PKG_PARTITION_LABEL};
use crate::global::{CloudConnectionStatus, Global, WifiConnectionStatus, DISPLAY_BUFFER_CONSUMERS};

/// How long Wi-Fi and cloud both need to have been continuously connected
/// before the status overlay stops being drawn.
const HIDE_REPORT_AFTER: Duration = Duration::from_secs(5);

// pub(crate): written by `CommandListener` on every "page=<value>" line,
// removed by "#factory".
pub(crate) const NVS_NAMESPACE: &str = "nodem";
pub(crate) const NVS_KEY_LAST_PAGE: &str = "last_page";
// pub(crate): read by `main` (via `read_nodem_config`) before `Global::new`
// sizes the framebuffer, written by "#nodem"/"#factory" and shown by "#cfg" -
// see `NodemConfig`.
pub(crate) const NVS_KEY_CONFIG: &str = "nodem";

/// Upper bound on either `NodemConfig` dimension - keeps the framebuffer
/// (`width * height / 8` bytes at 1 bit/pixel) to a sane heap allocation and
/// every coordinate well within `nodem_rs`'s `PosX`/`PosY` (`i16`).
const MAX_DIMENSION: u16 = 1024;

/// Shape of the nodem DOM framebuffer (`Global::display_buffer`), persisted as
/// `NVS_KEY_CONFIG` in the form "<width>:<height>:<bits_per_pixel>", e.g.
/// "128:64:1". Only 1 bit/pixel is supported for now. The framebuffer and the
/// DOM's `Surface` are sized from it in `Global::new` at boot, and resized
/// live by `Global::resize_display` whenever "#nodem"/"#factory" writes it.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct NodemConfig {
    pub width: u16,
    pub height: u16,
    pub bits_per_pixel: u8,
}

impl Default for NodemConfig {
    fn default() -> Self {
        Self { width: 128, height: 64, bits_per_pixel: 1 }
    }
}

impl NodemConfig {
    /// `None` for anything but exactly three colon-separated fields, a
    /// `width`/`height` outside `1..=MAX_DIMENSION`, or a `bits_per_pixel`
    /// other than 1.
    pub(crate) fn parse(s: &str) -> Option<Self> {
        let mut fields = s.split(':').map(str::trim);
        let width: u16 = fields.next()?.parse().ok()?;
        let height: u16 = fields.next()?.parse().ok()?;
        let bits_per_pixel: u8 = fields.next()?.parse().ok()?;
        if fields.next().is_some() {
            return None;
        }

        if !(1..=MAX_DIMENSION).contains(&width) || !(1..=MAX_DIMENSION).contains(&height) || bits_per_pixel != 1 {
            return None;
        }

        Some(Self { width, height, bits_per_pixel })
    }

    pub(crate) fn to_nvs_string(&self) -> String {
        format!("{}:{}:{}", self.width, self.height, self.bits_per_pixel)
    }

    /// Framebuffer size in bytes - pixels are packed contiguously, row after
    /// row with no per-row padding (see `nodem_rs::Surface::draw_pixel`).
    pub(crate) fn buffer_len(&self) -> usize {
        (self.width as usize * self.height as usize * self.bits_per_pixel as usize).div_ceil(8)
    }
}

/// Reads `NVS_KEY_CONFIG` back into a `NodemConfig`, the same way
/// `iled::read_iled_config` does its own: a missing value (first boot) or an
/// unparseable one is replaced in NVS by `NodemConfig::default()`, so "#cfg"
/// shows what's actually in use.
pub(crate) fn read_nodem_config(nvs: EspDefaultNvsPartition) -> NodemConfig {
    let nvs = match EspNvs::new(nvs, NVS_NAMESPACE, true) {
        Ok(nvs) => nvs,
        Err(e) => {
            log::error!("Failed to open '{NVS_NAMESPACE}' NVS namespace: {e:?}");
            return NodemConfig::default();
        }
    };

    let mut buf = [0u8; 32];
    let raw = nvs.get_str(NVS_KEY_CONFIG, &mut buf).ok().flatten();

    if let Some(config) = raw.and_then(NodemConfig::parse) {
        return config;
    }

    if let Some(raw) = raw {
        log::warn!("Malformed '{NVS_KEY_CONFIG}' in NVS ('{raw}'), falling back to default");
    }

    let config = NodemConfig::default();

    if let Err(e) = nvs.set_str(NVS_KEY_CONFIG, &config.to_nvs_string()) {
        log::error!("Failed to persist default '{NVS_KEY_CONFIG}' to NVS: {e:?}");
    }

    config
}

/// Advances the `nodem_rs` DOM runtime, 60 times a second, keeping its
/// `status_message` popup set to the current Wi-Fi/cloud status - the runtime
/// itself renders it into `Global::display_buffer` as part of `run()`. Only
/// touches the in-memory framebuffer (marking it dirty for every `Global::display_buffer_dirty`
/// consumer - `oled_task` and `iled_task` - at once) - never talks to a
/// display itself.
///
/// The overlay is suppressed once Wi-Fi and cloud have both been connected for
/// `HIDE_REPORT_AFTER` straight - `connected_since` tracks the start of the
/// current unbroken "both connected" streak (reset to `None` the moment either
/// drops), so a hidden report reappears immediately on any disconnect. It also
/// stays up for as long as `Global::sys_status.rollback_pending` - i.e. until
/// `heartbeat_task` has marked a freshly OTA-updated firmware valid.
///
/// Before the first frame, the pkg in the "pkg" partition (if any - written
/// by a "pkg" upload) is loaded, and the page last shown (`NVS_KEY_LAST_PAGE`, see `CommandListener`'s
/// "page=" handling) restored by running "page=<last_page>" through
/// `process_command`, just as if it had been received over UART/websocket.
/// Every later "pkg" upload is loaded as soon as it's written
/// (`Global::pkg_reload`), without a restart - the page isn't restored then.
pub async fn nodem_task(global: Rc<RefCell<Global>>, nvs: EspDefaultNvsPartition) {
    let mut connected_since: Option<Instant> = None;
    let mut pkg_mapping: Option<*const u8> = None;

    {
        let mut g = global.borrow_mut();
        let loaded = load_pkg_partition(&mut g, &mut pkg_mapping);

        let mut buf = [0u8; 8];
        let last_page = EspNvs::new(nvs.clone(), NVS_NAMESPACE, false).ok().and_then(|nvs| nvs.get_str(NVS_KEY_LAST_PAGE, &mut buf).ok().flatten().map(str::to_string));
        if let Some(page) = last_page.filter(|_| loaded) {
            let mut listener = CommandListener::new(nvs.clone());
            let mut response = String::new();
            let command = format!("page={page}\n");
            let _ = g.runtime.process_command(command.as_bytes(), &mut response, &mut listener);
            log::info!("restored {}: {}", command.trim(), response.trim());
        }
    }

    loop {
        {
            let mut g = global.borrow_mut();

            if core::mem::take(&mut g.pkg_reload) {
                load_pkg_partition(&mut g, &mut pkg_mapping);
            }

            let both_connected = matches!(g.wifi_status.status, WifiConnectionStatus::Connected)
                && matches!(g.cloud_status.connection, CloudConnectionStatus::Connected);

            connected_since = match (both_connected, connected_since) {
                (true, since @ Some(_)) => since,
                (true, None) => Some(Instant::now()),
                (false, _) => None,
            };

            let show_report = g.sys_status.rollback_pending
                || connected_since.map_or(true, |since| since.elapsed() < HIDE_REPORT_AFTER);

            let message = show_report.then(|| format!("{}{{br}}{}{{br}}{}", g.sys_status, g.wifi_status, g.cloud_status));
            set_status_message(&mut g.runtime, message);

            g.runtime.run();

            g.display_buffer_dirty = [true; DISPLAY_BUFFER_CONSUMERS];
        }

        Timer::after_millis(1000 / 60).await;
    }
}

/// Points `runtime.status_message` at `message`, but only swaps in a new
/// string when the text actually changed - the status only changes on
/// Wi-Fi/cloud events, not every frame. `DOM::status_message` borrows a
/// `&'static str`, so each new message is leaked into one and the previous
/// one freed here once `runtime` no longer points at it.
fn set_status_message(runtime: &mut nodem_rs::runtime::DOM<'static>, message: Option<String>) {
    if runtime.status_message == message.as_deref() {
        return;
    }

    let old = core::mem::replace(&mut runtime.status_message, message.map(|m| &*Box::leak(m.into_boxed_str())));

    if let Some(old) = old {
        // SAFETY: every `status_message` is set only here, from a
        // `Box::leak`ed `Box<str>`, and `runtime` - its sole holder - has just
        // been pointed elsewhere, so nothing else still references it.
        drop(unsafe { Box::from_raw(old as *const str as *mut str) });
    }
}

/// Loads the pkg stored in the "pkg" partition, if there is one, into the
/// live `Media` (as source 2). Layout: b"PKG0" magic, then a native-endian u32
/// giving the pkg's total length (magic included) - see `Media::load_pkg`,
/// which CRCs every byte up to that length.
///
/// The loaded `Media` points straight into flash, so the partition is
/// memory-mapped once - the whole of it, on first use - and then stays mapped
/// for the rest of the process's life (`mapping`), with every later load
/// reusing it: a pkg upload writes the same flash region again, and ESP-IDF
/// keeps the mapped view in sync with such writes. Not a fresh mapping per
/// load, releasing the previous one: ESP-IDF hands out the *existing* mapping
/// for a region that's already mapped (without a handle of its own), so
/// releasing the "old" one unmapped the new pkg too - an MMU fault on the
/// next access.
fn load_pkg_partition(g: &mut Global, mapping: &mut Option<*const u8>) -> bool {
    let Ok(label) = std::ffi::CString::new(PKG_PARTITION_LABEL) else { return false; };
    let partition = unsafe {
        sys::esp_partition_find_first(sys::esp_partition_type_t_ESP_PARTITION_TYPE_DATA, sys::esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_ANY, label.as_ptr())
    };
    let Some(partition_size) = (unsafe { partition.as_ref() }).map(|p| p.size as usize) else { return false; };

    let mut header = [0u8; 8];
    if let Err(e) = sys::esp!(unsafe { sys::esp_partition_read(partition, 0, header.as_mut_ptr() as *mut _, header.len()) }) {
        log::error!("'{PKG_PARTITION_LABEL}' partition read failed: {e:?}");
        return false;
    }
    if &header[0..4] != b"PKG0" {
        log::info!("'{PKG_PARTITION_LABEL}' partition has no pkg, skipping load");
        return false;
    }
    let len = u32::from_ne_bytes(header[4..8].try_into().unwrap()) as usize;
    if len > partition_size {
        log::error!("'{PKG_PARTITION_LABEL}': pkg header claims {len} bytes, partition only {partition_size}");
        return false;
    }

    let ptr = match *mapping {
        Some(ptr) => ptr,
        None => {
            let mut ptr: *const core::ffi::c_void = core::ptr::null();
            // Never unmapped - see above.
            let mut handle: esp_partition_mmap_handle_t = 0;
            if let Err(e) = sys::esp!(unsafe { sys::esp_partition_mmap(partition, 0, partition_size, sys::esp_partition_mmap_memory_t_ESP_PARTITION_MMAP_DATA, &mut ptr, &mut handle) }) {
                log::error!("'{PKG_PARTITION_LABEL}' partition mmap failed: {e:?}");
                return false;
            }
            *mapping = Some(ptr as *const u8);
            ptr as *const u8
        }
    };

    let ret = g.runtime.surface.media.load_pkg(ptr, len, 2);
    let loaded = matches!(ret, Ret::Ok);
    log::info!("pkg load: {}", ret as u8);
    loaded
}
