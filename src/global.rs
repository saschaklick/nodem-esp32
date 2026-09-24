use std::net::Ipv4Addr;

use esp_idf_svc::ws::client::EspWebSocketClient;
use nodem_rs::runtime::DOM;

use crate::iled::IledConfig;
use crate::nodem::NodemConfig;
use crate::oled::OledConfig;

const COMMAND_BUF_LEN: usize = 1024;

/// One dirty bit per `Global::display_buffer` consumer, rather than a single
/// shared flag - `oled_task` and `iled_task` each redraw at their own pace,
/// and with just one flag, whichever of them clears it first (in practice
/// always `oled_task`, since it polls far more often) clears it out from
/// under the other before it gets a chance to see it set. That was tried:
/// `iled_task`'s own, much slower ~250ms-interval check would almost always
/// land after `oled_task` had already cleared the shared flag, making
/// `iled_task`'s updates look sporadic even though `nodem_task` was
/// re-rendering constantly. Add a consumer here (and a matching index
/// constant) for any future task that also needs to know when
/// `display_buffer` has changed.
pub const DISPLAY_BUFFER_CONSUMERS: usize = 2;
pub const DISPLAY_BUFFER_OLED: usize = 0;
pub const DISPLAY_BUFFER_ILED: usize = 1;

/// Where `wifi_task` currently is in its connect loop.
#[derive(Clone)]
pub enum WifiConnectionStatus {
    WaitingForConfiguration,
    Connecting,
    Connected,
    Failed(String),
}

/// Everything `wifi_task` knows about the current Wi-Fi link - updated by
/// `wifi_task` on every state change, read by `nodem_task`/`heartbeat_task`
/// for display and by `websocket_task` (via `status`) to know when it's safe
/// to start connecting. `ip`/`subnet_mask`/`gateway`/`rssi` are only ever
/// `Some` while `status` is `Connected`.
#[derive(Clone)]
pub struct WifiStatus {
    pub status: WifiConnectionStatus,
    pub ssid: Option<String>,
    pub ip: Option<Ipv4Addr>,
    pub subnet_mask: Option<Ipv4Addr>,
    pub gateway: Option<Ipv4Addr>,
    pub rssi: Option<i32>,
}

impl WifiStatus {
    pub fn new() -> Self {
        Self {
            status: WifiConnectionStatus::WaitingForConfiguration,
            ssid: None,
            ip: None,
            subnet_mask: None,
            gateway: None,
            rssi: None,
        }
    }
}

impl std::fmt::Display for WifiStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match &self.status {
            WifiConnectionStatus::WaitingForConfiguration => write!(f, "Wifi: waiting for config"),
            WifiConnectionStatus::Connecting => write!(f, "Wifi: connecting..."),
            WifiConnectionStatus::Connected => write!(f, "Wifi: {}", self.ip.map(|ip| ip.to_string()).unwrap_or_default()),
            WifiConnectionStatus::Failed(e) => write!(f, "Wifi: failed ({e})"),
        }
    }
}

/// Where `websocket_task` currently is in registering the device and keeping
/// the websocket connection alive - kept as two independent axes (unlike
/// `WifiConnectionStatus`) since a device stays `Registered` across every
/// later websocket reconnect.
#[derive(Clone)]
pub enum RegistrationStatus {
    WaitingForCode,
    Registering,
    Registered,
    Failed(String),
}

#[derive(Clone)]
pub enum CloudConnectionStatus {
    Disconnected,
    Connecting,
    Connected,
}

/// Everything `websocket_task` knows about the current cloud connection -
/// updated by `websocket_task` on every state change, read by
/// `nodem_task`/`heartbeat_task` for display.
#[derive(Clone)]
pub struct CloudStatus {
    pub host: Option<String>,
    pub device_name: Option<String>,
    pub registration: RegistrationStatus,
    pub connection: CloudConnectionStatus,
}

impl CloudStatus {
    pub fn new() -> Self {
        Self {
            host: None,
            device_name: None,
            registration: RegistrationStatus::WaitingForCode,
            connection: CloudConnectionStatus::Disconnected,
        }
    }
}

impl std::fmt::Display for CloudStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match &self.registration {
            RegistrationStatus::WaitingForCode => write!(f, "Reg: waiting for code"),
            RegistrationStatus::Registering => write!(f, "Reg: registering..."),
            RegistrationStatus::Failed(e) => write!(f, "Reg: failed ({e})"),
            RegistrationStatus::Registered => {
                match self.connection {
                    CloudConnectionStatus::Disconnected => write!(f, "WS: disconnected")?,
                    CloudConnectionStatus::Connecting => write!(f, "WS: connecting...")?,
                    CloudConnectionStatus::Connected => write!(f, "WS: connected")?,
                }
                if let Some(device_name) = &self.device_name {
                    write!(f, " ({device_name})")?;
                }
                Ok(())
            }
        }
    }
}

/// The running firmware itself - filled in by `heartbeat_task` at startup
/// (`ota_slot`/`rollback_pending`) and again once it ends the rollback phase,
/// read by `nodem_task` for the status popup.
#[derive(Clone)]
pub struct SysStatus {
    pub version: &'static str,
    // 0/1 for `ota_0`/`ota_1`, `None` until known (or if the running
    // partition isn't an OTA slot at all).
    pub ota_slot: Option<u8>,
    // The running image is still unverified - the bootloader will roll back
    // to the previous slot on the next reset unless it gets marked valid.
    pub rollback_pending: bool,
}

impl SysStatus {
    pub fn new() -> Self {
        Self {
            version: env!("CARGO_PKG_VERSION"),
            ota_slot: None,
            rollback_pending: false,
        }
    }
}

impl std::fmt::Display for SysStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Sys: {} (", self.version)?;
        match self.ota_slot {
            Some(slot) => write!(f, "{slot}")?,
            None => write!(f, "?")?,
        }
        if self.rollback_pending {
            write!(f, "r")?;
        }
        write!(f, ")")
    }
}

/// Where `oled_task` currently is with the SSD1306 - `Failed` after the
/// initial `init()` failed lasts until `Global::oled_config` changes,
/// whereas after a failed reinitialization the next flush simply tries again.
/// `Disabled` while `Global::oled_config` is `None`.
#[derive(Clone)]
pub enum OledConnectionStatus {
    Disabled,
    Initializing,
    Connected,
    Failed(String),
}

/// Everything `oled_task` knows about the display - updated by `oled_task`,
/// read by `heartbeat_task` for logging. `errors` counts every failed flush
/// attempt (each one of up to `MAX_ATTEMPTS` per frame), `reinits` every time
/// the display had to be reinitialized after all of a frame's attempts failed.
#[derive(Clone)]
pub struct OledStatus {
    pub status: OledConnectionStatus,
    pub errors: u32,
    pub reinits: u32,
}

impl OledStatus {
    pub fn new() -> Self {
        Self {
            status: OledConnectionStatus::Initializing,
            errors: 0,
            reinits: 0,
        }
    }
}

impl std::fmt::Display for OledStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match &self.status {
            OledConnectionStatus::Disabled => write!(f, "OLED: disabled"),
            OledConnectionStatus::Initializing => write!(f, "OLED: initializing..."),
            OledConnectionStatus::Connected => write!(f, "OLED: connected"),
            OledConnectionStatus::Failed(e) => write!(f, "OLED: failed ({e})"),
        }
    }
}

/// Shared, cross-task state. The executor is single-threaded and cooperative,
/// so `Rc<RefCell<_>>` (no atomics/locking needed) is enough - only one task
/// ever runs at a time.
pub struct Global {
    pub sys_status: SysStatus,
    pub wifi_status: WifiStatus,
    pub cloud_status: CloudStatus,
    pub oled_status: OledStatus,
    // Boxed so the heap allocation's address - which `runtime`'s `Surface` holds
    // a raw pointer into (see `nodem_rs::Surface::framebuffer`) - stays fixed
    // even as `Global` itself gets moved around (e.g. into the `Rc<RefCell<_>>`
    // in `main`). A plain `[u8; N]` field would move (and thus dangle the
    // pointer) right along with it.
    // Sized from `nodem_config` - see `NodemConfig::buffer_len`.
    pub display_buffer: Box<[u8]>,
    // Shape of `display_buffer` - read from NVS in `main`, changed at
    // runtime only through `resize_display`. See `display_pixel`.
    pub nodem_config: NodemConfig,
    // Set (every element at once) by `nodem_task` whenever it re-renders
    // into `display_buffer`; index `DISPLAY_BUFFER_OLED`/`DISPLAY_BUFFER_ILED`
    // cleared independently by `oled_task`/`iled_task` respectively, each
    // once it has flushed that frame to its own display - see
    // `DISPLAY_BUFFER_CONSUMERS`'s doc comment for why this isn't one shared
    // flag.
    pub display_buffer_dirty: [bool; DISPLAY_BUFFER_CONSUMERS],
    // `'static`: `runtime.status_message` borrows a string that `Global`
    // can't own alongside it - `nodem_task` leaks each new status text and
    // frees the previous one itself, see `set_status_message` there.
    pub runtime: DOM<'static>,
    // Incoming-command staging buffer for `uart_task`: bytes read off the UART
    // accumulate here until `runtime.process_command` has consumed a full
    // command; `command_buf_pos` is how much of it is currently filled.
    pub command_buf: [u8; COMMAND_BUF_LEN],
    pub command_buf_pos: usize,
    pub wifi_reconnect: bool,
    pub reregister: bool,
    // Read fresh by `iled_task` every refresh; written by `uart_task`/
    // `websocket_task` right after a "#iled" command, same pattern as
    // `cloud_status.device_name` above. `None` means the feature is
    // disabled - a bare "#iled" (no fields at all) both clears
    // `iled::NVS_KEY_CONFIG` and sets this to `None`, rather than falling
    // back to `IledConfig::default()` the way every individually-blank
    // field does - see `command_listener`'s "#iled" handling.
    pub iled_config: Option<IledConfig>,
    // Loaded from `oled::NVS_KEY_CONFIG` by `oled_task` at startup, then
    // written by `uart_task`/`websocket_task` right after a "#oled"/"#factory",
    // same pattern as `iled_config`. `None` means the display is disabled;
    // `oled_task` re-initializes it whenever this changes.
    pub oled_config: Option<OledConfig>,
    // Set (mirrored from `CommandListener::take_pkg_updated`) by `uart_task`/
    // `websocket_task` right after a "pkg" upload has been written to the
    // "pkg" partition, same pattern as `iled_config` above. Taken by
    // `nodem_task`, which owns the `Surface`/`Media` the pkg is loaded into.
    pub pkg_reload: bool,
    // The cloud websocket while `websocket_task` has one - kept here, not in
    // that task, so `websocket::close_websocket` can close it cleanly from
    // wherever Wi-Fi is about to go down.
    pub ws_client: Option<EspWebSocketClient<'static>>,
}

impl Global {
    pub fn new(nodem_config: NodemConfig) -> Self {
        let mut display_buffer = vec![0u8; nodem_config.buffer_len()].into_boxed_slice();
        let runtime = DOM::new(
            &mut display_buffer[..],
            nodem_config.width,
            nodem_config.height,
        );

        Self {
            sys_status: SysStatus::new(),
            wifi_status: WifiStatus::new(),
            cloud_status: CloudStatus::new(),
            oled_status: OledStatus::new(),
            display_buffer,
            nodem_config,
            display_buffer_dirty: [false; DISPLAY_BUFFER_CONSUMERS],
            runtime,
            command_buf: [0u8; COMMAND_BUF_LEN],
            command_buf_pos: 0,
            wifi_reconnect: false,
            reregister: false,
            iled_config: Some(IledConfig::default()),
            oled_config: None,
            pkg_reload: false,
            ws_client: None,
        }
    }

    /// Applies a new `nodem_config`. Device mappings take effect on their
    /// own (each driver's `DriverView` reads them every frame); only if the
    /// geometry changed is a fresh, blank `display_buffer` of the new size
    /// swapped in and the DOM's `Surface` pointed at it - it redraws
    /// everything every frame, so the next `run()` renders the current page
    /// at the new size. The new buffer is pointed to before the old one is
    /// dropped, so `Surface` never holds a dangling pointer.
    pub fn resize_display(&mut self, nodem_config: NodemConfig) {
        if nodem_config.same_geometry(&self.nodem_config) {
            self.nodem_config = nodem_config;
            return;
        }
        let mut display_buffer = vec![0u8; nodem_config.buffer_len()].into_boxed_slice();
        self.runtime.surface.resize(&mut display_buffer[..], nodem_config.width, nodem_config.height);
        self.display_buffer = display_buffer;
        self.nodem_config = nodem_config;
        self.display_buffer_dirty = [true; DISPLAY_BUFFER_CONSUMERS];
        log::info!("framebuffer resized to {}", nodem_config.to_nvs_string());
    }

    /// Whether pixel `(x, y)` of `display_buffer` is set - 1 bit/pixel,
    /// packed row-major and MSB-first exactly as `nodem_rs::Surface` draws
    /// it. Anything outside `nodem_config`'s `width`x`height` reads as off,
    /// negative coordinates included - see `driver::DriverView`, the only
    /// caller, which maps a driver's pixels into here.
    pub fn display_pixel(&self, x: i32, y: i32) -> bool {
        if x < 0 || y < 0 || x >= self.nodem_config.width as i32 || y >= self.nodem_config.height as i32 {
            return false;
        }
        let i = y as usize * self.nodem_config.width as usize + x as usize;
        (self.display_buffer[i / 8] >> (7 - i % 8)) & 1 != 0
    }
}
