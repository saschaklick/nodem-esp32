use std::net::Ipv4Addr;

use nodem_rs::runtime::DOM;

use crate::iled::IledConfig;

// pub(crate): `iled_task` downsamples `Global::display_buffer` into its own
// framebuffer using these same dimensions - see its doc comment.
pub(crate) const NODEM_DISPLAY_WIDTH: u32 = 128;
pub(crate) const NODEM_DISPLAY_HEIGHT: u32 = 64;
const NODEM_BUFFER_LEN: usize = (NODEM_DISPLAY_WIDTH * NODEM_DISPLAY_HEIGHT / 8) as usize;

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

/// Shared, cross-task state. The executor is single-threaded and cooperative,
/// so `Rc<RefCell<_>>` (no atomics/locking needed) is enough - only one task
/// ever runs at a time.
pub struct Global {
    pub wifi_status: WifiStatus,
    pub cloud_status: CloudStatus,
    // Boxed so the heap allocation's address - which `runtime`'s `Surface` holds
    // a raw pointer into (see `nodem_rs::Surface::framebuffer`) - stays fixed
    // even as `Global` itself gets moved around (e.g. into the `Rc<RefCell<_>>`
    // in `main`). A plain `[u8; N]` field would move (and thus dangle the
    // pointer) right along with it.
    pub display_buffer: Box<[u8; NODEM_BUFFER_LEN]>,
    // Set (every element at once) by `nodem_task` whenever it re-renders
    // into `display_buffer`; index `DISPLAY_BUFFER_OLED`/`DISPLAY_BUFFER_ILED`
    // cleared independently by `oled_task`/`iled_task` respectively, each
    // once it has flushed that frame to its own display - see
    // `DISPLAY_BUFFER_CONSUMERS`'s doc comment for why this isn't one shared
    // flag.
    pub display_buffer_dirty: [bool; DISPLAY_BUFFER_CONSUMERS],
    pub runtime: DOM,
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
    // Set (mirrored from `CommandListener::take_pkg_reload`) by `uart_task`/
    // `websocket_task` right after a "#pkg" upload finishes flashing, same
    // pattern as `iled_config`/`cloud_status.device_name` above. Taken by
    // `nodem_task`, which actually owns the `Surface`/`Media` a pkg needs to
    // be loaded into - the pointer/length of the upload's memory-mapped flash
    // region, valid forever (see `CommandListener::process_loader_end`).
    pub pkg_reload: Option<(*const u8, usize)>,
}

impl Global {
    pub fn new() -> Self {
        let mut display_buffer = Box::new([0u8; NODEM_BUFFER_LEN]);
        let runtime = DOM::new(
            &mut display_buffer[..],
            NODEM_DISPLAY_WIDTH as _,
            NODEM_DISPLAY_HEIGHT as _,
        );

        Self {
            wifi_status: WifiStatus::new(),
            cloud_status: CloudStatus::new(),
            display_buffer,
            display_buffer_dirty: [false; DISPLAY_BUFFER_CONSUMERS],
            runtime,
            command_buf: [0u8; COMMAND_BUF_LEN],
            command_buf_pos: 0,
            wifi_reconnect: false,
            reregister: false,
            iled_config: Some(IledConfig::default()),
            pkg_reload: None,
        }
    }
}
