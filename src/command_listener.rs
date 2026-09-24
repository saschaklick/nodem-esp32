use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs};
use esp_idf_svc::partition::EspPartition;
use esp_idf_svc::sys::{self, esp, esp_ota_handle_t, esp_partition_t};

use nodem_rs::{ control::{ Control, ControlMode, IControl, IControlLoader, LoaderRet }, media::Media };

use crate::global::{CloudConnectionStatus, CloudStatus, Global, OledConnectionStatus, OledStatus, RegistrationStatus, WifiConnectionStatus, WifiStatus};
use crate::iled::{self, IledConfig};
use crate::nodem::{self, NodemConfig};
use crate::oled::{self, OledConfig};
use crate::websocket;
use crate::wifi;

pub(crate) const PKG_PARTITION_LABEL: &str = "pkg";
const LOADER_CHUNK_LEN: usize = 512;

#[repr(u8)]
#[derive(PartialEq)]
enum Ret {
    Ok = 0,
    Error = 1,
    MalformedValue = 2
}

/// The command handler `process_command` invokes for each complete `#...`
/// line - shared by `uart_task` (UART bytes) and `websocket_task` (cloud
/// messages), so the same admin command set ("#wifi", "#factory", ...) works
/// over either transport. Deliberately does *not* hold an `Rc<RefCell<Global>>`:
/// both callers already hold `Global` borrowed (as `g`) for the entire
/// `process_command` call this feeds into, and a second `.borrow_mut()` on the
/// same `RefCell` from in here would panic ("already borrowed") - it did,
/// before this was a plain `bool` instead. Callers copy these into
/// `g.wifi_reconnect`/`g.reregister` (via `take_wifi_reconnect`/
/// `take_reregister`) right after `process_command` returns, once it's safe to.
pub(crate) struct CommandListener {
    nvs: EspDefaultNvsPartition,
    wifi_reconnect: bool,
    reregister: bool,
    // Set by "#name"/"#reg" (`Some(Some(name))`) and "#factory" (`Some(None)`,
    // the name having been removed) so callers can mirror it into
    // `Global::cloud_status` right away, same reason/pattern as
    // `wifi_reconnect`/`reregister` above.
    device_name: Option<Option<String>>,
    // Set by "#reset" - taken (and acted on) by callers only once the ack
    // this produces has actually been sent, same reason/pattern as
    // `wifi_reconnect`/`reregister` above.
    restart: bool,
    // Set by "#iled" so callers can mirror it into `Global::iled_config`
    // right away, same reason/pattern as "#name"'s `device_name` above.
    // Double `Option`: the outer one is "#iled was received since the last
    // `take`" (same as every other field here); the inner one is the
    // command's actual effect - `Some(config)` to apply a new config,
    // `None` to disable the feature (a bare "#iled" - see its handling in
    // `process_line`).
    iled_config: Option<Option<IledConfig>>,
    // Set by "#nodem"/"#factory" so callers can resize the live framebuffer
    // (`Global::resize_display`) right away, same pattern as `iled_config`.
    nodem_config: Option<NodemConfig>,
    // Set by "#oled"/"#factory" so callers can mirror it into
    // `Global::oled_config` right away - same double `Option` as
    // `iled_config`, the inner `None` meaning the display is disabled.
    oled_config: Option<Option<OledConfig>>,
    // Snapshot of `Global::wifi_status`/`cloud_status`/`oled_status` and the
    // live iLED/OLED configs, refreshed by `update_status` right before each
    // `process_command` call (same reason this doesn't hold `Global` itself -
    // see the struct doc comment) so the "#stat" command below has something
    // to report.
    wifi_status: WifiStatus,
    cloud_status: CloudStatus,
    oled_status: OledStatus,
    live_oled_config: Option<OledConfig>,
    live_iled_config: Option<IledConfig>,
    // Backing store for "pkg" uploads (see `pkg_start` below).
    pkg_partition: Option<EspPartition>,
    // Absolute offset in the "pkg" partition of the next byte to be written.
    pkg_written: usize,
    // Set by `pkg_end` once a pkg upload has been fully written, for callers
    // to mirror into `Global::pkg_reload` (same reason/pattern as
    // `wifi_reconnect`/`reregister` above) - `nodem_task`, which owns the
    // live `Surface::media`, then loads it from the partition.
    pkg_updated: bool,
    // An "ota" upload in progress: the `esp_ota_begin` handle and the
    // inactive OTA slot it writes into. Uses the raw `esp_ota_*` API rather
    // than `esp_idf_svc::ota::EspOta`, which only allows a single instance at
    // a time - `heartbeat_task` holds that one - and whose `EspOtaUpdate`
    // borrows it, which a struct field here can't do.
    ota: Option<(esp_ota_handle_t, *const esp_partition_t)>,
    // Shared by both upload kinds (see `IControlLoader` below): which one is
    // in progress (`None` if its start was refused), the staging buffer the
    // loader's one-byte-at-a-time data is collected in before each
    // `pkg_write`/`ota_write`, and whether any of those writes has failed -
    // the rest of the upload is still consumed (the loader can't be stopped
    // midway), but `pkg_end`/`ota_end` then fail rather than activating it.
    loader_mode: Option<ControlMode>,
    loader_buf: [u8; LOADER_CHUNK_LEN],
    loader_buf_len: usize,
    loader_failed: bool,
}

impl CommandListener {
    pub(crate) fn new(nvs: EspDefaultNvsPartition) -> Self {
        Self {
            nvs,
            wifi_reconnect: false,
            reregister: false,
            device_name: None,
            restart: false,
            iled_config: None,
            nodem_config: None,
            oled_config: None,
            wifi_status: WifiStatus::new(),
            cloud_status: CloudStatus::new(),
            oled_status: OledStatus::new(),
            live_oled_config: None,
            live_iled_config: None,
            pkg_partition: None,
            pkg_written: 0,
            pkg_updated: false,
            ota: None,
            loader_mode: None,
            loader_buf: [0u8; LOADER_CHUNK_LEN],
            loader_buf_len: 0,
            loader_failed: false,
        }
    }

    pub(crate) fn take_wifi_reconnect(&mut self) -> bool {
        core::mem::take(&mut self.wifi_reconnect)
    }

    pub(crate) fn take_reregister(&mut self) -> bool {
        core::mem::take(&mut self.reregister)
    }

    pub(crate) fn take_device_name(&mut self) -> Option<Option<String>> {
        core::mem::take(&mut self.device_name)
    }

    pub(crate) fn take_restart(&mut self) -> bool {
        core::mem::take(&mut self.restart)
    }

    pub(crate) fn take_iled_config(&mut self) -> Option<Option<IledConfig>> {
        core::mem::take(&mut self.iled_config)
    }

    pub(crate) fn take_nodem_config(&mut self) -> Option<NodemConfig> {
        core::mem::take(&mut self.nodem_config)
    }

    pub(crate) fn take_oled_config(&mut self) -> Option<Option<OledConfig>> {
        core::mem::take(&mut self.oled_config)
    }

    pub(crate) fn take_pkg_updated(&mut self) -> bool {
        core::mem::take(&mut self.pkg_updated)
    }

    pub(crate) fn update_status(&mut self, g: &Global) {
        self.wifi_status = g.wifi_status.clone();
        self.cloud_status = g.cloud_status.clone();
        self.oled_status = g.oled_status.clone();
        self.live_oled_config = g.oled_config;
        self.live_iled_config = g.iled_config;
    }

    fn store(&self, namespace: &str, key: &str, value: &str, min_len: usize, max_len: usize, ret: &mut Ret) {
        if !(min_len..=max_len).contains(&value.len()) {
            log::error!("NVS write '{namespace}/{key}' failed: value is {} bytes, must be {min_len}..={max_len}", value.len());
            *ret = Ret::MalformedValue;
            return;
        }

        let result =
            EspNvs::new(self.nvs.clone(), namespace, true).and_then(|nvs| nvs.set_str(key, value));

        if let Err(e) = result {
            log::error!("NVS write '{namespace}/{key}' failed: {e:?}");
            *ret = Ret::Error;
        }
    }

    fn remove(&self, namespace: &str, key: &str) {
        let result =
            EspNvs::new(self.nvs.clone(), namespace, true).and_then(|nvs| nvs.remove(key).map(|_| ()));

        if let Err(e) = result {
            log::error!("NVS remove '{namespace}/{key}' failed: {e:?}");
        }
    }

    fn write_masked(&self, res: &mut dyn core::fmt::Write, namespace: &str, key: &str) -> core::fmt::Result {
        let mut buf = [0u8; 64];
        let len = EspNvs::new(self.nvs.clone(), namespace, false) .ok() .and_then(|nvs| nvs.get_str(key, &mut buf).ok().flatten()).map_or(0, str::len);
        write!(res, "{key}=")?;
        for _ in 0..len {
            res.write_char('*')?;
        }
        res.write_str("\r\n")
    }

    /// Every entry in the default NVS partition, across all namespaces
    /// (ESP-IDF's own, e.g. "nvs.net80211", included), unmasked, one
    /// "<namespace>/<key>=<value>" line each - blobs as their length only.
    /// The iterator is drained into a list first, since reading the values
    /// means opening each namespace, and released before that.
    fn write_all_entries(&self, res: &mut dyn core::fmt::Write) -> core::fmt::Result {
        let mut entries = Vec::new();
        let mut it: sys::nvs_iterator_t = core::ptr::null_mut();
        let mut err = unsafe { sys::nvs_entry_find(c"nvs".as_ptr(), core::ptr::null(), sys::nvs_type_t_NVS_TYPE_ANY, &mut it) };
        while err == sys::ESP_OK {
            let mut info = sys::nvs_entry_info_t::default();
            unsafe { sys::nvs_entry_info(it, &mut info) };
            let to_string = |s: &[core::ffi::c_char]| unsafe { core::ffi::CStr::from_ptr(s.as_ptr()) }.to_string_lossy().into_owned();
            entries.push((to_string(&info.namespace_name), to_string(&info.key), info.type_));
            err = unsafe { sys::nvs_entry_next(&mut it) };
        }
        unsafe { sys::nvs_release_iterator(it) };
        if err != sys::ESP_ERR_NVS_NOT_FOUND {
            log::error!("NVS entry iteration failed: {:?}", sys::EspError::from(err));
        }

        for (namespace, key, type_) in entries {
            let Ok(nvs) = EspNvs::new(self.nvs.clone(), &namespace, false) else {
                write!(res, "{namespace}/{key}=?\r\n")?;
                continue;
            };
            let value = match type_ {
                sys::nvs_type_t_NVS_TYPE_U8 => nvs.get_u8(&key).ok().flatten().map(|v| v.to_string()),
                sys::nvs_type_t_NVS_TYPE_I8 => nvs.get_i8(&key).ok().flatten().map(|v| v.to_string()),
                sys::nvs_type_t_NVS_TYPE_U16 => nvs.get_u16(&key).ok().flatten().map(|v| v.to_string()),
                sys::nvs_type_t_NVS_TYPE_I16 => nvs.get_i16(&key).ok().flatten().map(|v| v.to_string()),
                sys::nvs_type_t_NVS_TYPE_U32 => nvs.get_u32(&key).ok().flatten().map(|v| v.to_string()),
                sys::nvs_type_t_NVS_TYPE_I32 => nvs.get_i32(&key).ok().flatten().map(|v| v.to_string()),
                sys::nvs_type_t_NVS_TYPE_U64 => nvs.get_u64(&key).ok().flatten().map(|v| v.to_string()),
                sys::nvs_type_t_NVS_TYPE_I64 => nvs.get_i64(&key).ok().flatten().map(|v| v.to_string()),
                sys::nvs_type_t_NVS_TYPE_STR => nvs.str_len(&key).ok().flatten().and_then(|len| {
                    let mut buf = vec![0u8; len.max(1)];
                    nvs.get_str(&key, &mut buf).ok().flatten().map(str::to_string)
                }),
                sys::nvs_type_t_NVS_TYPE_BLOB => nvs.blob_len(&key).ok().flatten().map(|len| format!("<blob {len} bytes>")),
                _ => None,
            };
            write!(res, "{namespace}/{key}={}\r\n", value.as_deref().unwrap_or("?"))?;
        }
        Ok(())
    }

    fn write_plain(&self, res: &mut dyn core::fmt::Write, namespace: &str, key: &str) -> core::fmt::Result {
        let mut buf = [0u8; 256];
        let value = EspNvs::new(self.nvs.clone(), namespace, false).ok().and_then(|nvs| nvs.get_str(key, &mut buf).ok().flatten()).unwrap_or("");
        write!(res, "{key}={value}\r\n")
    }
}

impl IControl for CommandListener {
    fn process_line(&mut self, line: &str, _media: &Media, res: &mut dyn core::fmt::Write) -> (bool, core::fmt::Result) {
        let prefix = "#";
        if line.starts_with(prefix){
            let mut split = line[prefix.len()..].splitn(2, ',');
            let mut ret = Ret::Ok;
            let command = split.next().unwrap_or("").trim();
            let args = split.next().unwrap_or("");
            let line = &mut args.split(',');
            let arg_0 = line.next().unwrap_or("").trim();
            let arg_1 = line.next().unwrap_or("").trim();
            match command {
                "reset" => { self.restart = true; }
                "factory" => {
                    self.remove(wifi::NVS_NAMESPACE, wifi::NVS_KEY_SSID);
                    self.remove(wifi::NVS_NAMESPACE, wifi::NVS_KEY_PASS);
                    self.remove(wifi::NVS_NAMESPACE, wifi::NVS_KEY_CONNECT_SUCCESS_COUNT);
                    self.remove(wifi::NVS_NAMESPACE, wifi::NVS_KEY_CONNECT_FAIL_COUNT);
                    self.remove(websocket::NVS_NAMESPACE, websocket::NVS_KEY_DEVICE_NAME);
                    self.remove(websocket::NVS_NAMESPACE, websocket::NVS_KEY_REGISTRATION_CODE);
                    self.remove(websocket::NVS_NAMESPACE, websocket::NVS_KEY_DEVICE_ID);
                    self.remove(websocket::NVS_NAMESPACE, websocket::NVS_KEY_DEVICE_SECRET);
                    self.remove(websocket::NVS_NAMESPACE, websocket::NVS_KEY_CLOUD_HOST);
                    self.remove(nodem::NVS_NAMESPACE, nodem::NVS_KEY_LAST_PAGE);
                    // Keys with a default are reset to it rather than removed,
                    // the same value `read_nodem_config`/`read_iled_config`
                    // would write on a first boot. Like every NVS write here,
                    // each is mirrored into its runtime value right away.
                    self.store(nodem::NVS_NAMESPACE, nodem::NVS_KEY_CONFIG, &NodemConfig::default().to_nvs_string(), 1, nodem::NVS_VALUE_MAX_LEN, &mut ret);
                    self.store(iled::NVS_NAMESPACE, iled::NVS_KEY_CONFIG, &IledConfig::default().to_csv(), 1, 80, &mut ret);
                    self.store(oled::NVS_NAMESPACE, oled::NVS_KEY_CONFIG, &OledConfig::default().to_nvs_string(), 1, oled::NVS_VALUE_MAX_LEN, &mut ret);
                    self.nodem_config = Some(NodemConfig::default());
                    self.oled_config = Some(Some(OledConfig::default()));
                    self.erase_pkg_header();
                    self.iled_config = Some(Some(IledConfig::default()));
                    self.device_name = Some(None);
                    self.wifi_reconnect = true;
                    self.reregister = true;
                }
                "wifi" => {
                    self.store(wifi::NVS_NAMESPACE, wifi::NVS_KEY_SSID, arg_0, 1, 32, &mut ret);
                    if ret == Ret::Ok {
                        self.store(wifi::NVS_NAMESPACE, wifi::NVS_KEY_PASS, arg_1, 8, 63, &mut ret);
                    }
                    if ret == Ret::Ok { self.wifi_reconnect = true; }
                }
                // Both args are validated *before* either is written - a
                // naive "store code, then store name" order let a
                // too-short/too-long device name leave the registration
                // code alone persisted in NVS (the first store already
                // succeeded before the second one failed), which
                // `websocket::ensure_device_credentials` then treats as "a
                // code is present, go register" every retry, only to fail
                // every time on the missing device name - a permanently
                // stuck, self-inflicted `RegistrationStatus::Failed`. Validating
                // first makes the whole command atomic: either both get
                // written, or neither does.
                "reg" => {
                    if !(6..=6).contains(&arg_0.len())
                        || !(websocket::DEVICE_NAME_MIN_LEN..=websocket::DEVICE_NAME_MAX_LEN).contains(&arg_1.len())
                    {
                        ret = Ret::MalformedValue;
                    } else {
                        self.store(websocket::NVS_NAMESPACE, websocket::NVS_KEY_REGISTRATION_CODE, arg_0, 6, 6, &mut ret);
                        if ret == Ret::Ok {
                            self.store(websocket::NVS_NAMESPACE, websocket::NVS_KEY_DEVICE_NAME, arg_1, websocket::DEVICE_NAME_MIN_LEN, websocket::DEVICE_NAME_MAX_LEN, &mut ret);
                        }
                    }
                    if ret == Ret::Ok {
                        self.device_name = Some(Some(arg_1.to_string()));
                        self.reregister = true;
                    }
                }
                "name" => {
                    self.store(websocket::NVS_NAMESPACE, websocket::NVS_KEY_DEVICE_NAME, arg_0, websocket::DEVICE_NAME_MIN_LEN, websocket::DEVICE_NAME_MAX_LEN, &mut ret);
                    if ret == Ret::Ok { self.device_name = Some(Some(arg_0.to_string())); }
                }                
                // "#iled,<geometry>,<protocol>,<color>,<off_color>", where
                // <geometry> is itself "<width>:<height>:<layout>" and
                // <layout> is up to 4 order-independent characters - one of
                // "r"/"l" (which horizontal edge the chain enters from), one
                // of "t"/"b" (which vertical edge), an optional "x"/"y"
                // (row-major vs column-major scan, default "x"), and an
                // optional "i" (serpentine wiring) - see `iled::parse_layout`
                // - and <protocol> is either a known
                // chip name (e.g. "ws2816b_rgbw" - see `iled::CHIP_PROTOCOLS`)
                // or itself
                // "<pattern_high>:<pattern_low>:<pattern_ns>:<reset_ns>:<color_pattern>"
                // (all five required if <protocol> isn't left entirely
                // blank) - persisted to NVS like the commands above, so it survives a
                // reboot; see `IledConfig::parse`/`to_csv` for what each
                // field means, which ones tolerate being left empty, and how
                // they're validated. A completely bare "#iled" (no fields at
                // all, not even a single comma - distinct from every field
                // being individually left blank, which just means "use every
                // default") instead disables the feature: it clears
                // `NVS_KEY_CONFIG` and mirrors `None` into `Global::iled_config`
                // rather than falling back to `IledConfig::default()`.
                "iled" if args.trim().is_empty() => {
                    self.remove(iled::NVS_NAMESPACE, iled::NVS_KEY_CONFIG);
                    self.iled_config = Some(None);
                }
                "iled" => {
                    match IledConfig::parse(args) {
                        Some(config) => {
                            self.store(iled::NVS_NAMESPACE, iled::NVS_KEY_CONFIG, &config.to_csv(), 1, 80, &mut ret);
                            if ret == Ret::Ok { self.iled_config = Some(Some(config)); };
                        }
                        None => ret = Ret::MalformedValue,
                    }
                }
                // "#nodem,<width>:<height>:<bits_per_pixel>[,<device>...]",
                // each <device> "<driver>:<x>:<y>:<scale_x>:<scale_y>" - the
                // nodem framebuffer size and which drivers show which part
                // of it, see `nodem::NodemConfig`. A bare "#nodem" resets it
                // to `NodemConfig::default()`. Applied live right away - see
                // `Global::resize_display`.
                "nodem" => {
                    let value = args.trim();
                    let config = if value.is_empty() { Some(NodemConfig::default()) } else { NodemConfig::parse(value) };
                    match config {
                        Some(config) => {
                            self.store(nodem::NVS_NAMESPACE, nodem::NVS_KEY_CONFIG, &config.to_nvs_string(), 1, nodem::NVS_VALUE_MAX_LEN, &mut ret);
                            if ret == Ret::Ok { self.nodem_config = Some(config); }
                        }
                        None => ret = Ret::MalformedValue,
                    }
                }
                // "#oled,<protocol>:<width>:<height>:<x>:<y>[:<rotation>]" -
                // see `oled::OledConfig`. An empty value ("#oled") or any
                // protocol other than "ssd1306" is stored as given and
                // disables the display; a malformed "ssd1306" value is
                // rejected. Applied right away (`Global::oled_config`).
                "oled" => {
                    let value = args.trim();
                    match OledConfig::parse(value) {
                        Ok(config) => {
                            let stored = config.map(|c| c.to_nvs_string()).unwrap_or_else(|| value.to_string());
                            self.store(oled::NVS_NAMESPACE, oled::NVS_KEY_CONFIG, &stored, 0, oled::NVS_VALUE_MAX_LEN, &mut ret);
                            if ret == Ret::Ok { self.oled_config = Some(config); }
                        }
                        Err(()) => ret = Ret::MalformedValue,
                    }
                }
                // No value ("#host" with nothing after it, or an empty field)
                // clears back to `websocket::DEFAULT_CLOUD_HOST` rather than
                // erroring. A device's registration (`NVS_KEY_DEVICE_ID`/
                // `NVS_KEY_DEVICE_SECRET`) is only meaningful against
                // whichever host issued it, so switching hosts also clears
                // both here - without this, `reregister` below would just
                // reconnect and reuse those stale credentials against the
                // new host (`ensure_device_credentials` only wipes/replaces
                // them itself when a registration code is present), rather
                // than actually re-registering. With them gone,
                // `ensure_device_credentials` finds no credentials *and* no
                // code and reports `RegisterError::MissingCode`, which
                // `websocket_task`'s retry loop turns into
                // `RegistrationStatus::WaitingForCode` on its own - no need
                // to set that status here directly.
                "host" => {
                    if !arg_0.is_empty() {
                        self.store(websocket::NVS_NAMESPACE, websocket::NVS_KEY_CLOUD_HOST, arg_0, 1, 128, &mut ret);
                    }else{
                        self.remove(websocket::NVS_NAMESPACE, websocket::NVS_KEY_CLOUD_HOST);
                    }
                    self.remove(websocket::NVS_NAMESPACE, websocket::NVS_KEY_DEVICE_ID);
                    self.remove(websocket::NVS_NAMESPACE, websocket::NVS_KEY_DEVICE_SECRET);
                    self.reregister = true;
                }
                "cfg" => {
                    let _ = self.write_plain(res, wifi::NVS_NAMESPACE, wifi::NVS_KEY_SSID);
                    let _ = self.write_masked(res, wifi::NVS_NAMESPACE, wifi::NVS_KEY_PASS);
                    let _ = self.write_plain(res, websocket::NVS_NAMESPACE, websocket::NVS_KEY_DEVICE_NAME);
                    let _ = self.write_masked(res, websocket::NVS_NAMESPACE, websocket::NVS_KEY_REGISTRATION_CODE);
                    let _ = self.write_plain(res, websocket::NVS_NAMESPACE, websocket::NVS_KEY_CLOUD_HOST);
                    let _ = self.write_plain(res, iled::NVS_NAMESPACE, iled::NVS_KEY_CONFIG);
                    let _ = self.write_plain(res, nodem::NVS_NAMESPACE, nodem::NVS_KEY_LAST_PAGE);
                    let _ = self.write_plain(res, nodem::NVS_NAMESPACE, nodem::NVS_KEY_CONFIG);
                    let _ = self.write_plain(res, oled::NVS_NAMESPACE, oled::NVS_KEY_CONFIG);
                }
                // Four CSV lines: wifi, cloud, oled and iled - the last field
                // of each of the first three (an error message) is left
                // unescaped and thus may itself contain commas, so a parser
                // should treat it as "everything from here to end of line"
                // rather than a fixed column. "oled,<on>,<w>x<h>,<i2c>" and
                // "iled,<on>,<w>x<h>": <on> is 1/0 (configured or disabled),
                // the resolution empty while off; <i2c> is "ok" or the
                // `OledConnectionStatus` in words ("initializing", or the
                // failure's error).
                "stat" => {
                    let w = &self.wifi_status;
                    let (wifi_state, wifi_error) = match &w.status {
                        WifiConnectionStatus::WaitingForConfiguration => ("waiting", ""),
                        WifiConnectionStatus::Connecting => ("connecting", ""),
                        WifiConnectionStatus::Connected => ("connected", ""),
                        WifiConnectionStatus::Failed(e) => ("failed", e.as_str()),
                    };
                    let _ = write!(
                        res,
                        "wifi,{wifi_state},{},{},{},{},{},{wifi_error}\r\n",
                        w.ssid.as_deref().unwrap_or(""),
                        w.ip.map(|v| v.to_string()).unwrap_or_default(),
                        w.subnet_mask.map(|v| v.to_string()).unwrap_or_default(),
                        w.gateway.map(|v| v.to_string()).unwrap_or_default(),
                        w.rssi.map(|v| v.to_string()).unwrap_or_default(),
                    );

                    let c = &self.cloud_status;
                    let (reg_state, reg_error) = match &c.registration {
                        RegistrationStatus::WaitingForCode => ("waiting", ""),
                        RegistrationStatus::Registering => ("registering", ""),
                        RegistrationStatus::Registered => ("registered", ""),
                        RegistrationStatus::Failed(e) => ("failed", e.as_str()),
                    };
                    let conn_state = match c.connection {
                        CloudConnectionStatus::Disconnected => "disconnected",
                        CloudConnectionStatus::Connecting => "connecting",
                        CloudConnectionStatus::Connected => "connected",
                    };
                    let _ = write!(
                        res,
                        "cloud,{reg_state},{conn_state},{},{},{reg_error}\r\n",
                        c.host.as_deref().unwrap_or(""),
                        c.device_name.as_deref().unwrap_or(""),
                    );

                    let _ = match self.live_oled_config {
                        Some(o) => {
                            let i2c = match &self.oled_status.status {
                                OledConnectionStatus::Connected => "ok",
                                OledConnectionStatus::Initializing => "initializing",
                                OledConnectionStatus::Disabled => "disabled",
                                OledConnectionStatus::Failed(e) => e.as_str(),
                            };
                            write!(res, "oled,1,{}x{},{i2c}\r\n", o.width, o.height)
                        }
                        None => write!(res, "oled,0,,\r\n"),
                    };

                    let _ = match self.live_iled_config {
                        Some(i) => write!(res, "iled,1,{}x{}\r\n", i.width, i.height),
                        None => write!(res, "iled,0,\r\n"),
                    };
                }
                "nvs" => {
                    let _ = self.write_all_entries(res);
                }
                _ => { ret = Ret::Error; }
            }
            Control::send_result(prefix, ret as u8, res)
        }else{
            // "page=<value>" is remembered as `nodem::NVS_KEY_LAST_PAGE` (so
            // `nodem_task` can replay it at boot), but otherwise left alone:
            // returning `false` hands the same line on to the next listener -
            // the DOM's own `control::dom` listener - which actually switches
            // the page and sends the result. Only written when the value
            // changed, since the boot replay itself comes through here too
            // and page switches may be frequent - no need to wear the flash.
            // Values the DOM would reject (anything but a `u8`) aren't stored.
            if let Some(("page", value)) = line.split_once('=').map(|(k, v)| (k.trim(), v.trim())).filter(|(_, v)| v.parse::<u8>().is_ok()) {
                let mut buf = [0u8; 8];
                let stored = EspNvs::new(self.nvs.clone(), nodem::NVS_NAMESPACE, false).ok().and_then(|nvs| nvs.get_str(nodem::NVS_KEY_LAST_PAGE, &mut buf).ok().flatten().map(str::to_string));
                if stored.as_deref() != Some(value) {
                    let mut ret = Ret::Ok;
                    self.store(nodem::NVS_NAMESPACE, nodem::NVS_KEY_LAST_PAGE, value, 1, 3, &mut ret);
                }
            }
            (false, Ok(()))
        }
    }

    fn get_loader(&mut self) -> Option<&mut dyn IControlLoader> { Some(self) }    
}


/// Both "pkg" and "ota" uploads go through here: `process_loader_start`
/// picks the mode, then the data is staged in `loader_buf` and handed on in
/// `LOADER_CHUNK_LEN` pieces to that mode's `*_write`, and
/// `process_loader_end` flushes the remainder and finishes via its `*_end`.
/// A new pkg is loaded live (see `pkg_updated`); a new firmware takes a
/// restart (see `ota_end`).
impl IControlLoader for CommandListener {
    fn process_loader_start(&mut self, mode: ControlMode, len: usize) -> usize {
        log::info!("process_loader_start");

        // A previous "ota" upload that never reached `process_loader_end`
        // (e.g. the connection dropped midway) - release its handle first.
        if let Some((handle, _)) = self.ota.take() {
            unsafe { sys::esp_ota_abort(handle) };
        }

        self.loader_buf_len = 0;
        self.loader_failed = false;

        let size = match mode {
            ControlMode::PKGMode => self.pkg_start(len),
            ControlMode::OTAMode => self.ota_start(len),
            _ => 0,
        };
        self.loader_mode = (size > 0).then_some(mode);
        size
    }

    fn process_loader_data(&mut self, buf: &[u8], _pos: usize) {
        for &byte in buf {
            self.loader_buf[self.loader_buf_len] = byte;
            self.loader_buf_len += 1;
            if self.loader_buf_len == LOADER_CHUNK_LEN {
                self.loader_flush();
            }
        }
    }

    fn process_loader_end(&mut self) -> LoaderRet {
        log::info!("process_loader_end");
        self.loader_flush();

        match self.loader_mode.take() {
            Some(ControlMode::PKGMode) => self.pkg_end(),
            Some(ControlMode::OTAMode) => self.ota_end(),
            _ => LoaderRet::NotEnoughSpace,
        }
    }
}

impl CommandListener {
    /// Hands whatever is staged in `loader_buf` on to the current mode's
    /// `*_write` - skipped once a write has failed, since the upload is
    /// already lost at that point.
    fn loader_flush(&mut self) {
        let len = core::mem::take(&mut self.loader_buf_len);
        if len == 0 || self.loader_failed {
            return;
        }

        let ok = match self.loader_mode {
            Some(ControlMode::PKGMode) => self.pkg_write(len),
            Some(ControlMode::OTAMode) => self.ota_write(len),
            _ => true,
        };
        if !ok {
            self.loader_failed = true;
        }
    }
}

/// "pkg" uploads, into the "pkg" partition - loaded from there by
/// `nodem_task` right away (via `pkg_updated`), and again on every boot.
impl CommandListener {
    fn pkg_start(&mut self, len: usize) -> usize {
        self.pkg_written = 0;
        self.pkg_partition = None;

        let mut partition = match unsafe { EspPartition::new(PKG_PARTITION_LABEL) } {
            Ok(Some(partition)) => partition,
            Ok(None) => {
                log::error!("'{PKG_PARTITION_LABEL}' partition not found");
                return 0;
            }
            Err(e) => {
                log::error!("'{PKG_PARTITION_LABEL}' partition lookup failed: {e:?}");
                return 0;
            }
        };
        let size = partition.size();
        if len > size {
            log::error!("'{PKG_PARTITION_LABEL}': pkg is {len} bytes, partition only {size}");
            return 0;
        }

        let erase_size = partition.erase_size();
        let erase_len = (len + erase_size - 1) / erase_size * erase_size;

        // Flash can only clear bits, never set them, so the region the
        // package will occupy has to be erased up front, rounded up to whole
        // flash sectors (`erase_size`) - a per-chunk erase would repeatedly
        // re-erase (and thus wipe) earlier chunks that share the same erase
        // block as a later one.
        if let Err(e) = partition.erase(0, erase_len) {
            log::error!("'{PKG_PARTITION_LABEL}' partition erase failed: {e:?}");
            return 0;
        }

        self.pkg_partition = Some(partition);
        size
    }

    /// Invalidates whatever pkg is in the "pkg" partition by erasing its
    /// first erase block - enough to wipe the "PKG0" header, so
    /// `nodem::load_pkg_partition` no longer finds one. Flags `pkg_updated`
    /// so `nodem_task` drops the loaded pkg from the live runtime right away.
    fn erase_pkg_header(&mut self) {
        let mut partition = match unsafe { EspPartition::new(PKG_PARTITION_LABEL) } {
            Ok(Some(partition)) => partition,
            Ok(None) => {
                log::error!("'{PKG_PARTITION_LABEL}' partition not found");
                return;
            }
            Err(e) => {
                log::error!("'{PKG_PARTITION_LABEL}' partition lookup failed: {e:?}");
                return;
            }
        };
        let erase_size = partition.erase_size();
        if let Err(e) = partition.erase(0, erase_size) {
            log::error!("'{PKG_PARTITION_LABEL}' partition erase failed: {e:?}");
            return;
        }
        self.pkg_updated = true;
    }

    fn pkg_write(&mut self, len: usize) -> bool {
        let Some(partition) = self.pkg_partition.as_mut() else { return false; };
        if let Err(e) = partition.write(self.pkg_written, &self.loader_buf[..len]) {
            log::error!("'{PKG_PARTITION_LABEL}' partition write at {} failed: {e:?}", self.pkg_written);
            return false;
        }
        self.pkg_written += len;
        true
    }

    fn pkg_end(&mut self) -> LoaderRet {
        if self.pkg_partition.take().is_none() || self.loader_failed {
            return LoaderRet::NotEnoughSpace;
        }
        self.pkg_updated = true;
        LoaderRet::Ok
    }
}

/// "ota" uploads. The image goes into whichever OTA slot isn't running
/// (`esp_ota_get_next_update_partition`) and only replaces the running
/// firmware once `esp_ota_end` has verified it - image checksum/hash, and in
/// release builds its signature against the key that signed the running app
/// (`CONFIG_SECURE_SIGNED_ON_UPDATE_NO_SECURE_BOOT`, see
/// sdkconfig.defaults.release). Only then is it made the boot slot and a
/// restart requested (`restart`, acted on by callers once the result has
/// been sent). It boots in the rollback phase - see `heartbeat_task`.
impl CommandListener {
    fn ota_start(&mut self, len: usize) -> usize {
        let partition = unsafe { sys::esp_ota_get_next_update_partition(core::ptr::null()) };
        let Some(size) = (unsafe { partition.as_ref() }).map(|p| p.size as usize) else {
            log::error!("OTA: no update partition");
            return 0;
        };
        if len > size {
            log::error!("OTA: image is {len} bytes, update partition only {size}");
            return 0;
        }

        // `OTA_WITH_SEQUENTIAL_WRITES`: erase sector by sector as data comes
        // in, rather than the whole image's worth up front.
        let mut handle: esp_ota_handle_t = 0;
        // Also refused (`ESP_ERR_OTA_ROLLBACK_INVALID_STATE`) while the
        // running firmware is itself still in its rollback phase.
        if let Err(e) = esp!(unsafe { sys::esp_ota_begin(partition, sys::OTA_WITH_SEQUENTIAL_WRITES as usize, &mut handle) }) {
            log::error!("OTA: begin failed: {e:?}");
            return 0;
        }

        self.ota = Some((handle, partition));
        size
    }

    fn ota_write(&mut self, len: usize) -> bool {
        let Some((handle, _)) = self.ota else { return false; };
        if let Err(e) = esp!(unsafe { sys::esp_ota_write(handle, self.loader_buf.as_ptr() as *const _, len) }) {
            log::error!("OTA: write failed: {e:?}");
            return false;
        }
        true
    }

    fn ota_end(&mut self) -> LoaderRet {
        let Some((handle, partition)) = self.ota.take() else { return LoaderRet::Aborted; };

        if self.loader_failed {
            unsafe { sys::esp_ota_abort(handle) };
            return LoaderRet::Aborted;
        }

        // Validates the whole image (and, in release builds, its signature) -
        // frees `handle` either way.
        if let Err(e) = esp!(unsafe { sys::esp_ota_end(handle) }) {
            log::error!("OTA: image verification failed: {e:?}");
            return LoaderRet::Aborted;
        }

        if let Err(e) = esp!(unsafe { sys::esp_ota_set_boot_partition(partition) }) {
            log::error!("OTA: setting boot partition failed: {e:?}");
            return LoaderRet::Aborted;
        }

        log::info!("OTA: update verified, restarting into it");
        self.restart = true;
        LoaderRet::Ok
    }
}
