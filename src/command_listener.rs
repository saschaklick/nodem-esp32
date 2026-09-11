use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs};
use esp_idf_svc::partition::{EspMemMapType, EspPartition};

use nodem_rs::{ control::{ Control, IControl, IControlLoader }, media::Media };

use crate::global::{CloudConnectionStatus, CloudStatus, RegistrationStatus, WifiConnectionStatus, WifiStatus};
use crate::iled::{self, IledConfig};
use crate::websocket;
use crate::wifi;

pub(crate) const PKG_PARTITION_LABEL: &str = "pkg";
const PKG_CHUNK_LEN: usize = 512;

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
    // Set by "#name" so callers can mirror it into `Global::cloud_status`
    // right away, same reason/pattern as `wifi_reconnect`/`reregister` above.
    device_name: Option<String>,
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
    // Snapshot of `Global::wifi_status`/`cloud_status`, refreshed by
    // `update_status` right before each `process_command` call (same reason
    // this doesn't hold `Global` itself - see the struct doc comment) so the
    // "#stat" command below has something to report.
    wifi_status: WifiStatus,
    cloud_status: CloudStatus,
    // Backing store for "#pkg" uploads (see `IControlLoader` below).
    pkg_partition: Option<EspPartition>,
    pkg_buf: [u8; PKG_CHUNK_LEN],
    pkg_buf_len: usize,
    // Absolute offset in the "pkg" partition of the next byte to be written -
    // `process_loader_end` needs this to flush a final, sub-`PKG_CHUNK_LEN`
    // chunk, since (unlike `process_loader_data`) it isn't given a position.
    pkg_written: usize,
    // Set by `process_loader_end` once the upload has been flashed and
    // memory-mapped - the pointer/length of that mapping, for a caller to
    // mirror into `Global::pkg_reload` (same reason/pattern as
    // `wifi_reconnect`/`reregister` above) so `nodem_task` can actually call
    // `Media::load_pkg` on the *live* `Surface::media` next time it runs,
    // rather than on a `Media` here that nothing ever renders from. The
    // mapping itself is never unmapped (see `process_loader_end`) - it has to
    // stay valid for as long as whatever `Media` loads it is in use.
    pkg_reload: Option<(*const u8, usize)>,
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
            wifi_status: WifiStatus::new(),
            cloud_status: CloudStatus::new(),
            pkg_partition: None,
            pkg_buf: [0u8; PKG_CHUNK_LEN],
            pkg_buf_len: 0,
            pkg_written: 0,
            pkg_reload: None,
        }
    }

    pub(crate) fn take_wifi_reconnect(&mut self) -> bool {
        core::mem::take(&mut self.wifi_reconnect)
    }

    pub(crate) fn take_reregister(&mut self) -> bool {
        core::mem::take(&mut self.reregister)
    }

    pub(crate) fn take_device_name(&mut self) -> Option<String> {
        core::mem::take(&mut self.device_name)
    }

    pub(crate) fn take_restart(&mut self) -> bool {
        core::mem::take(&mut self.restart)
    }

    pub(crate) fn take_iled_config(&mut self) -> Option<Option<IledConfig>> {
        core::mem::take(&mut self.iled_config)
    }

    pub(crate) fn take_pkg_reload(&mut self) -> Option<(*const u8, usize)> {
        core::mem::take(&mut self.pkg_reload)
    }

    pub(crate) fn update_status(&mut self, wifi_status: &WifiStatus, cloud_status: &CloudStatus) {
        self.wifi_status = wifi_status.clone();
        self.cloud_status = cloud_status.clone();
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

    fn write_plain(&self, res: &mut dyn core::fmt::Write, namespace: &str, key: &str) -> core::fmt::Result {
        let mut buf = [0u8; 64];
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
                    if ret == Ret::Ok { self.reregister = true; }
                }
                "name" => {
                    self.store(websocket::NVS_NAMESPACE, websocket::NVS_KEY_DEVICE_NAME, arg_0, websocket::DEVICE_NAME_MIN_LEN, websocket::DEVICE_NAME_MAX_LEN, &mut ret);
                    if ret == Ret::Ok { self.device_name = Some(arg_0.to_string()); }
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
                }
                // Two CSV lines, one per status struct - the last field of
                // each ("failed"'s error message) is left unescaped and thus
                // may itself contain commas, so a parser should treat it as
                // "everything from here to end of line" rather than a fixed
                // column.
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
                }
                "nvs" => {
                    let _ = self.write_plain(res, wifi::NVS_NAMESPACE, wifi::NVS_KEY_SSID);
                    let _ = self.write_plain(res, wifi::NVS_NAMESPACE, wifi::NVS_KEY_PASS);
                    let _ = self.write_plain(res, websocket::NVS_NAMESPACE, websocket::NVS_KEY_DEVICE_NAME);
                    let _ = self.write_plain(res, websocket::NVS_NAMESPACE, websocket::NVS_KEY_REGISTRATION_CODE);                    
                    let _ = self.write_plain(res, websocket::NVS_NAMESPACE, websocket::NVS_KEY_CLOUD_HOST);
                    let _ = self.write_plain(res, websocket::NVS_NAMESPACE, websocket::NVS_KEY_DEVICE_ID);
                    let _ = self.write_plain(res, websocket::NVS_NAMESPACE, websocket::NVS_KEY_DEVICE_SECRET);
                }
                _ => { ret = Ret::Error; }
            }
            Control::send_result(prefix, ret as u8, res)
        }else{
            (false, Ok(()))
        }
    }

    fn get_loader(&mut self) -> Option<&mut dyn IControlLoader> { Some(self) }    
}


impl IControlLoader for CommandListener {
    fn process_loader_start(&mut self, _len: usize) -> usize {
        log::info!("process_loader_start");
        self.pkg_buf_len = 0;
        self.pkg_written = 0;
        self.pkg_partition = None;

        let partition = match unsafe { EspPartition::new(PKG_PARTITION_LABEL) } {
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
        self.pkg_partition = Some(partition);

        // Flash can only clear bits, never set them, so the whole region has
        // to be erased up front before `process_loader_data` can write into
        // it - a per-chunk erase would repeatedly re-erase (and thus wipe)
        // earlier chunks that share the same erase block as a later one.
        if let Err(e) = self.pkg_partition.as_mut().unwrap().erase(0, size) {
            log::error!("'{PKG_PARTITION_LABEL}' partition erase failed: {e:?}");
            self.pkg_partition = None;
            return 0;
        }

        size
    }

    fn process_loader_data(&mut self, buf: &[u8], pos: usize) {
        for (i, &byte) in buf.iter().enumerate() {            
            self.pkg_buf[self.pkg_buf_len] = byte;
            self.pkg_buf_len += 1;

            if self.pkg_buf_len == PKG_CHUNK_LEN {
                let Some(partition) = self.pkg_partition.as_mut() else { return; };
                let offset = pos + i + 1 - PKG_CHUNK_LEN;
                if let Err(e) = partition.write(offset, &self.pkg_buf) {
                    log::error!("'{PKG_PARTITION_LABEL}' partition write at {offset} failed: {e:?}");
                }
                self.pkg_written = offset + PKG_CHUNK_LEN;
                self.pkg_buf_len = 0;
            }
        }
    }

    fn process_loader_end(&mut self) -> u8 {
        log::info!("process_loader_end");
        let Some(partition) = self.pkg_partition.as_mut() else { return 1; };

        if self.pkg_buf_len > 0 {
            if let Err(e) = partition.write(self.pkg_written, &self.pkg_buf[..self.pkg_buf_len]) {
                log::error!("'{PKG_PARTITION_LABEL}' partition write at {} failed: {e:?}", self.pkg_written);
                return 1;
            }
            self.pkg_written += self.pkg_buf_len;
            self.pkg_buf_len = 0;
        }

        let mapped = match unsafe { partition.mmap(0, self.pkg_written, EspMemMapType::Data) } {
            Ok(mapped) => mapped,
            Err(e) => {
                log::error!("'{PKG_PARTITION_LABEL}' partition mmap failed: {e:?}");
                return 1;
            }
        };
        let ptr = mapped.start() as *const u8;
        let len = self.pkg_written;
        // Leaked deliberately: whichever `Media` eventually loads this (see
        // `pkg_reload`'s doc comment) will point straight into this mapping,
        // so it must never be unmapped (which is what
        // `EspMemMappedPartition::drop` would otherwise do).
        core::mem::forget(mapped);

        self.pkg_reload = Some((ptr, len));
        0
    }
}
