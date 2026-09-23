use std::cell::RefCell;
use std::rc::Rc;
use std::sync::mpsc;
use std::time::Duration;

use embassy_time::{Instant, Timer};
use embedded_svc::http::client::Client as HttpClient;
use embedded_svc::io::Write as _;
use embedded_svc::utils::io;
use esp_idf_svc::http::client::{Configuration as HttpConfiguration, EspHttpConnection};
use esp_idf_svc::io::EspIOError;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};
use esp_idf_svc::sys::{esp, esp_mac_type_t_ESP_MAC_WIFI_STA, esp_read_mac};
use esp_idf_svc::tls::X509;
use esp_idf_svc::ws::client::{
    EspWebSocketClient, EspWebSocketClientConfig, FrameType, WebSocketEvent, WebSocketEventType,
};
use nodem_rs::runtime::Runtime;

use crate::command_listener::CommandListener;
use crate::global::{CloudConnectionStatus, Global, RegistrationStatus, WifiConnectionStatus};

const DEFAULT_CLOUD_HOST: &str = "docker.ws-ai-call-bot.workers.dev";

// Trusted root CAs for the websocket/HTTPS server's TLS certificate: the
// self-signed dev-server root, plus Google Trust Services' GTS Root R4 - the
// root Cloudflare's own edge certificates (workers.dev, pages.dev, and thus
// any Worker without a custom domain) currently chain up through. Fetched
// straight from Google's PKI repo (https://i.pki.goog/r4.pem) and cross-checked
// against the live chain `openssl s_client -connect workers.dev:443 -showcerts`
// presents. `mbedtls_x509_crt_parse` accepts multiple concatenated PEM certs in
// one buffer, appending each to the trusted set, so both roots are trusted at
// once - no per-connection swap needed between the dev server and Cloudflare.
const ROOT_CA_CERT: &[u8] = concat!(
    include_str!("../certs/rootCA.pem"),
    include_str!("../certs/gts-root-r4.pem"),
    "\0",
)
.as_bytes();

// pub(crate): `uart_task`'s command handler writes `NVS_KEY_REGISTRATION_CODE`
// and `NVS_KEY_DEVICE_NAME` when it gets provisioned over UART - see
// `crate::command_listener::CommandListener`.
pub(crate) const NVS_NAMESPACE: &str = "device";
pub(crate) const NVS_KEY_REGISTRATION_CODE: &str = "reg_code";
pub(crate) const NVS_KEY_DEVICE_NAME: &str = "device_name";
// pub(crate): bounds `uart_task`'s "#reg" command enforces when
// writing `NVS_KEY_DEVICE_NAME` - `ensure_device_credentials` re-checks a
// stored name against these same bounds before registering with it.
pub(crate) const DEVICE_NAME_MIN_LEN: usize = 3;
pub(crate) const DEVICE_NAME_MAX_LEN: usize = 64;
// pub(crate): read (unmasked) by `uart_task`'s "#nvs" debug dump.
pub(crate) const NVS_KEY_DEVICE_ID: &str = "device_id";
pub(crate) const NVS_KEY_DEVICE_SECRET: &str = "device_secret";
// pub(crate): `uart_task`'s "#host" command writes/removes this - see
// `crate::command_listener::CommandListener`. Overrides `DEFAULT_CLOUD_HOST` for every
// cloud connection (websocket + HTTP registration) when set.
pub(crate) const NVS_KEY_CLOUD_HOST: &str = "cloud_host";

/// Value `NVS_KEY_REGISTRATION_CODE` gets replaced with once a registration
/// attempt is rejected - too short to ever collide with a real code (`uart`'s
/// "#reg" command only accepts a 6-character code as its first argument).
/// Marks the code as spent (so it isn't retried forever) while leaving a
/// visible trace, via "#cfg"/"#nvs", that the last attempt failed.
const NVS_VALUE_REGISTRATION_REJECTED: &str = "0";

/// Manages device registration and the websocket client connection.
///
/// The device must be registered (a device id/secret pair, sent as headers on
/// the websocket connection) before the websocket can be opened.
/// `ensure_device_credentials` handles that: reusing a previously-registered
/// id/secret pair from NVS if there is one, or - once a registration code has
/// been provisioned into NVS (by whoever is setting the device up; there's no
/// way to register without one) - performing the one-time HTTP registration
/// handshake and persisting the result for future boots.
///
/// `EspWebSocketClient`'s connection (including reconnects) is driven entirely
/// by its own background ESP-IDF task, not by this executor - `disable_auto_reconnect`
/// defaults to `false`, so unlike `wifi_task` there's no manual reconnect loop
/// needed here for a *dropped* connection. This task's job is just to wait for
/// Wi-Fi, register/load credentials, create the client, and then keep it alive
/// for as long as the device runs - unless `Global::reregister` is set (by
/// `uart_task`, when a fresh registration code is provisioned), in which case
/// it wipes the stored device id/secret, closes the client, and starts the
/// whole registration dance over.
pub async fn websocket_task(global: Rc<RefCell<Global>>, nvs: EspDefaultNvsPartition) {
    wait_for_wifi(&global).await;

    // Same admin command listener `uart_task` uses, so any "#..." command
    // valid over UART (including "#wifi"/"#factory", which is why this needs
    // its own NVS handle rather than reusing `device_nvs` below) works the
    // same way when it arrives as a cloud message instead.
    let mut command_listener = CommandListener::new(nvs.clone());

    let device_nvs = match EspNvs::new(nvs, NVS_NAMESPACE, true) {
        Ok(device_nvs) => device_nvs,
        Err(e) => {
            log::error!("Failed to open '{NVS_NAMESPACE}' NVS namespace: {e:?}");
            global.borrow_mut().cloud_status.registration = RegistrationStatus::Failed(e.to_string());
            return;
        }
    };

    loop {
        // Re-checked on every lap, not just once at task start, so a client
        // torn down for any reason (reregister, a failed ping) waits out a
        // Wi-Fi reconnect already in progress instead of racing ahead of it -
        // and, since `wifi_task` flips this the instant it has an IP, picks
        // back up immediately once Wi-Fi actually is up rather than after
        // this loop's own backoff delays.
        wait_for_wifi(&global).await;

        let (device_id, device_secret) = loop {
            match ensure_device_credentials(&device_nvs, &global) {
                Ok(credentials) => break credentials,
                Err(RegisterError::MissingCode) => {
                    global.borrow_mut().cloud_status.registration = RegistrationStatus::WaitingForCode;
                    Timer::after_secs(2).await;
                }
                Err(RegisterError::Failed(e)) => {
                    log::error!("Device registration failed: {e:?}");
                    global.borrow_mut().cloud_status.registration = RegistrationStatus::Failed(e.to_string());
                    Timer::after_secs(5).await;
                }
            }
        };

        {
            let mut g = global.borrow_mut();
            g.cloud_status.registration = RegistrationStatus::Registered;
            g.cloud_status.connection = CloudConnectionStatus::Connecting;
        }

        // `EspWebSocketClientConfig::headers` is a single string of "Header: value\r\n"
        // lines, not a list - see `esp_websocket_client_set_headers`'s doc comment.
        let headers = format!("X-Device-Id: {device_id}\r\nX-Device-Secret: {device_secret}\r\n");

        let config = EspWebSocketClientConfig {
            server_cert: Some(X509::pem_until_nul(ROOT_CA_CERT)),
            headers: Some(&headers),
            ..Default::default()
        };

        // Re-read on every (re)connect, not just once at task start, so a fresh
        // "#host" provisioned while already connected takes effect as soon as
        // `reregister` drops the client and loops back around here.
        let host = read_cloud_host(&device_nvs);
        global.borrow_mut().cloud_status.host = Some(host.clone());

        // Text/binary messages the server sends arrive on `EspWebSocketClient`'s
        // own hidden ESP-IDF thread (see the event callback below), not this
        // executor - `message_tx` hands them off across that thread boundary so
        // they can be run through `process_command` (which touches `Global`,
        // `!Send`) back on this task instead.
        let (message_tx, message_rx) = mpsc::channel::<Vec<u8>>();

        let client = EspWebSocketClient::new(
            format!("wss://{host}/ws/device").as_str(),
            &config,
            Duration::from_secs(10),
            move |event: &Result<WebSocketEvent, EspIOError>| {
                log_websocket_event(event);
                forward_incoming_message(event, &message_tx);
            },
        );

        let client = match client {
            Ok(client) => client,
            Err(e) => {
                log::error!("Websocket client creation failed: {e:?}");
                global.borrow_mut().cloud_status.connection = CloudConnectionStatus::Disconnected;
                Timer::after_secs(5).await;
                continue;
            }
        };

        // `EspWebSocketClient::new` returns once the client (and its background
        // ESP-IDF task) is created, not once it's actually connected - poll
        // `is_connected()` to keep `cloud_status` reasonably current for
        // display purposes. Also watch `reregister` here so it gets picked up
        // promptly rather than only between (long-lived) connections. Polled
        // much faster than the status/reregister checks below actually need,
        // so a queued incoming message doesn't sit for up to a second before
        // `process_command` (and thus a reply) runs.
        // Application-level heartbeat: the underlying ESP-IDF client has its
        // own auto-reconnect for a *dropped* TCP connection (see this
        // function's doc comment), but that doesn't catch a link that's still
        // "connected" yet silently stopped passing data - a failed send here
        // is the signal for that, so it forces the explicit reconnect below
        // rather than trusting `is_connected()` alone.
        let mut last_ping = Instant::now();

        // Kept in `Global` rather than here, so whatever is about to take
        // Wi-Fi down (`wifi_task`'s reconnect, a restart) can close it
        // cleanly first - see `close_websocket`. Gone from there means exactly
        // that happened: back to waiting for Wi-Fi.
        global.borrow_mut().ws_client = Some(client);

        loop {
            Timer::after_millis(20).await;

            while let Ok(message) = message_rx.try_recv() {
                handle_incoming_message(&global, &mut command_listener, &message);
            }

            let Some(connected) = global.borrow().ws_client.as_ref().map(EspWebSocketClient::is_connected) else {
                global.borrow_mut().cloud_status.connection = CloudConnectionStatus::Disconnected;
                break;
            };

            if connected && last_ping.elapsed() >= embassy_time::Duration::from_secs(10) {
                last_ping = Instant::now();

                let ping = format!("ping,{}", Instant::now().as_secs());
                if let Err(e) = send_text(&global, &ping) {
                    log::error!("Failed to send websocket ping, reconnecting: {e:?}");
                    close_websocket(&global);
                    global.borrow_mut().cloud_status.connection = CloudConnectionStatus::Disconnected;
                    break;
                }
            }

            if global.borrow().reregister {
                global.borrow_mut().reregister = false;

                // The actual NVS wipe (device id/secret) happens in
                // `ensure_device_credentials`, driven by the fresh registration
                // code this flag was set in response to - just get back there.
                close_websocket(&global);

                let mut g = global.borrow_mut();
                g.cloud_status.registration = RegistrationStatus::Registering;
                g.cloud_status.connection = CloudConnectionStatus::Disconnected;
                break;
            }

            global.borrow_mut().cloud_status.connection = if connected {
                CloudConnectionStatus::Connected
            } else {
                CloudConnectionStatus::Disconnected
            };
        }
    }
}

/// Runs one message received over the websocket through `runtime.process_command`
/// - same as `uart_task` does for bytes read off UART - and sends back whatever
/// it wrote to its response buffer. Unlike UART, a websocket message is already
/// a complete, discrete unit (no partial-command buffering needed), but it may
/// still contain more than one "#..." command back to back, so this keeps
/// feeding `process_command` the unconsumed remainder until either nothing is
/// left or a call stops making progress.
fn handle_incoming_message(
    global: &Rc<RefCell<Global>>,
    command_listener: &mut CommandListener,
    message: &[u8],
) {
    let mut remaining = message;

    while !remaining.is_empty() {
        let mut response = String::new();

        let (consumed, write_result) = {
            let mut g = global.borrow_mut();
            command_listener.update_status(&g.wifi_status, &g.cloud_status);
            let ret = g.runtime.process_command(remaining, &mut response, command_listener);

            if command_listener.take_wifi_reconnect() {
                g.wifi_reconnect = true;
            }
            if command_listener.take_reregister() {
                g.reregister = true;
            }
            if let Some(device_name) = command_listener.take_device_name() {
                g.cloud_status.device_name = Some(device_name);
            }
            if let Some(iled_config) = command_listener.take_iled_config() {
                g.iled_config = iled_config;
            }
            if command_listener.take_pkg_updated() {
                g.pkg_reload = true;
            }

            ret
        };
        let restart = command_listener.take_restart();

        if let Err(e) = write_result {
            log::error!("Failed to build response to websocket message: {e:?}");
        }

        if !response.is_empty() {
            if let Err(e) = send_text(global, &response) {
                log::error!("Failed to send websocket response: {e:?}");
            }
        }

        // Checked last, once the "#reset" ack above has actually been handed
        // to the client - `restart()` never returns.
        if restart {
            log::info!("Reset requested, restarting...");
            self::restart(global);
        }

        if consumed == 0 {
            break;
        }
        remaining = &remaining[consumed.min(remaining.len())..];
    }
}

/// Sends `text` over the websocket, if there currently is one.
fn send_text(global: &RefCell<Global>, text: &str) -> Result<(), EspIOError> {
    match global.borrow_mut().ws_client.as_mut() {
        Some(client) => client.send(FrameType::Text(false), text.as_bytes()).map_err(EspIOError),
        None => Ok(()),
    }
}

/// Closes the websocket cleanly - a close frame, and waiting (up to the
/// client's 10s timeout) for the server's reply - which is what dropping an
/// `EspWebSocketClient` does. Call before anything takes Wi-Fi down
/// (`wifi.disconnect()`, a restart): once the link is gone, the connection
/// can only be aborted, which the server just sees as a dropped TCP
/// connection. `websocket_task` notices the client is gone and starts over
/// from waiting for Wi-Fi.
pub(crate) fn close_websocket(global: &RefCell<Global>) {
    // Taken out first, then dropped with `global` no longer borrowed.
    let client = global.borrow_mut().ws_client.take();
    if let Some(client) = client {
        log::info!("Closing websocket...");
        drop(client);
    }
}

/// `restart()`, but with the websocket closed cleanly first - see
/// `close_websocket`.
pub(crate) fn restart(global: &RefCell<Global>) -> ! {
    close_websocket(global);
    esp_idf_svc::hal::reset::restart()
}

/// Extracts a `Text`/`Binary` websocket event's payload and hands it to
/// `handle_incoming_message` (via `tx`) back on the executor - runs on
/// `EspWebSocketClient`'s own hidden ESP-IDF thread, so it must not touch
/// `Global` or anything else that isn't `Send`.
fn forward_incoming_message(event: &Result<WebSocketEvent, EspIOError>, tx: &mpsc::Sender<Vec<u8>>) {
    let Ok(event) = event else { return };

    let payload: &[u8] = match event.event_type {
        WebSocketEventType::Text(text) => text.as_bytes(),
        WebSocketEventType::Binary(data) => data,
        _ => return,
    };

    if let Err(e) = tx.send(payload.to_vec()) {
        log::error!("Failed to queue incoming websocket message: {e:?}");
    }
}

/// The host (and, if applicable, port) for every cloud connection - both the
/// websocket and the HTTP registration handshake. `NVS_KEY_CLOUD_HOST` when
/// set (via the "#host" UART command), else `DEFAULT_CLOUD_HOST`.
fn read_cloud_host(nvs: &EspNvs<NvsDefault>) -> String {
    let mut host_buf = [0u8; 128];

    nvs.get_str(NVS_KEY_CLOUD_HOST, &mut host_buf)
        .ok()
        .flatten()
        .filter(|host| !host.is_empty())
        .map(str::to_string)
        .unwrap_or_else(|| DEFAULT_CLOUD_HOST.to_string())
}

enum RegisterError {
    /// No registration code has been provisioned into NVS yet - not a failure,
    /// just something to wait out.
    MissingCode,
    Failed(anyhow::Error),
}

/// Returns the device's id/secret pair, from NVS if already registered, or by
/// registering (and then persisting the result to NVS) if a registration code
/// is available but the device hasn't registered yet.
///
/// A registration code in NVS always wins over any already-stored id/secret -
/// its presence means "(re)register with this code", so it's treated as
/// authoritative even if the device already has credentials from a previous
/// registration. Only once that code has been consumed (removed on success,
/// or replaced with `NVS_VALUE_REGISTRATION_REJECTED` on failure) do the
/// stored id/secret get reused as-is.
fn ensure_device_credentials(
    nvs: &EspNvs<NvsDefault>,
    global: &Rc<RefCell<Global>>,
) -> Result<(String, String), RegisterError> {
    let mut code_buf = [0u8; 64];
    let registration_code = match nvs.get_str(NVS_KEY_REGISTRATION_CODE, &mut code_buf) {
        Ok(Some(code)) if code != NVS_VALUE_REGISTRATION_REJECTED => Some(code),
        Ok(_) => None,
        Err(e) => return Err(RegisterError::Failed(e.into())),
    };

    let Some(registration_code) = registration_code else {
        let mut id_buf = [0u8; 256];
        let mut secret_buf = [0u8; 256];
        let mut name_buf = [0u8; 128];

        let credentials = match (
            nvs.get_str(NVS_KEY_DEVICE_ID, &mut id_buf),
            nvs.get_str(NVS_KEY_DEVICE_SECRET, &mut secret_buf),
        ) {
            (Ok(Some(id)), Ok(Some(secret))) => Ok((id.to_string(), secret.to_string())),
            _ => Err(RegisterError::MissingCode),
        };

        if credentials.is_ok() {
            global.borrow_mut().cloud_status.device_name =
                nvs.get_str(NVS_KEY_DEVICE_NAME, &mut name_buf).ok().flatten().map(str::to_string);
        }

        return credentials;
    };

    // Checked alongside the registration code, before anything below has a
    // chance to wipe stored credentials or make an actual HTTP request.
    // `command_listener`'s "#reg" handler validates both its code and
    // device-name arguments before writing either, so a registration code
    // with no matching valid device name shouldn't be reachable going
    // forward - but a device that got stuck in that inconsistent state
    // before that fix landed (or had NVS poked directly) would otherwise
    // fail here forever, every retry, with nothing short of a manual NVS
    // fix able to clear it. Instead, treat it the same as no code ever
    // having been stored: clear the stray code and report `MissingCode`
    // (not a fabricated placeholder name, still) - the caller already
    // turns that into `RegistrationStatus::WaitingForCode` and keeps
    // retrying, so a fresh, valid "#reg" recovers the device on its own.
    let mut name_buf = [0u8; 128];
    let device_name = match nvs.get_str(NVS_KEY_DEVICE_NAME, &mut name_buf) {
        Ok(Some(name)) if (DEVICE_NAME_MIN_LEN..=DEVICE_NAME_MAX_LEN).contains(&name.len()) => name,
        Ok(_) => {
            if let Err(e) = nvs.remove(NVS_KEY_REGISTRATION_CODE) {
                log::error!("Failed to remove stray '{NVS_KEY_REGISTRATION_CODE}' from NVS: {e:?}");
            }
            return Err(RegisterError::MissingCode);
        }
        Err(e) => return Err(RegisterError::Failed(e.into())),
    };

    // Only reached (and thus only shown as `Registering`) once there's an
    // actual registration attempt about to happen, not while just waiting for
    // a code or reusing already-stored credentials.
    {
        let mut g = global.borrow_mut();
        g.cloud_status.registration = RegistrationStatus::Registering;
        g.cloud_status.device_name = Some(device_name.to_string());
    }

    // A registration code is present, so it takes priority: wipe whatever
    // credentials might already be stored before attempting to register -
    // this is what actually performs a "reregister" (see `Global::reregister`).
    if let Err(e) = nvs.remove(NVS_KEY_DEVICE_ID) {
        log::error!("Failed to remove '{NVS_KEY_DEVICE_ID}' from NVS: {e:?}");
    }
    if let Err(e) = nvs.remove(NVS_KEY_DEVICE_SECRET) {
        log::error!("Failed to remove '{NVS_KEY_DEVICE_SECRET}' from NVS: {e:?}");
    }

    let host = read_cloud_host(nvs);

    let (device_id, device_secret) = match register_device(registration_code, device_name, &host) {
        Ok(credentials) => credentials,
        Err(e) => {
            if let Err(e) = nvs.set_str(NVS_KEY_REGISTRATION_CODE, NVS_VALUE_REGISTRATION_REJECTED) {
                log::error!("Failed to mark '{NVS_KEY_REGISTRATION_CODE}' as rejected in NVS: {e:?}");
            }
            return Err(RegisterError::Failed(e));
        }
    };

    nvs.set_str(NVS_KEY_DEVICE_ID, &device_id)
        .map_err(|e| RegisterError::Failed(e.into()))?;
    nvs.set_str(NVS_KEY_DEVICE_SECRET, &device_secret)
        .map_err(|e| RegisterError::Failed(e.into()))?;

    if let Err(e) = nvs.remove(NVS_KEY_REGISTRATION_CODE) {
        log::error!("Failed to remove '{NVS_KEY_REGISTRATION_CODE}' from NVS: {e:?}");
    }

    Ok((device_id, device_secret))
}

/// Performs the one-time HTTP registration handshake: `POST`s
/// `#reg,<registration_code>,<mac_address>,<device_name>\r\n` as a plaintext
/// body to `/api/devices/register` and expects a plaintext
/// `#id,<id>,<secret>,<name>` response back.
fn register_device(registration_code: &str, device_name: &str, host: &str) -> anyhow::Result<(String, String)> {
    let url = format!("https://{host}/api/devices/register");
    let mac_address = device_mac_address()?;
    let payload = format!("#reg,{registration_code},{mac_address},{device_name}\r\n");

    let http_config = HttpConfiguration {
        server_certificate: Some(X509::pem_until_nul(ROOT_CA_CERT)),
        ..Default::default()
    };
    let mut client = HttpClient::wrap(EspHttpConnection::new(&http_config)?);

    let content_length = payload.len().to_string();
    let headers = [
        ("content-type", "text/plain"),
        ("content-length", content_length.as_str()),
    ];

    let mut request = client.post(&url, &headers)?;
    request.write_all(payload.as_bytes())?;
    request.flush()?;

    let mut response = request.submit()?;

    let status = response.status();
    if !(200..300).contains(&status) {
        anyhow::bail!("registration request failed with HTTP status {status}");
    }

    let mut buf = [0u8; 256];
    let bytes_read = io::try_read_full(&mut response, &mut buf).map_err(|e| e.0)?;
    let body = std::str::from_utf8(&buf[..bytes_read])?;

    parse_registration_response(body)
}

/// The device's Wi-Fi station MAC address, formatted as `AA:BB:CC:DD:EE:FF`.
///
/// Reads straight from eFuse via `esp_read_mac` rather than through the `wifi`
/// handle in `main` - this way `register_device` doesn't need one threaded in,
/// and the base MAC eFuse holds is available independent of Wi-Fi driver state.
fn device_mac_address() -> anyhow::Result<String> {
    let mut mac = [0u8; 6];

    unsafe {
        esp!(esp_read_mac(
            mac.as_mut_ptr(),
            esp_mac_type_t_ESP_MAC_WIFI_STA,
        ))?;
    }

    Ok(mac.iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(":"))
}

fn parse_registration_response(body: &str) -> anyhow::Result<(String, String)> {
    let mut parts = body.trim().split(',');

    if parts.next() != Some("#id") {
        anyhow::bail!("unexpected registration response: {body:?}");
    }

    let device_id = parts
        .next()
        .ok_or_else(|| anyhow::anyhow!("registration response missing device id: {body:?}"))?;
    let device_secret = parts
        .next()
        .ok_or_else(|| anyhow::anyhow!("registration response missing device secret: {body:?}"))?;

    Ok((device_id.to_string(), device_secret.to_string()))
}

async fn wait_for_wifi(global: &Rc<RefCell<Global>>) {
    while !matches!(global.borrow().wifi_status.status, WifiConnectionStatus::Connected) {
        Timer::after_millis(200).await;
    }
}

fn log_websocket_event(event: &Result<WebSocketEvent, EspIOError>) {
    let event = match event {
        Ok(event) => event,
        Err(e) => {
            log::error!("Websocket error: {e:?}");
            return;
        }
    };

    match event.event_type {
        WebSocketEventType::BeforeConnect => log::info!("Websocket connecting..."),
        WebSocketEventType::Connected => log::info!("Websocket connected"),
        WebSocketEventType::Disconnected => log::warn!("Websocket disconnected"),
        WebSocketEventType::Close(reason) => log::info!("Websocket close, reason: {reason:?}"),
        WebSocketEventType::Closed => log::info!("Websocket closed"),
        WebSocketEventType::Text(text) => log::info!("Websocket recv text: {text}"),
        WebSocketEventType::Binary(data) => {
            log::info!("Websocket recv binary ({} bytes)", data.len())
        }
        WebSocketEventType::Ping => log::debug!("Websocket ping"),
        WebSocketEventType::Pong => log::debug!("Websocket pong"),
    }
}
