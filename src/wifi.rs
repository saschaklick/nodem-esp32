use std::cell::RefCell;
use std::rc::Rc;

use embassy_time::Timer;
use esp_idf_svc::ipv4::IpInfo;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};
use esp_idf_svc::wifi::{AsyncWifi, AuthMethod, ClientConfiguration, Configuration, EspWifi};

use crate::global::{Global, WifiConnectionStatus};

// pub(crate): `uart_task`'s command handler writes these same keys when it
// gets provisioned over UART - see `crate::command_listener::CommandListener`.
pub(crate) const NVS_NAMESPACE: &str = "wifi";
pub(crate) const NVS_KEY_SSID: &str = "ssid";
pub(crate) const NVS_KEY_PASS: &str = "pass";

// Attempt counters for the currently-configured SSID - incremented on every
// `do_connect_wifi` call, reset once `wifi_reconnect` fires (new "#wifi"
// provisioned) since the counts no longer say anything about the new network.
// pub(crate): also directly removed by `uart_task`'s "#factory" command - see
// `crate::command_listener::CommandListener`.
pub(crate) const NVS_KEY_CONNECT_SUCCESS_COUNT: &str = "conn_ok";
pub(crate) const NVS_KEY_CONNECT_FAIL_COUNT: &str = "conn_fail";

/// Keeps the Wi-Fi connection up for as long as the device runs.
///
/// Won't attempt anything until NVS has an SSID and password in it - there's
/// no compiled-in fallback, unlike the old hardcoded consts this replaced.
/// Sits in `WaitingForConfiguration` (polling NVS every 2s) until whoever
/// provisions the device writes those two keys.
///
/// Once configured: connects, then sits in the inner loop below polling for
/// either the link dropping on its own (AP reboot, moving out of range, ...)
/// or `Global::wifi_reconnect` being set (by `uart_task`, after a fresh
/// "#wifi" write) - either way it disconnects (if not already) and
/// loops back around to reconnect, re-reading NVS so a credential change
/// actually takes effect rather than reusing what was read at startup.
/// Never returns.
///
/// Doesn't touch the websocket client directly on a Wi-Fi drop or reconnect:
/// `websocket_task`'s `EspWebSocketClient` has its own auto-reconnect
/// (`disable_auto_reconnect: false`) and detects a dead connection the same
/// way regardless of *why* the underlying link went away, so a Wi-Fi credential
/// change looks to it like any other transient network blip - no special
/// handling needed there.
pub async fn wifi_task(
    wifi: &mut AsyncWifi<EspWifi<'static>>,
    global: Rc<RefCell<Global>>,
    nvs: EspDefaultNvsPartition,
) {
    let wifi_nvs = match EspNvs::new(nvs, NVS_NAMESPACE, true) {
        Ok(wifi_nvs) => wifi_nvs,
        Err(e) => {
            log::error!("Failed to open '{NVS_NAMESPACE}' NVS namespace: {e:?}");
            global.borrow_mut().wifi_status.status = WifiConnectionStatus::WaitingForConfiguration;
            return;
        }
    };

    loop {
        let (ssid, password) = loop {
            match read_wifi_config(&wifi_nvs) {
                Some(config) => break config,
                None => {
                    global.borrow_mut().wifi_status.status = WifiConnectionStatus::WaitingForConfiguration;
                    Timer::after_secs(2).await;
                }
            }
        };

        {
            let mut g = global.borrow_mut();
            g.wifi_status.status = WifiConnectionStatus::Connecting;
            g.wifi_status.ssid = Some(ssid.clone());
            g.wifi_status.ip = None;
            g.wifi_status.subnet_mask = None;
            g.wifi_status.gateway = None;
            g.wifi_status.rssi = None;
        }

        let result = do_connect_wifi(wifi, &ssid, &password).await;

        increment_counter(
            &wifi_nvs,
            if result.is_ok() { NVS_KEY_CONNECT_SUCCESS_COUNT } else { NVS_KEY_CONNECT_FAIL_COUNT },
        );

        {
            let mut g = global.borrow_mut();
            match &result {
                Ok(ip_info) => {
                    g.wifi_status.status = WifiConnectionStatus::Connected;
                    g.wifi_status.ip = Some(ip_info.ip);
                    g.wifi_status.gateway = Some(ip_info.subnet.gateway);
                    g.wifi_status.subnet_mask = Some(ip_info.subnet.mask.into());
                }
                Err(e) => {
                    log::error!("Wifi connect failed: {e:?}");
                    g.wifi_status.status = WifiConnectionStatus::Failed(e.to_string());
                }
            }
        }

        if result.is_err() {
            // Back off before retrying so a persistently unreachable AP (wrong
            // password, out of range, ...) doesn't spin this loop hot.
            Timer::after_secs(5).await;
            continue;
        }

        // Connected - poll rather than the previous event-driven `wifi_wait`,
        // so `wifi_reconnect` gets noticed promptly instead of only whenever
        // the link happens to drop on its own.
        loop {
            Timer::after_secs(1).await;

            if global.borrow().wifi_reconnect {
                global.borrow_mut().wifi_reconnect = false;
                log::info!("Wifi reconnect requested, disconnecting to pick up new settings...");
                reset_counter(&wifi_nvs, NVS_KEY_CONNECT_SUCCESS_COUNT);
                reset_counter(&wifi_nvs, NVS_KEY_CONNECT_FAIL_COUNT);
                if let Err(e) = wifi.disconnect().await {
                    log::error!("Wifi disconnect failed: {e:?}");
                }
                break;
            }

            match wifi.is_connected() {
                Ok(true) => {
                    if let Ok(rssi) = wifi.wifi().get_rssi() {
                        global.borrow_mut().wifi_status.rssi = Some(rssi);
                    }
                }
                Ok(false) => {
                    log::warn!("Wifi disconnected, reconnecting...");
                    break;
                }
                Err(e) => log::error!("Wifi status check failed: {e:?}"),
            }
        }

        global.borrow_mut().wifi_status.status = WifiConnectionStatus::Connecting;
    }
}

fn increment_counter(nvs: &EspNvs<NvsDefault>, key: &str) {
    let count = nvs.get_u32(key).ok().flatten().unwrap_or(0);
    if let Err(e) = nvs.set_u32(key, count.saturating_add(1)) {
        log::error!("Failed to update NVS counter '{key}': {e:?}");
    }
}

fn reset_counter(nvs: &EspNvs<NvsDefault>, key: &str) {
    if let Err(e) = nvs.set_u32(key, 0) {
        log::error!("Failed to reset NVS counter '{key}': {e:?}");
    }
}

fn read_wifi_config(nvs: &EspNvs<NvsDefault>) -> Option<(String, String)> {
    let mut ssid_buf = [0u8; 64];
    let mut pass_buf = [0u8; 64];

    let ssid = nvs.get_str(NVS_KEY_SSID, &mut ssid_buf).ok().flatten()?;
    let password = nvs.get_str(NVS_KEY_PASS, &mut pass_buf).ok().flatten()?;

    Some((ssid.to_string(), password.to_string()))
}

async fn do_connect_wifi(
    wifi: &mut AsyncWifi<EspWifi<'static>>,
    ssid: &str,
    password: &str,
) -> anyhow::Result<IpInfo> {
    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: ssid.try_into().unwrap(),
        password: password.try_into().unwrap(),
        auth_method: AuthMethod::WPA2Personal,
        ..Default::default()
    }))?;

    // The driver stays started across a disconnect - only reconnecting needs
    // to happen - so only start it if this is the first connect attempt.
    if !wifi.is_started()? {
        wifi.start().await?;
        log::info!("Wifi started");
    }

    wifi.connect().await?;
    log::info!("Wifi connected");

    wifi.wait_netif_up().await?;
    log::info!("Wifi netif up");

    let ip_info = wifi.wifi().sta_netif().get_ip_info()?;
    log::info!("Wifi connected, DHCP info: {ip_info:?}");

    Ok(ip_info)
}
