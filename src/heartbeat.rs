use std::cell::RefCell;
use std::rc::Rc;

use embassy_time::{Duration, Instant, Timer};
use embedded_svc::ota::SlotState;
use esp_idf_svc::ota::EspOta;

use crate::global::Global;

/// How long a freshly OTA-updated firmware has to keep running before it
/// counts as working - until then the bootloader would roll back to the
/// previous slot on the next reset (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`).
const ROLLBACK_PHASE: Duration = Duration::from_secs(60);

/// Logs the Wi-Fi/cloud/OLED status every 5s, and owns the rollback phase:
/// fills in `Global::sys_status`'s OTA slot and rollback state at startup,
/// then marks the running slot valid once `ROLLBACK_PHASE` has passed since
/// boot (`Instant` counts from boot).
pub async fn heartbeat_task(global: Rc<RefCell<Global>>) {
    let mut ota = match EspOta::new() {
        Ok(ota) => Some(ota),
        Err(e) => {
            log::error!("OTA init failed: {e:?}");
            None
        }
    };

    if let Some(slot) = ota.as_ref().and_then(|ota| ota.get_running_slot().inspect_err(|e| log::error!("OTA running slot lookup failed: {e:?}")).ok()) {
        let mut g = global.borrow_mut();
        g.sys_status.ota_slot = match slot.label.as_str() {
            "ota_0" => Some(0),
            "ota_1" => Some(1),
            _ => None,
        };
        g.sys_status.rollback_pending = matches!(slot.state, SlotState::Unverified);
    }

    loop {
        if global.borrow().sys_status.rollback_pending && Instant::now().as_ticks() >= ROLLBACK_PHASE.as_ticks() {
            if let Some(ota) = ota.as_mut() {
                match ota.mark_running_slot_valid() {
                    Ok(()) => {
                        log::info!("Rollback phase over, running slot marked valid");
                        global.borrow_mut().sys_status.rollback_pending = false;
                    }
                    Err(e) => log::error!("Marking running slot valid failed: {e:?}"),
                }
            }
        }

        {
            let g = global.borrow();
            log::info!("{}", g.sys_status);
            log::info!(
                "{} | ssid={} ip={} mask={} gateway={} rssi={}",
                g.wifi_status,
                g.wifi_status.ssid.as_deref().unwrap_or("-"),
                g.wifi_status.ip.map(|v| v.to_string()).unwrap_or_else(|| "-".to_string()),
                g.wifi_status.subnet_mask.map(|v| v.to_string()).unwrap_or_else(|| "-".to_string()),
                g.wifi_status.gateway.map(|v| v.to_string()).unwrap_or_else(|| "-".to_string()),
                g.wifi_status.rssi.map(|v| format!("{v}dBm")).unwrap_or_else(|| "-".to_string()),
            );
            log::info!(
                "{} | host={} device={} | {} errors={} reinits={}",
                g.cloud_status,
                g.cloud_status.host.as_deref().unwrap_or("-"),
                g.cloud_status.device_name.as_deref().unwrap_or("-"),
                g.oled_status,
                g.oled_status.errors,
                g.oled_status.reinits,
            );
        }
        Timer::after_secs(5).await;
    }
}
