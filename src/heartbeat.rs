use std::cell::RefCell;
use std::rc::Rc;

use embassy_time::Timer;

use crate::global::Global;

pub async fn heartbeat_task(global: Rc<RefCell<Global>>) {
    loop {
        {
            let g = global.borrow();
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
                "{} | host={} device={}",
                g.cloud_status,
                g.cloud_status.host.as_deref().unwrap_or("-"),
                g.cloud_status.device_name.as_deref().unwrap_or("-"),
            );
        }
        Timer::after_secs(5).await;
    }
}
