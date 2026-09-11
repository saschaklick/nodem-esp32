use std::cell::RefCell;
use std::rc::Rc;

use embassy_time::{Duration, Instant, Timer};
use esp_idf_svc::partition::{EspMemMapType, EspPartition};
use nodem_rs::runtime::Runtime;

use crate::command_listener::PKG_PARTITION_LABEL;
use crate::global::{CloudConnectionStatus, Global, WifiConnectionStatus, DISPLAY_BUFFER_CONSUMERS};

/// How long Wi-Fi and cloud both need to have been continuously connected
/// before the status overlay stops being drawn.
const HIDE_REPORT_AFTER: Duration = Duration::from_secs(5);

/// Advances the `nodem_rs` DOM runtime and re-renders the Wi-Fi status overlay
/// into `Global::display_buffer`, 60 times a second. Only touches the in-memory
/// framebuffer (marking it dirty for every `Global::display_buffer_dirty`
/// consumer - `oled_task` and `iled_task` - at once) - never talks to a
/// display itself.
///
/// The overlay is suppressed once Wi-Fi and cloud have both been connected for
/// `HIDE_REPORT_AFTER` straight - `connected_since` tracks the start of the
/// current unbroken "both connected" streak (reset to `None` the moment either
/// drops), so a hidden report reappears immediately on any disconnect. Safe to
/// just skip the draw calls below rather than explicitly clearing the overlay
/// area first: `g.runtime.run()` clears the whole surface every frame (outside
/// the intro/loader-busy states) before anything here draws to it.
pub async fn nodem_task(global: Rc<RefCell<Global>>) {
    let mut connected_since: Option<Instant> = None;

    // One-time check at startup: if a "pkg" partition already exists (i.e.
    // it was flashed by a previous "#pkg" upload), wire `pkg_reload` up to it
    // so the loop below loads it on its very first iteration, the same way
    // it would after a fresh upload - see `CommandListener::process_loader_end`.
    {
        let mut g = global.borrow_mut();
        match unsafe { EspPartition::new(PKG_PARTITION_LABEL) } {
            Ok(Some(mut partition)) => {
                // Layout: b"PKG0" magic, then a native-endian u32 giving the
                // pkg's total length (magic included) - see `Media::load_pkg`,
                // which CRCs every byte up to that length.
                let mut header = [0u8; 8];
                match partition.read(0, &mut header) {
                    Ok(()) if &header[0..4] == b"PKG0" => {
                        let len = u32::from_ne_bytes(header[4..8].try_into().unwrap()) as usize;
                        match unsafe { partition.mmap(0, len, EspMemMapType::Data) } {
                            Ok(mapped) => {
                                // Leaked deliberately - see `process_loader_end`'s
                                // matching comment.
                                g.pkg_reload = Some((mapped.start() as *const u8, len));
                                core::mem::forget(mapped);
                            }
                            Err(e) => log::error!("'{PKG_PARTITION_LABEL}' partition mmap failed: {e:?}"),
                        }
                    }
                    Ok(()) => log::info!("'{PKG_PARTITION_LABEL}' partition has no pkg, skipping reload"),
                    Err(e) => log::error!("'{PKG_PARTITION_LABEL}' partition read failed: {e:?}"),
                }
            }
            Ok(None) => {}
            Err(e) => log::error!("'{PKG_PARTITION_LABEL}' partition lookup failed: {e:?}"),
        }
    }

    loop {
        {
            let mut g = global.borrow_mut();

            if let Some((ptr, len)) = g.pkg_reload.take() {
                let ret = g.runtime.surface.media.load_pkg(ptr, len, 2) as u8;
                log::info!("pkg reload: {ret}");
            }

            g.runtime.run();

            let both_connected = matches!(g.wifi_status.status, WifiConnectionStatus::Connected)
                && matches!(g.cloud_status.connection, CloudConnectionStatus::Connected);

            connected_since = match (both_connected, connected_since) {
                (true, since @ Some(_)) => since,
                (true, None) => Some(Instant::now()),
                (false, _) => None,
            };

            let show_report = connected_since.map_or(true, |since| since.elapsed() < HIDE_REPORT_AFTER);

            if show_report {
                let font = 0;
                let messages = format!("{}{{br}}{}", g.wifi_status, g.cloud_status);
                let size = g.runtime.surface.get_text_size(nodem_rs::media::Identifier::Index(font), messages.as_str());
                g.runtime.surface.draw_rect(nodem_rs::Area { point: nodem_rs::Point { x: 0, y: 0 }, size: nodem_rs::Size { width: size.width + 6, height: size.height + 6 } }, 1);
                g.runtime.surface.fill_rect(nodem_rs::Area { point: nodem_rs::Point { x: 1, y: 1 }, size: nodem_rs::Size { width: size.width + 4, height: size.height + 4 } }, 0);
                g.runtime.surface.draw_text(nodem_rs::media::Identifier::Index(font), messages.as_str(), nodem_rs::Point { x: 3, y: 3 });
            }

            g.display_buffer_dirty = [true; DISPLAY_BUFFER_CONSUMERS];
        }

        Timer::after_millis(1000 / 60).await;
    }
}
