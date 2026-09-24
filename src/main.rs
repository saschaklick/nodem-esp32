use std::cell::RefCell;
use std::rc::Rc;

use edge_executor::{block_on, LocalExecutor};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::uart::{config::Config as UartConfig, AsyncUartDriver};
use esp_idf_svc::hal::units::FromValueType;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::timer::EspTaskTimerService;
use esp_idf_svc::wifi::{AsyncWifi, EspWifi};

mod command_listener;
mod driver;
mod global;
mod heartbeat;
mod iled;
mod nodem;
mod oled;
mod uart;
mod websocket;
mod wifi;

use global::Global;

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    let logger = esp_idf_svc::log::init_from_esp_idf();
    #[cfg(not(debug_assertions))]
    if option_env!("LOG_LEVEL").is_none() {
        log::set_max_level(log::LevelFilter::Off);
    }
    apply_log_levels(logger.filter());

    // A release build doesn't log (the Rust side - ESP-IDF's own C
    // components keep their `info` default) unless `LOG_LEVEL` says so - not
    // just once a real client starts talking the "#..." protocol over UART,
    // the way a debug build does (see `uart::uart_task`'s doc comment for that
    // dynamic, first-byte-triggered switch, which still applies here too but
    // starts from an already-silent baseline). A runtime filter rather than
    // compiling logging out (`log`'s `release_max_level_off` feature, as
    // before), so `LOG_LEVEL=info cargo run --release` works too - at the
    // cost of the log strings staying in the binary.

    let peripherals = Peripherals::take()?;
    let sys_loop = EspSystemEventLoop::take()?;
    let timer_service = EspTaskTimerService::new()?;
    let nvs = EspDefaultNvsPartition::take()?;

    let mut wifi = AsyncWifi::wrap(
        EspWifi::new(peripherals.modem, sys_loop.clone(), Some(nvs.clone()))?,
        sys_loop,
        timer_service,
    )?;

    let i2c_config = I2cConfig::new().baudrate(1000.kHz().into());
    let i2c = I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio8, // SDA
        peripherals.pins.gpio9, // SCL
        &i2c_config,
    )?;

    let uart_config = UartConfig::new().baudrate(115_200.Hz());
    let uart = AsyncUartDriver::new(
        peripherals.uart0,
        peripherals.pins.gpio21, // TX
        peripherals.pins.gpio20, // RX
        Option::<esp_idf_svc::hal::gpio::AnyInputPin>::None,
        Option::<esp_idf_svc::hal::gpio::AnyOutputPin>::None,
        &uart_config,
    )?;

    // `iled::build_i2s` builds (and, when a "#iled" command changes
    // `TimingConfig`, rebuilds) its own I2S driver on GPIO1/2/3 + I2S0,
    // via `unsafe` `steal()` calls - see its doc comment for why it needs to
    // own that rather than being handed an already-constructed driver from
    // here.
    let global = Rc::new(RefCell::new(Global::new(nodem::read_nodem_config(nvs.clone()))));

    let executor: LocalExecutor = Default::default();

    executor.spawn(heartbeat::heartbeat_task(global.clone())).detach();
    executor.spawn(iled::iled_task(global.clone(), nvs.clone())).detach();
    executor.spawn(nodem::nodem_task(global.clone(), nvs.clone())).detach();
    executor.spawn(oled::oled_task(i2c, global.clone(), nvs.clone())).detach();
    executor.spawn(uart::uart_task(uart, global.clone(), nvs.clone())).detach();
    executor.spawn(websocket::websocket_task(global.clone(), nvs.clone())).detach();
    executor.spawn(wifi::wifi_task(&mut wifi, global, nvs)).detach();

    block_on(executor.run(core::future::pending::<()>()));

    Ok(())
}

/// Log levels from `LOG_LEVEL`, read at *build* time - the device can't be
/// handed anything at runtime, so e.g. `LOG_LEVEL=debug cargo run`. Cargo
/// rebuilds on its own whenever the variable changes (`option_env!` is
/// tracked). Same shape as `RUST_LOG`: a comma-separated list of either a
/// bare level (`off`/`error`/`warn`/`info`/`debug`/`trace`), applied to
/// everything - ESP-IDF's own C components (wifi, lwip, ...) included - or
/// `<target>=<level>` for a single Rust module path or ESP-IDF tag, e.g.
/// `LOG_LEVEL=warn,nodem_esp32=debug`. Unset keeps ESP-IDF's default (`info`,
/// see sdkconfig.defaults).
///
/// Levels above `info` need `CONFIG_LOG_MAXIMUM_LEVEL` raised, which only
/// dev builds do (sdkconfig.defaults.debug) - release builds go up to `info`.
/// Unset, release builds keep the Rust side silent (see `main`).
fn apply_log_levels(filter: &esp_idf_svc::log::EspIdfLogFilter) {
    let Some(spec) = option_env!("LOG_LEVEL") else { return; };

    let mut max = log::max_level();
    for entry in spec.split(',').map(str::trim).filter(|e| !e.is_empty()) {
        let (target, level) = entry.split_once('=').map_or(("*", entry), |(t, l)| (t.trim(), l.trim()));
        let Ok(level) = level.parse::<log::LevelFilter>() else {
            log::warn!("LOG_LEVEL: invalid level in '{entry}'");
            continue;
        };
        if target == "*" {
            max = level;
        } else {
            max = max.max(level);
        }
        if let Err(e) = filter.set_target_level(target, level) {
            log::warn!("LOG_LEVEL: setting '{entry}' failed: {e:?}");
        }
    }
    log::set_max_level(max);
}
