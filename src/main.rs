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
    esp_idf_svc::log::EspLogger::initialize_default();

    // A release build never logs over UART at all, from boot - not just
    // once a real client starts talking the "#..." protocol over it, the
    // way a debug build does (see `uart::uart_task`'s doc comment for that
    // dynamic, first-byte-triggered switch, which still applies here too but
    // starts from an already-silent baseline). This isn't done here at
    // runtime - `Cargo.toml`'s `log` dependency has
    // `features = ["release_max_level_off"]`, which compiles every `log::`
    // macro call above that level out of the binary entirely whenever
    // `debug_assertions` is disabled, rather than just filtering them at
    // runtime: smaller/faster release binary, and no risk of a call site
    // that captures something expensive still paying for it just to have
    // the result thrown away.

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
    let global = Rc::new(RefCell::new(Global::new()));

    let executor: LocalExecutor = Default::default();

    executor.spawn(heartbeat::heartbeat_task(global.clone())).detach();
    executor.spawn(iled::iled_task(global.clone(), nvs.clone())).detach();
    executor.spawn(nodem::nodem_task(global.clone())).detach();
    executor.spawn(oled::oled_task(i2c, global.clone())).detach();
    executor.spawn(uart::uart_task(uart, global.clone(), nvs.clone())).detach();
    executor.spawn(websocket::websocket_task(global.clone(), nvs.clone())).detach();
    executor.spawn(wifi::wifi_task(&mut wifi, global, nvs)).detach();

    block_on(executor.run(core::future::pending::<()>()));

    Ok(())
}
