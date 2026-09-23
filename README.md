# VS Code tasks

Tasks defined in [.vscode/tasks.json](.vscode/tasks.json), run via **Terminal → Run Task...** (or `Ctrl+Shift+B` for the default build task).

## rust: cargo build

Debug build (`cargo build`). Fast to iterate with; not what gets flashed to the device (see `sdkconfig.defaults`'s debug-vs-release notes if you need to check).

## rust: cargo build --release

Release build (`cargo build --release`) - optimized (`opt-level = "s"`, LTO, stripped, per `Cargo.toml`'s `[profile.release]`) and small enough to fit the target partition layout. This is the build the remaining tasks below package/flash.

## rust: cargo bloat

Runs `cargo bloat --symbols-section .flash.text` to break down what's actually consuming flash space in the built binary - useful when a change unexpectedly grows the image or you're hunting for something to trim.

## rust: build esp32c3 release

Depends on **rust: cargo build --release** (runs it first). Runs `scripts/sign.sh`, which signs the release build and packages it:
- `nodem_ESP32-C3-ota.bin` - the signed app image, the file to upload via `ota`
- `nodem_ESP32-C3.bin` - bootloader + release partition table (`partitions.release.csv`) + blank `otadata` + signed app, merged into one image to write at `0x0` (`espflash write-bin 0x0 nodem_ESP32-C3.bin`) - meant for factory-fresh devices: it leaves the `pkg` partition untouched, but wipes `nvs` (Wi-Fi, registration and other settings). `cargo run --release` doesn't use it, and keeps `nvs`.

Both are written to the repo root (gitignored).

### Signing

Release firmware only accepts OTA updates signed with the same RSA-3072 key as the firmware it's running, and won't start at all unless it's signed itself (see `sdkconfig.defaults.release`). The private key is read from `$OTA_SIGNING_KEY`, default `keys/ota_signing_key.pem` (gitignored - never commit it). Generate one once with:

```
scripts/gen-key.sh
```

Both scripts use the `espsecure.py`/`esptool.py` from the ESP-IDF Python environment in `.embuild/` (installed by the first build), not whatever is on `PATH`. Losing the key means devices in the field can only be updated over USB again. `cargo run --release` signs too (via `scripts/flash.sh`), so it needs the key as well. Dev builds aren't signed and don't support OTA.

## Log level

Set at build time via `LOG_LEVEL`, e.g. `LOG_LEVEL=debug cargo run` - cargo rebuilds whenever it changes. Takes a level (`off`/`error`/`warn`/`info`/`debug`/`trace`) for everything, ESP-IDF's own components included, and/or `<target>=<level>` pairs for single Rust modules or ESP-IDF tags, comma-separated: `LOG_LEVEL=warn,nodem_esp32=debug`. Unset means `info` in dev builds and no Rust logging at all in release builds (ESP-IDF's components still log at `info`). Release builds go up to `info` at most. The nodem-rs library's target is `nodem_rs`, e.g. `LOG_LEVEL=warn,nodem_rs=info`. Logging also switches off for good once the first byte arrives on UART0 (see `uart_task`).

## Flashing speed

`cargo run` flashes at 1843200 baud. Override with `FLASH_BAUD`, e.g. `FLASH_BAUD=3000000 cargo run` - see `scripts/flash.sh` for the usual values and which adapters handle them. If flashing fails to connect or verify, go one step lower.
