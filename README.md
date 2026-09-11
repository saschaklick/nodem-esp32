# VS Code tasks

Tasks defined in [.vscode/tasks.json](.vscode/tasks.json), run via **Terminal → Run Task...** (or `Ctrl+Shift+B` for the default build task).

## rust: cargo build

Debug build (`cargo build`). Fast to iterate with; not what gets flashed to the device (see `sdkconfig.defaults`'s debug-vs-release notes if you need to check).

## rust: cargo build --release

Release build (`cargo build --release`) - optimized (`opt-level = "s"`, LTO, stripped, per `Cargo.toml`'s `[profile.release]`) and small enough to fit the target partition layout. This is the build the remaining tasks below package/flash.

## rust: cargo bloat

Runs `cargo bloat --symbols-section .flash.text` to break down what's actually consuming flash space in the built binary - useful when a change unexpectedly grows the image or you're hunting for something to trim.

## rust: build esp32c3 release

Depends on **rust: cargo build --release** (runs it first). Packages that release build into a single flashable image via `espflash save-image`:
- Chip: `esp32c3`, 4MB flash
- Merges the app binary with the bootloader (`target/riscv32imc-esp-espidf/release/build/bootloader.bin`) and the partition table (`partitions.csv`)
- Output: `nodem_ESP32-C3.bin` at the repo root

This is the file to flash to hardware or hand off for distribution - use `espflash write-bin`/`espflash flash` (or the vendor's flashing tool) with `nodem_ESP32-C3.bin` afterward.
