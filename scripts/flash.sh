#!/bin/sh
# Cargo runner (see .cargo/config.toml): flashes the ELF cargo hands us as the
# last argument, using partitions.<profile>.csv for whichever profile
# (target/<triple>/<profile>/...) it was built with, and the bootloader
# esp-idf-sys built from sdkconfig.defaults (for
# `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`) rather than espflash's bundled
# default. `otadata` is erased on every flash so the bootloader starts from
# `ota_0`, where the app is written, rather than from whichever slot a
# previous OTA update last selected. The device is always reset into the
# new firmware afterwards (`--after hard-reset`), then its serial output is
# followed with `espflash monitor` (CTRL+C exits, CTRL+R resets). Not with
# `--non-interactive`: espflash 3.x still switches the terminal to raw mode
# there but then ignores key presses, so CTRL+C can't exit it anymore.
#
# Release builds must be signed (see sdkconfig.defaults.release), which
# `espflash flash` can't do - those go through scripts/sign.sh, and the
# bootloader, partition table, blank otadata and signed app are then written
# one by one with esptool. Not as sign.sh's merged image: that one is padded
# with 0xFF between its parts, which covers (and so wipes) nvs.
set -e
for elf; do :; done
dir=$(dirname "$elf")
profile=$(basename "$dir")
root=$(dirname "$0")/..
# Flashing speed, e.g. `FLASH_BAUD=3000000 cargo run`. Typical steps:
# 115200 (espflash's default), 460800, 921600, 1500000, 1843200 (the default
# here), 2000000, 3000000. How high actually works depends on the USB-serial
# adapter and cable - FTDI FT232R (0403:6001) goes up to 3000000, CH340-family
# (1a86:*) chips usually top out at 2000000. Too high shows up as the flash
# failing to connect or verify; step back down one notch then. Only the
# flashing itself - the monitor always reads the firmware's own 115200.
baud=${FLASH_BAUD:-2000000}

if [ "$profile" = release ]; then
    tmp=$(mktemp -d)
    trap 'rm -rf "$tmp"' EXIT
    "$root/scripts/sign.sh" "$elf" "$tmp"

    # ESP-IDF's own python env (installed by esp-idf-sys) has esptool.
    pyenv=$(ls -d "$root"/.embuild/espressif/python_env/*/bin 2>/dev/null | head -1)
    [ -n "$pyenv" ] && PATH="$pyenv:$PATH"

    # Offsets as in partitions.release.csv.
    head -c 8192 /dev/zero | tr '\0' '\377' > "$tmp/otadata.bin"
    esptool.py --chip esp32c3 --baud $baud --after hard_reset write_flash \
        0x0 "$dir/build/bootloader.bin" \
        0x8000 "$dir/build/partition-table.bin" \
        0xd000 "$tmp/otadata.bin" \
        0x10000 "$tmp/nodem_ESP32-C3-ota.bin"
else
    espflash flash --baud=$baud --after hard-reset \
        --partition-table "$root/partitions.$profile.csv" \
        --bootloader "$dir/build/bootloader.bin" \
        --erase-parts otadata \
        "$@"
fi

# `--elf` lets the monitor decode panic backtraces into source locations.
espflash monitor --elf "$elf"
