#!/bin/sh
# Turns a release ELF into signed flashable images:
#   <out>/nodem_ESP32-C3-ota.bin  signed app image - what gets uploaded via "ota"
#   <out>/nodem_ESP32-C3.bin      bootloader + partition table + blank otadata +
#                                 signed app, merged - to flash at 0x0 over USB.
#                                 For factory-fresh devices: the gaps between
#                                 its parts are 0xFF-filled, which wipes nvs.
#
# Usage: scripts/sign.sh [elf] [out-dir]
#   elf      defaults to target/riscv32imc-esp-espidf/release/nodem-esp32
#   out-dir  defaults to the repo root
#
# The private key comes from $OTA_SIGNING_KEY (default keys/ota_signing_key.pem,
# gitignored). Every device only accepts updates signed with the same key as
# the firmware it's running - keep it safe, and never commit it. Generate one
# with scripts/gen-key.sh.
set -e
root=$(cd "$(dirname "$0")/.." && pwd)
elf=${1:-$root/target/riscv32imc-esp-espidf/release/nodem-esp32}
out=${2:-$root}
key=${OTA_SIGNING_KEY:-$root/keys/ota_signing_key.pem}
build=$(dirname "$elf")/build

# ESP-IDF's own python env (installed by esp-idf-sys) has espsecure/esptool.
pyenv=$(ls -d "$root"/.embuild/espressif/python_env/*/bin 2>/dev/null | head -1)
[ -n "$pyenv" ] && PATH="$pyenv:$PATH"

if [ ! -f "$key" ]; then
    echo "signing key not found: $key" >&2
    echo "set OTA_SIGNING_KEY, or generate one with scripts/gen-key.sh" >&2
    exit 1
fi

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

espflash save-image --chip esp32c3 --flash-size 4mb \
    --partition-table "$root/partitions.release.csv" \
    "$elf" "$tmp/app.bin"
espsecure.py sign_data --version 2 --keyfile "$key" \
    --output "$out/nodem_ESP32-C3-ota.bin" "$tmp/app.bin"

# Blank otadata so a USB flash always boots `ota_0`, whichever slot a previous
# OTA update left active - offsets as in partitions.release.csv. `pkg` (past
# the end of the image) is not touched, but nvs (in the gap between the
# partition table and otadata) is.
head -c 8192 /dev/zero | tr '\0' '\377' > "$tmp/otadata.bin"
esptool.py --chip esp32c3 merge_bin --flash_size 4MB -o "$out/nodem_ESP32-C3.bin" \
    0x0 "$build/bootloader.bin" \
    0x8000 "$build/partition-table.bin" \
    0xd000 "$tmp/otadata.bin" \
    0x10000 "$out/nodem_ESP32-C3-ota.bin"
