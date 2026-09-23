#!/bin/sh
# Generates the RSA-3072 OTA signing key scripts/sign.sh uses - at
# $OTA_SIGNING_KEY, default keys/ota_signing_key.pem (gitignored). Refuses to
# overwrite an existing key: devices only accept updates signed with the key
# their running firmware was signed with.
set -e
root=$(cd "$(dirname "$0")/.." && pwd)
key=${OTA_SIGNING_KEY:-$root/keys/ota_signing_key.pem}

# ESP-IDF's own python env (installed by esp-idf-sys) has espsecure - any
# other espsecure.py on PATH may be broken or a different version.
pyenv=$(ls -d "$root"/.embuild/espressif/python_env/*/bin 2>/dev/null | head -1)
[ -n "$pyenv" ] && PATH="$pyenv:$PATH"

if [ -e "$key" ]; then
    echo "signing key already exists, not overwriting: $key" >&2
    exit 1
fi

mkdir -p "$(dirname "$key")"
umask 077
espsecure.py generate_signing_key --version 2 --scheme rsa3072 "$key"
