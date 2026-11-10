#!/usr/bin/env bash
# Build the PostcardRTK firmware with arduino-cli (same command as CI).
# Needs the esp32:esp32 core 3.0.7 and the libraries ArduinoJson (7.x) and ESP32_BleSerial.
# Output in build/: PostcardRTK.ino.merged.bin (bootloader, partition table and application: write at 0x0 with
# the Espressif web flasher for a first install) and PostcardRTK.ino.bin (application only: write at 0x10000
# to update, keeping the settings).
set -euo pipefail
cd "$(dirname "$0")/.."
python3 tools/embed_web.py
VERSION="${POSTCARDRTK_VERSION:-$(git describe --tags --always --dirty 2>/dev/null || echo dev)}"
arduino-cli compile \
  --fqbn "esp32:esp32:esp32:FlashSize=16M,PSRAM=enabled,PartitionScheme=custom,DebugLevel=none" \
  --build-property "compiler.cpp.extra_flags=-MMD -c -DPOSTCARDRTK_VERSION=\"$VERSION\"" \
  --export-binaries --output-dir build firmware/PostcardRTK "$@"
# The merged image is the whole 16 MB flash; drop the empty tail so it flashes in seconds (the file system
# partition is not touched, the start-up formats it if needed)
python3 - <<'PY'
d = open("build/PostcardRTK.ino.merged.bin", "rb").read()
n = (len(d.rstrip(b"\xff")) + 4095) // 4096 * 4096
open("build/PostcardRTK.ino.merged.bin", "wb").write(d[:n])
print(f"merged image trimmed to {n} bytes")
PY
ls -la build/*.bin
