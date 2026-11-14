#!/usr/bin/env bash
# Build the PostcardRTK firmware with arduino-cli (same command as CI).
# Needs the esp32:esp32 core 3.0.7 and the libraries ArduinoJson (7.x) and ESP32_BleSerial.
#
#   BOARD=postcard (default): SparkFun RTK Postcard, variant V3. Output in build/:
#     PostcardRTK.ino.merged.bin (bootloader, partition table and application: write at 0x0 with the Espressif web
#     flasher for a first install) and PostcardRTK.ino.bin (application only: write at 0x10000 to update, keeping
#     the settings).
#   BOARD=c6: Waveshare LG290P module + ESP32-C6 SuperMini, variant V2 (4 MB flash, Bluetooth LE only, console on
#     the C6's own USB). Output in build/c6/, same two files.
#   BOARD=f9r: AgriBIT receiver, ESP32-DevKitC + u-blox ZED-F9R, F9R build (4 MB flash, no PSRAM, optional OLED; needs
#     the U8g2 library). Output in build/f9r/, same two files.
set -euo pipefail
cd "$(dirname "$0")/.."
python3 tools/embed_web.py
VERSION="${POSTCARDRTK_VERSION:-$(git describe --tags --always --dirty 2>/dev/null || echo dev)}"
BOARD="${BOARD:-postcard}"
FLAGS="compiler.cpp.extra_flags=-MMD -c -DPOSTCARDRTK_VERSION=\"$VERSION\""

case "$BOARD" in
postcard)
  OUT=build
  SKETCH=firmware/PostcardRTK
  FQBN="esp32:esp32:esp32:FlashSize=16M,PSRAM=enabled,PartitionScheme=custom,DebugLevel=none"
  ;;
c6)
  OUT=build/c6
  # Same sources; the sketch is copied so that its partitions.csv is the 4 MB table
  SKETCH=build/c6-src/PostcardRTK
  rm -rf build/c6-src && mkdir -p build/c6-src
  cp -r firmware/PostcardRTK "$SKETCH"
  cp firmware/partitions_c6.csv "$SKETCH/partitions.csv"
  FQBN="esp32:esp32:esp32c6:FlashSize=4M,PartitionScheme=custom,CDCOnBoot=cdc,DebugLevel=none"
  ;;
f9r)
  OUT=build/f9r
  # AgriBIT receiver: ESP32-DevKitC (WROOM-32D, 4 MB, no PSRAM) + u-blox ZED-F9R; same 4 MB partition table as the C6
  SKETCH=build/f9r-src/PostcardRTK
  rm -rf build/f9r-src && mkdir -p build/f9r-src
  cp -r firmware/PostcardRTK "$SKETCH"
  cp firmware/partitions_c6.csv "$SKETCH/partitions.csv"
  FLAGS="$FLAGS -DBOARD_F9R"
  FQBN="esp32:esp32:esp32:FlashSize=4M,PartitionScheme=custom,PSRAM=disabled,DebugLevel=none"
  ;;
*)
  echo "BOARD must be postcard, c6 or f9r" >&2
  exit 1
  ;;
esac

mkdir -p "$OUT"
arduino-cli compile --fqbn "$FQBN" --build-property "$FLAGS" --export-binaries --output-dir "$OUT" "$SKETCH" "$@"
# The merged image is the whole flash; drop the empty tail so it flashes in seconds (the file system
# partition is not touched, the start-up formats it if needed)
python3 - "$OUT/PostcardRTK.ino.merged.bin" <<'PY'
import sys
p = sys.argv[1]
d = open(p, "rb").read()
n = (len(d.rstrip(b"\xff")) + 4095) // 4096 * 4096
open(p, "wb").write(d[:n])
print(f"merged image trimmed to {n} bytes")
PY
rm -rf build/c6-src build/f9r-src
# The application must fit its partition (the board menu reports the 16 MB default for custom tables)
APP=$(stat -c %s "$OUT/PostcardRTK.ino.bin")
MAX=$([ "$BOARD" = postcard ] && echo $((0x600000)) || echo $((0x2C0000)))
echo "application $APP bytes of $MAX ($((APP * 100 / MAX)) %)"
[ "$APP" -le "$MAX" ] || { echo "application too large for its partition" >&2; exit 1; }
ls -la "$OUT"/*.bin
