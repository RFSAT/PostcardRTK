# PostcardRTK

Firmware for the **SparkFun RTK Postcard** (ESP32 + Quectel LG290P quad-band GNSS) running **on its own**, without the Portability Shield or a screen. It is powered from the USB-C port and starts as soon as it gets power. It sends positions as NMEA over Bluetooth, and picks the best positioning method by itself.

The same firmware also runs on a **Waveshare LG290P module joined to an ESP32-C6 SuperMini** by a UART (variant V2 of the RFSAT RTK documentation): see [ESP32-C6 build](#esp32-c6-build-variant-v2); and on the **AgriBIT receiver** (ESP32-DevKitC + u-blox ZED-F9R, variant V7): see [AgriBIT receiver build](#agribit-receiver-build-variant-v7).

RFSAT Ltd. Developed from [PicoRTK](https://github.com/RFSAT/PicoRTK), the same rover on a Raspberry Pi Pico 2 W, and sharing its tested modules: NMEA and RTCM handling, the NTRIP client with failover, method tracking, sky view, history and the event log.

## How it chooses the method

The order of preference is accuracy first, then reliability:

| Method | Typical accuracy | Needs |
|---|---|---|
| RTK fixed / float | 1–2 cm fixed | Corrections: NTRIP over a configured Wi-Fi network, or RTCM sent by a phone app over Bluetooth |
| Galileo HAS | about 20 cm after several minutes | An antenna that receives Galileo E6, and LG290P firmware 2.x |
| SBAS (EGNOS) | about 1 m | Nothing (from the satellites) |
| GNSS only | a few metres | Nothing |

The LG290P computes one solution at a time and falls back on its own: when RTCM stops it keeps RTK for up to its differential timeout, then uses Galileo HAS or SBAS, then GNSS only. So the firmware's job is deciding **whether to feed RTK corrections**, and that is what the method ladder (`firmware/PostcardRTK/ladder.cpp`) does.

**When RTK is used:**
- RTK is used whenever a correction source is available: a configured Wi-Fi network in range with a caster, or an app sending RTCM.
- Without Wi-Fi, the Postcard scans for the configured networks every 60 s (`wifiScanSec`). This is how it notices that RTK has become available.

**When RTK gets paused:**
- RTK gets a **5-minute trial** (`trialSec`), counted only while corrections actually arrive. It "performs" when it is fixed, or float with the receiver's estimate at or below 0.5 m (`rtkGoodM`).
- If RTK has not performed for a whole trial period, it is compared with what Galileo HAS or SBAS achieved at this place in the last hour. If neither has been seen yet, it is compared with a typical SBAS result of 1.5 m.
- If the fallback is better, RTK is **paused for 15 minutes**: the caster connection is closed so the receiver falls back. Then RTK is tried again. The pause doubles after each failed retry, up to 1 hour, and a minute of RTK fixed resets it.
- The same applies when corrections arrive for 5 minutes but the receiver never reaches RTK, for example with a base too far away.

**Honest reporting:**
- Positions go out exactly as the receiver computes them, with no holding or smoothing.
- Every decision is logged with its reason, in plain words, and shown on the web page.

Mode **Galileo HAS only** (`corrections.mode = "has"`) never uses RTK corrections.

## Bluetooth output

The Postcard appears as **PostcardRTK-xxxx**, using classic SPP for Android apps and BLE for iPhone apps. Apps get the receiver's NMEA (GGA, RMC, GSA, GSV, GST and any others enabled) plus one status sentence after every GGA:

```
$PRFSM,<ladder>,<method>,<corrections>,<age>,<station>,<estH>,<estV>*hh
$PRFSM,RTK,FIX,live,1.0,1234,0.014,0.025*hh
$PRFSM,PAUSE,SBAS,live,,123,0.812,1.402*hh
```

| Field | Values |
|---|---|
| ladder | `RTK`, `TRY` (corrections arriving, not RTK yet), `PAUSE`, `NOSRC` (no correction source), `HASONLY` |
| method | `FIX`, `FLOAT`, `DGNSS`, `HASCONV` (HAS converging), `HAS`, `SBAS`, `GNSS`, `DR`, `NONE` |
| corrections | `live`, `stale` (the receiver still uses corrections that stopped arriving), `none` |
| age | correction age in seconds |
| station | RTCM base id, `9002` = Galileo HAS, `120`–`158` = EGNOS satellite |
| estH, estV | the receiver's own horizontal and vertical accuracy estimate, metres |

Apps that ignore unknown sentences (most do) are not affected. To switch it off, set `corrections.statusSentence` to `false`.

**RTK without Wi-Fi:** apps with their own NTRIP client (for example SW Maps or Lefebure NTRIP Client on Android) can send corrections from the phone's mobile data over the same Bluetooth link. The Postcard passes them to the receiver when it is not streaming NTRIP itself.

## Flashing

You need Chrome or Edge, and a USB-C data cable to the Postcard. The CH342 chip gives two COM ports: **port A** (the lower number) is the ESP32, **port B** is the LG290P.

1. Open the **Espressif web flasher** at https://espressif.github.io/esptool-js/
2. Set the baud rate to 921600, press **Connect** and pick port A.
3. Add file `PostcardRTK-<version>-merged_0x0_V3.bin` at **Flash Address 0x0**, press **Program**, and wait for "Leaving…".
4. Press the reset button or re-plug the cable.

The merged image holds the bootloader, the partition table (the same as RTK Everywhere) and PostcardRTK. The settings of the previous firmware are not used.

**Later updates** keep the settings: write only `PostcardRTK-<version>_V3.bin` at address **0x10000**.

**File names** end with the RFSAT variant ID, so each file shows which prototype it belongs on: `_V3` for the SparkFun RTK Postcard, `_V2` for the Waveshare LG290P module with the ESP32-C6 SuperMini, `_V7` for the AgriBIT receiver (ESP32-DevKitC + ZED-F9R). Private builds add `-private` before the ID. Never write a `_V2` file to a Postcard or the reverse: the pins and flash layout differ.

**Back to RTK Everywhere:** use SparkFun's RTK Firmware Uploader as before.

## First set-up

Choose either:
- **Over Wi-Fi:** with no Wi-Fi configured (public build), the Postcard opens the access point **PostcardRTK-Setup-xxxx** (password `postcardrtk-setup`). Join it and open http://192.168.4.1, user `admin`, password `postcardrtk`. Enter your Wi-Fi networks and correction sources on the page, then save.
- **Over USB:** open port A at 115200 baud (the RFSAT configurator, PuTTY or the Arduino serial monitor) and type `help`.

```
wifi MyNetwork MyPassword
set {"ntrip":[{"name":"VRS","host":"gr.nrtk.eu","port":7801,"mount":"MSM_VRS","user":"me","pass":"secret","gga":true}]}
save
restart
```

Once on your Wi-Fi, the page is at http://postcardrtk.local, or at the IP address the console prints. It has the same tabs as PicoRTK (Home, Methods, Sky, Events, history download), plus the ladder's current decision and its reason.

If no configured network is found for 2 minutes, the setup access point opens again, and it closes once the Postcard joins Wi-Fi. The console command `ap` opens it on demand.

## Console commands (USB port A, 115200 baud)

| Command | |
|---|---|
| `status` | one-line status (also printed every 30 s) |
| `ladder` | the method ladder's state and reason, retry time, best fallback estimate |
| `config` / `set {json}` / `wifi <ssid> <pass>` / `save` | settings |
| `rx <command>` / `replies` | talk to the LG290P, e.g. `rx $PQTMVERNO` |
| `events` | the event log |
| `ap` / `restart` | |

## What the firmware sets in the receiver at every start

None of this is saved in the receiver:
- GGA, RMC, GSA, GSV and GST at 1 Hz;
- PQTMEPE and PQTMPPPNAV for the accuracy estimate and the Galileo HAS status;
- Galileo HAS on (`$PQTMCFGPPP,W,2,1,120,0.10,0.15`).

It also checks the receiver mode. A receiver left in **base** mode by RTK Everywhere is switched back to rover, saved and restarted, and this is logged.

The UART speed is found automatically if it does not match the setting (460800 by default).

## Building

```
bash tools/build.sh            # needs arduino-cli, esp32:esp32@3.0.7, ArduinoJson 7.4.2, ESP32_BleSerial 3.0.0
```

**Board settings:** ESP32 Dev Module, 16 MB flash, PSRAM on, partition table `firmware/PostcardRTK/partitions.csv` (the same as RTK Everywhere).

**CI:** runs the PC unit tests (`test/test_main.cpp`, including the method ladder scenarios) and the NTRIP protocol test, then builds the `.bin` files for both boards.

**Private builds** with your Wi-Fi and caster logins compiled in:

```
python3 tools/make_defaults.py my-settings.json
bash tools/build.sh            # and/or BOARD=c6 bash tools/build.sh
python3 tools/make_defaults.py --remove
```

`my-settings.json` is a settings file downloaded from the page. The generated `defaults_local.h` is git-ignored: never commit it or publish a binary built with it.

## ESP32-C6 build (variant V2)

One source, one version: `firmware/PostcardRTK/board.h` holds what differs, and the build picks the board.

| | SparkFun RTK Postcard (V3) | LG290P module + ESP32-C6 SuperMini (V2) |
|---|---|---|
| Release files | `PostcardRTK-<version>_V3.bin`, `PostcardRTK-<version>-merged_0x0_V3.bin` | `PostcardRTK-<version>_V2.bin`, `PostcardRTK-<version>-merged_0x0_V2.bin` |
| Receiver UART | GPIO21 RX, GPIO22 TX, reset GPIO33 | GPIO17 RX ← TXD3, GPIO16 TX → RXD3 (UART3 socket of the module), no reset line |
| Bluetooth | classic SPP and/or BLE | **BLE only** (Nordic UART Service); the settings `spp` and `both` are served as BLE |
| Flash, partitions | 16 MB, as RTK Everywhere | 4 MB, `firmware/partitions_c6.csv` (one 2.75 MB application, 1.1 MB file system) |
| Event log, history | 64 KB, 2 × 384 KB (about 30 h) | 32 KB, 2 × 256 KB (about 20 h) |
| Console | USB port A (CH342), 115200 | the C6's own USB-C (USB CDC) |
| LEDs | – | GPIO15: on = Wi-Fi connected, slow blink = looking for Wi-Fi, fast blink = setup network. RGB (GPIO8): green RTK fixed, yellow float, cyan HAS converged, blue HAS converging, magenta DGNSS/SBAS, white GNSS only, red blinking = no position |
| Button | – | GPIO19 to GND, hold 3 s: setup network |

Wiring: TXD3 → GPIO17, RXD3 ← GPIO16, GND, and the module's 5V pad from the C6's 5V pin (never join the two 3.3 V rails). Avoid GPIO4, 5, 8, 9, 15 for anything else (strapping pins); GPIO12/13 are USB.

**Flashing:** Espressif web flasher on the C6's USB-C (hold BOOT while plugging in if it does not connect), file `PostcardRTK-<version>-merged_0x0_V2.bin` at 0x0; updates `PostcardRTK-<version>_V2.bin` at 0x10000.

**Building:** `BOARD=c6 bash tools/build.sh` (output in `build/c6/`). Board settings: ESP32C6 Dev Module, 4 MB flash, partition scheme custom, USB CDC on boot enabled.

**Receiver set-up:** the receiver's own USB-C (on the Waveshare module) takes QGNSS for firmware updates; unplug the UART link to the C6 meanwhile.

## AgriBIT receiver build (variant V7)

The AgriBIT GNSS receiver (2021-2023): an **ESP32-DevKitC V4** (ESP32-WROOM-32D, 4 MB flash, no PSRAM) stacked on the **ESP_F9x** board with a **u-blox ZED-F9R** (multi-band RTK with a built-in IMU for dead reckoning), optionally with the **ESP_BG95** board (Quectel BG95-M3 LTE-M modem, SD card) between them. This build replaces its original ESPrtk firmware: **back up the original first** (below). The first V7 version is **Wi-Fi only** (no modem, no SD card), with the optional 0.96" OLED.

| | AgriBIT receiver (V7) |
|---|---|
| Release files | `PostcardRTK-<version>_V7.bin`, `PostcardRTK-<version>-merged_0x0_V7.bin` |
| Receiver | u-blox ZED-F9R, configured with UBX CFG-VALSET at every start (RAM and battery-backed RAM; its flash configuration is not changed): NMEA GGA, RMC, GSA, GSV, GST, VTG on (GLL off), NMEA high precision, RTCM3 in, SBAS (EGNOS) on, IMU mount auto-alignment, 115200 baud. Every key gets its own message, so a key the receiver's firmware lacks is reported by name in the event log and the rest still applies |
| Receiver UART | found automatically: GPIO16 RX / GPIO4 TX or GPIO17 RX / GPIO5 TX (both wired through the stack), at 115200, 38400 (ZED-F9 default), 9600, 230400 or 460800; searched again if NMEA stops for 5 s. The receiver's firmware version (UBX MON-VER) is shown on the status page and in the event log |
| Methods | RTK (fixed / float), DGNSS, SBAS, GNSS only, dead reckoning (GGA quality 6). **No Galileo HAS** (not in the ZED-F9R): the HAS controls are hidden on this board |
| Bluetooth | classic SPP by default (as the original); BLE or both selectable (more RAM) |
| Flash, partitions | 4 MB, `firmware/partitions_c6.csv` (one 2.75 MB application, 1.1 MB file system) |
| Display | SSD1306 0.96" 128 x 64 on I2C GPIO26 SDA / GPIO27 SCL (optional; nothing happens without it). Boot screen: RFSAT Limited and the firmware version. Status: method and satellites, the receiver's accuracy estimate, **C:** caster (Metrica SmartNet, EUREF-IP, RTK2go by name; others by host) and **M:** mount point while streaming (otherwise the NTRIP state), correction station and age, IP address or the setup network |
| LEDs | GPIO15 ("ESP_RTK OK"): on = Wi-Fi connected, slow blink = looking for Wi-Fi, fast blink = setup network. GPIO12 ("ESP_RTK ERROR"): on = no position or no data from the receiver |
| Button | GPIO13 (top button), hold 3 s: setup network |
| Not used yet | BG95 modem (GPIO0 power key), SD card (GPIO23/39/18/19) |

**Back up the original ESPrtk firmware first** (once per unit; restores it exactly, licence and settings included). With the DevKitC off the stack, on its own USB, using esptool 5 (`esptool` 4 writes the commands with `_`):

```
esptool --chip esp32 --port COM5 flash-id
esptool --chip esp32 --port COM5 read-mac
esptool --chip esp32 --port COM5 --baud 115200 read-flash 0x000000 0x100000 part0.bin
esptool --chip esp32 --port COM5 --baud 115200 read-flash 0x100000 0x100000 part1.bin
esptool --chip esp32 --port COM5 --baud 115200 read-flash 0x200000 0x100000 part2.bin
esptool --chip esp32 --port COM5 --baud 115200 read-flash 0x300000 0x100000 part3.bin
copy /b part0.bin+part1.bin+part2.bin+part3.bin ESPrtk_unitN_backup.bin
```

Reading in 1 MB parts avoids "Digest mismatch" on long reads; repeat a part that fails. Restore: `esptool --chip esp32 --port COM5 --baud 115200 write-flash 0 ESPrtk_unitN_backup.bin`.

**Flashing V7:** DevKitC on its own USB (or the whole stack on the DevKitC's USB), first install `esptool --chip esp32 --port COM5 --baud 115200 write-flash 0 PostcardRTK-<version>-merged_0x0_V7.bin`; updates `write-flash 0x10000 PostcardRTK-<version>_V7.bin` (settings kept). The Espressif web flasher works too.

**Building:** `BOARD=v7 bash tools/build.sh` (output in `build/v7/`; needs the U8g2 library). Board settings: ESP32 Dev Module, 4 MB flash, partition scheme custom, PSRAM disabled, extra flag `-DBOARD_V7`.

## Hardware notes

- **No screen, buttons, battery or SD card:** those are on the Portability Shield. If the shield is fitted, this firmware ignores it.
- **The receiver's own USB channel (port B)** stays available for QGNSS: firmware updates, sky view and signal levels.
- **Galileo HAS needs an E6-capable antenna.** Without one the ladder still works, going RTK → SBAS → GNSS only.
