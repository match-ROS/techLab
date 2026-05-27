# Elecrow CrowPanel 1.28 Smart Knob

Dieses Setup ist fuer den Elecrow CrowPanel 1.28 Zoll Smart Knob mit
IPS-Touchscreen und ESP32-S3. Der Sketch liest den Drehencoder aus, erkennt die
Drehrichtung und sendet die Werte per USB-Serial an den angeschlossenen PC.
Auf dem Display wird ein Roboter-Icon mit der aktuell gesteuerten Achse
angezeigt.

## Aktueller Sketch

Projektordner:

```text
/home/rosmatch/Arduino/CrowPanel_P1_J1
```

Aktueller Stand fuer den angeschlossenen dritten Knob:

- `KNOB_ID`: `3`
- `DEVICE_ID`: `P3`
- Display-Icon: `icon_j3.h`
- Display-Label: `Achse 3`
- Serial-Ausgabe Encoder: `P3:ENC:<wert>`
- Serial-Ausgabe Button: `P3:BTN:1`
- Baudrate: `115200`

## Arduino IDE unter Ubuntu 24.04 installieren

Nicht das Ubuntu-Paket `arduino` verwenden, weil dieses nur die alte IDE 1.8.x
installiert. Stattdessen die offizielle Arduino IDE 2 als AppImage verwenden.

1. Arduino IDE 2 fuer Linux herunterladen:

   ```text
   https://www.arduino.cc/en/software
   ```

2. Die Datei ausfuehrbar machen:

   ```bash
   chmod +x ~/Downloads/arduino-ide_*_Linux_64bit.AppImage
   ```

3. Falls das AppImage nicht startet, die benoetigten Pakete installieren:

   ```bash
   sudo apt update
   sudo apt install libfuse2t64 dbus-x11
   ```

4. Arduino IDE ohne `sudo` starten:

   ```bash
   cd ~/Downloads
   ./arduino-ide_2.3.9_Linux_64bit.AppImage --no-sandbox
   ```

Wichtig: Die IDE nicht mit `sudo` starten. Electron/Chromium blockiert den
Root-Start sonst mit einer Sandbox-Fehlermeldung.

## Seriellen Port freigeben

Der CrowPanel-Knob erscheint auf diesem Rechner als:

```text
/dev/ttyACM0
```

USB-Geraet:

```text
Espressif USB JTAG/serial debug unit
```

Damit Upload und Serial Monitor funktionieren, muss der Benutzer in der Gruppe
`dialout` sein:

```bash
sudo usermod -a -G dialout rosmatch
```

Danach einmal abmelden und wieder anmelden oder den Rechner neu starten.

## Arduino IDE konfigurieren

In der Arduino IDE:

1. `File > Preferences`
2. `Sketchbook location` setzen auf:

   ```text
   /home/rosmatch/Arduino
   ```

3. `Additional boards manager URLs` setzen auf:

   ```text
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```

4. `Tools > Board > Boards Manager` oeffnen.
5. Nach `esp32` suchen.
6. `esp32 by Espressif Systems` installieren.

Fuer dieses Projekt Version `2.0.14` verwenden.

Wichtig: Nicht die aktuelle `3.x`-Version verwenden. Die lokale
`Arduino_GFX`-Library in diesem Arduino-Ordner nutzt noch die ESP32-SPI-API aus
Core 2.x. Mit Core `3.3.8` kommt beim Kompilieren ein Fehler zu
`spiFrequencyToClockDiv(...)`.

## Board-Einstellungen

In `Tools` diese Werte einstellen:

- Board: `ESP32S3 Dev Module`
- Port: `/dev/ttyACM0`
- USB CDC On Boot: `Enabled`
- Flash Size: `16MB (128Mb)`
- PSRAM: `OPI PSRAM`
- Partition Scheme: `Huge APP (3MB No OTA/1MB SPIFFS)`

Danach den Sketch oeffnen:

```text
/home/rosmatch/Arduino/CrowPanel_P1_J1/CrowPanel_P1_J1.ino
```

Dann `Verify` und anschliessend `Upload` ausfuehren.

## Pinout im Sketch

Display:

- `TFT_SCLK`: GPIO 10
- `TFT_MOSI`: GPIO 11
- `TFT_DC`: GPIO 3
- `TFT_CS`: GPIO 9
- `TFT_RES`: GPIO 14
- `TFT_BLK`: GPIO 46
- `LCD_PWR_EN1`: GPIO 1
- `LCD_PWR_EN2`: GPIO 2

Encoder:

- `ENCODER_CLK`: GPIO 45
- `ENCODER_DT`: GPIO 42
- `ENCODER_BTN`: GPIO 41

## Knob auswaehlen

Im Sketch muss nur noch `KNOB_ID` oben geaendert werden:

```cpp
#define KNOB_ID 3
```

Moegliche Werte:

- `1`: sendet als `P1`, zeigt `icon_j1`, Label `Achse 1`
- `2`: sendet als `P2`, zeigt `icon_j2`, Label `Achse 2`
- `3`: sendet als `P3`, zeigt `icon_j3`, Label `Achse 3`

`DEVICE_ID`, Icon-Auswahl und Achsen-Label werden daraus automatisch erzeugt.

Fuer einen spaeteren vierten Knob muss zusaetzlich eine passende
`icon_j4.h`-Datei vorhanden sein und der Switch im Sketch um `case 4`
erweitert werden:

```cpp
case 4:
  gfx->draw16bitRGBBitmap(0, 0, icon_j4, ICON_W, ICON_H);
  break;
```

## Troubleshooting

Wenn die IDE beim Doppelklick nicht startet, im Terminal starten:

```bash
~/Downloads/arduino-ide_2.3.9_Linux_64bit.AppImage --no-sandbox
```

Wenn `dbus-launch` fehlt:

```bash
sudo apt install dbus-x11
```

Wenn Upload oder Serial Monitor mit `Permission denied` fehlschlagen:

```bash
sudo usermod -a -G dialout rosmatch
```

Danach abmelden/anmelden.

Wenn kein Port sichtbar ist, Knob neu einstecken und pruefen:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
lsusb
```

Wenn beim Kompilieren Fehler wie diese erscheinen:

```text
invalid conversion from 'int32_t' to 'spi_t*'
too few arguments to function 'uint32_t spiFrequencyToClockDiv(spi_t*, uint32_t)'
```

dann ist sehr wahrscheinlich der ESP32-Core `3.x` installiert. In der Arduino
IDE im Boards Manager bei `esp32 by Espressif Systems` die Version auf `2.0.14`
wechseln:

1. `Tools > Board > Boards Manager` oeffnen.
2. Nach `esp32` suchen.
3. Beim installierten Paket `esp32 by Espressif Systems` Version `2.0.14`
   auswaehlen.
4. `Install` klicken und warten, bis die Installation abgeschlossen ist.
5. Danach den Sketch erneut mit `Verify` kompilieren.

Wenn danach mit Core `2.0.14` dieser Fehler erscheint:

```text
fatal error: esp32-hal-periman.h: No such file or directory
```

dann enthaelt die lokale `Arduino_GFX`-Library Code fuer Core 3.x. In diesem
Projekt wurde die Library-Datei
`libraries/Arduino_GFX/src/databus/Arduino_ESP32SPI.h` deshalb so angepasst,
dass `esp32-hal-periman.h` und `esp_private/periph_ctrl.h` nur fuer ESP32-Core
3.x verwendet werden. Fuer Core 2.x wird stattdessen `driver/periph_ctrl.h`
eingebunden.

Wenn danach ein Fehler zu `esp_rgb_panel_t` aus
`Arduino_ESP32RGBPanel.cpp` erscheint, liegt das ebenfalls an der gemischten
`Arduino_GFX`-Version. In diesem Projekt wurde in
`libraries/Arduino_GFX/src/databus/Arduino_ESP32RGBPanel.h` der
Kompatibilitaetsblock fuer Core 2.x wieder aktiviert:

```cpp
#if (!defined(ESP_ARDUINO_VERSION_MAJOR)) || (ESP_ARDUINO_VERSION_MAJOR < 3)
```
