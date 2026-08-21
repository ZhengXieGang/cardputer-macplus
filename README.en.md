# Cardputer Mac Plus Emulator

[简体中文](README.md) | [English](README.en.md)

A Macintosh Plus emulator for the M5Stack Cardputer-Adv, with tilt control of the mouse and viewport, audio, and Wi-Fi file transfer.

## Firmware editions

Firmware downloads: [GitHub Releases](https://github.com/ZhengXieGang/cardputer-macplus/releases/latest).

| File | Description |
| --- | --- |
| `macplus-launcher.bin` | Launcher edition install package. Launcher installs the app and creates a default 1,152 KiB `macplus` data partition; an existing partition is preserved and reused when large enough. |
| `macplus-full.bin` | Full standalone build, written at `0x0`, replacing Launcher. It includes a 6,528 KiB `macplus` partition and supports system-disk images up to 6,680,576 bytes. |

Launcher and Full editions have the same features; only their Flash layouts differ. The Launcher edition coexists with Launcher and other firmware. Full is intended for devices dedicated to Mac Plus and makes full use of the 8 MB Flash.

## System disk

Prepare `hd.img` yourself. Put it in the root of a FAT32 SD card, or upload it through Wi-Fi.

The image must be a partitioned Macintosh hard disk with an Apple partition map, boot driver, and HFS/MFS partition. Its size must be 512-byte aligned.

On first boot, the device copies the system disk to the `macplus` partition. Later emulation runs from the Flash mapping and never performs random system-disk reads from SD. Enlarging the partition automatically increases the available image capacity.

## Controls

- `Ctrl`: Command; `Opt`: Option; `Alt`: Control; `Shift`: Shift.
- G0: mouse button.
- Tilt: moves the mouse and viewport.
- Hold `Ctrl + Alt + Opt` for two seconds: Wi-Fi transfer mode.

## Software installation

Supported files are 400K/800K `.dsk`, `.img`, and Disk Copy 4.2 `.dc42` images. Software disks are stored on the SD card.

1. Hold `Ctrl + Alt + Opt` for two seconds to enter Wi-Fi transfer mode.
2. Connect to `MacPlus-Install` and open `http://192.168.4.1`.
3. Upload the software disk and wait for the automatic reboot.
4. Open the floppy icon in Finder and copy the application to the system disk.

Hold any key or G0 for two seconds to leave Wi-Fi transfer mode.

## Build

Requires PlatformIO, Arduino-ESP32, and M5Cardputer:

```bash
~/.platformio/penv/bin/pio run -e cardputer-adv-launcher
~/.platformio/penv/bin/pio run -e cardputer-adv-full
```

Normal builds compile out serial diagnostics to reduce Flash use and runtime overhead. Use the separate debug environments when a serial trace is needed:

```bash
~/.platformio/penv/bin/pio run -e cardputer-adv-launcher-debug
~/.platformio/penv/bin/pio run -e cardputer-adv-full-debug
```

`tools/generate_partitions.py` creates the Launcher/Full build layouts. The Launcher edition must be merged with its bootloader and partition description before Launcher installs it. Do not write the merged package directly into a Launcher app slot, and do not run `pio run -t upload`.

## References

- [pico-mac](https://github.com/evansm7/pico-mac)
- [M5Stack M5Cardputer](https://github.com/m5stack/M5Cardputer)
- [Musashi](https://github.com/kstenerud/Musashi)
- The authors and contributors of the original Mac Plus emulator and its TME, SCSI, and IWM code
