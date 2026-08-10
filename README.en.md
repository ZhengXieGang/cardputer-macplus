# Cardputer Mac Plus Emulator

[简体中文](README.md) | [English](README.en.md)

A Macintosh Plus emulator for the M5Stack Cardputer-Adv, with tilt control of the mouse and viewport, audio, and Wi-Fi file transfer.

## Firmware editions

Firmware downloads: [GitHub Releases](https://github.com/ZhengXieGang/cardputer-macplus/releases/latest).

| File | Description |
| --- | --- |
| `macplus-lite.bin` | For updates to an existing Launcher slot only; does not change the partition table and uses the existing `macplus` partition. |
| `macplus-standard.bin` | Complete Standard firmware, written at `0x0`; includes a 4,992 KiB `macplus` data partition and supports hard-disk images up to 4,243,456 bytes. |
| `macplus-full.bin` | Complete Full firmware, written at `0x0`; does not use Launcher and gives the remaining 8 MB Flash to `macplus`, with hard-disk images up to 5,844,992 bytes. |

Write Lite to the application slot marked `BOOT` by Launcher; it does not change the partition table. Standard and Full are complete images written at `0x0` and replace Launcher. Full is intended for a device running Mac Plus only.

## System disk

Prepare `hd.img` yourself. Put it in the root of a FAT32 SD card, or upload it through Wi-Fi.

The image must be a partitioned Macintosh hard disk with an Apple partition map, boot driver, and HFS/MFS partition. Its size must be 512-byte aligned. A raw 400K/800K floppy cannot be used as `hd.img`.

On first boot, the device copies the system disk to the Flash data partition. Later emulation runs from the Flash mapping; the SD card is only an initial or recovery source.

## Software installation

Supported files are 400K/800K `.dsk`, `.img`, and Disk Copy 4.2 `.dc42` images.

1. Hold `Ctrl + Alt + Opt` for two seconds to enter Wi-Fi transfer mode.
2. Connect to `MacPlus-Install` and open `http://192.168.4.1`.
3. Upload the software disk and wait for the automatic reboot.
4. Open the floppy icon in Finder and copy the application to the system disk.

To leave without uploading, release the entry keys, then hold any key or G0 for two seconds.

## Controls

- G0: mouse button.
- Tilt: moves the mouse and viewport.
- Hold `Ctrl + Alt + Opt` for two seconds: Wi-Fi transfer mode.

## Build

Requires PlatformIO, Arduino-ESP32, and M5Cardputer:

```bash
~/.platformio/penv/bin/pio run -e cardputer-adv
~/.platformio/penv/bin/pio run -e cardputer-adv-full
```

`tools/generate_partitions.py` creates the Standard/Full tables before each build. Do not run `pio run -t upload`; Launcher devices require the correct application slot to be selected manually.

## References

- [pico-mac](https://github.com/evansm7/pico-mac)
- [M5Stack M5Cardputer](https://github.com/m5stack/M5Cardputer)
- [Musashi](https://github.com/kstenerud/Musashi)
- The authors and contributors of the original Mac Plus emulator and its TME, SCSI, and IWM code
