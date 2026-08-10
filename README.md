# Cardputer Mac Plus 模拟器

[简体中文](README.md) | [English](README.en.md)

运行在 M5Stack Cardputer-Adv 上的 Macintosh Plus 模拟器，支持 倾斜设备控制鼠标和移动画面范围、音频和 Wi-Fi 文件传输。

## 固件版本

固件下载：[GitHub Releases](https://github.com/ZhengXieGang/cardputer-macplus/releases/latest)。

| 文件 | 说明 |
| --- | --- |
| `macplus-lite.bin` | 只用于已有 Launcher 槽位的后续更新，不修改分区表。使用设备现有的 `macplus` 分区。 |
| `macplus-standard.bin` | Standard 完整固件，直接从 `0x0` 刷入；含 4,992 KiB 的 `macplus` 数据分区，系统硬盘镜像最大 4,243,456 字节。 |
| `macplus-full.bin` | Full 完整固件，直接从 `0x0` 刷入；不使用 Launcher，把剩余 8 MB Flash 都交给 `macplus`，系统硬盘镜像最大 5,844,992 字节。 |

Lite 写入 Launcher 标记为 `BOOT` 的应用槽，不修改分区表。Standard 和 Full 是完整镜像，从 `0x0` 写入，会覆盖 Launcher；Full 适合只运行 Mac Plus 的设备。

## 准备系统盘

`hd.img` 由用户自行准备，可以放在 FAT32 SD 卡根目录，也可以通过 Wi-Fi 页面上传。

镜像必须是 Macintosh 分区硬盘，包含 Apple 分区表、启动驱动和 HFS/MFS 分区，并且大小按 512 字节对齐。400K/800K 裸软盘不能直接作为 `hd.img`。

首次启动时，设备会把系统盘复制到 Flash data 分区；之后模拟器从 Flash 映射运行，SD 卡只作为初始来源或恢复来源。

## 安装软件

支持 400K/800K 的 `.dsk`、`.img` 和 Disk Copy 4.2 `.dc42` 文件。

1. 长按 `Ctrl + Alt + Opt` 两秒，进入 Wi-Fi 传输模式。
2. 连接热点 `MacPlus-Install`，打开 `http://192.168.4.1`。
3. 上传软件盘，设备写入完成后自动重启。
4. 在 Finder 打开软盘图标，把应用复制到系统硬盘。

不上传时，先松开入口组合键，再长按任意键或 G0 两秒退出 Wi-Fi 模式。

## 操作

- `Ctrl`：Command；`Opt`：Option；`Alt`：Control；`Shift`：Shift。
- G0：鼠标键；
- 倾斜设备：移动鼠标和画面。
- `Ctrl + Alt + Opt` 长按两秒：Wi-Fi 传输模式。

## 构建

依赖 PlatformIO、Arduino-ESP32 和 M5Cardputer：

```bash
~/.platformio/penv/bin/pio run -e cardputer-adv
~/.platformio/penv/bin/pio run -e cardputer-adv-full
```

`tools/generate_partitions.py` 会在构建前生成 Standard/Full 分区表。不要使用 `pio run -t upload`；Launcher 设备必须手动选择正确的应用槽。

## 参考

- [pico-mac](https://github.com/evansm7/pico-mac)
- [M5Stack M5Cardputer](https://github.com/m5stack/M5Cardputer)
- [Musashi](https://github.com/kstenerud/Musashi)
- 原 Mac Plus 模拟器及 TME、SCSI、IWM 代码的作者和贡献者
