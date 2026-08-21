# Cardputer Mac Plus 模拟器

[简体中文](README.md) | [English](README.en.md)

运行在 M5Stack Cardputer-Adv 上的 Macintosh Plus 模拟器，支持 倾斜设备控制鼠标和移动画面范围、音频和 Wi-Fi 文件传输。

## 固件版本

固件下载：[GitHub Releases](https://github.com/ZhengXieGang/cardputer-macplus/releases/latest)。

| 文件 | 说明 |
| --- | --- |
| `macplus-launcher.bin` | Launcher 版安装包。Launcher 安装应用时会同时创建默认 1,152 KiB 的 `macplus` 数据分区；容量足够的已有分区会保留并复用。 |
| `macplus-full.bin` | Full 独立版，从 `0x0` 写入并替换 Launcher。内含 6,528 KiB 的 `macplus` 数据分区，系统硬盘镜像最大 6,680,576 字节。 |

Launcher 版和 Full 版功能相同，区别只在 Flash 布局。Launcher 版与 Launcher 及其他固件共存；Full 适合只运行 Mac Plus、希望充分使用 8 MB Flash 的设备。

## 准备系统盘

`hd.img` 自行准备，可以放在 FAT32 SD 卡根目录，也可以通过 Wi-Fi 页面上传。

镜像必须是 Macintosh 分区硬盘，包含 Apple 分区表、启动驱动和 HFS/MFS 分区，并且大小按 512 字节对齐。

首次启动时，设备会把系统盘复制到 `macplus` 分区；之后模拟器从 Flash 映射运行，不会从 SD 卡随机读取系统盘。分区扩大后，固件会自动使用新增容量。

## 操作

- `Ctrl`：Command；`Opt`：Option；`Alt`：Control；`Shift`：Shift。
- G0：鼠标键；
- 倾斜设备：移动鼠标和画面。
- `Ctrl + Alt + Opt` 长按两秒：Wi-Fi 传输模式。

## 安装软件

支持 400K/800K 的 `.dsk`、`.img` 和 Disk Copy 4.2 `.dc42` 文件。软件盘保存在 SD 卡。

1. 长按 `Ctrl + Alt + Opt` 两秒，进入 Wi-Fi 传输模式。
2. 连接热点 `MacPlus-Install`，打开 `http://192.168.4.1`。
3. 上传软件盘，设备写入完成后自动重启。
4. 在 Finder 打开软盘图标，把应用复制到系统硬盘。

长按任意键或 G0 两秒退出 Wi-Fi 模式。

## 构建

依赖 PlatformIO、Arduino-ESP32 和 M5Cardputer：

```bash
~/.platformio/penv/bin/pio run -e cardputer-adv-launcher
~/.platformio/penv/bin/pio run -e cardputer-adv-full
```

普通构建默认关闭串口诊断日志，以减少 Flash 占用和运行时开销。需要排障记录时使用独立的 debug 环境：

```bash
~/.platformio/penv/bin/pio run -e cardputer-adv-launcher-debug
~/.platformio/penv/bin/pio run -e cardputer-adv-full-debug
```

`tools/generate_partitions.py` 会在构建前生成 Launcher/Full 构建布局。Launcher 版需要合并 bootloader、分区描述和应用后交给 Launcher 安装；不要把合并包直接写入 Launcher 应用槽，也不要使用 `pio run -t upload`。

## 参考

- [pico-mac](https://github.com/evansm7/pico-mac)
- [M5Stack M5Cardputer](https://github.com/m5stack/M5Cardputer)
- [Musashi](https://github.com/kstenerud/Musashi)
- 原 Mac Plus 模拟器及 TME、SCSI、IWM 代码的作者和贡献者
