#pragma once

// M5Stack Cardputer-Adv (Stamp-S3A, ESP32-S3FN8, 8MB flash, no PSRAM)
//
// Board peripherals (from the reference README and the M5Stack schematics):
//   - LCD ST7789V2 240x135 : handled by M5GFX (CS/37, RS/34, MOSI/35, SCK/36,
//     RST/33, BL/38) through M5Unified's board_M5CardputerADV profile.
//   - Keyboard TCA8418     : I2C SDA/8 SCL/9, INT/11 (M5Cardputer.Keyboard).
//   - IMU BMI270           : I2C SDA/8 SCL/9, addr 0x69 (M5.Imu).
//   - Button G0 (GO)       : GPIO0 (M5.BtnA), active low.
//   - microSD (SPI)        : CS/12, MOSI/14, SCK/40, MISO/39.
//   - Audio ES8311         : M5Unified I2S output to the built-in speaker.

// microSD card, ESP-IDF SDSPI host on the shared SPI2 bus.
#define SD_SPI_HOST_ID SPI2_HOST
#define SD_SPI_MOSI_PIN 14
#define SD_SPI_CLK_PIN 40
#define SD_SPI_MISO_PIN 39
#define SD_SPI_CS_PIN 12
#define SD_SPI_FREQ_HZ (20 * 1000 * 1000)
#define SD_SPI_TRANSFER_BYTES (8 * 1024)
#define SD_SPI_PROBE_FREQ_HZ (400 * 1000)

// GO button (G0) acts as the Mac mouse button.  GPIO0 is also the boot strap
// pin; holding it during reset enters download mode, which is normal.
#define MOUSE_BUTTON_PIN 0

// Display geometry (see disp.cpp): the 240x135 panel is an aspect-correct
// viewport into the 512x342 Mac framebuffer.
#define CARD_DISP_W 240
#define CARD_DISP_H 135
