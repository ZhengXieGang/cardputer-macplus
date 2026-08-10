/*
 * microSD support for the Cardputer-Adv.
 *
 * The card is driven by the ESP-IDF SDSPI host on the shared SPI2 bus with
 * the M5Stack reference wiring (CS=12, MOSI=14, SCK=40, MISO=39).  A small
 * raw CMD0/ACMD41 preflight is performed first so a missing or stuck card is
 * skipped quickly instead of hanging boot inside FatFS.
 */
#include <Arduino.h>
#include <dirent.h>
#include <errno.h>
#include <esp_vfs_fat.h>
#include <SPI.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

extern "C" {
#include "ff.h"
}

#include "hardware_config.h"
#include "sdcard.h"

static bool mounted = false;
static sdmmc_card_t *card = nullptr;
static SemaphoreHandle_t storageMutex = nullptr;

bool sdcardMounted() {
    return mounted;
}

bool sdcardAcquire(uint32_t timeoutMs) {
    if (storageMutex == nullptr) return false;
    return xSemaphoreTake(storageMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

void sdcardRelease() {
    if (storageMutex != nullptr) xSemaphoreGive(storageMutex);
}

static void sdPinsIdle() {
    pinMode(SD_SPI_CS_PIN, OUTPUT);
    digitalWrite(SD_SPI_CS_PIN, HIGH);
    pinMode(SD_SPI_CLK_PIN, OUTPUT);
    digitalWrite(SD_SPI_CLK_PIN, LOW);
    pinMode(SD_SPI_MOSI_PIN, OUTPUT);
    digitalWrite(SD_SPI_MOSI_PIN, HIGH);
    pinMode(SD_SPI_MISO_PIN, INPUT_PULLUP);
}

// Raw SPI preflight: bring the card out of any stale state and initialize it
// enough that the ESP-IDF SDSPI driver can take over and mount FatFS.
static bool sdRawInit() {
    SPIClass preflight(static_cast<uint8_t>(SD_SPI_HOST_ID));
    preflight.begin(SD_SPI_CLK_PIN, SD_SPI_MISO_PIN, SD_SPI_MOSI_PIN,
                    SD_SPI_CS_PIN);
    pinMode(SD_SPI_CS_PIN, OUTPUT);
    digitalWrite(SD_SPI_CS_PIN, HIGH);
    pinMode(SD_SPI_MISO_PIN, INPUT_PULLUP);

    // At least 74 clocks with CS high to power up the card.
    preflight.beginTransaction(
        SPISettings(SD_SPI_PROBE_FREQ_HZ, MSBFIRST, SPI_MODE0));
    for (int i = 0; i < 256; ++i) preflight.transfer(0xFF);
    preflight.endTransaction();

    bool ok = false;
    for (int attempt = 0; attempt < 20; ++attempt) {
        preflight.beginTransaction(
            SPISettings(SD_SPI_PROBE_FREQ_HZ, MSBFIRST, SPI_MODE0));
        digitalWrite(SD_SPI_CS_PIN, LOW);
        const uint8_t cmd[] = {0x40, 0x00, 0x00, 0x00, 0x00, 0x95}; // CMD0
        uint8_t r1 = 0xFF;
        for (size_t i = 0; i < sizeof(cmd); ++i) preflight.transfer(cmd[i]);
        for (int i = 0; i < 32; ++i) {
            r1 = preflight.transfer(0xFF);
            if ((r1 & 0x80) == 0) break;
        }
        digitalWrite(SD_SPI_CS_PIN, HIGH);
        preflight.transfer(0xFF);
        preflight.endTransaction();
        if (r1 == 0x01) {
            ok = true;
            break;
        }
        delay(5);
    }
    if (!ok) {
        printf("SD: card did not answer CMD0\n");
        preflight.end();
        sdPinsIdle();
        return false;
    }

    preflight.end();
    sdPinsIdle();
    return true;
}

bool sdcardInit() {
    if (mounted) return true;

    if (storageMutex == nullptr) {
        storageMutex = xSemaphoreCreateMutex();
        if (storageMutex == nullptr) {
            printf("SD: mutex allocation failed\n");
            return false;
        }
    }

    // The M5GFX display bus is SPI2 on this board? No: M5GFX uses SPI3 for
    // the ST7789.  The SD card owns SPI2 exclusively, so a raw preflight on
    // the same host is safe as long as the ESP-IDF SDSPI host is not active.
    // NOTE: an Arduino SPIClass preflight on SPI2 breaks M5GFX's SPI3 display
    // on this board (subsequent panel writes hang), so it is intentionally
    // not used.  ESP-IDF SDSPI runs its own full init sequence at the probing
    // frequency, which is sufficient.

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SD_SPI_HOST_ID;
    host.max_freq_khz = SD_SPI_FREQ_HZ / 1000;

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = SD_SPI_MOSI_PIN;
    bus_cfg.miso_io_num = SD_SPI_MISO_PIN;
    bus_cfg.sclk_io_num = SD_SPI_CLK_PIN;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    // Match the 8KB SCSI DMA window. This avoids splitting each Mac disk
    // request into four FatFS/SPI transactions without adding a cache.
    bus_cfg.max_transfer_sz = SD_SPI_TRANSFER_BYTES;

    const spi_host_device_t hostId = static_cast<spi_host_device_t>(host.slot);
    esp_err_t err = spi_bus_initialize(hostId, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        printf("SD: SPI bus init failed (%s)\n", esp_err_to_name(err));
        return false;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.host_id = hostId;
    slot_config.gpio_cs = static_cast<gpio_num_t>(SD_SPI_CS_PIN);
    slot_config.gpio_int = GPIO_NUM_NC;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 1,
        // Keep the FatFS work area small: the no-PSRAM SRAM budget cannot
        // spare a large allocation-unit buffer alongside the 32KB SCSI DMA
        // window and the 256KB emulated RAM.
        .allocation_unit_size = 4 * 1024,
    };

    err = esp_vfs_fat_sdspi_mount("/sd", &host, &slot_config, &mount_config,
                                  &card);
    if (err != ESP_OK) {
        printf("SD: mount failed (%s)\n", esp_err_to_name(err));
        spi_bus_free(hostId);
        sdPinsIdle();
        return false;
    }

    mounted = true;
    printf("SD: mounted via SDSPI\n");
    sdmmc_card_print_info(stdout, card);
    return true;
}

bool sdcardRetry() {
    if (mounted) return true;
    return sdcardInit();
}

bool sdcardRunWriteReadTest(uint32_t bytes) {
    if (!mounted) return false;
    if (!sdcardAcquire(5000)) return false;

    static const char *kPath = "/sd/.sdcard_test.bin";
    uint8_t *buffer = (uint8_t *)malloc(bytes);
    bool ok = false;
    if (buffer != nullptr) {
        FILE *fp = fopen(kPath, "wb");
        if (fp != nullptr) {
            for (uint32_t i = 0; i < bytes; ++i) {
                buffer[i] = static_cast<uint8_t>((i * 7U + (i >> 8)) & 0xFF);
            }
            const size_t written = fwrite(buffer, 1, bytes, fp);
            const int closeErr = fclose(fp);
            if (written == bytes && closeErr == 0) {
                fp = fopen(kPath, "rb");
                if (fp != nullptr) {
                    ok = fread(buffer, 1, bytes, fp) == bytes;
                    fclose(fp);
                    for (uint32_t i = 0; ok && i < bytes; ++i) {
                        if (buffer[i] !=
                            static_cast<uint8_t>((i * 7U + (i >> 8)) & 0xFF)) {
                            ok = false;
                        }
                    }
                }
            }
            remove(kPath);
        }
        free(buffer);
    }
    sdcardRelease();
    return ok;
}

bool sdcardApplyPendingUpdates() {
    // The upload/update pipeline was removed with the network services.
    return true;
}

void sdcardPrintRoot() {
    if (!mounted) {
        printf("SD root: not mounted\n");
        return;
    }
    DIR *dir = opendir("/sd");
    if (dir == nullptr) {
        printf("SD root: opendir failed (errno=%d)\n", errno);
        return;
    }
    printf("SD root contents:\n");
    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_name[0] == '.') continue;
        printf("  %s\n", entry->d_name);
    }
    closedir(dir);
}
