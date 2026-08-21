#include "debug_log.h"
/*
 * microSD support for the Cardputer-Adv.
 *
 * The card is driven by the ESP-IDF SDSPI host on SPI2 using the M5Cardputer
 * wiring (CS=12, MOSI=14, SCK=40, MISO=39).  Keeping the mount in IDF avoids
 * Arduino's global SD/SPI state colliding with M5GFX and the emulator's
 * display transactions.
 */
#include <Arduino.h>
#include <esp_vfs_fat.h>

#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

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

static uint8_t sdProbeTransfer(uint8_t value) {
    uint8_t result = 0;
    for (int bit = 7; bit >= 0; --bit) {
        digitalWrite(SD_SPI_MOSI_PIN, (value & (1U << bit)) != 0U);
        digitalWrite(SD_SPI_CLK_PIN, HIGH);
        delayMicroseconds(1);
        result = static_cast<uint8_t>((result << 1U) |
                                      (digitalRead(SD_SPI_MISO_PIN) ? 1U : 0U));
        digitalWrite(SD_SPI_CLK_PIN, LOW);
        delayMicroseconds(1);
    }
    return result;
}

// Reset a card left in an interrupted SPI transaction before handing the bus
// to the IDF SDSPI driver. This stays bit-banged so it cannot disturb the
// display's separate SPI service and makes repeated app resets deterministic.
static bool sdRawInit() {
    (void)SD_SPI_PROBE_FREQ_HZ;
    sdPinsIdle();
    for (int i = 0; i < 16; ++i) sdProbeTransfer(0xFFU);

    for (int attempt = 0; attempt < 20; ++attempt) {
        digitalWrite(SD_SPI_CS_PIN, LOW);
        sdProbeTransfer(0x40U);
        sdProbeTransfer(0x00U);
        sdProbeTransfer(0x00U);
        sdProbeTransfer(0x00U);
        sdProbeTransfer(0x00U);
        sdProbeTransfer(0x95U); // CMD0 CRC
        uint8_t response = 0xFFU;
        for (int poll = 0; poll < 32; ++poll) {
            response = sdProbeTransfer(0xFFU);
            if ((response & 0x80U) == 0U) break;
        }
        digitalWrite(SD_SPI_CS_PIN, HIGH);
        sdProbeTransfer(0xFFU);
        if (response == 0x01U) {
            sdPinsIdle();
            return true;
        }
        delay(5);
    }
    sdPinsIdle();
    MACPLUS_LOG("SD: card did not answer CMD0\n");
    return false;
}

bool sdcardInit() {
    if (mounted) return true;

    if (storageMutex == nullptr) {
        storageMutex = xSemaphoreCreateMutex();
        if (storageMutex == nullptr) {
            MACPLUS_LOG("SD: mutex allocation failed\n");
            return false;
        }
    }

    if (!sdRawInit()) return false;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SD_SPI_HOST_ID;
    host.max_freq_khz = SD_SPI_FREQ_HZ / 1000;

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = SD_SPI_MOSI_PIN;
    bus_cfg.miso_io_num = SD_SPI_MISO_PIN;
    bus_cfg.sclk_io_num = SD_SPI_CLK_PIN;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    // Keep the DMA reservation modest: the emulator has no PSRAM and also
    // needs space for its 256 KiB emulated RAM and display buffers.
    bus_cfg.max_transfer_sz = SD_SPI_TRANSFER_BYTES;

    const spi_host_device_t hostId = static_cast<spi_host_device_t>(host.slot);
    esp_err_t err = spi_bus_initialize(hostId, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        MACPLUS_LOG("SD: SPI bus init failed (%s, 0x%x)\n",
               esp_err_to_name(err), static_cast<unsigned>(err));
        return false;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.host_id = hostId;
    slot_config.gpio_cs = static_cast<gpio_num_t>(SD_SPI_CS_PIN);
    slot_config.gpio_int = GPIO_NUM_NC;

    // The no-PSRAM target has a tight FatFS heap budget. setup() keeps the
    // software-disk stream open only after the raw system cache is valid, so
    // one descriptor is sufficient and leaves room for the emulator.
    constexpr uint8_t kMaxOpenFiles = 1;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = kMaxOpenFiles,
        .allocation_unit_size = 4 * 1024,
    };

    err = esp_vfs_fat_sdspi_mount("/sd", &host, &slot_config, &mount_config,
                                  &card);
    if (err != ESP_OK) {
        MACPLUS_LOG("SD: SDSPI mount failed (%s, 0x%x)\n",
               esp_err_to_name(err), static_cast<unsigned>(err));
        // The mount helper may have registered a partial device before
        // failing.  Unmount defensively, then release the bus for a retry.
        if (card != nullptr) {
            esp_vfs_fat_sdcard_unmount("/sd", card);
            card = nullptr;
        }
        spi_bus_free(hostId);
        pinMode(SD_SPI_CS_PIN, OUTPUT);
        digitalWrite(SD_SPI_CS_PIN, HIGH);
        return false;
    }

    mounted = true;
    MACPLUS_LOG("SD: mounted via SDSPI (max_files=%u)\n",
           static_cast<unsigned>(kMaxOpenFiles));
#if MACPLUS_ENABLE_DEBUG_LOG
    sdmmc_card_print_info(stdout, card);
#endif
    MACPLUS_LOG("SD: heap after mount free=%u largest=%u\n",
           static_cast<unsigned>(esp_get_free_heap_size()),
           static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
    return true;
}

bool sdcardRetry() {
    if (mounted) return true;
    return sdcardInit();
}
