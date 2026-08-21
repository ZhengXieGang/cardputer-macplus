#include "debug_log.h"
/*
 * Mac Plus emulator for the M5Stack Cardputer-Adv (ESP32-S3FN8, no PSRAM).
 *
 * The emulated Mac runs with 256KB RAM in the chip's internal SRAM and boots
 * from the raw-flash cache populated from /sd/hd.img. The patched Mac Plus
 * ROM is embedded in the application binary; SD /sd/macplus.rom is unused.
 *
 * Input: TCA8418 keyboard -> Mac ADB scancodes, BMI270 accel -> mouse, G0/GO
 * button -> mouse button.  Output: 512x342 1bpp screen scaled to the 240x135
 * ST7789 panel by the display task on core 1.
 */
#include <Arduino.h>
#include <errno.h>
#include <string.h>

#include <M5Cardputer.h>
#include "esp_heap_caps.h"
#include "esp_ota_ops.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "hardware_config.h"
#include "input.h"
#include "partition_setup.h"
#include "sdcard.h"
#include "settings.h"
#include "webinstall.h"

extern "C" {
#include "tme/emu.h"
#include "tme/rtc.h"
#include "tme/tmeconfig.h"
#include "tme/disp.h"
#include "tme/hd.h"
#include "tme/ncr.h"
#include "tme/snd.h"
}

static char pram[32];
static volatile bool bootOk = false;
static const uint8_t *romData = nullptr;

#define MAC_PLUS_ROM_INITIAL_PC 0x0040002AUL
#define MAC_PLUS_ROM_CHECKSUM 0x4D1F8172UL

static void initNvs() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // The Launcher owns this shared partition. A Mac image must never
        // erase it merely because its current layout is incompatible.
        MACPLUS_LOG("NVS: shared partition unavailable (%s); using default PRAM\n",
               esp_err_to_name(err));
    }
    if (err != ESP_OK) MACPLUS_LOG("NVS: init failed (%s)\n", esp_err_to_name(err));
}

static void initCardputerHardware(bool speaker, bool keyboard) {
    auto cfg = M5.config();
    cfg.internal_mic = false;
    cfg.internal_spk = speaker;
    cfg.internal_rtc = false;
    cfg.external_rtc = false;
    M5Cardputer.begin(cfg, keyboard);
    M5.Display.setRotation(1);
    M5.Display.setBrightness(100);
    M5.Display.fillScreen(0x0000);
}

// The pre-patched Mac Plus v3 ROM is embedded in the firmware image
// (src/rom_embedded.S -> roms/macplus-patched-256.rom, see tools/patch_rom.py).
// Embedding makes the app self-contained so the M5 Launcher can install it
// as a plain app binary without needing custom data partitions.
extern "C" {
extern const unsigned char macplus_rom_embedded[];
extern const uint32_t macplus_rom_embedded_size;
}

// Allocated very early, before M5/SD/FatFS fragment the heap, so the 68K RAM
// can get one contiguous internal-SRAM block.
extern "C" unsigned char *macRam;

static uint32_t readBigEndianLong(const uint8_t *data) {
    return (static_cast<uint32_t>(data[0]) << 24) |
           (static_cast<uint32_t>(data[1]) << 16) |
           (static_cast<uint32_t>(data[2]) << 8) |
           static_cast<uint32_t>(data[3]);
}

// The patched ROM keeps its original header, so we verify the stored checksum
// and the initial PC but do not recompute the (now modified) checksum.
static bool validateRomBuffer(const uint8_t *rom, const char *source) {
    const uint32_t storedChecksum = readBigEndianLong(rom);
    const uint32_t initPC = readBigEndianLong(rom + 4);
    MACPLUS_LOG("ROM: %s first8=%02X%02X%02X%02X-%02X%02X%02X%02X pc=%08lX sum=%08lX\n",
           source,
           rom[0], rom[1], rom[2], rom[3],
           rom[4], rom[5], rom[6], rom[7],
           static_cast<unsigned long>(initPC),
           static_cast<unsigned long>(storedChecksum));
    if (rom[0] == 0xFF && rom[1] == 0xFF) {
        MACPLUS_LOG("ROM: blank partition\n");
        return false;
    }
    if (storedChecksum != MAC_PLUS_ROM_CHECKSUM) {
        MACPLUS_LOG("ROM: not a Mac Plus v3 ROM (checksum mismatch)\n");
        return false;
    }
    if (initPC != MAC_PLUS_ROM_INITIAL_PC) {
        MACPLUS_LOG("ROM: unexpected initial PC\n");
        return false;
    }
    return true;
}

static bool validateEmbeddedRom() {
    if (macplus_rom_embedded_size != TME_ROMSIZE) {
        MACPLUS_LOG("ROM: embedded size mismatch (%u)\n",
               (unsigned)macplus_rom_embedded_size);
        return false;
    }
    return validateRomBuffer(macplus_rom_embedded, "embedded");
}

// Called by rtc.c when PRAM changes — save to NVS
extern "C" void saveRtcMem(char *mem) {
    nvs_handle_t handle;
    if (nvs_open("macplus", NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_blob(handle, "pram", mem, 32);
        nvs_commit(handle);
        nvs_close(handle);
    }
}

static void loadPram() {
    memset(pram, 0, sizeof(pram));
    nvs_handle_t handle;
    if (nvs_open("macplus", NVS_READONLY, &handle) == ESP_OK) {
        size_t len = 32;
        if (nvs_get_blob(handle, "pram", pram, &len) == ESP_OK) {
            MACPLUS_LOG("PRAM loaded from NVS (%d bytes)\n", (int)len);
        } else {
            MACPLUS_LOG("No PRAM in NVS, using defaults\n");
        }
        nvs_close(handle);
    }
}

static void emuTask(void *param) {
    (void)param;
    MACPLUS_LOG("Starting Mac Plus emulation on core %d...\n", xPortGetCoreID());
    tmeStartEmu(const_cast<uint8_t *>(romData));
    vTaskDelete(NULL);
}

void setup() {
    Serial.begin(115200);
    MACPLUS_LOG("\n\n=== Cardputer-Adv Mac Plus ===\n");
    MACPLUS_LOG("CPU: %dMHz, flash: %uMB, PSRAM: %uMB\n", ESP.getCpuFreqMHz(),
           ESP.getFlashChipSize() / (1024 * 1024),
           ESP.getPsramSize() / (1024 * 1024));
    MACPLUS_LOG("Free heap before init: %d bytes\n", (int)esp_get_free_heap_size());

    if (webInstallModeRequested()) {
        MACPLUS_LOG("BOOT: integrated WiFi transfer mode selected\n");
        initNvs();
        loadMacSettings();
        initCardputerHardware(false, true);
        webInstallRun();
        return;
    }

    // Reserve the emulated Mac RAM while the heap is still pristine. The SD
    // driver is initialized after M5GFX so it cannot collide with the
    // display's SPI bus probe.
    macRam = static_cast<unsigned char *>(malloc(TME_RAMSIZE));
    if (macRam == nullptr) {
        MACPLUS_LOG("FATAL: cannot allocate %d bytes for Mac RAM (free=%u largest=%u internal=%u largest_internal=%u)\n",
               TME_RAMSIZE,
               static_cast<unsigned>(esp_get_free_heap_size()),
               static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)),
               static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
               static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));
        return;
    }
    MACPLUS_LOG("Mac RAM reserved at %p (%d bytes)\n", macRam, TME_RAMSIZE);
    // Defer speaker allocation until after the contiguous Mac RAM block is
    // reserved. sndInit() configures the speaker and codec itself, so the
    // ES8311 does not get enabled abruptly by M5Unified during board setup.
    // The WiFi transfer path above deliberately keeps the speaker disabled.
    initCardputerHardware(false, false);
    bool sdReady = sdcardInit();
    for (int attempt = 0; !sdReady && attempt < 2; ++attempt) {
        delay(100);
        sdReady = sdcardRetry();
    }
    // Allocate the small display crop immediately after the large contiguous
    // Mac RAM block, before the remaining emulator services fragment the heap.
    dispInit();
    // Reserve the SCSI DMA window early too.  It lives in IRAM (data-bus
    // accessible on ESP32-S3) so it does not shrink the internal-DRAM heap
    // needed by the 256KB Mac RAM, SD mount and display.
    scsiDataBuffer = static_cast<uint8_t *>(heap_caps_malloc(
        SCSI_DATA_BUFFER_BYTES, MALLOC_CAP_EXEC | MALLOC_CAP_8BIT));
    if (scsiDataBuffer == nullptr) {
        scsiDataBuffer = static_cast<uint8_t *>(malloc(SCSI_DATA_BUFFER_BYTES));
    }
    MACPLUS_LOG("SCSI buffer reserved at %p (%d bytes)\n",
           scsiDataBuffer, SCSI_DATA_BUFFER_BYTES);
    hdReserveStorage();
    bootOk = true;

    // NVS for PRAM. Keep this after the large contiguous Mac RAM allocation.
    initNvs();
    loadMacSettings();

    // Cardputer board: display, keyboard, IMU, G0 button and speaker.  The
    // microphone is unused.  Mac PRAM is emulated in software and kept in
    // NVS, so the hardware RTC remains disabled.
    // The normal emulator path polls the TCA8418 FIFO directly after audio
    // is ready. Do not also arm M5Cardputer's GPIO11-only keyboard reader.
    // Reserve the small I2S DMA buffers before SD and emulator services
    // fragment the no-PSRAM heap.
    sndInit();

    const char *startingMsg[] = {
        "BOOT CARDPUTER MAC PLUS",
        "[RUN] HARDWARE INIT",
        "[RUN] CHECKING STORAGE",
    };
    dispShowMessage(startingMsg, 3);

    cardputerInputInit();

    if (sdReady) macplusStorageSetup();
    if (sdReady) {
        /* Open the optional software disk on the setup task. Its FATFS/newlib
           call chain is too deep for the 68000 task's tight no-PSRAM stack;
           the normal runtime already has a valid raw Flash system cache, so
           the single SD file slot can be dedicated to this stream. */
        hdPrepareInstallVolume();
    }

    loadPram();
    rtcInit(pram);
    const bool romReady = validateEmbeddedRom();
    if (romReady) {
        romData = macplus_rom_embedded;
        MACPLUS_LOG("ROM: embedded patched Mac Plus image ready (256KB RAM build)\n");
    }

    const char *bootMsg[] = {
        "BOOT CARDPUTER MAC PLUS",
        sdReady ? "[OK] SD CARD" : "[FAIL] SD CARD",
        romReady ? "[OK] ROM" : "[FAIL] ROM",
    };
    dispShowMessage(bootMsg, 3);

    if (!romReady) {
        MACPLUS_LOG("\nEmbedded Mac Plus ROM is missing or invalid.\n");
        MACPLUS_LOG("Regenerate roms/macplus-patched-256.rom with tools/patch_rom.py.\n");
        const char *missingMsg[] = {
            "MACPLUS.ROM MISSING",
            "REGENERATE ROM",
            "AND REBUILD",
        };
        dispShowMessage(missingMsg, 3);
        return;
    }

    // Start emulation on core 0; the display task starts inside tmeStartEmu
    // on core 1.
    const BaseType_t taskResult =
        // The optional software disk is prepared above, so keep the task at
        // the 12 KiB stack that fits alongside the no-PSRAM display/SD heap.
        xTaskCreatePinnedToCore(emuTask, "emu", 3072, nullptr, 1, nullptr, 0);
    if (taskResult != pdPASS) {
        MACPLUS_LOG("FATAL: cannot create emulation task\n");
    }
}

void loop() {
    if (!bootOk) {
        delay(1000);
        return;
    }
    cardputerInputPoll();
    dispService();
    delay(2);
}
