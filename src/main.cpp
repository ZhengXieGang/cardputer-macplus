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
#include "webinstall.h"

extern "C" {
#include "tme/emu.h"
#include "tme/rtc.h"
#include "tme/tmeconfig.h"
#include "tme/hd.h"
#include "tme/iwm.h"
#include "tme/disp.h"
#include "tme/mouse.h"
#include "tme/scc.h"
#include "tme/snd.h"
#include "tme/via.h"
}

static char pram[32];
static volatile bool controlTaskActive = false;
static volatile bool setupComplete = false;
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
        printf("NVS: shared partition unavailable (%s); using default PRAM\n",
               esp_err_to_name(err));
    }
    if (err != ESP_OK) printf("NVS: init failed (%s)\n", esp_err_to_name(err));
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
    printf("ROM: %s first8=%02X%02X%02X%02X-%02X%02X%02X%02X pc=%08lX sum=%08lX\n",
           source,
           rom[0], rom[1], rom[2], rom[3],
           rom[4], rom[5], rom[6], rom[7],
           static_cast<unsigned long>(initPC),
           static_cast<unsigned long>(storedChecksum));
    if (rom[0] == 0xFF && rom[1] == 0xFF) {
        printf("ROM: blank partition\n");
        return false;
    }
    if (storedChecksum != MAC_PLUS_ROM_CHECKSUM) {
        printf("ROM: not a Mac Plus v3 ROM (checksum mismatch)\n");
        return false;
    }
    if (initPC != MAC_PLUS_ROM_INITIAL_PC) {
        printf("ROM: unexpected initial PC\n");
        return false;
    }
    return true;
}

static bool validateEmbeddedRom() {
    if (macplus_rom_embedded_size != TME_ROMSIZE) {
        printf("ROM: embedded size mismatch (%u)\n",
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
            printf("PRAM loaded from NVS (%d bytes)\n", (int)len);
        } else {
            printf("No PRAM in NVS, using defaults\n");
        }
        nvs_close(handle);
    }
}

static void emuTask(void *param) {
    (void)param;
    printf("Starting Mac Plus emulation on core %d...\n", xPortGetCoreID());
    tmeStartEmu(const_cast<uint8_t *>(romData));
    vTaskDelete(NULL);
}

static void handleSerialCommand(const char *line) {
    if (line == nullptr || line[0] == '\0') return;
    String cmd(line);
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd == "help") {
        printf("Commands: help, status, disp, imu, mouse, mousehw, keys, keytest [scan], hdstats, hdcache clear, install, sens [value], flipx, flipy, swapxy, sdtest, sdwritetest, reboot\n");
    } else if (cmd.startsWith("sens ")) {
        imuSensitivity = atof(cmd.substring(5).c_str());
        if (imuSensitivity < 0.1f) imuSensitivity = 0.1f;
        if (imuSensitivity > 1000.0f) imuSensitivity = 1000.0f;
        printf("IMU max speed = %.1f px/s at full tilt\n", imuSensitivity);
    } else if (cmd == "sens") {
        printf("IMU max speed = %.1f px/s at full tilt\n", imuSensitivity);
    } else if (cmd == "imu") {
        cardputerInputImuStatus();
    } else if (cmd == "mouse") {
        cardputerInputMouseStatus();
    } else if (cmd == "mousehw") {
        mouseDebugPrint();
        viaDebugPrint();
        sccDebugPrint();
        printf("MOUSEHW: cursor=%u,%u\n", (unsigned)tmeGetMouseX(),
               (unsigned)tmeGetMouseY());
    } else if (cmd == "keys") {
        cardputerInputKeysStatus();
    } else if (cmd.startsWith("keytest ")) {
        const int scan = atoi(cmd.substring(8).c_str());
        if (scan < 0 || scan > 0x7F) {
            printf("KEYTEST: scan must be 0..127\n");
        } else {
            kbdPushKey(static_cast<uint8_t>(scan), 0);
            kbdPushKey(static_cast<uint8_t>(scan), 1);
            printf("KEYTEST: queued scan=0x%02X press+release\n", scan);
        }
    } else if (cmd == "hdstats") {
        printf("HD: commands=%lu reads=%lu sectors writes=%lu sectors floppy_bytes=%lu io_us=%lu "
               "read_err=%lu write_err=%lu seek_err=%lu last_cmd=0x%02lX "
               "lba=%lu len=%lu err=%lu\n",
               (unsigned long)hdGetCommandCount(),
               (unsigned long)hdGetReadSectors(),
               (unsigned long)hdGetWriteSectors(),
               (unsigned long)iwmGetFloppyReadCount(),
               (unsigned long)hdGetReadIOMicros(),
               (unsigned long)hdGetReadErrorCount(),
               (unsigned long)hdGetWriteErrorCount(),
               (unsigned long)hdGetSeekErrorCount(),
               (unsigned long)hdGetLastCommand(),
               (unsigned long)hdGetLastLba(),
               (unsigned long)hdGetLastLength(),
               (unsigned long)hdGetLastIoError());
    } else if (cmd == "disp") {
        printf("DISP: panel=%d frames=%lu requests=%lu dropped=%lu alive=%lu\n",
               M5.Display.getPanel() != nullptr ? 1 : 0,
               (unsigned long)dispGetFrameCount(),
               (unsigned long)dispGetRequestCount(),
               (unsigned long)dispGetDroppedFrameCount(),
               (unsigned long)dispGetAliveTicks());
    } else if (cmd == "hdcache clear") {
        hdInvalidateRawCache();
    } else if (cmd == "install" || cmd == "web") {
        requestWebInstallMode();
    } else if (cmd == "flipx") {
        imuFlipX = -imuFlipX;
        printf("IMU X direction flipped (now %d)\n", imuFlipX);
    } else if (cmd == "flipy") {
        imuFlipY = -imuFlipY;
        printf("IMU Y direction flipped (now %d)\n", imuFlipY);
    } else if (cmd == "swapxy") {
        imuSwapXY = !imuSwapXY;
        printf("IMU X/Y axes swapped (now %d)\n", imuSwapXY);
    } else if (cmd == "status") {
        printf("STATUS: SD=%s, free heap=%d, largest=%d, free IRAM=%d, "
               "largest IRAM=%d, free PSRAM=%d\n",
               sdcardMounted() ? "mounted" : "not mounted",
               (int)esp_get_free_heap_size(),
               (int)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
               (int)heap_caps_get_free_size(MALLOC_CAP_EXEC),
               (int)heap_caps_get_largest_free_block(MALLOC_CAP_EXEC),
               (int)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    } else if (cmd == "sdtest") {
        if (!setupComplete) {
            printf("SDTEST: boot initialization still active\n");
            return;
        }
        const bool ready = sdcardRetry();
        printf("SDTEST: %s\n", ready ? "mounted" : "failed");
        sdcardPrintRoot();
    } else if (cmd == "sdwritetest") {
        if (!setupComplete) {
            printf("SDTESTIO: boot initialization still active\n");
            return;
        }
        const bool ok = sdcardRunWriteReadTest(16384);
        printf("SDTESTIO: command=%s\n", ok ? "PASS" : "FAIL");
    } else if (cmd == "reboot") {
        hdFlushNow();
        ESP.restart();
    } else if (cmd.startsWith("mousetest ")) {
        const int dx = atoi(cmd.substring(10).c_str());
        const int comma = cmd.indexOf(',', 10);
        const int dy = comma >= 0
            ? atoi(cmd.substring(comma + 1).c_str()) : 0;
        mouseMove(dx, dy, 0);
        printf("MOUSETEST: dx=%d dy=%d\n", dx, dy);
    }
}

static void pollSerialCommands() {
    static char line[192] = {};
    static size_t length = 0;
    static bool overflow = false;
    while (Serial.available() > 0) {
        const int value = Serial.read();
        if (value < 0) break;
        const char byte = static_cast<char>(value);
        if (byte == '\r' || byte == '\n') {
            if (overflow) {
                printf("SERIAL: command rejected (too long)\n");
            } else if (length != 0) {
                line[length] = '\0';
                handleSerialCommand(line);
            }
            length = 0;
            overflow = false;
        } else if (!overflow && length + 1 < sizeof(line)) {
            line[length++] = byte;
        } else {
            overflow = true;
        }
    }
}

static void controlTask(void *param) {
    (void)param;
    for (;;) {
        pollSerialCommands();
        cardputerInputPoll();
        dispService();
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void setup() {
    Serial.begin(115200);
    delay(300);
    printf("\n\n=== Cardputer-Adv Mac Plus ===\n");
    printf("CPU: %dMHz, flash: %uMB, PSRAM: %uMB\n", ESP.getCpuFreqMHz(),
           ESP.getFlashChipSize() / (1024 * 1024),
           ESP.getPsramSize() / (1024 * 1024));
    printf("Free heap before init: %d bytes\n", (int)esp_get_free_heap_size());

    if (webInstallModeRequested()) {
        printf("BOOT: integrated WiFi transfer mode selected\n");
        initNvs();
        initCardputerHardware(false, true);
        webInstallRun();
        return;
    }

    // Reserve the emulated Mac RAM while the heap is still pristine.
    macRam = static_cast<unsigned char *>(malloc(TME_RAMSIZE));
    if (macRam == nullptr) {
        printf("FATAL: cannot allocate %d bytes for Mac RAM\n", TME_RAMSIZE);
        return;
    }
    printf("Mac RAM reserved at %p (%d bytes)\n", macRam, TME_RAMSIZE);
    // Reserve the SCSI DMA window early too.  It lives in IRAM (data-bus
    // accessible on ESP32-S3) so it does not shrink the internal-DRAM heap
    // needed by the 256KB Mac RAM, SD mount and display.
    scsiDataBuffer = static_cast<uint8_t *>(heap_caps_malloc(
        SCSI_DATA_BUFFER_BYTES, MALLOC_CAP_EXEC | MALLOC_CAP_8BIT));
    if (scsiDataBuffer == nullptr) {
        scsiDataBuffer = static_cast<uint8_t *>(malloc(SCSI_DATA_BUFFER_BYTES));
    }
    printf("SCSI buffer reserved at %p (%d bytes)\n",
           scsiDataBuffer, SCSI_DATA_BUFFER_BYTES);
    hdReserveStorage();
    bootOk = true;

    // NVS for PRAM. Keep this after the large contiguous Mac RAM allocation.
    initNvs();

    // Cardputer board: display, keyboard, IMU, G0 button and speaker.  The
    // microphone is unused.  Mac PRAM is emulated in software and kept in
    // NVS, so the hardware RTC remains disabled.
    // The normal emulator path polls the TCA8418 FIFO directly after audio
    // is ready. Do not also arm M5Cardputer's GPIO11-only keyboard reader.
    initCardputerHardware(true, false);
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

    bool sdReady = sdcardInit();
    // A card can still be busy recovering from the previous reset. Retry a
    // few times before starting the emulator without storage.
    for (int attempt = 0; !sdReady && attempt < 2; ++attempt) {
        delay(100);
        sdReady = sdcardRetry();
    }
    sdcardPrintRoot();

    if (sdReady) macplusStorageSetup();

    loadPram();
    rtcInit(pram);
    const bool romReady = validateEmbeddedRom();
    if (romReady) {
        romData = macplus_rom_embedded;
        printf("ROM: embedded patched Mac Plus image ready (256KB RAM build)\n");
    }

    const char *bootMsg[] = {
        "BOOT CARDPUTER MAC PLUS",
        sdReady ? "[OK] SD CARD" : "[FAIL] SD CARD",
        romReady ? "[OK] ROM" : "[FAIL] ROM",
    };
    dispShowMessage(bootMsg, 3);

    if (!romReady) {
        printf("\nEmbedded Mac Plus ROM is missing or invalid.\n");
        printf("Regenerate roms/macplus-patched-256.rom with tools/patch_rom.py.\n");
        const char *missingMsg[] = {
            "MACPLUS.ROM MISSING",
            "REGENERATE ROM",
            "AND REBUILD",
        };
        dispShowMessage(missingMsg, 3);
        setupComplete = true;
        return;
    }

    const BaseType_t controlResult = xTaskCreatePinnedToCore(
        controlTask, "control", 3072, nullptr, 4, nullptr, 1);
    controlTaskActive = controlResult == pdPASS;
    printf("CONTROL: input service task %s\n",
           controlTaskActive ? "ready" : "unavailable");

    // Start emulation on core 0; the display task starts inside tmeStartEmu
    // on core 1.
    const BaseType_t taskResult =
        xTaskCreatePinnedToCore(emuTask, "emu", 6144, nullptr, 1, nullptr, 0);
    if (taskResult != pdPASS) {
        printf("FATAL: cannot create emulation task\n");
    }
    setupComplete = true;
}

void loop() {
    if (!bootOk) {
        delay(1000);
        return;
    }
    if (!controlTaskActive) {
        pollSerialCommands();
        cardputerInputPoll();
        dispService();
    }
    delay(2);
}
