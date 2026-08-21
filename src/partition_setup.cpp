#include "debug_log.h"
#include "partition_setup.h"

#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "esp_flash.h"
#include "esp_flash_partitions.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_rom_md5.h"
#include "nvs.h"

#include "input.h"
#include "storage_layout.h"

extern "C" {
#include "tme/disp.h"
}

namespace {

constexpr uint32_t kTableSectorBytes = 0x1000U;
constexpr uint32_t kImageSectorBytes = 512U;
constexpr uint32_t kMinHdImageBytes = 0xC402U;
constexpr uint8_t kMacKeyY = 0x10;
constexpr uint8_t kMacKeyEscape = 0x35;
constexpr uint8_t kNoKey = 0xFF;

esp_flash_os_functions_t partitionTableOsFunctions;

esp_err_t IRAM_ATTR allowValidatedPartitionTableWrite(void *, size_t, size_t) {
    return ESP_OK;
}

struct TableInfo {
    size_t entryCount = 0;
    size_t endOffset = 0;
    int macplusIndex = -1;
    uint32_t maxEnd = 0;
};

uint32_t alignUp(uint32_t value, uint32_t alignment) {
    return (value + alignment - 1U) & ~(alignment - 1U);
}

bool labelMatches(const uint8_t label[16], const char *expected) {
    char copy[17] = {};
    memcpy(copy, label, 16);
    return strcmp(copy, expected) == 0;
}

bool validateRanges(const uint8_t *table, size_t entryCount) {
    const auto *entries =
        reinterpret_cast<const esp_partition_info_t *>(table);
    for (size_t i = 0; i < entryCount; ++i) {
        const auto &entry = entries[i];
        if ((entry.pos.offset & 0xFFFU) != 0 ||
            (entry.pos.size & 0xFFFU) != 0 ||
            entry.pos.offset < ESP_PARTITION_TABLE_OFFSET +
                                   kTableSectorBytes ||
            (entry.type == ESP_PARTITION_TYPE_APP &&
             (entry.pos.offset & 0xFFFFU) != 0)) {
            return false;
        }
        const uint32_t end = entry.pos.offset + entry.pos.size;
        for (size_t j = i + 1; j < entryCount; ++j) {
            const auto &other = entries[j];
            const uint32_t otherEnd = other.pos.offset + other.pos.size;
            if (entry.pos.offset < otherEnd && other.pos.offset < end) {
                return false;
            }
        }
    }
    return true;
}

bool inspectTable(uint8_t *table, TableInfo &info) {
    int verifiedEntries = 0;
    if (esp_partition_table_verify(
            reinterpret_cast<const esp_partition_info_t *>(table), true,
            &verifiedEntries) != ESP_OK) {
        MACPLUS_LOG("STORAGE: current partition table verification failed\n");
        return false;
    }

    for (size_t offset = 0;
         offset + sizeof(esp_partition_info_t) <= ESP_PARTITION_TABLE_MAX_LEN;
         offset += sizeof(esp_partition_info_t)) {
        auto *entry = reinterpret_cast<esp_partition_info_t *>(table + offset);
        if (entry->magic == ESP_PARTITION_MAGIC) {
            if (entry->pos.size == 0 ||
                entry->pos.offset > UINT32_MAX - entry->pos.size) {
                return false;
            }
            const uint32_t end = entry->pos.offset + entry->pos.size;
            if (end > info.maxEnd) info.maxEnd = end;
            if (labelMatches(entry->label, MACPLUS_DATA_PARTITION_LABEL)) {
                if (info.macplusIndex >= 0) return false;
                info.macplusIndex = static_cast<int>(info.entryCount);
            }
            ++info.entryCount;
            continue;
        }
        if (entry->magic == ESP_PARTITION_MAGIC_MD5 ||
            entry->magic == 0xFFFFU) {
            info.endOffset = offset;
            return info.entryCount == static_cast<size_t>(verifiedEntries) &&
                   validateRanges(table, info.entryCount);
        }
        return false;
    }
    return false;
}

bool readHdImageSize(uint32_t &imageBytes) {
    struct stat info = {};
    if (stat("/sd/hd.img", &info) != 0 || info.st_size <= kMinHdImageBytes ||
        static_cast<uint64_t>(info.st_size) > UINT32_MAX ||
        (info.st_size % kImageSectorBytes) != 0) {
        return false;
    }
    imageBytes = static_cast<uint32_t>(info.st_size);
    return true;
}

bool calculatePartitionSize(uint32_t imageBytes, uint32_t &partitionBytes) {
    const uint64_t wanted = static_cast<uint64_t>(imageBytes) +
                            MACPLUS_HD_METADATA_BYTES +
                            MACPLUS_STORAGE_HEADROOM_BYTES;
    if (wanted > UINT32_MAX - (MACPLUS_STORAGE_ALIGNMENT_BYTES - 1U)) {
        return false;
    }
    partitionBytes = alignUp(static_cast<uint32_t>(wanted),
                             MACPLUS_STORAGE_ALIGNMENT_BYTES);
    return true;
}

void showError(const char *reason, const char *action) {
    const char *lines[] = {
        "MACPLUS STORAGE",
        reason,
        action,
    };
    dispShowMessage(lines, 3);
    MACPLUS_LOG("STORAGE: %s; %s\n", reason, action);
}

bool waitForConfirmation(const char *operation, uint32_t imageBytes,
                         uint32_t partitionBytes) {
    while (cardputerInputAnyKeyPressed()) delay(10);
    while (cardputerInputReadKeyPress() != kNoKey) {}

    char sizeLine[40];
    snprintf(sizeLine, sizeof(sizeLine), "HD %luK / PART %luK",
             static_cast<unsigned long>((imageBytes + 1023U) / 1024U),
             static_cast<unsigned long>(partitionBytes / 1024U));
    const char *lines[] = {
        "MACPLUS STORAGE",
        operation,
        sizeLine,
        "Y=CONFIRM  ESC=SKIP",
    };
    dispShowMessage(lines, 4);

    for (;;) {
        const uint8_t key = cardputerInputReadKeyPress();
        if (key == kMacKeyY) return true;
        if (key == kMacKeyEscape) return false;
        if (Serial.available() > 0) {
            const int serialKey = Serial.read();
            if (serialKey == 'y' || serialKey == 'Y') return true;
            if (serialKey == 0x1B) return false;
        }
        delay(10);
    }
}

void rebuildMd5(uint8_t *table, size_t md5Offset) {
    auto *md5 = reinterpret_cast<esp_partition_info_t *>(table + md5Offset);
    memset(md5, 0xFF, sizeof(*md5));
    md5->magic = ESP_PARTITION_MAGIC_MD5;

    md5_context_t context;
    esp_rom_md5_init(&context);
    esp_rom_md5_update(&context, table, static_cast<uint32_t>(md5Offset));
    esp_rom_md5_final(table + md5Offset + ESP_PARTITION_MD5_OFFSET,
                      &context);
}

bool verifyGeneratedTable(uint8_t *table, size_t expectedEntries) {
    int entries = 0;
    return esp_partition_table_verify(
               reinterpret_cast<const esp_partition_info_t *>(table), true,
               &entries) == ESP_OK &&
           entries == static_cast<int>(expectedEntries);
}

bool writeTable(const uint8_t *table) {
    if (esp_flash_default_chip == nullptr ||
        esp_flash_default_chip->os_func == nullptr) {
        MACPLUS_LOG("STORAGE: flash OS functions unavailable\n");
        return false;
    }

    const esp_flash_os_functions_t *originalOsFunctions =
        esp_flash_default_chip->os_func;
    partitionTableOsFunctions = *originalOsFunctions;
    partitionTableOsFunctions.region_protected =
        allowValidatedPartitionTableWrite;
    esp_flash_default_chip->os_func = &partitionTableOsFunctions;

    uint8_t *verify = static_cast<uint8_t *>(malloc(kTableSectorBytes));
    if (verify == nullptr) {
        esp_flash_default_chip->os_func = originalOsFunctions;
        return false;
    }

    bool success = false;
    for (int attempt = 0; attempt < 3; ++attempt) {
        esp_err_t error = esp_flash_erase_region(
            nullptr, ESP_PARTITION_TABLE_OFFSET, kTableSectorBytes);
        if (error != ESP_OK) {
            MACPLUS_LOG("STORAGE: table erase %d/3 failed: %s\n", attempt + 1,
                   esp_err_to_name(error));
            continue;
        }

        error = esp_flash_write(nullptr, table, ESP_PARTITION_TABLE_OFFSET,
                                kTableSectorBytes);
        if (error != ESP_OK) {
            MACPLUS_LOG("STORAGE: table write %d/3 failed: %s\n", attempt + 1,
                   esp_err_to_name(error));
            continue;
        }

        error = esp_flash_read(nullptr, verify, ESP_PARTITION_TABLE_OFFSET,
                               kTableSectorBytes);
        const bool matches = error == ESP_OK &&
                             memcmp(verify, table, kTableSectorBytes) == 0;
        if (matches) {
            MACPLUS_LOG("STORAGE: partition table write verified\n");
            success = true;
            break;
        }
        MACPLUS_LOG("STORAGE: table verify %d/3 failed: %s\n", attempt + 1,
               error == ESP_OK ? "data mismatch" : esp_err_to_name(error));
    }

    free(verify);
    esp_flash_default_chip->os_func = originalOsFunctions;
    return success;
}

bool registerLauncherDataPartition() {
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running == nullptr || running->type != ESP_PARTITION_TYPE_APP ||
        running->subtype < ESP_PARTITION_SUBTYPE_APP_OTA_0 ||
        running->subtype > ESP_PARTITION_SUBTYPE_APP_OTA_15) {
        return true;
    }

    char key[16] = "s_";
    strncat(key, running->label, sizeof(key) - strlen(key) - 1U);
    nvs_handle_t handle;
    esp_err_t error = nvs_open("l_apps", NVS_READWRITE, &handle);
    if (error != ESP_OK) return false;
    error = nvs_set_str(handle, key, MACPLUS_DATA_PARTITION_LABEL);
    if (error == ESP_OK) error = nvs_commit(handle);
    nvs_close(handle);
    return error == ESP_OK;
}

} // namespace

void macplusStorageSetup() {
    const esp_partition_t *existing = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY,
        MACPLUS_DATA_PARTITION_LABEL);

    uint32_t imageBytes = 0;
    const bool haveImage = readHdImageSize(imageBytes);
    if (existing != nullptr &&
        (!haveImage || (existing->size > MACPLUS_HD_DATA_OFFSET &&
                        imageBytes <= existing->size - MACPLUS_HD_DATA_OFFSET))) {
        return;
    }
    if (!haveImage) {
        showError("[FAIL] INVALID HD.IMG", "COPY HD.IMG TO SD");
        return;
    }

    uint32_t partitionBytes = 0;
    if (!calculatePartitionSize(imageBytes, partitionBytes)) {
        showError("[FAIL] IMAGE TOO LARGE", "USE A SMALLER HD.IMG");
        return;
    }

    uint8_t *table = static_cast<uint8_t *>(malloc(kTableSectorBytes));
    if (table == nullptr) {
        showError("[FAIL] LOW MEMORY", "REBOOT AND TRY AGAIN");
        return;
    }
    esp_err_t error = esp_flash_read(nullptr, table,
                                     ESP_PARTITION_TABLE_OFFSET,
                                     kTableSectorBytes);
    TableInfo info;
    if (error != ESP_OK || !inspectTable(table, info)) {
        free(table);
        showError("[FAIL] TABLE INVALID", "USE LAUNCHER PMAN");
        return;
    }

    uint32_t flashBytes = 0;
    if (esp_flash_get_size(nullptr, &flashBytes) != ESP_OK) flashBytes = 0;

    bool resize = false;
    uint32_t partitionOffset = 0;
    if (info.macplusIndex >= 0) {
        auto *entry = reinterpret_cast<esp_partition_info_t *>(table) +
                      info.macplusIndex;
        if (entry->type != ESP_PARTITION_TYPE_DATA || existing == nullptr) {
            free(table);
            showError("[FAIL] LABEL CONFLICT", "FIX 'macplus' IN PMAN");
            return;
        }
        if (entry->pos.offset + entry->pos.size != info.maxEnd) {
            free(table);
            showError("[FAIL] CANNOT EXPAND", "ENLARGE IN LAUNCHER PMAN");
            return;
        }
        partitionOffset = entry->pos.offset;
        resize = true;
    } else {
        if (info.endOffset + 2U * sizeof(esp_partition_info_t) >
            ESP_PARTITION_TABLE_MAX_LEN) {
            free(table);
            showError("[FAIL] TABLE FULL", "USE LAUNCHER PMAN");
            return;
        }
        partitionOffset = alignUp(info.maxEnd,
                                  MACPLUS_STORAGE_ALIGNMENT_BYTES);
    }

    const uint64_t requiredEnd = static_cast<uint64_t>(partitionOffset) +
                                 partitionBytes +
                                 MACPLUS_STORAGE_TAIL_RESERVE_BYTES;
    if (flashBytes == 0 || requiredEnd > flashBytes) {
        free(table);
        showError("[FAIL] NOT ENOUGH SPACE", "FREE SPACE IN LAUNCHER");
        return;
    }

    MACPLUS_LOG("STORAGE: %s request image=%lu old=%lu target=%lu offset=0x%lX "
           "flash=%lu reserve=%lu\n",
           resize ? "resize" : "create",
           static_cast<unsigned long>(imageBytes),
           static_cast<unsigned long>(existing == nullptr ? 0U : existing->size),
           static_cast<unsigned long>(partitionBytes),
           static_cast<unsigned long>(partitionOffset),
           static_cast<unsigned long>(flashBytes),
           static_cast<unsigned long>(MACPLUS_STORAGE_TAIL_RESERVE_BYTES));

    if (!waitForConfirmation(resize ? "RESIZE DATA PARTITION?"
                                    : "CREATE DATA PARTITION?",
                             imageBytes, partitionBytes)) {
        free(table);
        showError("[OK] CREATION SKIPPED", "REBOOT TO TRY AGAIN");
        return;
    }

    size_t md5Offset = info.endOffset;
    if (resize) {
        auto *entry = reinterpret_cast<esp_partition_info_t *>(table) +
                      info.macplusIndex;
        entry->pos.size = partitionBytes;
        memset(table + info.endOffset, 0xFF,
               kTableSectorBytes - info.endOffset);
    } else {
        memset(table + info.endOffset, 0xFF,
               kTableSectorBytes - info.endOffset);
        auto *entry = reinterpret_cast<esp_partition_info_t *>(
            table + info.endOffset);
        memset(entry, 0, sizeof(*entry));
        entry->magic = ESP_PARTITION_MAGIC;
        entry->type = ESP_PARTITION_TYPE_DATA;
        entry->subtype = ESP_PARTITION_SUBTYPE_DATA_SPIFFS;
        entry->pos.offset = partitionOffset;
        entry->pos.size = partitionBytes;
        memcpy(entry->label, MACPLUS_DATA_PARTITION_LABEL,
               strlen(MACPLUS_DATA_PARTITION_LABEL));
        md5Offset += sizeof(*entry);
    }
    rebuildMd5(table, md5Offset);

    const size_t expectedEntries = info.entryCount + (resize ? 0U : 1U);
    if (!verifyGeneratedTable(table, expectedEntries)) {
        free(table);
        showError("[FAIL] NEW TABLE INVALID", "NO CHANGES WRITTEN");
        return;
    }

    const char *writing[] = {
        "MACPLUS STORAGE",
        "[RUN] WRITING TABLE",
        "DO NOT POWER OFF",
    };
    dispShowMessage(writing, 3);
    const bool written = writeTable(table);
    free(table);
    if (!written) {
        showError("[FAIL] WRITE ERROR", "REFLASH LAUNCHER");
        return;
    }

    const bool launcherLinked = registerLauncherDataPartition();

    MACPLUS_LOG("STORAGE: %s '%s' at 0x%lX, %lu bytes\n",
           resize ? "resized" : "created", MACPLUS_DATA_PARTITION_LABEL,
           static_cast<unsigned long>(partitionOffset),
           static_cast<unsigned long>(partitionBytes));
    MACPLUS_LOG("STORAGE: Launcher data link %s\n",
           launcherLinked ? "ready" : "unavailable");
    const char *done[] = {
        "MACPLUS STORAGE",
        "[OK] PARTITION READY",
        launcherLinked ? "RESTARTING..." : "[WARN] LINK NOT SAVED",
    };
    dispShowMessage(done, 3);
    delay(500);
    ESP.restart();
}
