#include "debug_log.h"
/* SCSI hard disk backed by the named `macplus` data partition. */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <sys/stat.h>
#include <Arduino.h>
#include "esp_heap_caps.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdcard.h"
#include "storage_layout.h"

extern "C" {
#include "tme/disp.h"
#include "tme/ncr.h"
#include "tme/hd.h"
#include "ff.h"
}

// One flash-sector-sized cache block is both the runtime write-back buffer
// and the one-time SD-to-flash copy buffer.  This keeps the no-PSRAM SRAM
// budget flat while allowing raw flash writes after startup.
static constexpr size_t HD_SD_IO_CHUNK_BYTES = 8 * 1024;
static constexpr size_t HD_CACHE_BLOCK_BYTES = 4096;
static constexpr size_t HD_CACHE_BLOCK_COUNT = 1;
// xTaskCreatePinnedToCore takes stack depth in words (not bytes).  Two
// thousand words cover the flash journal/write-back call chain while staying
// within the no-PSRAM heap, allowing flushes to stay off the 68K task.
static constexpr uint32_t HD_FLUSH_TASK_STACK_WORDS = 2048U;
// Keep the power-loss window short.  Reads still come from the mapped Flash
// image; this only affects the low-priority write-back task.
static constexpr uint32_t HD_FLUSH_INTERVAL_MS = 250;

static constexpr uint32_t HD_RAW_FLASH_VALID_MAGIC = 0x4D414344; // "MACD"
static constexpr uint32_t HD_RAW_FLASH_PINNED_MAGIC = 0x4D414357; // "MACW"
static constexpr uint32_t HD_RAW_FLASH_METADATA_BYTES = MACPLUS_HD_METADATA_BYTES;
// The metadata sector also contains a tiny write-ahead journal.  A pending
// record means the last 4 KiB write may have been interrupted; boot must then
// rebuild the cache from /sd/hd.img instead of trusting a torn block.
static constexpr uint32_t HD_RAW_FLASH_JOURNAL_OFFSET = 32U;
static constexpr uint32_t HD_RAW_FLASH_JOURNAL_RECORD_BYTES = 16U;
static constexpr uint32_t HD_RAW_FLASH_JOURNAL_SLOTS =
    (HD_RAW_FLASH_METADATA_BYTES - HD_RAW_FLASH_JOURNAL_OFFSET) /
    HD_RAW_FLASH_JOURNAL_RECORD_BYTES;
static constexpr uint32_t HD_RAW_FLASH_JOURNAL_PENDING = 0xF0F0F0F0U;
static constexpr uint32_t HD_RAW_FLASH_JOURNAL_COMMITTED = 0xF0F00000U;
static constexpr uint32_t INSTALL_400K_BYTES =
    MACPLUS_INSTALL_400K_BYTES;
static constexpr uint32_t INSTALL_800K_BYTES =
    MACPLUS_INSTALL_800K_BYTES;
// IWM requests sectors in Macintosh interleave order (0,6,1,7,...). Reading
// one 512-byte sector with a fresh fseek for every request makes FatFS repeat
// the same SD lookup many times while Finder scans an HFS catalog. One 8 KiB
// window covers a complete outer-track side (12 sectors) within the no-PSRAM
// memory budget.
// The no-PSRAM target needs one contiguous 256 KiB block for Mac RAM. Keep
// this cache at 8 KiB so it does not compete with the contiguous Mac RAM
// allocation.
static constexpr size_t INSTALL_READ_CACHE_BYTES = 8 * 1024;
static_assert((INSTALL_READ_CACHE_BYTES % 512U) == 0,
              "install read cache must be sector aligned");
static constexpr const char *INSTALL_IMAGE_PATH = "/sd/macplus-install.img";
static constexpr const char *INSTALL_BACKUP_PATH =
    "/sd/macplus-install.backup";
static constexpr const char *INSTALL_TEMP_PATH = "/sd/macplus-install.upload";
static const void *hdFlashMap = nullptr;
static spi_flash_mmap_handle_t hdFlashMapHandle = 0;
static const esp_partition_t *hdCachePartition = nullptr;
static uint32_t hdCacheBase = 0;
static uint32_t hdCacheMetadata = 0;
static uint32_t hdCacheImageMax = 0;

struct InstallVolume {
    FIL *file;
    uint32_t bytes;
    bool isMfs;
    bool readOnly;
};

static InstallVolume installVolume = {};
// FatFS keeps a private 512-byte sector window in each FIL.  This object is
// reserved dynamically immediately after Mac RAM, before board/SD setup
// fragments the remaining heap.
static FIL *reservedInstallFile = nullptr;
alignas(4) static uint8_t installReadCache[INSTALL_READ_CACHE_BYTES];
alignas(4) static uint8_t installWarmupSector[512];
struct InstallReadCache {
    uint32_t baseSector;
    uint32_t sectorCount;
    bool valid;
};
static InstallReadCache installReadCacheState = {};

struct RawFlashMetadata {
    uint32_t magic;
    uint32_t imageBytes;
    uint32_t fingerprint;
    uint32_t contentCrc;
};

static uint32_t hdCrc32(const uint8_t *data, size_t len, uint32_t crc);

struct RawFlashJournalRecord {
    uint32_t sequence;
    uint32_t blockIndex;
    uint32_t recordCrc;
    uint32_t state;
};

static uint32_t rawFlashJournalCrc(uint32_t sequence,
                                   uint32_t blockIndex) {
    uint32_t values[2] = {sequence, blockIndex};
    return hdCrc32(reinterpret_cast<const uint8_t *>(values),
                   sizeof(values), 0);
}

static void showHdStorageError(const char *error, const char *action) {
    const char *lines[] = {
        "MACPLUS STORAGE",
        error,
        action,
    };
    dispShowMessage(lines, 3);
    delay(3000);
}

static bool selectHdCacheStorage(uint32_t requiredBytes) {
    hdCachePartition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY,
        MACPLUS_DATA_PARTITION_LABEL);
    if (hdCachePartition == nullptr ||
        hdCachePartition->size <= MACPLUS_HD_DATA_OFFSET) {
        hdCacheBase = 0;
        hdCacheMetadata = 0;
        hdCacheImageMax = 0;
        return false;
    }
    hdCacheMetadata = hdCachePartition->address;
    hdCacheBase = hdCachePartition->address + MACPLUS_HD_DATA_OFFSET;
    hdCacheImageMax = hdCachePartition->size - MACPLUS_HD_DATA_OFFSET;
    return requiredBytes == 0 || requiredBytes <= hdCacheImageMax;
}

bool hdRawFlashStorageIsSafe(uint32_t bytes) {
    return bytes != 0 && selectHdCacheStorage(bytes);
}

uint32_t hdRawFlashStorageAddress(void) { return hdCacheBase; }

uint32_t hdRawFlashStorageMetadataAddress(void) { return hdCacheMetadata; }

uint32_t hdRawFlashStorageMaxImageBytes(void) { return hdCacheImageMax; }

bool hdRawFlashStorageRead(uint32_t offset, uint8_t *buffer, size_t bytes) {
    if (hdCachePartition == nullptr && !selectHdCacheStorage(0)) return false;
    if (buffer == nullptr || offset > hdCacheImageMax ||
        bytes > hdCacheImageMax - offset) {
        return false;
    }
    return esp_partition_read(hdCachePartition,
                              MACPLUS_HD_DATA_OFFSET + offset,
                              buffer, bytes) == ESP_OK;
}

static bool writeHdCache(uint32_t offset, const uint8_t *data, size_t bytes) {
    return hdCachePartition != nullptr &&
           esp_partition_write(hdCachePartition,
                               MACPLUS_HD_DATA_OFFSET + offset,
                               data, bytes) == ESP_OK;
}

static bool eraseHdCache(uint32_t offset, size_t bytes) {
    return hdCachePartition != nullptr &&
           esp_partition_erase_range(hdCachePartition,
                                     MACPLUS_HD_DATA_OFFSET + offset,
                                     bytes) == ESP_OK;
}

static bool readHdMetadata(uint32_t offset, void *data, size_t bytes) {
    return hdCachePartition != nullptr &&
           offset <= MACPLUS_HD_METADATA_BYTES &&
           bytes <= MACPLUS_HD_METADATA_BYTES - offset &&
           esp_partition_read(hdCachePartition, offset, data, bytes) == ESP_OK;
}

static bool writeHdMetadata(uint32_t offset, const void *data, size_t bytes) {
    return hdCachePartition != nullptr &&
           offset <= MACPLUS_HD_METADATA_BYTES &&
           bytes <= MACPLUS_HD_METADATA_BYTES - offset &&
           esp_partition_write(hdCachePartition, offset, data, bytes) == ESP_OK;
}

static bool eraseHdMetadata(void) {
    return hdCachePartition != nullptr &&
           esp_partition_erase_range(hdCachePartition, 0,
                                     MACPLUS_HD_METADATA_BYTES) == ESP_OK;
}

static bool rawImageBytesIsValid(uint32_t imageBytes) {
    return imageBytes > 0xC402U && (imageBytes % 512U) == 0 &&
           imageBytes <= hdCacheImageMax;
}

static uint32_t hdCrc32(const uint8_t *data, size_t len, uint32_t crc) {
    crc = ~crc;
    while (len--) {
        crc ^= *data++;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

static bool rawFlashFingerprint(uint32_t imageBytes, uint32_t *fingerprint) {
    if (fingerprint == nullptr || !rawImageBytesIsValid(imageBytes)) return false;
    uint8_t sector[512];
    uint32_t crc = 0;
    if (!hdRawFlashStorageRead(0, sector, sizeof(sector))) {
        return false;
    }
    crc = hdCrc32(sector, sizeof(sector), crc);
    if (!hdRawFlashStorageRead(imageBytes - sizeof(sector), sector,
                               sizeof(sector))) {
        return false;
    }
    crc = hdCrc32(sector, sizeof(sector), crc);
    *fingerprint = crc ^ imageBytes;
    return true;
}

static bool readRawFlashMetadata(RawFlashMetadata *metadata) {
    if (metadata == nullptr || !readHdMetadata(0, metadata, sizeof(*metadata))) {
        return false;
    }
    return (metadata->magic == HD_RAW_FLASH_VALID_MAGIC ||
            metadata->magic == HD_RAW_FLASH_PINNED_MAGIC) &&
           rawImageBytesIsValid(metadata->imageBytes);
}

static bool writeRawFlashMetadata(uint32_t magic, uint32_t imageBytes,
                                  uint32_t fingerprint, uint32_t contentCrc) {
    if ((magic != HD_RAW_FLASH_VALID_MAGIC &&
         magic != HD_RAW_FLASH_PINNED_MAGIC) ||
        !rawImageBytesIsValid(imageBytes)) {
        return false;
    }
    RawFlashMetadata metadata = {magic, imageBytes, fingerprint, contentCrc};
    return eraseHdMetadata() && writeHdMetadata(0, &metadata, sizeof(metadata));
}

static bool rawFlashJournalRecordIsErased(
    const RawFlashJournalRecord &record) {
    const uint32_t *words = reinterpret_cast<const uint32_t *>(&record);
    return words[0] == 0xFFFFFFFFU && words[1] == 0xFFFFFFFFU &&
           words[2] == 0xFFFFFFFFU && words[3] == 0xFFFFFFFFU;
}

static bool rawFlashJournalRecordIsValid(
    const RawFlashJournalRecord &record) {
    return (record.state == HD_RAW_FLASH_JOURNAL_PENDING ||
            record.state == HD_RAW_FLASH_JOURNAL_COMMITTED) &&
           record.recordCrc == rawFlashJournalCrc(record.sequence,
                                                   record.blockIndex);
}

// Return true when the newest journal entry is not a completed transaction.
// A damaged/non-erased entry is treated conservatively as pending.
static bool rawFlashJournalHasPending(void) {
    uint32_t newestSequence = 0;
    uint32_t newestState = 0;
    bool haveNewest = false;
    int lastNonErasedSlot = -1;
    int lastDamagedSlot = -1;
    for (uint32_t slot = 0; slot < HD_RAW_FLASH_JOURNAL_SLOTS; ++slot) {
        RawFlashJournalRecord record = {};
        const uint32_t offset = HD_RAW_FLASH_JOURNAL_OFFSET +
                                slot * HD_RAW_FLASH_JOURNAL_RECORD_BYTES;
        if (!readHdMetadata(offset, &record, sizeof(record))) {
            return true;
        }
        if (rawFlashJournalRecordIsErased(record)) continue;
        lastNonErasedSlot = static_cast<int>(slot);
        if (!rawFlashJournalRecordIsValid(record)) {
            lastDamagedSlot = static_cast<int>(slot);
            continue;
        }
        if (!haveNewest ||
            static_cast<int32_t>(record.sequence - newestSequence) > 0) {
            haveNewest = true;
            newestSequence = record.sequence;
            newestState = record.state;
        }
    }
    if (lastNonErasedSlot < 0) return false;
    return (lastDamagedSlot == lastNonErasedSlot) ||
           (haveNewest && newestState != HD_RAW_FLASH_JOURNAL_COMMITTED);
}

static bool rawFlashJournalAppend(uint32_t blockIndex, uint32_t state) {
    if (state != HD_RAW_FLASH_JOURNAL_PENDING &&
        state != HD_RAW_FLASH_JOURNAL_COMMITTED) {
        return false;
    }

    int freeSlot = -1;
    uint32_t newestSequence = 0;
    bool haveSequence = false;
    for (uint32_t slot = 0; slot < HD_RAW_FLASH_JOURNAL_SLOTS; ++slot) {
        RawFlashJournalRecord record = {};
        const uint32_t offset = HD_RAW_FLASH_JOURNAL_OFFSET +
                                slot * HD_RAW_FLASH_JOURNAL_RECORD_BYTES;
        if (!readHdMetadata(offset, &record, sizeof(record))) {
            return false;
        }
        if (rawFlashJournalRecordIsErased(record)) {
            if (freeSlot < 0) freeSlot = static_cast<int>(slot);
            continue;
        }
        if (rawFlashJournalRecordIsValid(record) &&
            (!haveSequence ||
             static_cast<int32_t>(record.sequence - newestSequence) > 0)) {
            newestSequence = record.sequence;
            haveSequence = true;
        }
    }

    if (freeSlot < 0) {
        // Rotate the journal.  Preserve the current image metadata, then
        // start the new transaction in the freshly erased sector.
        RawFlashMetadata metadata = {};
        if (!readRawFlashMetadata(&metadata) ||
            !eraseHdMetadata() || !writeHdMetadata(0, &metadata,
                                                   sizeof(metadata))) {
            return false;
        }
        freeSlot = 0;
        haveSequence = false;
    }

    RawFlashJournalRecord record = {
        haveSequence ? newestSequence + 1U : 1U,
        blockIndex,
        0,
        state,
    };
    record.recordCrc = rawFlashJournalCrc(record.sequence, record.blockIndex);
    const uint32_t offset = HD_RAW_FLASH_JOURNAL_OFFSET +
                            static_cast<uint32_t>(freeSlot) *
                                HD_RAW_FLASH_JOURNAL_RECORD_BYTES;
    return writeHdMetadata(offset, &record, sizeof(record));
}

static bool prepareInstallVolume(void) {
    if (installVolume.file != nullptr) return true;
    if (!sdcardMounted() || !sdcardAcquire(2000)) return false;

    struct stat info = {};
    struct stat backupInfo = {};
    const bool haveImage = stat(INSTALL_IMAGE_PATH, &info) == 0;
    if (!haveImage &&
        stat(INSTALL_BACKUP_PATH, &backupInfo) == 0) {
        rename(INSTALL_BACKUP_PATH, INSTALL_IMAGE_PATH);
    } else if (haveImage &&
               stat(INSTALL_BACKUP_PATH, &backupInfo) == 0) {
        remove(INSTALL_BACKUP_PATH);
    }
    uint8_t signature[2] = {};
    // FIL contains FatFS's sector buffer. It was reserved immediately after
    // Mac RAM, before SD/display/audio allocations fragment the heap.
    FIL *file = reservedInstallFile;
    if (file == nullptr) {
        sdcardRelease();
        MACPLUS_LOG("INSTALL: FatFS file buffer allocation failed\n");
        return false;
    }
    memset(file, 0, sizeof(*file));
    const bool sizeValid = stat(INSTALL_IMAGE_PATH, &info) == 0 &&
        (info.st_size == INSTALL_400K_BYTES ||
         info.st_size == INSTALL_800K_BYTES);
    if (!sizeValid) {
        MACPLUS_LOG("INSTALL: missing or invalid image size=%ld errno=%d\n",
               static_cast<long>(info.st_size), errno);
    }
    FRESULT openResult = FR_NO_FILE;
    if (sizeValid) {
        openResult = f_open(file, "0:/macplus-install.img", FA_READ | FA_WRITE);
    }
    bool readOnly = false;
    if (openResult != FR_OK && sizeValid) {
        // A physically or FAT-level write-protected card can still expose the
        // software disk for reading.  Report that state to the Mac instead of
        // pretending a writable disk is available.
        memset(file, 0, sizeof(*file));
        openResult = f_open(file, "0:/macplus-install.img", FA_READ);
        readOnly = openResult == FR_OK;
    }
    UINT signatureRead = 0;
    const bool valid = openResult == FR_OK &&
        f_lseek(file, 1024U) == FR_OK &&
        f_read(file, signature, sizeof(signature), &signatureRead) == FR_OK &&
        signatureRead == sizeof(signature) &&
        ((signature[0] == 0x42 && signature[1] == 0x44) ||
         (signature[0] == 0xD2 && signature[1] == 0xD7));
    if (!valid) {
        if (openResult != FR_OK && sizeValid) {
            MACPLUS_LOG("INSTALL: cannot open %s (FatFS=%d)\n",
                   INSTALL_IMAGE_PATH, static_cast<int>(openResult));
        }
        if (openResult == FR_OK) f_close(file);
        memset(file, 0, sizeof(*file));
        sdcardRelease();
        return false;
    }
    installVolume.file = file;
    installVolume.bytes = static_cast<uint32_t>(info.st_size);
    installVolume.isMfs = signature[0] == 0xD2;
    installVolume.readOnly = readOnly;
    memset(&installReadCacheState, 0, sizeof(installReadCacheState));
    sdcardRelease();
    MACPLUS_LOG("INSTALL: SD volume ready for IWM (%luKB %s)\n",
           static_cast<unsigned long>(installVolume.bytes / 1024U),
           installVolume.isMfs ? "MFS" : "HFS");
    MACPLUS_LOG("INSTALL: media %s\n", installVolume.readOnly ?
           "read-only" : "read/write");
    return true;
}

bool hdPrepareInstallVolume(void) {
    return prepareInstallVolume();
}

uint32_t hdGetInstallVolumeBytes(void) {
    return installVolume.file != nullptr ? installVolume.bytes : 0;
}

int hdIsInstallVolumeMfs(void) {
    return installVolume.file != nullptr && installVolume.isMfs ? 1 : 0;
}

int hdIsInstallVolumeReadOnly(void) {
    return installVolume.file == nullptr || installVolume.readOnly ? 1 : 0;
}

int hdReadInstallSector(uint32_t sector, uint8_t *destination) {
    if (installVolume.file == nullptr || destination == nullptr ||
        sector >= installVolume.bytes / 512U) {
        return 0;
    }

    const uint32_t cacheSectors = INSTALL_READ_CACHE_BYTES / 512U;
    const bool cached = installReadCacheState.valid &&
        sector >= installReadCacheState.baseSector &&
        sector - installReadCacheState.baseSector <
            installReadCacheState.sectorCount;
    bool ok = true;
    if (!cached) {
        if (!sdcardAcquire(1000)) return 0;
        // The IWM primes a track from its first sector.  Start the window at
        // that request instead of aligning to an arbitrary 16-sector FAT
        // boundary: 400K tracks contain 12 sectors and 800K sides contain
        // 12 sectors, so alignment otherwise makes one track cross two SD
        // reads on nearly every seek.
        const uint32_t base = sector;
        const uint32_t totalSectors = installVolume.bytes / 512U;
        const uint32_t count = (totalSectors - base) < cacheSectors
                                   ? totalSectors - base : cacheSectors;
        int seekResult = -1;
        size_t bytesRead = 0;
        const size_t requestedBytes = static_cast<size_t>(count) * 512U;
        int streamError = 0;
        int errorNumber = 0;
        /* A card can return a short FatFS read after a long seek. Reopen is
           intentionally avoided here; the direct FIL state remains usable
           after a transient retry. */
        ok = false;
        for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
            const long fileOffset = static_cast<long>(base) * 512L;
            /* Keep every cache fill anchored by an absolute file offset. The
               same FIL is also used by the .Sony block path; relying on its
               previous cursor lets an interleaved request return a valid but
               unrelated sector window. */
            seekResult = f_lseek(installVolume.file,
                                 static_cast<FSIZE_t>(fileOffset)) == FR_OK
                ? 0 : -1;
            UINT transferred = 0;
            const FRESULT readResult = seekResult == 0
                ? f_read(installVolume.file, installReadCache,
                         static_cast<UINT>(requestedBytes), &transferred)
                : FR_DISK_ERR;
            bytesRead = transferred;
            streamError = readResult == FR_OK ? 0 : static_cast<int>(readResult);
            errorNumber = 0;
            ok = seekResult == 0 && readResult == FR_OK &&
                 bytesRead == requestedBytes;
        }
        if (ok) {
            installReadCacheState.baseSector = base;
            installReadCacheState.sectorCount = count;
            installReadCacheState.valid = true;
        } else {
            installReadCacheState.valid = false;
            installReadCacheState.sectorCount = 0;
            MACPLUS_LOG("INSTALL: SD read failed at sector %lu "
                   "seek=%d read=%lu/%lu ferror=%d errno=%d\n",
                   static_cast<unsigned long>(sector), seekResult,
                   static_cast<unsigned long>(bytesRead),
                   static_cast<unsigned long>(requestedBytes), streamError,
                   errorNumber);
        }
        sdcardRelease();
    }
    if (ok) {
        memcpy(destination, installReadCache +
                   (sector - installReadCacheState.baseSector) * 512U,
               512U);
    }
    return ok ? 1 : 0;
}

int hdWriteInstallSector(uint32_t sector, const uint8_t *source) {
    if (installVolume.file == nullptr || installVolume.readOnly ||
        source == nullptr || sector >= installVolume.bytes / 512U) {
        return 0;
    }
    if (!sdcardAcquire(1000)) return 0;
    const bool seekOk = f_lseek(installVolume.file,
                                static_cast<FSIZE_t>(sector) * 512U) == FR_OK;
    UINT written = 0;
    const bool ok = seekOk &&
                    f_write(installVolume.file, source, 512U, &written) == FR_OK &&
                    written == 512U;
    sdcardRelease();
    if (ok) {
        if (installReadCacheState.valid &&
            sector >= installReadCacheState.baseSector &&
            sector - installReadCacheState.baseSector <
                installReadCacheState.sectorCount) {
            memcpy(installReadCache +
                       (sector - installReadCacheState.baseSector) * 512U,
                   source, 512U);
        }
    }
    if (!ok) {
        MACPLUS_LOG("INSTALL: SD write failed at sector %lu\n",
               static_cast<unsigned long>(sector));
    }
    return ok ? 1 : 0;
}

int hdReadInstallBytes(uint32_t offset, uint8_t *destination, uint32_t bytes) {
    if (installVolume.file == nullptr || destination == nullptr ||
        (offset & 511U) != 0U ||
        (bytes & 511U) != 0U ||
        offset > installVolume.bytes || bytes > installVolume.bytes - offset) {
        return 0;
    }

    // Block-level .Sony reads are already sector aligned. Handle one bounded
    // multi-sector request at a time; the IWM bitstream path continues to use
    // the same cache for small random sector reads.
    if (bytes >= 2048U && bytes <= sizeof(installReadCache)) {
        if (!sdcardAcquire(2000)) return 0;
        const FSIZE_t fileOffset = static_cast<FSIZE_t>(offset);
        const bool seekOk = f_lseek(installVolume.file, fileOffset) == FR_OK;
        UINT transferred = 0;
        /* SDSPI/FatFS can use DMA for a large read. Read into the aligned
           cache buffer first, then copy to the emulated RAM, whose 24-bit
           address may be only 2-byte aligned or wrap at 256 KiB. */
        const bool ok = seekOk &&
            f_read(installVolume.file, installReadCache,
                   static_cast<UINT>(bytes),
                   &transferred) == FR_OK && transferred == bytes;
        if (ok) memcpy(destination, installReadCache, bytes);
        // The direct stream may overlap the existing cache; do not let the
        // next IWM request consume stale sectors from that window.
        installReadCacheState.valid = false;
        sdcardRelease();
        return ok ? 1 : 0;
    }

    uint32_t sector = offset / 512U;
    while (bytes != 0U) {
        if (installReadCacheState.valid &&
            sector >= installReadCacheState.baseSector &&
            sector < installReadCacheState.baseSector +
                         installReadCacheState.sectorCount) {
            const uint32_t available =
                installReadCacheState.baseSector +
                installReadCacheState.sectorCount - sector;
            const uint32_t count = available * 512U < bytes
                ? available * 512U : bytes;
            memcpy(destination,
                   installReadCache +
                       (sector - installReadCacheState.baseSector) * 512U,
                   count);
            sector += count / 512U;
            destination += count;
            bytes -= count;
        } else {
            if (!hdReadInstallSector(sector, destination)) return 0;
            ++sector;
            destination += 512U;
            bytes -= 512U;
        }
    }
    return 1;
}

int hdWriteInstallBytes(uint32_t offset, const uint8_t *source, uint32_t bytes) {
    if (installVolume.file == nullptr || installVolume.readOnly ||
        source == nullptr || (offset & 511U) != 0U ||
        (bytes & 511U) != 0U || offset > installVolume.bytes ||
        bytes > installVolume.bytes - offset) {
        return 0;
    }
    if (bytes == 0U) return 1;
    if (!sdcardAcquire(2000)) return 0;
    const bool seekOk = f_lseek(installVolume.file,
                                static_cast<FSIZE_t>(offset)) == FR_OK;
    UINT written = 0;
    const bool ok = seekOk && bytes <= UINT_MAX &&
                    f_write(installVolume.file, source,
                            static_cast<UINT>(bytes), &written) == FR_OK &&
                    written == bytes;
    sdcardRelease();
    if (!ok) {
        MACPLUS_LOG("INSTALL: SD write failed at offset %lu (%lu bytes)\n",
               static_cast<unsigned long>(offset),
               static_cast<unsigned long>(bytes));
        return 0;
    }

    const uint32_t firstSector = offset / 512U;
    const uint32_t sectors = bytes / 512U;
    if (installReadCacheState.valid) {
        const uint32_t cacheEnd = installReadCacheState.baseSector +
                                  installReadCacheState.sectorCount;
        const uint32_t writeEnd = firstSector + sectors;
        const uint32_t begin = firstSector > installReadCacheState.baseSector
            ? firstSector : installReadCacheState.baseSector;
        const uint32_t end = writeEnd < cacheEnd ? writeEnd : cacheEnd;
        if (begin < end) {
            memcpy(installReadCache +
                       (begin - installReadCacheState.baseSector) * 512U,
                   source + (begin - firstSector) * 512U,
                   (end - begin) * 512U);
        }
    }
    return 1;
}

int hdFlushInstallVolume(void) {
    if (installVolume.file == nullptr || installVolume.readOnly) return 0;
    if (!sdcardAcquire(2000)) return 0;
    const FRESULT result = f_sync(installVolume.file);
    sdcardRelease();
    if (result != FR_OK) {
        MACPLUS_LOG("INSTALL: SD flush failed (FatFS=%d)\n",
               static_cast<int>(result));
        return 0;
    }
    return 1;
}

struct HdCacheBlock {
    bool valid;
    bool dirty;
    uint32_t blockIndex;
    uint32_t lastUse;
    uint8_t data[HD_CACHE_BLOCK_BYTES];
};

typedef struct {
    bool primary;
    bool ready;
    bool usingSd;
    bool readOnly;
    // Reserved for an external writer that temporarily marks the image
    // busy.  Always false in this build (network features removed).
    volatile bool externalBusy;
    // SD card file mode
    FILE *fp;
    char path[32];
    long lastFileOffset; // cached fseek position; -1 = unknown
    const uint8_t *flashData; // raw-flash memory-mapped image (if active)
    int size;
    SemaphoreHandle_t mutex;
    HdCacheBlock cache[HD_CACHE_BLOCK_COUNT];
    uint32_t cacheClock;
    uint32_t dirtyBlocks;
    uint32_t lastFlushMs;
    volatile uint32_t lastIoError;
    // Preserve the first failed stdio operation. Later SCSI requests after
    // the image is disabled must not overwrite the useful root cause.
    volatile uint32_t firstFailureStage;
    volatile uint32_t firstFailureLba;
    volatile uint32_t firstFailureBytes;
    volatile uint32_t firstFailureTransferred;
    volatile int32_t firstFailureSeekResult;
    volatile int32_t firstFailureFlushResult;
    volatile int32_t firstFailureStreamError;
    volatile int32_t firstFailureErrno;
    volatile uint32_t lastFailureStage;
    volatile uint32_t lastFailureLba;
    volatile uint32_t lastFailureBytes;
    volatile uint32_t lastFailureTransferred;
    volatile int32_t lastFailureSeekResult;
    volatile int32_t lastFailureFlushResult;
    volatile int32_t lastFailureStreamError;
    volatile int32_t lastFailureErrno;
    volatile uint32_t offlineFailureStage;
    volatile uint32_t offlineFailureLba;
    volatile uint32_t offlineFailureBytes;
    volatile uint32_t offlineFailureTransferred;
    volatile int32_t offlineFailureSeekResult;
    volatile int32_t offlineFailureFlushResult;
    volatile int32_t offlineFailureStreamError;
    volatile int32_t offlineFailureErrno;
    volatile uint32_t offlineError;
    volatile uint8_t senseKey;
    volatile uint8_t senseAsc;
    volatile uint8_t senseAscq;
} HdPriv;

static HdPriv *activeHd = nullptr;
static TaskHandle_t flushTaskHandle = nullptr;
static HdPriv *reservedHd = nullptr;
static SCSIDevice *reservedDev = nullptr;

// Reserve the FatFS object immediately after Mac RAM, then reserve the HD
// objects after the SCSI/task block.  Keeping these phases separate avoids
// consuming the last usable contiguous block during board setup.
void hdReserveInstallFile(void) {
    if (reservedInstallFile == nullptr) {
        reservedInstallFile = static_cast<FIL *>(malloc(sizeof(FIL)));
        if (reservedInstallFile != nullptr) {
            memset(reservedInstallFile, 0, sizeof(*reservedInstallFile));
        }
    }
    MACPLUS_LOG("HD: reserved floppy %s\n",
           reservedInstallFile != nullptr ? "OK" : "FAIL");
}

void hdReserveStorage(void) {
    if (reservedHd == nullptr) {
        reservedHd = static_cast<HdPriv *>(malloc(sizeof(HdPriv)));
    }
    if (reservedDev == nullptr) {
        reservedDev = static_cast<SCSIDevice *>(malloc(sizeof(SCSIDevice)));
    }
    MACPLUS_LOG("HD: reserved storage %s%s%s\n",
           reservedHd != nullptr ? "hd" : "hd-FAIL",
           reservedDev != nullptr ? "+dev" : "+dev-FAIL",
           reservedInstallFile != nullptr ? "+floppy" : "+floppy-FAIL");
}

int hdIsDeviceReady(const SCSIDevice *device) {
    if (device == nullptr || device->arg == nullptr) return 0;
    const HdPriv *hd = static_cast<const HdPriv *>(device->arg);
    return hd->ready && !hd->externalBusy ? 1 : 0;
}

static bool lockHd(HdPriv *hd);
static void unlockHd(HdPriv *hd);
static bool flushCache(HdPriv *hd);
static bool readImageCached(HdPriv *hd, unsigned int lba, uint8_t *buffer,
                            size_t bytes);
static bool writeImageCached(HdPriv *hd, unsigned int lba,
                             const uint8_t *buffer, size_t bytes);

const uint8_t *hdGetRawFlashData(void) {
    return static_cast<const uint8_t *>(hdFlashMap);
}
uint32_t hdGetImageSize(void) { return activeHd ? activeHd->size : 0; }
int hdIsReady(void) { return activeHd && activeHd->ready && !activeHd->externalBusy ? 1 : 0; }
int hdIsUsingSd(void) { return activeHd && activeHd->usingSd ? 1 : 0; }
int hdIsReadOnly(void) { return activeHd && activeHd->readOnly ? 1 : 0; }
uint32_t hdGetLastIoError(void) { return activeHd ? activeHd->lastIoError : 0; }
uint32_t hdGetFirstFailureStage(void) {
    return activeHd ? activeHd->firstFailureStage : 0;
}
uint32_t hdGetFirstFailureLba(void) {
    return activeHd ? activeHd->firstFailureLba : 0;
}
uint32_t hdGetFirstFailureBytes(void) {
    return activeHd ? activeHd->firstFailureBytes : 0;
}
uint32_t hdGetFirstFailureTransferred(void) {
    return activeHd ? activeHd->firstFailureTransferred : 0;
}
int32_t hdGetFirstFailureSeekResult(void) {
    return activeHd ? activeHd->firstFailureSeekResult : 0;
}
int32_t hdGetFirstFailureFlushResult(void) {
    return activeHd ? activeHd->firstFailureFlushResult : 0;
}
int32_t hdGetFirstFailureStreamError(void) {
    return activeHd ? activeHd->firstFailureStreamError : 0;
}
int32_t hdGetFirstFailureErrno(void) {
    return activeHd ? activeHd->firstFailureErrno : 0;
}
uint32_t hdGetOfflineError(void) {
    return activeHd ? activeHd->offlineError : 0;
}
uint32_t hdGetOfflineFailureStage(void) {
    return activeHd ? activeHd->offlineFailureStage : 0;
}
uint32_t hdGetOfflineFailureLba(void) {
    return activeHd ? activeHd->offlineFailureLba : 0;
}
uint32_t hdGetOfflineFailureBytes(void) {
    return activeHd ? activeHd->offlineFailureBytes : 0;
}
uint32_t hdGetOfflineFailureTransferred(void) {
    return activeHd ? activeHd->offlineFailureTransferred : 0;
}
int32_t hdGetOfflineFailureSeekResult(void) {
    return activeHd ? activeHd->offlineFailureSeekResult : 0;
}
int32_t hdGetOfflineFailureFlushResult(void) {
    return activeHd ? activeHd->offlineFailureFlushResult : 0;
}
int32_t hdGetOfflineFailureStreamError(void) {
    return activeHd ? activeHd->offlineFailureStreamError : 0;
}
int32_t hdGetOfflineFailureErrno(void) {
    return activeHd ? activeHd->offlineFailureErrno : 0;
}

static void recordFirstFailure(HdPriv *hd, uint32_t stage, unsigned int lba,
                               size_t bytes, size_t transferred,
                               int seekResult, int flushResult,
                               int streamError, int errorNumber) {
    if (hd == nullptr) return;
    hd->lastFailureLba = lba;
    hd->lastFailureBytes = static_cast<uint32_t>(bytes);
    hd->lastFailureTransferred = static_cast<uint32_t>(transferred);
    hd->lastFailureSeekResult = seekResult;
    hd->lastFailureFlushResult = flushResult;
    hd->lastFailureStreamError = streamError;
    hd->lastFailureErrno = errorNumber;
    hd->lastFailureStage = stage;

    if (hd->firstFailureStage == 0) {
        hd->firstFailureLba = lba;
        hd->firstFailureBytes = static_cast<uint32_t>(bytes);
        hd->firstFailureTransferred = static_cast<uint32_t>(transferred);
        hd->firstFailureSeekResult = seekResult;
        hd->firstFailureFlushResult = flushResult;
        hd->firstFailureStreamError = streamError;
        hd->firstFailureErrno = errorNumber;
        // Publish the stage last so readers never treat a partial snapshot as
        // complete.
        hd->firstFailureStage = stage;
    }
}

static const uint8_t inq_resp[95]={
    0, //HD
    0, //0x80 if removable
    0x49, //Obsolete SCSI standard 1 all the way
    0, //response version etc
    31, //extra data
    0,0, //reserved
    0, //features
    'A','P','P','L','E',' ',' ',' ', //vendor id
    '2','0','S','C',' ',' ',' ',' ', //prod id
    '1','.','0',' ',' ',' ',' ',' ', //prod rev lvl
};

static uint32_t readBe32(const uint8_t *data) {
    return (static_cast<uint32_t>(data[0]) << 24) |
           (static_cast<uint32_t>(data[1]) << 16) |
           (static_cast<uint32_t>(data[2]) << 8) | data[3];
}

typedef bool (*MacImageReader)(void *context, uint32_t offset,
                               uint8_t *buffer, size_t bytes);

static bool macImageLayoutIsValid(MacImageReader reader, void *context,
                                  uint32_t imageBytes) {
    if (reader == nullptr || imageBytes < 1024U ||
        (imageBytes % 512U) != 0) {
        return false;
    }
    uint8_t block[512] = {};
    if (!reader(context, 0, block, sizeof(block)) || block[0] != 0x45 ||
        block[1] != 0x52 ||
        !reader(context, 512, block, sizeof(block)) || block[0] != 0x50 ||
        block[1] != 0x4D) {
        return false;
    }
    const uint32_t mapBlocks = readBe32(block + 4);
    if (mapBlocks == 0 || mapBlocks > 128U ||
        mapBlocks > imageBytes / 512U) {
        return false;
    }
    for (uint32_t index = 1; index <= mapBlocks; ++index) {
        const uint32_t offset = index * 512U;
        if (!reader(context, offset, block, sizeof(block)) ||
            block[0] != 0x50 || block[1] != 0x4D) {
            return false;
        }
        const bool isHfs = memcmp(block + 48, "Apple_HFS", 9) == 0;
        const bool isMfs = memcmp(block + 48, "Apple_MFS", 9) == 0;
        if (!isHfs && !isMfs) continue;
        const uint32_t partitionStart = readBe32(block + 8);
        const uint32_t partitionBlocks = readBe32(block + 12);
        const uint32_t dataStart = readBe32(block + 80);
        if (partitionBlocks < 2U || dataStart >= partitionBlocks ||
            partitionStart >= imageBytes / 512U ||
            partitionBlocks > imageBytes / 512U - partitionStart ||
            dataStart > partitionBlocks - 2U) {
            continue;
        }
        const uint64_t volumeHeader =
            (static_cast<uint64_t>(partitionStart) + dataStart) * 512ULL +
            1024ULL;
        if (volumeHeader + 2ULL > imageBytes ||
            !reader(context, static_cast<uint32_t>(volumeHeader), block, 2)) {
            continue;
        }
        const bool hfsSignature = block[0] == 0x42 && block[1] == 0x44;
        const bool mfsSignature = block[0] == 0xD2 && block[1] == 0xD7;
        if ((isHfs && hfsSignature) || (isMfs && mfsSignature)) return true;
    }
    return false;
}

static bool readFileBytes(void *context, uint32_t offset, uint8_t *buffer,
                          size_t bytes) {
    FILE *file = static_cast<FILE *>(context);
    return file != nullptr && fseek(file, static_cast<long>(offset), SEEK_SET) == 0 &&
           fread(buffer, 1, bytes, file) == bytes;
}

static bool readRawFlashBytes(void *, uint32_t offset, uint8_t *buffer,
                              size_t bytes) {
    return hdRawFlashStorageRead(offset, buffer, bytes);
}

static bool imageRangeIsValid(const HdPriv *hd, unsigned int lba, size_t bytes) {
    const uint64_t imageBytes = static_cast<uint64_t>(hd->size);
    const uint64_t offset = static_cast<uint64_t>(lba) * 512ULL;
    return offset <= imageBytes && bytes <= imageBytes - offset;
}

static bool macImageHeaderIsValid(FILE *file, long imageSize) {
    return file != nullptr && imageSize > 0 &&
           macImageLayoutIsValid(readFileBytes, file,
                                 static_cast<uint32_t>(imageSize));
}

bool hdRawFlashImageIsValid(uint32_t imageBytes) {
    return rawImageBytesIsValid(imageBytes) &&
           macImageLayoutIsValid(readRawFlashBytes, nullptr, imageBytes);
}

static void setSense(HdPriv *hd, uint8_t key, uint8_t asc, uint8_t ascq) {
    if (hd == nullptr) return;
    hd->senseKey = key;
    hd->senseAsc = asc;
    hd->senseAscq = ascq;
}

static int copyData(SCSITransferData *data, const uint8_t *source,
                    size_t sourceBytes, unsigned int allocationLength) {
    if (data->data == nullptr || data->dataCapacity == 0) return 0;
    size_t count = sourceBytes;
    if (count > allocationLength) count = allocationLength;
    if (count > data->dataCapacity) count = data->dataCapacity;
    memcpy(data->data, source, count);
    return static_cast<int>(count);
}

static void markStorageOffline(HdPriv *hd, uint32_t error) {
    if (hd == nullptr || hd->path[0] == '\0') return;

    hd->offlineFailureLba = hd->lastFailureLba;
    hd->offlineFailureBytes = hd->lastFailureBytes;
    hd->offlineFailureTransferred = hd->lastFailureTransferred;
    hd->offlineFailureSeekResult = hd->lastFailureSeekResult;
    hd->offlineFailureFlushResult = hd->lastFailureFlushResult;
    hd->offlineFailureStreamError = hd->lastFailureStreamError;
    hd->offlineFailureErrno = hd->lastFailureErrno;
    hd->offlineFailureStage = hd->lastFailureStage;

    if (hd->fp != nullptr) {
        fclose(hd->fp);
        hd->fp = nullptr;
    }
    hd->path[0] = '\0';
    hd->size = 0;
    hd->dirtyBlocks = 0;
    for (size_t index = 0; index < HD_CACHE_BLOCK_COUNT; ++index) {
        hd->cache[index].valid = false;
        hd->cache[index].dirty = false;
    }

    // Do not unmount/reinitialize FATFS from the emulator task. The active
    // FILE and VFS must be torn down in a controlled reset path; otherwise a
    // failed write leaves the SCSI layer pointing at a stale file handle.
    hd->ready = false;
    hd->usingSd = false;
    hd->offlineError = error;
    hd->lastIoError = error;
    MACPLUS_LOG("HD: %s image disabled after I/O failure, errno=%lu; reboot required\n",
           hd->primary ? "primary" : "shared", static_cast<unsigned long>(error));
}

static bool reopenSdImage(HdPriv *hd) {
    if (hd->fp != nullptr) {
        fclose(hd->fp);
        hd->fp = nullptr;
    }
    if (hd->path[0] == '\0') return false;

    errno = 0;
    hd->fp = fopen(hd->path, hd->readOnly ? "rb" : "r+b");
    if (hd->fp == nullptr) {
        hd->lastIoError = errno != 0 ? static_cast<uint32_t>(errno)
                                     : static_cast<uint32_t>(EIO);
        return false;
    }
    // Do not let newlib issue hidden multi-sector FatFS transfers. The cache
    // above provides the required coalescing without making card I/O opaque.
    setvbuf(hd->fp, nullptr, _IONBF, 0);
    hd->lastFileOffset = -1;
    return true;
}

// Copy /sd/hd.img into the raw flash cache (once) and map it for fast reads.
// Returns true when the mapped image is usable; the caller then switches the
// emulated disk to flash mode.
static bool prepareRawFlashHd(HdPriv *hd) {
    if (hd == nullptr) return false;
    if (!selectHdCacheStorage(0)) {
        MACPLUS_LOG("HD: data partition '%s' is missing\n",
               MACPLUS_DATA_PARTITION_LABEL);
        showHdStorageError("[FAIL] PARTITION MISSING",
                           "CREATE 'macplus' IN PMAN");
        return false;
    }
    MACPLUS_LOG("HD: partition '%s' at 0x%lX, %lu bytes (%lu image bytes)\n",
           hdCachePartition->label,
           static_cast<unsigned long>(hdCachePartition->address),
           static_cast<unsigned long>(hdCachePartition->size),
           static_cast<unsigned long>(hdCacheImageMax));

    RawFlashMetadata metadata = {};
    const bool haveMetadata = readRawFlashMetadata(&metadata);
    const uint32_t sdImageBytes = hd->size > 0
                                      ? static_cast<uint32_t>(hd->size)
                                      : 0;
    const bool preferPinned = haveMetadata &&
        metadata.magic == HD_RAW_FLASH_PINNED_MAGIC;
    uint32_t imageBytes = haveMetadata ? metadata.imageBytes : sdImageBytes;
    if (!preferPinned && sdImageBytes != 0) imageBytes = sdImageBytes;
    if (!rawImageBytesIsValid(imageBytes)) {
        if (sdImageBytes > hdCacheImageMax) {
            MACPLUS_LOG("HD: image needs %lu bytes, partition allows %lu; enlarge "
                   "'%s' in Launcher PMan\n",
                   static_cast<unsigned long>(sdImageBytes),
                   static_cast<unsigned long>(hdCacheImageMax),
                   MACPLUS_DATA_PARTITION_LABEL);
            showHdStorageError("[FAIL] PARTITION TOO SMALL",
                               "ENLARGE 'macplus' IN PMAN");
        } else {
            MACPLUS_LOG("HD: no usable Flash image and no valid /sd/hd.img\n");
            showHdStorageError("[FAIL] NO SYSTEM DISK",
                               "COPY /hd.img TO SD CARD");
        }
        return false;
    }
    hd->size = static_cast<int>(imageBytes);
    if (hdFlashMap != nullptr) {
        hd->flashData = static_cast<const uint8_t *>(hdFlashMap);
        return true;
    }

    uint32_t sdFingerprint = 0;
    bool haveSd = false;
    if (hd->fp != nullptr && !preferPinned) {
        haveSd = true;
        if (!sdcardAcquire(5000)) return false;
        uint8_t sector[512];
        if (fseek(hd->fp, 0, SEEK_SET) == 0 &&
            fread(sector, 1, sizeof(sector), hd->fp) == sizeof(sector)) {
            sdFingerprint = hdCrc32(sector, sizeof(sector), sdFingerprint);
            if (imageBytes > sizeof(sector) &&
                fseek(hd->fp, static_cast<long>(imageBytes) -
                                  static_cast<long>(sizeof(sector)), SEEK_SET) == 0 &&
                fread(sector, 1, sizeof(sector), hd->fp) == sizeof(sector)) {
                sdFingerprint = hdCrc32(sector, sizeof(sector), sdFingerprint);
            } else if (imageBytes > sizeof(sector)) {
                haveSd = false;
            }
            sdFingerprint ^= imageBytes;
        } else {
            haveSd = false;
        }
        sdcardRelease();
    }

    bool valid = haveMetadata && metadata.imageBytes == imageBytes &&
                 hdRawFlashImageIsValid(imageBytes);
    if (valid && rawFlashJournalHasPending()) {
        MACPLUS_LOG("HD: interrupted Flash write detected; restoring from SD\n");
        valid = false;
    }
    uint32_t flashFingerprint = 0;
    if (valid) {
        valid = rawFlashFingerprint(imageBytes, &flashFingerprint) &&
                metadata.fingerprint == flashFingerprint;
    }
    if (valid && haveSd && !preferPinned) {
        valid = metadata.fingerprint == sdFingerprint;
    }

    if (!valid) {
        if (hd->fp == nullptr) {
            hd->size = static_cast<int>(sdImageBytes);
            showHdStorageError("[FAIL] FLASH IMAGE DAMAGED",
                               "RESTORE /hd.img ON SD");
            return false;
        }
        MACPLUS_LOG("HD: copying SD image to partition at 0x%X (%u bytes)...\n",
               static_cast<unsigned int>(hdCacheBase),
               static_cast<unsigned int>(imageBytes));
        if (!sdcardAcquire(5000)) return false;
        uint8_t *copyChunk = hd->cache[0].data;
        const uint32_t eraseBytes = (imageBytes + 4095U) & ~4095U;
        dispShowProgress("HD CACHE INIT", "[RUN] PREPARING FLASH",
                         "DO NOT POWER OFF", 0, eraseBytes);
        // Erase in moderate chunks so the first-boot activity marker keeps
        // changing instead of appearing frozen during one long erase call.
        // Invalidate the old image before touching its data. A reset during
        // this copy must never leave stale metadata pointing at a partial image.
        bool ok = eraseHdMetadata();
        for (uint32_t erased = 0; ok && erased < eraseBytes; erased += 0x10000U) {
            const uint32_t left = eraseBytes - erased;
            const uint32_t chunk = left < 0x10000U ? left : 0x10000U;
            ok = eraseHdCache(erased, chunk);
            dispShowProgress("HD CACHE INIT", "[RUN] PREPARING FLASH",
                             "DO NOT POWER OFF", erased + chunk, eraseBytes);
        }
        if (ok) ok = fseek(hd->fp, 0, SEEK_SET) == 0;
        if (ok) dispShowHdCacheProgress(0, imageBytes);
        uint32_t lastProgressMs = millis();
        uint32_t copiedContentCrc = 0;
        for (size_t offset = 0; ok && offset < imageBytes;
             offset += HD_CACHE_BLOCK_BYTES) {
            const size_t want = imageBytes - offset;
            const size_t n = want < HD_CACHE_BLOCK_BYTES
                                 ? want : HD_CACHE_BLOCK_BYTES;
            if (fread(copyChunk, 1, n, hd->fp) != n ||
                !writeHdCache(static_cast<uint32_t>(offset), copyChunk, n)) {
                ok = false;
                break;
            }
            copiedContentCrc = hdCrc32(copyChunk, n, copiedContentCrc);
            const uint32_t now = millis();
            if (now - lastProgressMs >= 250U) {
                lastProgressMs = now;
                dispShowHdCacheProgress(static_cast<uint32_t>(offset + n),
                                        imageBytes);
            }
        }
        if (ok) {
            uint32_t copiedFingerprint = 0;
            ok = rawFlashFingerprint(imageBytes, &copiedFingerprint) &&
                 writeRawFlashMetadata(HD_RAW_FLASH_VALID_MAGIC, imageBytes,
                                       copiedFingerprint, copiedContentCrc);
        }
        sdcardRelease();
        if (!ok) {
            MACPLUS_LOG("HD: Flash copy failed; hard disk unavailable\n");
            const char *lines[] = {
                "HD CACHE",
                "[FAIL] COPY ERROR",
                "CHECK SD CARD AND REBOOT",
            };
            dispShowMessage(lines, 3);
            return false;
        }
        dispShowHdCacheProgress(imageBytes, imageBytes);
        MACPLUS_LOG("HD: Flash copy complete\n");
    }

    const size_t mapBytes = (static_cast<size_t>(hd->size) + 0xFFFFU) &
                            ~static_cast<size_t>(0xFFFFU);
    const esp_err_t mapError = esp_partition_mmap(
        hdCachePartition, MACPLUS_HD_DATA_OFFSET, mapBytes,
        SPI_FLASH_MMAP_DATA, &hdFlashMap, &hdFlashMapHandle);
    if (mapError != ESP_OK) {
        MACPLUS_LOG("HD: Flash cache mmap failed; hard disk unavailable\n");
        return false;
    }
    hd->flashData = static_cast<const uint8_t *>(hdFlashMap);
    hd->lastIoError = 0;
    MACPLUS_LOG("HD: raw flash image mapped at %p (%d bytes)\n",
           hd->flashData, hd->size);
    return true;
}

// Invalidate the raw-flash HD cache (removes the completion magic).  The
// next boot re-copies /sd/hd.img from the SD card, so an updated image takes
// effect without reflashing the firmware.
bool hdInvalidateRawCache(void) {
    if (!selectHdCacheStorage(0)) return false;
    return eraseHdMetadata();
}

bool hdRemoveInstallImage(void) {
    if (!sdcardMounted() || !sdcardAcquire(2000)) return false;

    if (installVolume.file != nullptr) {
        f_sync(installVolume.file);
        f_close(installVolume.file);
        memset(installVolume.file, 0, sizeof(*installVolume.file));
        installVolume.file = nullptr;
        installVolume.bytes = 0;
        installVolume.isMfs = false;
        installVolume.readOnly = false;
        memset(&installReadCacheState, 0, sizeof(installReadCacheState));
    }

    bool ok = true;
    const char *paths[] = {
        INSTALL_IMAGE_PATH,
        INSTALL_BACKUP_PATH,
        INSTALL_TEMP_PATH,
    };
    for (const char *path : paths) {
        errno = 0;
        if (remove(path) != 0 && errno != ENOENT) ok = false;
    }
    sdcardRelease();
    return ok;
}

static bool readSdImage(HdPriv *hd, unsigned int lba, uint8_t *buffer,
                        size_t bytes) {
    const long offset = static_cast<long>(static_cast<uint64_t>(lba) * 512ULL);
    memset(buffer, 0, bytes);

    if (!sdcardAcquire(5000)) {
        hd->lastIoError = EBUSY;
        recordFirstFailure(hd, 1, lba, bytes, 0, -1, 0, 0, EBUSY);
        MACPLUS_LOG("HD: read storage lock timeout LBA=%u bytes=%u\n", lba,
               static_cast<unsigned int>(bytes));
        return false;
    }

    for (int attempt = 1; attempt <= 3; ++attempt) {
        if (hd->fp == nullptr && !reopenSdImage(hd)) {
            recordFirstFailure(hd, 2, lba, bytes, 0, -1, 0, 0,
                               static_cast<int>(hd->lastIoError));
            MACPLUS_LOG("HD: read reopen %d/3 failed, errno=%lu\n", attempt,
                   static_cast<unsigned long>(hd->lastIoError));
            continue;
        }
        clearerr(hd->fp);
        errno = 0;
        int seekResult = 0;
        if (hd->lastFileOffset != offset) {
            seekResult = fseek(hd->fp, offset, SEEK_SET);
            if (seekResult == 0) hd->lastFileOffset = offset;
            else hd->lastFileOffset = -1;
        }
        int savedErrno = errno;
        size_t count = 0;
        if (seekResult == 0) {
            while (count < bytes) {
                const size_t want = (bytes - count) < HD_SD_IO_CHUNK_BYTES
                    ? bytes - count : HD_SD_IO_CHUNK_BYTES;
                errno = 0;
                const size_t transferred =
                    fread(buffer + count, 1, want, hd->fp);
                count += transferred;
                if (errno != 0) savedErrno = errno;
                if (transferred != want || ferror(hd->fp) != 0) break;
            }
        }
        const int streamError = ferror(hd->fp);

        if (seekResult == 0 && count == bytes && streamError == 0) {
            hd->lastIoError = 0;
            hd->lastFileOffset = offset + static_cast<long>(count);
            sdcardRelease();
            return true;
        }

        hd->lastIoError = savedErrno != 0 ? static_cast<uint32_t>(savedErrno)
                                          : static_cast<uint32_t>(EIO);
        recordFirstFailure(hd, 3, lba, bytes, count, seekResult, 0,
                           streamError, savedErrno);
        MACPLUS_LOG("HD: read retry %d/3 LBA=%u bytes=%u seek=%d read=%u ferror=%d errno=%d\n",
               attempt, lba, static_cast<unsigned int>(bytes), seekResult,
               static_cast<unsigned int>(count), streamError, savedErrno);

        // FatFS latches FIL.err after FR_DISK_ERR. Reopening is required;
        // clearerr() only resets the newlib FILE wrapper.
        reopenSdImage(hd);
    }

    markStorageOffline(hd, hd->lastIoError);
    memset(buffer, 0, bytes);
    sdcardRelease();
    return false;
}

static bool writeSdRange(HdPriv *hd, unsigned int lba, const uint8_t *buffer,
                         size_t bytes, bool flushStream) {
    const long offset = static_cast<long>(static_cast<uint64_t>(lba) * 512ULL);

    if (!sdcardAcquire(5000)) {
        hd->lastIoError = EBUSY;
        recordFirstFailure(hd, 4, lba, bytes, 0, -1, 0, 0, EBUSY);
        MACPLUS_LOG("HD: write storage lock timeout LBA=%u bytes=%u\n", lba,
               static_cast<unsigned int>(bytes));
        return false;
    }

    for (int attempt = 1; attempt <= 3; ++attempt) {
        if (hd->fp == nullptr && !reopenSdImage(hd)) {
            recordFirstFailure(hd, 5, lba, bytes, 0, -1, 0, 0,
                               static_cast<int>(hd->lastIoError));
            MACPLUS_LOG("HD: write reopen %d/3 failed, errno=%lu\n", attempt,
                   static_cast<unsigned long>(hd->lastIoError));
            continue;
        }
        clearerr(hd->fp);
        errno = 0;
        int seekResult = 0;
        if (hd->lastFileOffset != offset) {
            seekResult = fseek(hd->fp, offset, SEEK_SET);
            if (seekResult == 0) hd->lastFileOffset = offset;
            else hd->lastFileOffset = -1;
        }
        int savedErrno = errno;
        size_t count = 0;
        int flushResult = 0;
        if (seekResult == 0) {
            while (count < bytes) {
                const size_t want = (bytes - count) < HD_SD_IO_CHUNK_BYTES
                    ? bytes - count : HD_SD_IO_CHUNK_BYTES;
                errno = 0;
                const size_t transferred =
                    fwrite(buffer + count, 1, want, hd->fp);
                count += transferred;
                if (errno != 0) savedErrno = errno;
                if (transferred != want || ferror(hd->fp) != 0) break;
            }
            if (flushStream) flushResult = fflush(hd->fp);
            if (errno != 0) savedErrno = errno;
        }
        const int streamError = ferror(hd->fp);

        if (seekResult == 0 && count == bytes &&
            (!flushStream || flushResult == 0) && streamError == 0) {
            hd->lastIoError = 0;
            hd->lastFileOffset = offset + static_cast<long>(count);
            sdcardRelease();
            return true;
        }

        hd->lastIoError = savedErrno != 0 ? static_cast<uint32_t>(savedErrno)
                                          : static_cast<uint32_t>(EIO);
        recordFirstFailure(hd, 6, lba, bytes, count, seekResult, flushResult,
                           streamError, savedErrno);
        MACPLUS_LOG("HD: write retry %d/3 LBA=%u bytes=%u seek=%d wrote=%u flush=%d ferror=%d errno=%d\n",
               attempt, lba, static_cast<unsigned int>(bytes), seekResult,
               static_cast<unsigned int>(count), flushResult, streamError,
               savedErrno);

        reopenSdImage(hd);
    }

    markStorageOffline(hd, hd->lastIoError);
    sdcardRelease();
    return false;
}

static size_t cacheBlockBytes(const HdPriv *hd, uint32_t blockIndex) {
    const uint64_t offset = static_cast<uint64_t>(blockIndex) *
                            HD_CACHE_BLOCK_BYTES;
    if (offset >= static_cast<uint64_t>(hd->size)) return 0;
    const uint64_t remaining = static_cast<uint64_t>(hd->size) - offset;
    return remaining < HD_CACHE_BLOCK_BYTES
        ? static_cast<size_t>(remaining) : HD_CACHE_BLOCK_BYTES;
}

static int findCacheBlock(const HdPriv *hd, uint32_t blockIndex) {
    for (size_t index = 0; index < HD_CACHE_BLOCK_COUNT; ++index) {
        if (hd->cache[index].valid &&
            hd->cache[index].blockIndex == blockIndex) {
            return static_cast<int>(index);
        }
    }
    return -1;
}

static int chooseCacheBlock(const HdPriv *hd) {
    int selected = -1;
    for (size_t index = 0; index < HD_CACHE_BLOCK_COUNT; ++index) {
        if (!hd->cache[index].valid) return static_cast<int>(index);
        if (selected < 0 || hd->cache[index].lastUse <
                            hd->cache[selected].lastUse) {
            selected = static_cast<int>(index);
        }
    }
    return selected;
}

static bool rawFlashBlockMatches(uint32_t offset, const uint8_t *data) {
    uint8_t verify[512];
    for (size_t part = 0; part < HD_CACHE_BLOCK_BYTES; part += sizeof(verify)) {
        if (!hdRawFlashStorageRead(offset + static_cast<uint32_t>(part),
                                   verify, sizeof(verify)) ||
            memcmp(verify, data + part, sizeof(verify)) != 0) {
            return false;
        }
    }
    return true;
}

static bool flushFlashRawBlock(HdPriv *hd, HdCacheBlock *block) {
    if (hd->flashData == nullptr || block == nullptr || !block->valid ||
        !block->dirty) return false;

    const uint32_t offset = block->blockIndex * HD_CACHE_BLOCK_BYTES;
    if (!rawFlashJournalAppend(block->blockIndex,
                               HD_RAW_FLASH_JOURNAL_PENDING)) {
        hd->lastIoError = EIO;
        return false;
    }
    esp_err_t error = eraseHdCache(offset, 4096) ? ESP_OK : ESP_FAIL;
    if (error == ESP_OK) {
        error = writeHdCache(offset, block->data, HD_CACHE_BLOCK_BYTES)
            ? ESP_OK : ESP_FAIL;
    }
    if (error == ESP_OK &&
        !rawFlashBlockMatches(offset, block->data)) {
        error = ESP_FAIL;
    }
    hd->lastIoError = static_cast<uint32_t>(error);
    if (error != ESP_OK) return false;

    if (!rawFlashJournalAppend(block->blockIndex,
                               HD_RAW_FLASH_JOURNAL_COMMITTED)) {
        hd->lastIoError = EIO;
        return false;
    }

    // PINNED images deliberately skip the full-image CRC at boot.  Do not
    // rescan a multi-megabyte disk and erase the metadata sector after every
    // ordinary 4KB write.  The short fingerprint only changes when the first
    // or last sector changes.
    RawFlashMetadata metadata = {};
    const uint32_t imageBytes = static_cast<uint32_t>(hd->size);
    const bool touchesFingerprint = offset < 512U ||
        offset + HD_CACHE_BLOCK_BYTES > imageBytes - 512U;
    const bool metadataPinned = readRawFlashMetadata(&metadata) &&
        metadata.magic == HD_RAW_FLASH_PINNED_MAGIC &&
        metadata.imageBytes == imageBytes;
    if (!metadataPinned || touchesFingerprint) {
        uint32_t fingerprint = 0;
        if (!rawFlashFingerprint(imageBytes, &fingerprint) ||
            !writeRawFlashMetadata(HD_RAW_FLASH_PINNED_MAGIC, imageBytes,
                                   fingerprint, 0)) {
            hd->lastIoError = EIO;
            return false;
        }
    }

    block->dirty = false;
    if (hd->dirtyBlocks != 0) --hd->dirtyBlocks;
    return true;
}

static bool flushCache(HdPriv *hd) {
    if (hd == nullptr || hd->dirtyBlocks == 0) return true;

    bool ok = true;
    for (size_t index = 0; index < HD_CACHE_BLOCK_COUNT; ++index) {
        HdCacheBlock &block = hd->cache[index];
        if (!block.valid || !block.dirty) continue;

        if (!flushFlashRawBlock(hd, &block)) {
            ok = false;
            break;
        }
    }

    if (!ok) {
        // Avoid retrying a failed card transaction on every emulator frame.
        hd->lastFlushMs = millis();
        return false;
    }

    hd->lastFlushMs = millis();
    hd->lastIoError = 0;
    return true;
}

static void hdFlushTask(void *param) {
    HdPriv *hd = static_cast<HdPriv *>(param);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(20));
        if (hd != activeHd || hd->dirtyBlocks == 0 ||
            millis() - hd->lastFlushMs < HD_FLUSH_INTERVAL_MS) {
            continue;
        }

        // Keep SD latency off the 68000 task. The HD mutex still serializes
        // this with SCSI commands and explicit SYNCHRONIZE CACHE requests.
        if (lockHd(hd)) {
            flushCache(hd);
            unlockHd(hd);
        }
    }
}

static HdCacheBlock *loadCacheBlock(HdPriv *hd, uint32_t blockIndex) {
    int index = findCacheBlock(hd, blockIndex);
    if (index >= 0) {
        hd->cache[index].lastUse = ++hd->cacheClock;
        return &hd->cache[index];
    }

    index = chooseCacheBlock(hd);
    if (index < 0) return nullptr;
    HdCacheBlock &block = hd->cache[index];
    if (block.valid && block.dirty) {
        if (!flushCache(hd)) return nullptr;
    }

    const size_t bytes = cacheBlockBytes(hd, blockIndex);
    if (bytes == 0) {
        hd->lastIoError = EINVAL;
        return nullptr;
    }
    memset(block.data, 0, sizeof(block.data));

    bool ok = false;
    if (hd->flashData != nullptr) {
        memcpy(block.data,
               hd->flashData + blockIndex * HD_CACHE_BLOCK_BYTES, bytes);
        ok = true;
    }
    if (!ok) {
        block.valid = false;
        return nullptr;
    }

    block.valid = true;
    block.dirty = false;
    block.blockIndex = blockIndex;
    block.lastUse = ++hd->cacheClock;
    return &block;
}

static bool readImageCached(HdPriv *hd, unsigned int lba, uint8_t *buffer,
                            size_t bytes) {
    // Raw-flash mode: the whole disk image is memory-mapped, so reads are
    // just memcpy (instant).  Only fall back to the cache while dirty blocks
    // are pending so a just-written sector is still visible.
    if (hd->flashData != nullptr && hd->dirtyBlocks == 0 && bytes >= 512) {
        memcpy(buffer, hd->flashData + static_cast<size_t>(lba) * 512U, bytes);
        return true;
    }

    size_t remaining = bytes;
    uint32_t currentLba = lba;
    uint8_t *destination = buffer;
    while (remaining != 0) {
        const uint64_t byteOffset = static_cast<uint64_t>(currentLba) * 512U;
        const uint32_t blockIndex = byteOffset / HD_CACHE_BLOCK_BYTES;
        const size_t blockOffset = byteOffset % HD_CACHE_BLOCK_BYTES;
        const size_t count = min(remaining,
                                 HD_CACHE_BLOCK_BYTES - blockOffset);
        HdCacheBlock *block = loadCacheBlock(hd, blockIndex);
        if (block == nullptr) {
            memset(buffer, 0, bytes);
            return false;
        }
        memcpy(destination, block->data + blockOffset, count);
        destination += count;
        currentLba += count / 512U;
        remaining -= count;
    }
    return true;
}

static uint32_t exportedBlockCount(const HdPriv *hd) {
    return static_cast<uint32_t>(hd->size / 512U);
}

static bool exportedRangeIsValid(const HdPriv *hd, unsigned int lba,
                                 size_t bytes) {
    if (hd == nullptr || (bytes & 511U) != 0) return false;
    const uint32_t sectors = static_cast<uint32_t>(bytes / 512U);
    const uint32_t blocks = exportedBlockCount(hd);
    return lba <= blocks && sectors <= blocks - lba;
}

static bool readExportedImage(HdPriv *hd, unsigned int lba, uint8_t *buffer,
                              size_t bytes) {
    if (!exportedRangeIsValid(hd, lba, bytes)) return false;
    return readImageCached(hd, lba, buffer, bytes);
}

static bool writeExportedBaseImage(HdPriv *hd, unsigned int lba,
                                   const uint8_t *buffer, size_t bytes) {
    const uint32_t baseBlocks = static_cast<uint32_t>(hd->size / 512U);
    const uint32_t sectors = static_cast<uint32_t>(bytes / 512U);
    if ((bytes & 511U) != 0 || lba > baseBlocks ||
        sectors > baseBlocks - lba) {
        return false;
    }
    return writeImageCached(hd, lba, buffer, bytes);
}

bool hdReadSector(unsigned int lba, uint8_t *destination) {
    HdPriv *hd = activeHd;
    if (destination == nullptr || hd == nullptr || !hd->ready ||
        !imageRangeIsValid(hd, lba, 512U) || !lockHd(hd)) {
        return false;
    }
    const bool ok = readImageCached(hd, lba, destination, 512U);
    unlockHd(hd);
    return ok;
}

static bool writeImageCached(HdPriv *hd, unsigned int lba,
                             const uint8_t *buffer, size_t bytes) {
    size_t remaining = bytes;
    uint32_t currentLba = lba;
    const uint8_t *source = buffer;
    while (remaining != 0) {
        const uint64_t byteOffset = static_cast<uint64_t>(currentLba) * 512U;
        const uint32_t blockIndex = byteOffset / HD_CACHE_BLOCK_BYTES;
        const size_t blockOffset = byteOffset % HD_CACHE_BLOCK_BYTES;
        const size_t count = min(remaining,
                                 HD_CACHE_BLOCK_BYTES - blockOffset);
        HdCacheBlock *block = loadCacheBlock(hd, blockIndex);
        if (block == nullptr) return false;
        if (!block->dirty) ++hd->dirtyBlocks;
        memcpy(block->data + blockOffset, source, count);
        block->dirty = true;
        block->lastUse = ++hd->cacheClock;
        source += count;
        currentLba += count / 512U;
        remaining -= count;
    }
    return true;
}

static bool lockHd(HdPriv *hd) {
    return hd != nullptr &&
           (hd->mutex == nullptr ||
            xSemaphoreTake(hd->mutex, pdMS_TO_TICKS(2000)) == pdTRUE);
}

static void unlockHd(HdPriv *hd) {
    if (hd != nullptr && hd->mutex != nullptr) xSemaphoreGive(hd->mutex);
}


int hdFlushNow(void) {
    HdPriv *hd = activeHd;
    if (hd == nullptr || hd->mutex == nullptr) return hd == nullptr ? 1 : 0;
    bool ok = true;
    if (hd != nullptr && lockHd(hd)) {
        ok = flushCache(hd);
        unlockHd(hd);
    } else {
        ok = false;
    }
    return ok ? 1 : 0;
}

void hdFlushIfDue(void) {
    // New builds flush from a low-priority task. Keep this function as a
    // fallback for the unlikely case where task creation fails.
    if (flushTaskHandle != nullptr) return;

    HdPriv *hd = activeHd;
    if (hd == nullptr || !lockHd(hd)) return;
    if (hd->dirtyBlocks != 0 &&
        millis() - hd->lastFlushMs >= HD_FLUSH_INTERVAL_MS) {
        flushCache(hd);
    }
    unlockHd(hd);
}

static int hdScsiCmd(SCSITransferData *data, unsigned int cmd,
                     unsigned int len, unsigned int lba, void *arg) {
    int ret = 0;
    HdPriv *hd = (HdPriv *)arg;

    bool commandOk = true;

    switch (cmd) {
    case 0x00: // TEST UNIT READY
        if (!hd->ready || hd->externalBusy) {
            commandOk = false;
            setSense(hd, 0x02, 0x3A, 0x00); // NOT READY, medium not present
        }
        break;

    case 0x03: { // REQUEST SENSE (6)
        uint8_t sense[18] = {};
        sense[0] = 0x70; // fixed-format current error
        sense[2] = hd->senseKey;
        sense[7] = 10;
        sense[12] = hd->senseAsc;
        sense[13] = hd->senseAscq;
        ret = copyData(data, sense, sizeof(sense), len);
        // Sense is reported once, as expected by the classic SCSI driver.
        setSense(hd, 0, 0, 0);
        break;
    }

    case 0x08: // READ(6)
    case 0x28: { // READ(10)
        const size_t bytes = static_cast<size_t>(len) * 512U;
        const bool bufferOk = data->data != nullptr &&
                              bytes <= data->dataCapacity;
        if (!hd->ready || hd->externalBusy) {
            commandOk = false;
            setSense(hd, 0x02, 0x3A, 0x00); // medium not present
            if (data->data != nullptr && data->dataCapacity != 0) {
                memset(data->data, 0, data->dataCapacity);
            }
        } else if (!bufferOk || !exportedRangeIsValid(hd, lba, bytes)) {
            hd->lastIoError = EINVAL;
            commandOk = false;
            setSense(hd, 0x05, 0x21, 0x00); // ILLEGAL REQUEST, LBA out of range
            if (data->data != nullptr && data->dataCapacity != 0) {
                memset(data->data, 0, data->dataCapacity);
            }
            MACPLUS_LOG("HD: invalid read LBA=%u sectors=%u bytes=%u capacity=%u image=%d\n",
                   lba, len, static_cast<unsigned int>(bytes),
                   static_cast<unsigned int>(data->dataCapacity), hd->size);
        } else if (hd->flashData != nullptr && lockHd(hd)) {
            commandOk = readExportedImage(hd, lba, data->data, bytes);
            unlockHd(hd);
            if (!commandOk) {
                setSense(hd, 0x03, 0x11, 0x00);
                memset(data->data, 0, bytes);
            }
        } else {
            commandOk = false;
            setSense(hd, 0x02, 0x3A, 0x00);
        }
        ret = commandOk ? static_cast<int>(bytes) : 0;
        break;
    }

    case 0x0A: // WRITE(6)
    case 0x2A: { // WRITE(10)
        const size_t bytes = static_cast<size_t>(len) * 512U;
        const bool bufferOk = data->data != nullptr &&
                              bytes <= data->dataCapacity;
        if (!hd->ready) {
            commandOk = false;
            setSense(hd, 0x02, 0x3A, 0x00); // medium not present
        } else if (!bufferOk || !exportedRangeIsValid(hd, lba, bytes)) {
            hd->lastIoError = EINVAL;
            commandOk = false;
            setSense(hd, 0x05, 0x21, 0x00);
            MACPLUS_LOG("HD: invalid write LBA=%u sectors=%u bytes=%u capacity=%u image=%d\n",
                   lba, len, static_cast<unsigned int>(bytes),
                   static_cast<unsigned int>(data->dataCapacity), hd->size);
        } else if (hd->flashData != nullptr && lockHd(hd)) {
            commandOk = writeExportedBaseImage(hd, lba, data->data, bytes);
            unlockHd(hd);
            if (!commandOk) {
                setSense(hd, 0x03, 0x0C, 0x02);
            }
        } else {
            commandOk = false;
            setSense(hd, 0x02, 0x3A, 0x00);
        }
        if (commandOk) {
            ret = static_cast<int>(bytes);
        }
        break;
    }

    case 0x35: { // SYNCHRONIZE CACHE (10)
        const bool locked = lockHd(hd);
        if (!locked || !flushCache(hd)) {
            commandOk = false;
            setSense(hd, 0x03, 0x0C, 0x02);
        }
        if (locked) unlockHd(hd);
        break;
    }

    case 0x12: { // INQUIRY
        MACPLUS_LOG("HD: Inquiry\n");
        ret = copyData(data, inq_resp, sizeof(inq_resp), len);
        break;
    }

    case 0x1A: { // MODE SENSE(6), minimal direct-access response
        const uint8_t mode[] = {3, 0, 0, 0};
        ret = copyData(data, mode, sizeof(mode), len);
        break;
    }

    case 0x1B: // START STOP UNIT
    case 0x1E: // PREVENT/ALLOW MEDIUM REMOVAL
    case 0x01: // REZERO UNIT
    case 0x0B: // SEEK(6)
        break;

    case 0x25: { // READ CAPACITY(10)
        if (data->data == nullptr || data->dataCapacity < 8 ||
            !hd->ready || hd->externalBusy || hd->size < 512) {
            commandOk = false;
            setSense(hd, 0x02, 0x3A, 0x00);
            break;
        }
        const uint32_t lastLba = exportedBlockCount(hd) - 1U;
        data->data[0] = static_cast<uint8_t>(lastLba >> 24);
        data->data[1] = static_cast<uint8_t>(lastLba >> 16);
        data->data[2] = static_cast<uint8_t>(lastLba >> 8);
        data->data[3] = static_cast<uint8_t>(lastLba);
        data->data[4] = 0;
        data->data[5] = 0;
        data->data[6] = 2; // 512-byte sectors
        data->data[7] = 0;
        ret = 8;
        MACPLUS_LOG("HD: Read capacity (%lu sectors)\n",
               static_cast<unsigned long>(lastLba + 1));
        break;
    }

    default:
        commandOk = false;
        setSense(hd, 0x05, 0x20, 0x00); // ILLEGAL REQUEST, invalid command
        MACPLUS_LOG("********** hdScsiCmd: unrecognized command %x\n", cmd);
        break;
    }

    data->cmd[0] = commandOk ? 0 : 2; // SCSI GOOD or CHECK CONDITION
    data->msg[0] = 0;
    return ret;
}

SCSIDevice *hdCreate(void) {
    SCSIDevice *ret = reservedDev;
    HdPriv *hd = reservedHd;
    reservedDev = nullptr;
    reservedHd = nullptr;
    if (ret == nullptr) ret = (SCSIDevice*)malloc(sizeof(SCSIDevice));
    if (hd == nullptr) hd = (HdPriv*)malloc(sizeof(HdPriv));
    if (ret == nullptr || hd == nullptr) {
        MACPLUS_LOG("HD: allocation failed (free heap=%d, largest=%d)\n",
               (int)esp_get_free_heap_size(),
               (int)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        free(ret);
        free(hd);
        return nullptr;
    }
    memset(ret, 0, sizeof(SCSIDevice));
    memset(hd, 0, sizeof(HdPriv));
    hd->lastFileOffset = -1;
    hd->mutex = xSemaphoreCreateMutex();
    hd->primary = activeHd == nullptr;
    hd->lastFlushMs = millis();

    // The protected Flash image is the normal runtime backend.  Check it
    // before opening /sd/hd.img so the emulator task does not enter the
    // deepest FatFS/stat/fopen path during startup.  SD remains the fallback
    // used to create or repair the cache.
    bool rawFlashReady = false;
    RawFlashMetadata cachedMetadata = {};
    if (readRawFlashMetadata(&cachedMetadata) &&
        rawImageBytesIsValid(cachedMetadata.imageBytes) &&
        !rawFlashJournalHasPending() &&
        hdRawFlashImageIsValid(cachedMetadata.imageBytes)) {
        hd->size = static_cast<int>(cachedMetadata.imageBytes);
        rawFlashReady = prepareRawFlashHd(hd);
    }

    // Try SD card only when the Flash cache is absent, invalid, or damaged.
    if (!rawFlashReady && sdcardMounted()) {
        if (sdcardAcquire(5000)) {
            const char *names[] = { "/sd/hd.img", "/sd/hd.hd",
                                    "/sd/hd.dsk", NULL };
            for (int i = 0; names[i] && !hd->fp; i++) {
                struct stat fileInfo = {};
                const bool statOk = stat(names[i], &fileInfo) == 0;
                const bool sizeOk = statOk && fileInfo.st_size > 0 &&
                                    fileInfo.st_size <= INT_MAX &&
                                    (fileInfo.st_size % 512) == 0;
                if (!sizeOk) {
                    hd->lastIoError = statOk ? EINVAL :
                        (errno != 0 ? static_cast<uint32_t>(errno) : ENOENT);
                    MACPLUS_LOG("HD: cannot stat %s or invalid size=%ld errno=%lu\n",
                           names[i], statOk ? static_cast<long>(fileInfo.st_size) : 0L,
                           static_cast<unsigned long>(hd->lastIoError));
                    continue;
                }
                errno = 0;
                hd->fp = fopen(names[i], "r+b");
                if (hd->fp) {
                    setvbuf(hd->fp, nullptr, _IONBF, 0);
                    if (!macImageHeaderIsValid(hd->fp, static_cast<long>(fileInfo.st_size))) {
                        MACPLUS_LOG("HD: refusing %s; Apple Driver Map/HFS header is invalid\n",
                               names[i]);
                        fclose(hd->fp);
                        hd->fp = nullptr;
                        hd->lastIoError = EINVAL;
                        continue;
                    }
                    hd->size = static_cast<int>(fileInfo.st_size);
                    strlcpy(hd->path, names[i], sizeof(hd->path));
                    hd->usingSd = true;
                    hd->ready = true;
                    MACPLUS_LOG("HD: Using SD card %s (%d bytes, read/write; stat)\n",
                           names[i], hd->size);
                } else {
                    const int openError = errno != 0 ? errno : EIO;
                    hd->lastIoError = static_cast<uint32_t>(openError);
                    errno = 0;
                    FILE *readOnly = fopen(names[i], "rb");
                    const bool readOnlyAvailable = readOnly != nullptr;
                    if (readOnly != nullptr) {
                        setvbuf(readOnly, nullptr, _IONBF, 0);
                        if (!macImageHeaderIsValid(readOnly, static_cast<long>(fileInfo.st_size))) {
                            fclose(readOnly);
                            readOnly = nullptr;
                            hd->lastIoError = EINVAL;
                        }
                    }
                    if (readOnly != nullptr) {
                        hd->fp = readOnly;
                        hd->size = static_cast<int>(fileInfo.st_size);
                        strlcpy(hd->path, names[i], sizeof(hd->path));
                        hd->usingSd = true;
                        hd->readOnly = true;
                        hd->ready = true;
                        MACPLUS_LOG("HD: Using SD card %s (%d bytes, read-only; r+b errno=%d; stat)\n",
                               names[i], hd->size, openError);
                    }
                    if (!hd->fp) {
                        MACPLUS_LOG("HD: cannot open %s r+b, errno=%d; rb=%s\n",
                               names[i], openError,
                               readOnlyAvailable ? "ok" : "failed");
                    }
                }
            }
            sdcardRelease();
        } else {
            MACPLUS_LOG("HD: SD storage lock timeout while opening image\n");
        }
    }

    // Prefer the protected Flash cache: copy /sd/hd.img there on first boot,
    // then serve all reads from the memory-mapped copy. SD is never used for
    // runtime random reads.
    if (!rawFlashReady) rawFlashReady = prepareRawFlashHd(hd);
    if (rawFlashReady) {
        if (hd->fp != nullptr) {
            fclose(hd->fp);
            hd->fp = nullptr;
        }
        hd->path[0] = '\0';
        hd->usingSd = false;
        hd->readOnly = false;
        hd->ready = true;
        MACPLUS_LOG("HD: Using raw flash image at 0x%X (%d bytes, read/write)\n",
               static_cast<unsigned int>(hdCacheBase), hd->size);
    } else if (hd->fp != nullptr) {
        fclose(hd->fp);
        hd->fp = nullptr;
        hd->path[0] = '\0';
        hd->usingSd = false;
        hd->ready = false;
        hd->size = 0;
        MACPLUS_LOG("HD: Flash cache unavailable; SD random I/O disabled\n");
    }

    if (!hd->fp && hd->flashData == nullptr) {
        hd->ready = false;
        hd->size = 0;
        hd->lastIoError = EIO;
        MACPLUS_LOG("HD: hard disk unavailable; only the named '%s' partition is allowed\n",
               MACPLUS_DATA_PARTITION_LABEL);
    }

    if (hd->ready && prepareInstallVolume()) {
        // Do the first 8 KiB cache fill before the 68K reset.  Without this
        // warm-up the first IWM revolution performs an SD seek/read directly
        // from the emulation core and produces a visible frame-time spike.
        if (!hdReadInstallSector(0, installWarmupSector)) {
            MACPLUS_LOG("INSTALL: initial cache warm-up failed\n");
        } else {
            MACPLUS_LOG("INSTALL: initial track cache warm\n");
        }
    }

    if (activeHd == nullptr) {
        activeHd = hd;
    }

    if (hd->primary && hd->mutex == nullptr) {
        MACPLUS_LOG("HD: mutex unavailable; async flush disabled\n");
    } else if (hd->primary && flushTaskHandle == nullptr) {
        // Keep the flash write-back task small enough for the no-PSRAM heap;
        // xTaskCreatePinnedToCore's depth is measured in words.
        const BaseType_t flushTaskResult = xTaskCreatePinnedToCore(
            hdFlushTask, "hd-flush", HD_FLUSH_TASK_STACK_WORDS, hd, 1,
            &flushTaskHandle, 1);
        if (flushTaskResult != pdPASS) {
            flushTaskHandle = nullptr;
            MACPLUS_LOG("HD: async flush task unavailable; using emulator fallback\n");
        } else {
            MACPLUS_LOG("HD: async flush task enabled, interval=%lums, cache=%uKiB\n",
                   static_cast<unsigned long>(HD_FLUSH_INTERVAL_MS),
                   static_cast<unsigned int>(HD_CACHE_BLOCK_COUNT *
                                             HD_CACHE_BLOCK_BYTES / 1024));
        }
    }

    ret->arg=hd;
    ret->scsiCmd=hdScsiCmd;
    return ret;
}
