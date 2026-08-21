#include "debug_log.h"
/*
 * Paravirtual Macintosh .Sony driver backend.
 *
 * The ROM stub is derived from UMAC, whose block-level Sony backend is based
 * on Basilisk II's GPL-2.0-or-later driver.  It serves normal 400K/800K raw
 * Macintosh images as 512-byte blocks, avoiding raw IWM write timing on a
 * microcontroller without PSRAM.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "m68k.h"
#include "tmeconfig.h"
#include "hd.h"
#include "softfloppy.h"

extern unsigned char *macRam;

enum {
    kNoErr = 0,
    kControlErr = -17,
    kStatusErr = -18,
    kParamErr = -50,
    kNoSuchDriveErr = -56,
    kOfflineErr = -65,
    kWriteProtectedErr = -44,

    kSonyRefNum = -5,
    kDriveNumber = 1,

    kIoTrap = 6,
    kIoVRefNum = 22,
    kIoBuffer = 32,
    kIoReqCount = 36,
    kIoActCount = 40,
    kCsCode = 26,
    kCsParam = 28,

    kDCtlPosition = 16,

    kDsTrack = 0,
    kDsWriteProtected = 2,
    kDsDiskInPlace = 3,
    kDsInstalled = 4,
    kDsSides = 5,
    kDsQType = 10,
    kDsQDrive = 12,
    kDsQRefNum = 14,
    kDsTwoSideFormat = 18,
    kDsMfmDrive = 22,
    kDsMfmDisk = 23,

    kReadCommand = 2,
};

typedef struct {
    uint32_t bytes;
    uint32_t status;
    uint8_t available;
    uint8_t readOnly;
    uint8_t mountPending;
    uint16_t mountDelayFrames;
} SoftFloppy;

static SoftFloppy floppy;

static uint32_t ramOffset(uint32_t address) {
    return address & (TME_RAMSIZE - 1U);
}

static uint8_t ramRead8(uint32_t address) {
    return macRam[ramOffset(address)];
}

static uint16_t ramRead16(uint32_t address) {
    return ((uint16_t)ramRead8(address) << 8U) | ramRead8(address + 1U);
}

static uint32_t ramRead32(uint32_t address) {
    return ((uint32_t)ramRead8(address) << 24U) |
           ((uint32_t)ramRead8(address + 1U) << 16U) |
           ((uint32_t)ramRead8(address + 2U) << 8U) |
           ramRead8(address + 3U);
}

static void ramWrite8(uint32_t address, uint8_t value) {
    macRam[ramOffset(address)] = value;
}

static void ramWrite16(uint32_t address, uint16_t value) {
    ramWrite8(address, (uint8_t)(value >> 8U));
    ramWrite8(address + 1U, (uint8_t)value);
}

static void ramWrite32(uint32_t address, uint32_t value) {
    ramWrite8(address, (uint8_t)(value >> 24U));
    ramWrite8(address + 1U, (uint8_t)(value >> 16U));
    ramWrite8(address + 2U, (uint8_t)(value >> 8U));
    ramWrite8(address + 3U, (uint8_t)value);
}

static void ramCopyFrom(uint8_t *destination, uint32_t source, uint32_t bytes) {
    while (bytes != 0U) {
        const uint32_t offset = ramOffset(source);
        uint32_t count = TME_RAMSIZE - offset;
        if (count > bytes) count = bytes;
        memcpy(destination, macRam + offset, count);
        destination += count;
        source += count;
        bytes -= count;
    }
}

static void ramCopyTo(uint32_t destination, const uint8_t *source, uint32_t bytes) {
    while (bytes != 0U) {
        const uint32_t offset = ramOffset(destination);
        uint32_t count = TME_RAMSIZE - offset;
        if (count > bytes) count = bytes;
        memcpy(macRam + offset, source, count);
        destination += count;
        source += count;
        bytes -= count;
    }
}

static int16_t setDiskError(int16_t error) {
    ramWrite16(0x142U, (uint16_t)error);
    return error;
}

static int driveMatches(uint32_t parameterBlock) {
    return (int16_t)ramRead16(parameterBlock + kIoVRefNum) == kDriveNumber;
}

static void setDriveStatus(void) {
    if (floppy.status == 0U) return;
    ramWrite16(floppy.status + kDsTrack, 0);
    ramWrite8(floppy.status + kDsWriteProtected, floppy.readOnly ? 0xFFU : 0U);
    ramWrite8(floppy.status + kDsDiskInPlace, floppy.available ? 1U : 0U);
    ramWrite8(floppy.status + kDsInstalled, 1U);
    ramWrite8(floppy.status + kDsSides,
              floppy.bytes == 800U * 1024U ? 0xFFU : 0U);
    ramWrite16(floppy.status + kDsQType, 0U);
    ramWrite16(floppy.status + kDsQDrive, kDriveNumber);
    ramWrite16(floppy.status + kDsQRefNum, (uint16_t)kSonyRefNum);
    ramWrite8(floppy.status + kDsTwoSideFormat,
              floppy.bytes == 800U * 1024U ? 0xFFU : 0U);
    ramWrite8(floppy.status + kDsMfmDrive, 0U);
    ramWrite8(floppy.status + kDsMfmDisk, 0U);
}

static int16_t softOpen(uint32_t dce, uint32_t status) {
    floppy.status = status;
    setDriveStatus();
    ramWrite32(dce + kDCtlPosition, 0U);
    /* The ROM .Sony client expects a non-null SonyVars pointer after Open. */
    ramWrite32(0x134U, 0xDEADBEEFU);
    /* Device-driver version 3 prevents later systems from replacing .Sony. */
    ramWrite16(dce + 6U, (uint16_t)((ramRead16(dce + 6U) & 0xFF00U) | 3U));
    MACPLUS_LOG("SONY: open status=%06lX drive=%d media=%s\n",
           (unsigned long)status, kDriveNumber,
           floppy.available ? "present" : "absent");
    return setDiskError(kNoErr);
}

static int16_t softPrime(uint32_t parameterBlock, uint32_t dce) {
    ramWrite32(parameterBlock + kIoActCount, 0U);
    if (!driveMatches(parameterBlock)) return setDiskError(kNoSuchDriveErr);
    if (!floppy.available) return setDiskError(kOfflineErr);

    /* Match the Sony/Basilisk driver: a successful request marks the disk as
       accessed before the actual transfer starts. */
    ramWrite8(floppy.status + kDsDiskInPlace, 2U);

    const uint32_t buffer = ramRead32(parameterBlock + kIoBuffer) & 0x00FFFFFFU;
    const uint32_t bytes = ramRead32(parameterBlock + kIoReqCount);
    const uint32_t offset = ramRead32(dce + kDCtlPosition);
    MACPLUS_LOG("SONY: prime req trap=%02X pos=%lu bytes=%lu buffer=%06lX\n",
           (unsigned)(ramRead16(parameterBlock + kIoTrap) & 0xFFU),
           (unsigned long)offset, (unsigned long)bytes,
           (unsigned long)buffer);
    if ((offset & 511U) != 0U || (bytes & 511U) != 0U ||
        offset > floppy.bytes || bytes > floppy.bytes - offset) {
        MACPLUS_LOG("SONY: prime reject drive=%d pos=%lu bytes=%lu buffer=%06lX\n",
               (int16_t)ramRead16(parameterBlock + kIoVRefNum),
               (unsigned long)offset, (unsigned long)bytes,
               (unsigned long)buffer);
        return setDiskError(kParamErr);
    }

    if ((ramRead16(parameterBlock + kIoTrap) & 0xFFU) == kReadCommand) {
        uint8_t sector[512];
        uint32_t position = offset;
        uint32_t remaining = bytes;
        uint32_t destination = buffer;
        while (remaining != 0U) {
            if (!hdReadInstallBytes(position, sector, sizeof(sector))) {
                MACPLUS_LOG("SONY: prime read failed pos=%lu sector=%lu\n",
                       (unsigned long)position,
                       (unsigned long)(position / 512U));
                return setDiskError(kOfflineErr);
            }
            ramCopyTo(destination, sector, sizeof(sector));
            position += sizeof(sector);
            destination += sizeof(sector);
            remaining -= sizeof(sector);
        }
    } else {
        if (floppy.readOnly) return setDiskError(kWriteProtectedErr);
        uint8_t sector[512];
        uint32_t position = offset;
        uint32_t remaining = bytes;
        uint32_t source = buffer;
        while (remaining != 0U) {
            ramCopyFrom(sector, source, sizeof(sector));
            if (!hdWriteInstallBytes(position, sector, sizeof(sector))) {
                return setDiskError(kOfflineErr);
            }
            position += sizeof(sector);
            source += sizeof(sector);
            remaining -= sizeof(sector);
        }
        if (!hdFlushInstallVolume()) return setDiskError(kOfflineErr);
    }

    ramWrite32(parameterBlock + kIoActCount, bytes);
    ramWrite32(dce + kDCtlPosition, offset + bytes);
    return setDiskError(kNoErr);
}

static int16_t softControl(uint32_t parameterBlock) {
    const uint16_t code = ramRead16(parameterBlock + kCsCode);
    MACPLUS_LOG("SONY: control request code=%u drive=%d media=%u status=%06lX\n",
           (unsigned)code,
           (int16_t)ramRead16(parameterBlock + kIoVRefNum),
           (unsigned)floppy.available,
           (unsigned long)floppy.status);

    /* General driver controls from the mature Basilisk implementation. */
    if (code == 1U) return setDiskError(-1); /* KillIO is unsupported. */
    if (code == 9U) return setDiskError(kNoErr); /* Ignore track cache. */
    if (code == 65U) {
        /* Basilisk mounts removable media by calling PostEvent(diskEvent)
           from the driver's periodic accRun callback. Return a one-shot
           positive signal; the 68K ROM stub performs that trap in the normal
           Mac execution context and disables further periodic callbacks. */
        if (!floppy.mountPending) return setDiskError(kNoErr);
        /* Do not advertise an uploaded software disk during the ROM boot
           scan.  A non-bootable tools disk would otherwise win over the
           system hard disk and leave the Mac on the crossed-disk screen.
           softFloppyFrameTick() releases the media after ten real emulated
           seconds, once Finder has finished its boot-time drive scan. */
        if (floppy.mountDelayFrames != 0U) return setDiskError(kNoErr);
        floppy.available = 1U;
        setDriveStatus();
        floppy.mountPending = 0U;
        return setDiskError(1);
    }
    if (!driveMatches(parameterBlock)) return setDiskError(kNoSuchDriveErr);

    switch (code) {
    case 5: /* verify */
        return setDiskError(floppy.available ? kNoErr : kOfflineErr);
    case 6: /* format */
        /* Uploaded media is already formatted.  A no-op is the behavior used
           by Basilisk for a writable mounted volume; never erase the SD file. */
        return setDiskError(floppy.readOnly ? kWriteProtectedErr : kNoErr);
    case 7: /* eject */
        /* There is no physical eject mechanism: the uploaded image remains
           mounted on SD until the next upload/reboot.  Treat the request as
           idempotent so System's boot-time cleanup cannot turn the drive
           offline and make the following Prime fail with -65. */
        setDriveStatus();
        return setDiskError(kNoErr);
    case 8: /* tag buffer, unused by raw 400K/800K images */
    case 23: /* drive information */
        return setDiskError(kNoErr);
    default:
        return setDiskError(kControlErr);
    }
}

static int16_t softStatus(uint32_t parameterBlock) {
    const uint16_t code = ramRead16(parameterBlock + kCsCode);
    if (!driveMatches(parameterBlock)) return setDiskError(kNoSuchDriveErr);

    switch (code) {
    case 6: { /* supported disk formats */
        const uint16_t count = ramRead16(parameterBlock + kCsParam);
        if (count == 0U) return setDiskError(kParamErr);
        const uint32_t result = ramRead32(parameterBlock + kCsParam + 2U);
        ramWrite16(parameterBlock + kCsParam, 1U);
        ramWrite32(result, floppy.bytes / 512U);
        /* Basilisk II's Sony driver advertises DD, 2 heads, 18 sectors and
           80 tracks as 0xD2120050.  The old 0xC2 value makes System reject an
           otherwise valid 800K HFS image and ask to initialize it. */
        ramWrite32(result + 4U, floppy.bytes == 800U * 1024U ?
                   0xD2120050U : 0xC1110050U);
        return setDiskError(kNoErr);
    }
    case 8: /* drive status */
        for (uint32_t index = 0; index < 22U; ++index) {
            ramWrite8(parameterBlock + kCsParam + index,
                      ramRead8(floppy.status + index));
        }
        return setDiskError(kNoErr);
    case 10: /* disk type / MFM information */
        ramWrite32(parameterBlock + kCsParam, 0x000000FEU);
        return setDiskError(kNoErr);
    case 0x5343U: /* address header format */
        ramWrite8(parameterBlock + kCsParam, 0x02U);
        return setDiskError(kNoErr);
    default:
        return setDiskError(kStatusErr);
    }
}

void softFloppyInit(void) {
    memset(&floppy, 0, sizeof(floppy));
    floppy.bytes = hdGetInstallVolumeBytes();
    const uint8_t validImage =
        floppy.bytes == 400U * 1024U || floppy.bytes == 800U * 1024U;
    /* Keep an uploaded software disk hidden during the ROM boot scan.  The
       patched .Sony accRun path enables it and posts diskEvent after the
       system disk has booted, so a non-bootable tools disk cannot hijack boot. */
    floppy.available = 0U;
    floppy.readOnly = hdIsInstallVolumeReadOnly() ? 1U : 0U;
    floppy.mountPending = validImage;
    floppy.mountDelayFrames = validImage ? 600U : 0U;
    if (validImage) {
        MACPLUS_LOG("INSTALL: .Sony block driver ready (%luKB %s)\n",
               (unsigned long)(floppy.bytes / 1024U),
               floppy.readOnly ? "read-only" : "read/write");
    }
}

void softFloppyFrameTick(void) {
    if (floppy.mountPending && floppy.mountDelayFrames != 0U) {
        --floppy.mountDelayFrames;
    }
    if (floppy.mountPending && floppy.mountDelayFrames == 0U &&
        !floppy.available) {
        /* Keep this state transition independent of accRun frequency. Some
           System versions call accRun only once during boot; updating the
           status record here still lets Finder notice the inserted volume. */
        floppy.available = 1U;
        setDriveStatus();
        MACPLUS_LOG("INSTALL: software disk visible after boot delay\n");
    }
}

int softFloppyPvHook(uint8_t opcode) {
    const uint32_t a0 = m68k_get_reg(NULL, M68K_REG_A0) & 0x00FFFFFFU;
    const uint32_t a1 = m68k_get_reg(NULL, M68K_REG_A1) & 0x00FFFFFFU;
    const uint32_t a2 = m68k_get_reg(NULL, M68K_REG_A2) & 0x00FFFFFFU;
    int16_t result = kControlErr;

    if (macRam == NULL) return 0;
    switch (opcode) {
    case 0: result = softOpen(a1, a2); break;
    case 1: result = softPrime(a0, a1); break;
    case 2: result = softControl(a0); break;
    case 3: result = softStatus(a0); break;
    default: break;
    }
    m68k_set_reg(M68K_REG_D0, (uint32_t)(int32_t)result);
    return 1;
}
