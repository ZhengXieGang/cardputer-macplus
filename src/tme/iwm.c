/*
 * Minimal Macintosh Plus IWM state used by the system ROM.
 *
 * Uploaded MFS/HFS volumes are read one sector at a time from SD.
 */
#include <stdint.h>
#include <string.h>

#include "iwm.h"
#include "hd.h"

#define IWM_CA0     (1 << 0)
#define IWM_CA1     (1 << 1)
#define IWM_CA2     (1 << 2)
#define IWM_LSTRB   (1 << 3)
#define IWM_ENABLE  (1 << 4)
#define IWM_SELECT  (1 << 5)
#define IWM_Q6      (1 << 6)
#define IWM_Q7      (1 << 7)

#define INSTALL_800K_BYTES (1600U * 512U)

#define TRACKS 80U
#define SYNC_WORD_BITS 10U
#define ADDRESS_SYNC_WORDS_OUTER 40U
#define ADDRESS_SYNC_WORDS_ZONE1 38U
#define DATA_SYNC_PREFIX_BITS 8U
#define DATA_SYNC_WORDS 4U
#define DATA_SYNC_BITS (DATA_SYNC_PREFIX_BITS + DATA_SYNC_WORDS * SYNC_WORD_BITS)
#define ADDRESS_MARK_BITS 24U
#define ADDRESS_FIELD_BITS 40U
#define ADDRESS_SUFFIX_BITS 24U
#define DATA_MARK_BITS 24U
#define DATA_SECTOR_BITS 8U
#define DATA_GCR_BYTES 703U
#define DATA_SUFFIX_BITS 24U

static const uint8_t kGcr[64] = {
    0x96, 0x97, 0x9A, 0x9B, 0x9D, 0x9E, 0x9F, 0xA6,
    0xA7, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB2, 0xB3,
    0xB4, 0xB5, 0xB6, 0xB7, 0xB9, 0xBA, 0xBB, 0xBC,
    0xBD, 0xBE, 0xBF, 0xCB, 0xCD, 0xCE, 0xCF, 0xD3,
    0xD6, 0xD7, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE,
    0xDF, 0xE5, 0xE6, 0xE7, 0xE9, 0xEA, 0xEB, 0xEC,
    0xED, 0xEE, 0xEF, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6,
    0xF7, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF,
};

static const uint8_t kSectorsPerZone[5] = {12, 11, 10, 9, 8};
static const uint8_t kSectorOrder[5][12] = {
    {0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5, 11},
    {0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5, 0},
    {0, 5, 1, 6, 2, 7, 3, 8, 4, 9, 0, 0},
    {0, 5, 1, 6, 2, 7, 3, 8, 4, 0, 0, 0},
    {0, 4, 1, 5, 2, 6, 3, 7, 0, 0, 0, 0},
};
// PCE's standard 500kbit/s Macintosh GCR track lengths.
static const uint32_t kTrackBits[5] = {74640, 68240, 62200, 55980, 49760};
static const uint8_t kAddressMark[3] = {0xD5, 0xAA, 0x96};
static const uint8_t kDataMark[3] = {0xD5, 0xAA, 0xAD};
// Macintosh GCR sectors terminate both the address and data fields with the
// three-byte bit-slip sequence DE AA FF.
static const uint8_t kBitSlip[3] = {0xDE, 0xAA, 0xFF};

static int iwmLines;
static int iwmModeReg;
static int iwmHeadSel;
static IwmSectorReader floppyReader;
static uint32_t floppyBytes;
static uint32_t cachedSourceSector;
static uint8_t sourceSector[512];
static uint8_t sourceSectorValid;
static uint32_t trackPosition;
static uint32_t cycleAccumulator;
static uint8_t readBitCount;
static uint8_t dataReg;
static uint8_t shiftReg;
static uint8_t activeTrack;
static uint8_t activeSide;
static uint8_t activeSector;
static uint8_t driveTrack;
static uint8_t motorOn;
static uint8_t diskInserted;
static uint8_t diskSwitched;
// The Sony driver samples the step-complete sense line as a pulse.  Keep the
// one-shot pending state instead of reporting "complete" permanently; a
// permanently asserted line makes System's formatter stop after track 0.
static uint8_t stepPending;
static uint8_t stepDown;
static uint8_t encodedSector[DATA_GCR_BYTES];
static int encodedValid;
// The IWM writes a raw GCR bitstream. Capture only one encoded sector while
// the stream parser walks the address/data fields, avoiding a 9 KiB track
// buffer that would starve the SD/FAT task on the no-PSRAM board.
static uint8_t writeTrackTrack;
static uint8_t writeTrackSide;
static uint8_t writeShift;
static uint8_t writeBitsRemaining;
static uint8_t writeBuffer;
static uint8_t writeBufferValid;
static uint8_t writing;
static uint32_t writeWindow;
static uint8_t writeParserByte;
static uint8_t writeParserBits;
static uint8_t writeParserState;
static uint8_t writeAddress[5];
static uint8_t writeAddressCount;
static uint8_t writeDataSector;
static uint16_t writeGcrCount;
static uint8_t writeGcr[DATA_GCR_BYTES];
// Keep the GCR decoder workspace out of the emulation task stack.  The write
// path is serialized by the single emulation task, so these buffers can be
// shared safely and avoid a second exception while the panic handler tries to
// print the original fault.
static uint8_t writeDecodedGcr[DATA_GCR_BYTES];
static uint8_t writeCooked[527];
static uint8_t writeDecoded[524];
static uint8_t writePayload[512];
enum {
    WRITE_SCAN = 0,
    WRITE_ADDRESS,
    WRITE_SKIP,
    WRITE_DATA_MARK,
    WRITE_DATA_SECTOR,
    WRITE_DATA_GCR,
};

static void finishWriting(void);
static void feedWriteBit(uint8_t bit);

static uint8_t zoneForTrack(uint8_t track) {
    return track < TRACKS ? track / 16U : 4U;
}

static uint32_t addressSyncBits(uint8_t zone) {
    const uint32_t words = zone == 1U ? ADDRESS_SYNC_WORDS_ZONE1
                                      : ADDRESS_SYNC_WORDS_OUTER;
    return words * SYNC_WORD_BITS;
}

static uint32_t sectorBits(uint8_t zone) {
    return addressSyncBits(zone) + ADDRESS_MARK_BITS +
           ADDRESS_FIELD_BITS + ADDRESS_SUFFIX_BITS + DATA_SYNC_BITS +
           DATA_MARK_BITS + DATA_SECTOR_BITS + DATA_GCR_BYTES * 8U +
           DATA_SUFFIX_BITS;
}

static uint8_t internalDriveSelected(void) {
    // Mac Plus SELECT low addresses the internal drive; SELECT high addresses
    // the optional external drive, which this single-drive emulator lacks.
    return (iwmLines & IWM_SELECT) == 0;
}

static uint8_t sideCount(void) {
    return floppyBytes == INSTALL_800K_BYTES ? 2U : 1U;
}

static uint8_t selectedSide(void) {
    return iwmHeadSel && sideCount() == 2U ? 1U : 0U;
}

static uint32_t sectorsBeforeTrack(uint8_t track) {
    uint32_t sectors = 0;
    for (uint8_t current = 0; current < track; ++current) {
        sectors += kSectorsPerZone[zoneForTrack(current)] * sideCount();
    }
    return sectors;
}

static uint32_t sectorOffset(uint8_t track, uint8_t side, uint8_t sector) {
    const uint8_t zone = zoneForTrack(track);
    return sectorsBeforeTrack(track) +
           (uint32_t)side * kSectorsPerZone[zone] + sector;
}

// Prime the SD/FatFS read-ahead window when a track becomes current. PCE's
// mature IWM backend materializes a complete track before the bit engine runs;
// on this no-PSRAM target the existing bounded HD read cache is the equivalent
// representation. Subsequent bit-loop accesses are then memory-only.
static int primeTrackCache(uint8_t track, uint8_t side) {
    if (floppyReader == NULL || track >= TRACKS || side >= sideCount()) {
        return 0;
    }
    const uint8_t sectors = kSectorsPerZone[zoneForTrack(track)];
    uint8_t scratch[512];
    for (uint8_t sector = 0; sector < sectors; ++sector) {
        if (!floppyReader(sectorOffset(track, side, sector), scratch)) {
            sourceSectorValid = 0;
            return 0;
        }
    }
    sourceSectorValid = 0;
    return 1;
}

static uint8_t sourceByte(uint8_t track, uint8_t side, uint8_t sector,
                          uint16_t index) {
    if (index < 12U) return 0;
    const uint32_t source = sectorOffset(track, side, sector);
    if (!sourceSectorValid || cachedSourceSector != source) {
        if (floppyReader == NULL || !floppyReader(source, sourceSector)) {
            sourceSectorValid = 0;
            diskInserted = 0;
            motorOn = 0;
            return 0;
        }
        cachedSourceSector = source;
        sourceSectorValid = 1;
    }
    return sourceSector[index - 12U];
}

static void encodeSector(uint8_t track, uint8_t side, uint8_t sector) {
    uint8_t cooked[527];
    uint32_t checksum[3] = {};
    for (uint16_t index = 0; index < 524U; ++index) {
        if ((index % 3U) == 0) {
            checksum[2] = ((checksum[2] << 1U) & 0x1FEU) |
                          ((checksum[2] >> 7U) & 1U);
        }
        const uint32_t value = sourceByte(track, side, sector, index);
        checksum[0] += value + ((checksum[2] >> 8U) & 1U);
        checksum[2] &= 0xFFU;
        cooked[index] = (uint8_t)(value ^ checksum[2]);

        const uint32_t next = checksum[0];
        checksum[0] = checksum[1];
        checksum[1] = checksum[2];
        checksum[2] = next;
    }
    checksum[2] &= 0xFFU;
    cooked[524] = (uint8_t)checksum[1];
    cooked[525] = (uint8_t)checksum[2];
    cooked[526] = (uint8_t)checksum[0];

    uint32_t output = 0;
    for (uint16_t index = 0; index < 522U; index += 3U) {
        const uint8_t high = (uint8_t)(((cooked[index] & 0xC0U) >> 2U) |
            ((cooked[index + 1U] & 0xC0U) >> 4U) |
            ((cooked[index + 2U] & 0xC0U) >> 6U));
        encodedSector[output++] = kGcr[high];
        encodedSector[output++] = kGcr[cooked[index] & 0x3FU];
        encodedSector[output++] = kGcr[cooked[index + 1U] & 0x3FU];
        encodedSector[output++] = kGcr[cooked[index + 2U] & 0x3FU];
    }

    uint8_t high = (uint8_t)(((cooked[522] & 0xC0U) >> 2U) |
                             ((cooked[523] & 0xC0U) >> 4U));
    encodedSector[output++] = kGcr[high];
    encodedSector[output++] = kGcr[cooked[522] & 0x3FU];
    encodedSector[output++] = kGcr[cooked[523] & 0x3FU];

    high = (uint8_t)(((cooked[524] & 0xC0U) >> 2U) |
                     ((cooked[525] & 0xC0U) >> 4U) |
                     ((cooked[526] & 0xC0U) >> 6U));
    encodedSector[output++] = kGcr[high];
    encodedSector[output++] = kGcr[cooked[524] & 0x3FU];
    encodedSector[output++] = kGcr[cooked[525] & 0x3FU];
    encodedSector[output++] = kGcr[cooked[526] & 0x3FU];
    encodedValid = output == DATA_GCR_BYTES;
    activeTrack = track;
    activeSide = side;
    activeSector = sector;
}

static uint8_t byteBit(uint8_t value, uint32_t position) {
    return (value >> (7U - (position & 7U))) & 1U;
}

static uint8_t bytesBit(const uint8_t *values, uint32_t position) {
    return byteBit(values[position >> 3U], position);
}

static uint8_t syncBit(uint32_t position) {
    return (position % SYNC_WORD_BITS) >= 2U;
}

static uint8_t dataSyncBit(uint32_t position) {
    if (position < DATA_SYNC_PREFIX_BITS) return 1;
    return syncBit(position - DATA_SYNC_PREFIX_BITS);
}

static uint8_t trackEndBit(uint32_t position, uint32_t remaining) {
    // Match PCE's end-of-track filler. It writes the final short 0xFF run
    // first, then continues with ten-bit 0xFF sync words.
    while (remaining != 0U) {
        uint32_t run = remaining % SYNC_WORD_BITS;
        if (run == 0U) run = SYNC_WORD_BITS;
        if (position < run) {
            const uint32_t leadingZeroes = run > 8U ? run - 8U : 0U;
            return position >= leadingZeroes;
        }
        position -= run;
        remaining -= run;
    }
    return 1;
}

static uint8_t trackBit(uint32_t position) {
    const uint8_t zone = zoneForTrack(driveTrack);
    const uint8_t sectors = kSectorsPerZone[zone];
    const uint32_t oneSectorBits = sectorBits(zone);
    const uint32_t record = position / oneSectorBits;
    uint32_t offset = position % oneSectorBits;
    if (record >= sectors) {
        const uint32_t used = (uint32_t)sectors * oneSectorBits;
        return trackEndBit(position - used, kTrackBits[zone] - used);
    }

    const uint8_t side = selectedSide();
    const uint8_t sector = kSectorOrder[zone][record];
    const uint32_t addressBits = addressSyncBits(zone);
    if (offset < addressBits) return syncBit(offset);
    offset -= addressBits;

    if (offset < ADDRESS_MARK_BITS) return bytesBit(kAddressMark, offset);
    offset -= ADDRESS_MARK_BITS;

    if (offset < ADDRESS_FIELD_BITS) {
        const uint8_t format = floppyBytes == INSTALL_800K_BYTES ? 0x22 : 0x02;
        const uint8_t headAndTrackHigh =
            (uint8_t)((side << 5) | ((driveTrack >> 6) & 0x1FU));
        const uint8_t values[5] = {
            driveTrack & 0x3F,
            sector & 0x1F,
            headAndTrackHigh,
            format,
            0,
        };
        const uint8_t checksum =
            (uint8_t)((driveTrack ^ headAndTrackHigh ^ sector ^ format) & 0x3F);
        const uint32_t field = offset >> 3U;
        return byteBit(kGcr[(field == 4U ? checksum : values[field]) & 0x3F],
                       offset);
    }
    offset -= ADDRESS_FIELD_BITS;

    if (offset < ADDRESS_SUFFIX_BITS) return bytesBit(kBitSlip, offset);
    offset -= ADDRESS_SUFFIX_BITS;

    if (offset < DATA_SYNC_BITS) return dataSyncBit(offset);
    offset -= DATA_SYNC_BITS;

    if (offset < DATA_MARK_BITS) return bytesBit(kDataMark, offset);
    offset -= DATA_MARK_BITS;

    if (offset < DATA_SECTOR_BITS) return byteBit(kGcr[sector & 0x1F], offset);
    offset -= DATA_SECTOR_BITS;

    if (offset < DATA_GCR_BYTES * 8U) {
        if (!encodedValid || activeTrack != driveTrack || activeSide != side ||
            activeSector != sector) {
            encodeSector(driveTrack, side, sector);
        }
        return bytesBit(encodedSector, offset);
    }
    offset -= DATA_GCR_BYTES * 8U;
    return bytesBit(kBitSlip, offset);
}

static void shiftTrackBit(uint8_t value) {
    shiftReg = (uint8_t)((shiftReg << 1U) | (value & 1U));
    if ((shiftReg & 0x80U) != 0) {
        dataReg = shiftReg;
        shiftReg = 0;
    }
}

static void writeDriveRegister(void) {
    const uint8_t reg = (uint8_t)(iwmLines & (IWM_CA0 | IWM_CA1)) |
                        (iwmHeadSel ? 4U : 0U);
    const uint8_t value = (iwmLines & IWM_CA2) != 0;

    // PCE applies the control register to the currently selected drive and
    // does not discard it at this point based on SELECT.  This firmware has a
    // single emulated drive, so doing the same is important: the Sony driver
    // can change SELECT while programming the control latch, and rejecting
    // that access leaves the head at track 0 even though the data stream is
    // otherwise valid.
    switch (reg) {
    case 0:
        stepDown = !value;
        break;
    case 1:
        if (!value) {
            if (stepDown) {
                if (driveTrack + 1U < TRACKS) ++driveTrack;
            } else if (driveTrack != 0) {
                --driveTrack;
            }
            stepPending = 1;
            trackPosition = 0;
            encodedValid = 0;
            cycleAccumulator = 0;
            shiftReg = 0;
            if (diskInserted) {
                primeTrackCache(driveTrack, selectedSide());
            }
        }
        break;
    case 2:
        if (motorOn != (uint8_t)(!value && diskInserted)) {
            motorOn = !value && diskInserted;
            cycleAccumulator = 0;
            dataReg = 0;
            shiftReg = 0;
        }
        break;
    case 3:
        if (value && diskInserted) {
            diskInserted = 0;
            motorOn = 0;
            diskSwitched = 1;
            // Finder may have dirtied the system disk while the removable
            // volume was being used.  Commit that cache before acknowledging
            // eject so an immediate power-off cannot lose the boot volume.
            (void)hdFlushNow();
        }
        break;
    case 4:
        if (value) diskSwitched = 0;
        break;
    default:
        break;
    }
}

static uint8_t driveSense(void) {
    const uint8_t reg = (uint8_t)(iwmLines & (IWM_CA0 | IWM_CA1 | IWM_CA2)) |
                        (iwmHeadSel ? 8U : 0U);
    const uint8_t zone = zoneForTrack(driveTrack);
    const uint32_t trackBits = kTrackBits[zone];
    uint8_t result = 0;

    if (!internalDriveSelected()) {
        switch (reg) {
        case 2: result = 1; break;  /* motor off */
        case 7: result = 0; break;  /* drive installed */
        case 8: result = 1; break;  /* no disk inserted */
        case 9: result = 1; break;  /* not write-protected */
        case 11: result = 1; break; /* no tachometer pulses */
        case 13: result = 1; break; /* not a SuperDrive */
        case 15: result = 1; break; /* IWM interface */
        default: result = 0; break;
        }
    } else {
        switch (reg) {
        case 0: result = stepDown == 0; break; /* step direction */
        case 1: {
            const uint8_t pending = stepPending;
            stepPending = 0;
            // Match PCE's active-low step sense: low for the first poll after
            // a step, then high once the drive is settled.
            result = pending == 0;
            break;
        }
        case 2: result = motorOn == 0; break;
        case 3: result = diskSwitched; break;
        case 4: result = 0; break;          /* head 0 exists */
        case 5: result = 0; break;          /* not a SuperDrive */
        case 6: result = sideCount() == 2U; break;
        case 7: result = 0; break;          /* drive installed */
        case 8: result = diskInserted == 0; break; /* CISTN */
        case 9: result = hdIsInstallVolumeReadOnly() ? 0 : 1; break;
        case 10: result = driveTrack != 0; break;
        case 11:
            result = !motorOn || ((trackPosition * 120U / trackBits) & 1U) == 0;
            break;
        case 12: result = 0; break;         /* head 1 exists */
        case 13: result = 1; break;         /* not a SuperDrive */
        case 14: result = 0; break;         /* drive ready */
        case 15: result = 1; break;         /* IWM interface */
        default: result = 1; break;
        }
    }

    return result;
}

void iwmInit(void) {
    iwmLines = 0;
    iwmModeReg = 0;
    iwmHeadSel = 0;
    floppyReader = NULL;
    floppyBytes = 0;
    cachedSourceSector = 0;
    sourceSectorValid = 0;
    trackPosition = 0;
    cycleAccumulator = 0;
    readBitCount = 0;
    dataReg = 0;
    shiftReg = 0;
    driveTrack = 0;
    stepPending = 0;
    motorOn = 0;
    diskInserted = 0;
    // `disk-switched` is a controller latch, not the disk-inserted signal.
    // The Sony driver polls it during initialization and expects a newly
    // inserted image to start cleared, as in the mature PCE IWM backend.
    // Disk presence is reported separately through sense register 8.
    diskSwitched = 0;
    stepDown = 0;
    encodedValid = 0;
    writeShift = 0;
    writeBitsRemaining = 0;
    writeBuffer = 0;
    writeBufferValid = 0;
    writing = 0;
    writeWindow = 0;
    writeParserByte = 0;
    writeParserBits = 0;
    writeParserState = WRITE_SCAN;
    writeAddressCount = 0;
    writeDataSector = 0;
    writeGcrCount = 0;
}

void iwmSetDiskReader(IwmSectorReader reader, uint32_t bytes, int inserted) {
    if (reader == NULL || (bytes != 400U * 1024U && bytes != 800U * 1024U)) {
        floppyReader = NULL;
        floppyBytes = 0;
        diskInserted = 0;
        motorOn = 0;
        diskSwitched = 0;
        sourceSectorValid = 0;
        return;
    }
    floppyReader = reader;
    floppyBytes = bytes;
    diskInserted = 0;
    motorOn = 0;
    diskSwitched = 0;
    driveTrack = 0;
    stepPending = 0;
    trackPosition = 0;
    cycleAccumulator = 0;
    readBitCount = 0;
    dataReg = 0;
    shiftReg = 0;
    encodedValid = 0;
    sourceSectorValid = 0;
    writeBufferValid = 0;
    writing = 0;
    writeWindow = 0;
    writeParserByte = 0;
    writeParserBits = 0;
    writeParserState = WRITE_SCAN;
    writeAddressCount = 0;
    writeGcrCount = 0;
    if (inserted) {
        if (!primeTrackCache(driveTrack, selectedSide())) {
            floppyReader = NULL;
            floppyBytes = 0;
            sourceSectorValid = 0;
            return;
        }
        diskInserted = 1;
        // Keep the disk-switched latch clear on insertion.  Setting it here
        // makes the ROM wait on sense register 3 and never reach its normal
        // head-step sequence.
        diskSwitched = 0;
    }
}

void iwmAccess(unsigned int addr) {
    const int bit = 1 << (addr >> 1);
    if (addr & 1U) iwmLines |= bit;
    else iwmLines &= ~bit;
    if ((iwmLines & IWM_Q7) == 0 && writing) finishWriting();
    if (iwmLines & IWM_LSTRB) {
        writeDriveRegister();
    }
}

void iwmWrite(unsigned int addr, unsigned int val) {
    iwmAccess(addr);
    const int reg = iwmLines & (IWM_Q7 | IWM_Q6);
    if (reg == (IWM_Q7 | IWM_Q6)) {
        if (iwmLines & IWM_ENABLE) {
            const uint8_t starting = !writing;
            if (starting) {
                writeTrackTrack = driveTrack;
                writeTrackSide = selectedSide();
                writeWindow = 0;
                writeParserByte = 0;
                writeParserBits = 0;
                writeParserState = WRITE_SCAN;
                writeAddressCount = 0;
                writeGcrCount = 0;
            }
            writing = 1;
            writeBuffer = (uint8_t)val;
            writeBufferValid = 1;
        } else {
            iwmModeReg = val;
        }
    }
}

void iwmSetHeadSel(int side) {
    const uint8_t next = side ? 1U : 0U;
    if (iwmHeadSel == next) return;
    iwmHeadSel = next;
    // Match the mature backend's select-head operation: selecting a side
    // rebinds the current track and starts its bit position from the current
    // index.  The old code only changed the address bit, leaving stale track
    // data/position after a side switch during formatting.
    trackPosition = 0;
    cycleAccumulator = 0;
    readBitCount = 0;
    dataReg = 0;
    shiftReg = 0;
    sourceSectorValid = 0;
    encodedValid = 0;
    if (diskInserted) primeTrackCache(driveTrack, selectedSide());
}

unsigned int iwmRead(unsigned int addr) {
    iwmAccess(addr);
    const int reg = iwmLines & (IWM_Q7 | IWM_Q6);
    unsigned int value;
    if (reg == 0) {
        if (!(iwmLines & IWM_ENABLE) || !internalDriveSelected()) {
            value = 0xFF;
        } else {
            value = dataReg;
            dataReg = 0;
        }
    } else if (reg == IWM_Q6) {
        value = iwmModeReg & 0x1F;
        if (iwmLines & IWM_ENABLE) value |= 0x20;
        if (driveSense()) value |= 0x80;
    } else if (reg == IWM_Q7) {
        value = writeBufferValid ? 0x7F : 0xFF;
    } else {
        value = (iwmModeReg & 0x1F) |
                ((iwmLines & IWM_ENABLE) ? 0x20 : 0);
    }
    return value;
}

void iwmTick(unsigned int cycles) {
    if (!internalDriveSelected() || !diskInserted || !motorOn ||
        floppyReader == NULL) return;
    // 500kbit/s at the Macintosh Plus 7.8336MHz CPU clock (500000/7833600 = 625/9792).
    cycleAccumulator += cycles * 625U;
    const uint8_t zone = zoneForTrack(driveTrack);
    const uint32_t trackBits = kTrackBits[zone];
    while (cycleAccumulator >= 9792U) {
        cycleAccumulator -= 9792U;
        if (writing) {
            if (writeBitsRemaining == 0U && writeBufferValid) {
                writeShift = writeBuffer;
                writeBitsRemaining = 8U;
                writeBufferValid = 0;
            }
            if (writeBitsRemaining != 0U) {
                const uint8_t bit = (writeShift & 0x80U) != 0U;
                feedWriteBit(bit);
                writeShift <<= 1U;
                --writeBitsRemaining;
            }
        } else if ((iwmLines & (IWM_Q6 | IWM_Q7)) == 0) {
            shiftTrackBit(trackBit(trackPosition));
        }
        trackPosition = (trackPosition + 1U) % trackBits;
        if (++readBitCount == 8U) {
            readBitCount = 0;
        }
    }
}

static uint8_t decodeGcrByte(uint8_t value) {
    for (uint8_t index = 0; index < 64U; ++index) {
        if (kGcr[index] == value) return index;
    }
    return 0xFFU;
}

static int decodeWrittenSector(const uint8_t *gcr, uint8_t *destination) {
    for (uint16_t index = 0; index < DATA_GCR_BYTES; ++index) {
        writeDecodedGcr[index] = decodeGcrByte(gcr[index]);
        if (writeDecodedGcr[index] > 0x3FU) return 0;
    }

    uint32_t output = 0;
    for (uint16_t index = 0; index < 522U; index += 3U) {
        const uint8_t high = writeDecodedGcr[output++];
        writeCooked[index] =
            (uint8_t)(((high << 2U) & 0xC0U) | writeDecodedGcr[output++]);
        writeCooked[index + 1U] =
            (uint8_t)(((high << 4U) & 0xC0U) | writeDecodedGcr[output++]);
        writeCooked[index + 2U] =
            (uint8_t)(((high << 6U) & 0xC0U) | writeDecodedGcr[output++]);
    }
    uint8_t high = writeDecodedGcr[output++];
    writeCooked[522] =
        (uint8_t)(((high << 2U) & 0xC0U) | writeDecodedGcr[output++]);
    writeCooked[523] =
        (uint8_t)(((high << 4U) & 0xC0U) | writeDecodedGcr[output++]);
    high = writeDecodedGcr[output++];
    writeCooked[524] =
        (uint8_t)(((high << 2U) & 0xC0U) | writeDecodedGcr[output++]);
    writeCooked[525] =
        (uint8_t)(((high << 4U) & 0xC0U) | writeDecodedGcr[output++]);
    writeCooked[526] =
        (uint8_t)(((high << 6U) & 0xC0U) | writeDecodedGcr[output++]);

    uint32_t checksum[3] = {};
    for (uint16_t index = 0; index < 524U; ++index) {
        if ((index % 3U) == 0U) {
            checksum[2] = ((checksum[2] << 1U) & 0x1FEU) |
                          ((checksum[2] >> 7U) & 1U);
        }
        const uint32_t carry = (checksum[2] >> 8U) & 1U;
        checksum[2] &= 0xFFU;
        const uint8_t value = (uint8_t)(writeCooked[index] ^ checksum[2]);
        writeDecoded[index] = value;
        checksum[0] += value + carry;
        checksum[2] &= 0xFFU;
        const uint32_t next = checksum[0];
        checksum[0] = checksum[1];
        checksum[1] = checksum[2];
        checksum[2] = next;
    }
    checksum[2] &= 0xFFU;
    if (writeCooked[524] != (uint8_t)checksum[1] ||
        writeCooked[525] != (uint8_t)checksum[2] ||
        writeCooked[526] != (uint8_t)checksum[0]) {
        return 0;
    }
    memcpy(destination, writeDecoded + 12U, 512U);
    return 1;
}

static uint8_t collectWriteByte(uint8_t bit, uint8_t *value) {
    writeParserByte = (uint8_t)((writeParserByte << 1U) | (bit & 1U));
    if (++writeParserBits != 8U) return 0;
    *value = writeParserByte;
    writeParserByte = 0;
    writeParserBits = 0;
    return 1;
}

static void feedWriteBit(uint8_t bit) {
    const uint8_t zone = zoneForTrack(writeTrackTrack);
    const uint8_t sectors = kSectorsPerZone[zone];
    uint8_t value = 0;

    if (writeParserState == WRITE_SCAN ||
        writeParserState == WRITE_DATA_MARK) {
        writeWindow = ((writeWindow << 1U) | (bit & 1U)) & 0x00FFFFFFU;
        const uint32_t mark = writeParserState == WRITE_SCAN
            ? 0xD5AA96U : 0xD5AAADU;
        if (writeWindow == mark) {
            writeParserByte = 0;
            writeParserBits = 0;
            writeAddressCount = 0;
            if (writeParserState == WRITE_SCAN) {
                writeParserState = WRITE_ADDRESS;
            } else {
                writeParserState = WRITE_DATA_SECTOR;
            }
        }
        return;
    }

    if (!collectWriteByte(bit, &value)) return;

    switch (writeParserState) {
    case WRITE_ADDRESS:
        if (writeAddressCount >= sizeof(writeAddress)) {
            writeAddressCount = 0;
            writeWindow = 0;
            writeParserState = WRITE_SCAN;
            return;
        }
        writeAddress[writeAddressCount++] = value;
        if (writeAddressCount == 5U) {
            const uint8_t c0 = decodeGcrByte(writeAddress[0]);
            const uint8_t sector = decodeGcrByte(writeAddress[1]);
            const uint8_t head = decodeGcrByte(writeAddress[2]);
            const uint8_t format = decodeGcrByte(writeAddress[3]);
            const uint8_t checksum = decodeGcrByte(writeAddress[4]);
            // During 800K formatting the classic Sony driver can emit the
            // 400K address format (0x02) while it is rebuilding both sides;
            // normal 800K media uses 0x22.  PCE accepts the address field
            // independently of this marker, so accept both values here and
            // let the track/side/checksum fields provide the real validation.
            const uint8_t formatValid =
                format == 0x02U ||
                (floppyBytes == INSTALL_800K_BYTES && format == 0x22U);
            const uint16_t decodedTrack =
                (uint16_t)(c0 & 0x3FU) |
                ((uint16_t)(head & 0x1FU) << 6U);
            const uint8_t decodedSide = (uint8_t)((head >> 5U) & 3U);
            const uint8_t valid = c0 <= 0x3FU && sector < sectors &&
                head <= 0x3FU && format <= 0x3FU && checksum <= 0x3FU &&
                decodedTrack == writeTrackTrack &&
                decodedSide == writeTrackSide &&
                ((writeTrackTrack ^ head ^ sector ^ format ^ checksum) & 0x3FU) == 0U &&
                formatValid;
            if (valid) {
                writeDataSector = sector;
                // The data marker is searched bitwise, so the exact number of
                // sync words emitted by the Mac drive routine is irrelevant.
                writeWindow = 0;
                writeParserState = WRITE_DATA_MARK;
            } else {
                writeWindow = 0;
                writeParserState = WRITE_SCAN;
            }
        }
        break;
    case WRITE_DATA_SECTOR:
        if (decodeGcrByte(value) == writeDataSector) {
            writeGcrCount = 0;
            writeParserState = WRITE_DATA_GCR;
        } else {
            writeWindow = 0;
            writeParserState = WRITE_SCAN;
        }
        break;
    case WRITE_DATA_GCR:
        if (writeGcrCount >= DATA_GCR_BYTES) {
            writeGcrCount = 0;
            writeWindow = 0;
            writeParserState = WRITE_SCAN;
            return;
        }
        writeGcr[writeGcrCount++] = value;
        if (writeGcrCount == DATA_GCR_BYTES) {
            if (decodeWrittenSector(writeGcr, writePayload) &&
                hdWriteInstallSector(
                    sectorOffset(writeTrackTrack, writeTrackSide,
                                 writeDataSector), writePayload)) {
                sourceSectorValid = 0;
                encodedValid = 0;
            }
            writeWindow = 0;
            writeParserState = WRITE_SCAN;
        }
        break;
    default:
        writeWindow = 0;
        writeParserState = WRITE_SCAN;
        break;
    }
}

static void finishWriting(void) {
    if (!writing) return;
    writing = 0;
    writeBufferValid = 0;
    writeBitsRemaining = 0;
    writeParserByte = 0;
    writeParserBits = 0;
    (void)hdFlushInstallVolume();
}
