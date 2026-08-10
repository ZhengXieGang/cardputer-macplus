/*
 * Minimal Macintosh Plus IWM state used by the system ROM.
 *
 * Uploaded MFS/HFS volumes are kept in a memory-mapped Flash slot and can be
 * inserted here after the system disk has finished booting.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "iwm.h"

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
#define TRACK_LEAD_SYNC_WORDS 32U
#define ADDRESS_SYNC_WORDS 38U
#define DATA_SYNC_WORDS 8U
#define ADDRESS_MARK_BITS 24U
#define ADDRESS_FIELD_BITS 40U
#define ADDRESS_SUFFIX_BITS 16U
#define DATA_MARK_BITS 24U
#define DATA_SECTOR_BITS 8U
#define DATA_GCR_BYTES 703U
#define DATA_SUFFIX_BITS 16U
#define TRACK_LEAD_BITS (TRACK_LEAD_SYNC_WORDS * SYNC_WORD_BITS)
#define SECTOR_BITS (ADDRESS_SYNC_WORDS * SYNC_WORD_BITS + ADDRESS_MARK_BITS + \
                     ADDRESS_FIELD_BITS + ADDRESS_SUFFIX_BITS + \
                     DATA_SYNC_WORDS * SYNC_WORD_BITS + DATA_MARK_BITS + \
                     DATA_SECTOR_BITS + DATA_GCR_BYTES * 8U + DATA_SUFFIX_BITS)

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
static const uint32_t kTrackBits[5] = {76262, 69902, 63540, 57190, 50838};
static const uint8_t kAddressMark[3] = {0xD5, 0xAA, 0x96};
static const uint8_t kDataMark[3] = {0xD5, 0xAA, 0xAD};
static const uint8_t kBitSlip[2] = {0xDE, 0xAA};

static int iwmLines;
static int iwmModeReg;
static int iwmHeadSel;
static const uint8_t *floppyData;
static uint32_t floppyBytes;
static uint32_t floppyReadBytes;
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
static uint8_t stepDown;
static uint8_t encodedSector[DATA_GCR_BYTES];
static int encodedValid;

static uint8_t zoneForTrack(uint8_t track) {
    return track < TRACKS ? track / 16U : 4U;
}

static uint8_t internalDriveSelected(void) {
    // Mac Plus SELECT low addresses the internal drive; SELECT high addresses
    // the optional external drive, which this single-drive emulator lacks.
    return (iwmLines & IWM_SELECT) == 0;
}

static uint8_t sideCount(void) {
    return floppyBytes == INSTALL_800K_BYTES ? 2U : 1U;
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

static uint8_t sourceByte(uint8_t track, uint8_t side, uint8_t sector,
                          uint16_t index) {
    if (index < 12U) return 0;
    return floppyData[sectorOffset(track, side, sector) * 512U + index - 12U];
}

static void encodeSector(uint8_t track, uint8_t side, uint8_t sector) {
    uint8_t cooked[527];
    uint32_t checksum[3] = {};
    for (uint16_t index = 0; index < 524U; ++index) {
        if ((index % 3U) == 0) {
            checksum[0] = ((checksum[0] << 1U) & 0x1FEU) |
                          ((checksum[0] >> 7U) & 1U);
        }
        uint32_t value = sourceByte(track, side, sector, index);
        checksum[2] += value + ((checksum[0] >> 8U) & 1U);
        checksum[0] &= 0xFFU;
        cooked[index] = (uint8_t)(value ^ checksum[0]);

        const uint32_t next = checksum[2];
        checksum[2] = checksum[1];
        checksum[1] = checksum[0];
        checksum[0] = next;
    }
    checksum[0] &= 0xFFU;
    cooked[524] = (uint8_t)checksum[1];
    cooked[525] = (uint8_t)checksum[0];
    cooked[526] = (uint8_t)checksum[2];

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

static uint8_t trackBit(uint32_t position) {
    const uint8_t zone = zoneForTrack(driveTrack);
    const uint8_t sectors = kSectorsPerZone[zone];
    if (position < TRACK_LEAD_BITS) return syncBit(position);

    position -= TRACK_LEAD_BITS;
    const uint32_t record = position / SECTOR_BITS;
    uint32_t offset = position % SECTOR_BITS;
    if (record >= sectors) return syncBit(offset);

    const uint8_t side = iwmHeadSel && sideCount() == 2U ? 1U : 0U;
    const uint8_t sector = kSectorOrder[zone][record];
    const uint32_t addressSyncBits = ADDRESS_SYNC_WORDS * SYNC_WORD_BITS;
    if (offset < addressSyncBits) return syncBit(offset);
    offset -= addressSyncBits;

    if (offset < ADDRESS_MARK_BITS) return bytesBit(kAddressMark, offset);
    offset -= ADDRESS_MARK_BITS;

    if (offset < ADDRESS_FIELD_BITS) {
        const uint8_t values[5] = {
            driveTrack & 0x3F,
            sector & 0x1F,
            (uint8_t)((side << 5) | ((driveTrack >> 6) & 1U)),
            floppyBytes == INSTALL_800K_BYTES ? 0x22 : 0x02,
            0,
        };
        const uint8_t checksum = values[0] ^ values[1] ^ values[2] ^ values[3];
        const uint32_t field = offset >> 3U;
        return byteBit(kGcr[(field == 4U ? checksum : values[field]) & 0x3F],
                       offset);
    }
    offset -= ADDRESS_FIELD_BITS;

    if (offset < ADDRESS_SUFFIX_BITS) return bytesBit(kBitSlip, offset);
    offset -= ADDRESS_SUFFIX_BITS;

    const uint32_t dataSyncBits = DATA_SYNC_WORDS * SYNC_WORD_BITS;
    if (offset < dataSyncBits) return syncBit(offset);
    offset -= dataSyncBits;

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
    if (!internalDriveSelected()) return;

    const uint8_t reg = (uint8_t)(iwmLines & (IWM_CA0 | IWM_CA1)) |
                        (iwmHeadSel ? 4U : 0U);
    const uint8_t value = (iwmLines & IWM_CA2) != 0;

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
            trackPosition = 0;
            encodedValid = 0;
            cycleAccumulator = 0;
            shiftReg = 0;
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
        if (value) {
            diskInserted = 0;
            motorOn = 0;
            diskSwitched = 1;
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

    if (!internalDriveSelected()) {
        switch (reg) {
        case 2: return 1;  /* motor off */
        case 7: return 0;  /* drive installed */
        case 8: return 1;  /* no disk inserted */
        case 9: return 1;  /* not write-protected */
        case 11: return 1; /* no tachometer pulses */
        case 13: return 1; /* not a SuperDrive */
        case 15: return 1; /* IWM interface */
        default: return 0;
        }
    }

    switch (reg) {
    case 0: return stepDown == 0;          /* step direction */
    case 1: return 1;                      /* step finished */
    case 2: return motorOn == 0;
    case 3: return diskSwitched;
    case 4: return 0;                      /* head 0 exists */
    case 5: return 0;                      /* not a SuperDrive */
    case 6: return sideCount() == 2U;
    case 7: return 0;                      /* drive installed */
    case 8: return diskInserted == 0;      /* CISTN */
    case 9: return 0;                      /* uploaded disk is write-protected */
    case 10: return driveTrack != 0;
    case 11:
        return !motorOn || ((trackPosition * 120U / trackBits) & 1U) == 0;
    case 12: return 0;                     /* head 1 exists */
    case 13: return 1;                     /* not a SuperDrive */
    case 14: return 0;                     /* drive ready */
    case 15: return 1;                     /* IWM interface */
    default: return 1;
    }
}

void iwmInit(void) {
    iwmLines = 0;
    iwmModeReg = 0;
    iwmHeadSel = 0;
    floppyData = NULL;
    floppyBytes = 0;
    floppyReadBytes = 0;
    trackPosition = 0;
    cycleAccumulator = 0;
    readBitCount = 0;
    dataReg = 0;
    shiftReg = 0;
    driveTrack = 0;
    motorOn = 0;
    diskInserted = 0;
    diskSwitched = 0;
    stepDown = 0;
    encodedValid = 0;

}

void iwmSetDisk(const uint8_t *data, uint32_t bytes, int inserted) {
    if (data == NULL || (bytes != 400U * 1024U && bytes != 800U * 1024U)) {
        floppyData = NULL;
        floppyBytes = 0;
        diskInserted = 0;
        motorOn = 0;
        diskSwitched = 0;
        return;
    }
    floppyData = data;
    floppyBytes = bytes;
    diskInserted = inserted ? 1U : 0U;
    motorOn = 0;
    diskSwitched = inserted ? 1U : 0U;
    driveTrack = 0;
    trackPosition = 0;
    cycleAccumulator = 0;
    readBitCount = 0;
    dataReg = 0;
    shiftReg = 0;
    encodedValid = 0;
}

void iwmAccess(unsigned int addr) {
    const int bit = 1 << (addr >> 1);
    if (addr & 1U) iwmLines |= bit;
    else iwmLines &= ~bit;
    if (iwmLines & IWM_LSTRB) {
        writeDriveRegister();
    }
}

void iwmWrite(unsigned int addr, unsigned int val) {
    iwmAccess(addr);
    const int reg = iwmLines & (IWM_Q7 | IWM_Q6);
    if (reg == (IWM_Q7 | IWM_Q6) && !(iwmLines & IWM_ENABLE)) {
        iwmModeReg = val;
    }
}

void iwmSetHeadSel(int side) {
    iwmHeadSel = side ? 1 : 0;
}

unsigned int iwmRead(unsigned int addr) {
    iwmAccess(addr);
    const int reg = iwmLines & (IWM_Q7 | IWM_Q6);
    if (reg == 0) {
        if (!(iwmLines & IWM_ENABLE)) return 0xFF;
        if (!internalDriveSelected()) return 0xFF;
        const uint8_t value = dataReg;
        dataReg = 0;
        return value;
    }
    if (reg == IWM_Q6) {
        uint8_t value = iwmModeReg & 0x1F;
        if (iwmLines & IWM_ENABLE) value |= 0x20;
        if (driveSense()) value |= 0x80;
        return value;
    }
    if (reg == IWM_Q7) return 0xFF;
    return (iwmModeReg & 0x1F) | ((iwmLines & IWM_ENABLE) ? 0x20 : 0);
}

void iwmTick(unsigned int cycles) {
    if (!internalDriveSelected() || !diskInserted || !motorOn ||
        floppyData == NULL) return;
    // 500kbit/s at the Macintosh Plus 7.8336MHz CPU clock (500000/7833600 = 625/9792).
    cycleAccumulator += cycles * 625U;
    const uint8_t zone = zoneForTrack(driveTrack);
    const uint32_t trackBits = kTrackBits[zone];
    while (cycleAccumulator >= 9792U) {
        cycleAccumulator -= 9792U;
        if ((iwmLines & (IWM_Q6 | IWM_Q7)) == 0) {
            shiftTrackBit(trackBit(trackPosition));
        }
        trackPosition = (trackPosition + 1U) % trackBits;
        if (++readBitCount == 8U) {
            readBitCount = 0;
            ++floppyReadBytes;
        }
    }
}

uint32_t iwmGetFloppyReadCount(void) { return floppyReadBytes; }
