#pragma once

#include <stdint.h>

// This data partition is declared by the standalone image and can also be
// created by a Launcher install.  All offsets below are relative to it.
static constexpr const char *MACPLUS_DATA_PARTITION_LABEL = "macplus";
static constexpr uint32_t MACPLUS_DATA_PARTITION_STANDARD_BYTES = 0x4E0000U;
static constexpr uint32_t MACPLUS_DATA_PARTITION_FULL_BYTES = 0x660000U;

// A 4 MiB HFS volume plus the 96-block SCSI wrapper.
static constexpr uint32_t MACPLUS_HD_MAX_IMAGE_BYTES = 0x40C000U;
// Full builds use the remaining flash after the application and partition
// table.  The install disk and metadata remain at the end of the partition.
static constexpr uint32_t MACPLUS_HD_FULL_MAX_IMAGE_BYTES = 0x593000U;
static constexpr uint32_t MACPLUS_HD_METADATA_OFFSET =
    MACPLUS_HD_MAX_IMAGE_BYTES;
static constexpr uint32_t MACPLUS_HD_METADATA_BYTES = 0x1000U;
static constexpr uint32_t MACPLUS_INSTALL_GAP_BYTES = 0x4000U;

// Keep the existing 400K/800K installer media protocol.  Its location moves
// only inside the named data partition; the legacy raw layout stays at the
// old absolute address for compatibility.
static constexpr uint32_t MACPLUS_INSTALL_OFFSET =
    MACPLUS_HD_METADATA_OFFSET + 0x4000U;
static constexpr uint32_t MACPLUS_INSTALL_400K_BYTES = 800U * 512U;
static constexpr uint32_t MACPLUS_INSTALL_800K_BYTES = 1600U * 512U;
static constexpr uint32_t MACPLUS_INSTALL_MARKER_OFFSET =
    MACPLUS_INSTALL_OFFSET + MACPLUS_INSTALL_800K_BYTES;
static constexpr uint32_t MACPLUS_STORAGE_REQUIRED_BYTES =
    MACPLUS_INSTALL_MARKER_OFFSET + 0x1000U;

static constexpr uint32_t macplusHdMaxForPartition(uint32_t partitionBytes) {
    return partitionBytes >= MACPLUS_DATA_PARTITION_FULL_BYTES
               ? MACPLUS_HD_FULL_MAX_IMAGE_BYTES
               : MACPLUS_HD_MAX_IMAGE_BYTES;
}
