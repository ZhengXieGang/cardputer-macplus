#pragma once

#include <stdint.h>

// The Launcher build never assumes an address. It uses the data partition
// whose label is `macplus`; PMan may resize that partition at any time.
static constexpr const char *MACPLUS_DATA_PARTITION_LABEL = "macplus";

// The first erase sector is reserved for the image metadata and write journal.
// The image itself starts at a stable offset, so enlarging `macplus` never
// moves existing disk data.
static constexpr uint32_t MACPLUS_HD_METADATA_BYTES = 0x1000U;
static constexpr uint32_t MACPLUS_HD_DATA_OFFSET = MACPLUS_HD_METADATA_BYTES;
static constexpr uint32_t MACPLUS_STORAGE_ALIGNMENT_BYTES = 0x10000U;
static constexpr uint32_t MACPLUS_STORAGE_HEADROOM_BYTES = 0x10000U;
static constexpr uint32_t MACPLUS_STORAGE_TAIL_RESERVE_BYTES = 0x10000U;
static constexpr uint32_t MACPLUS_INSTALL_400K_BYTES = 800U * 512U;
static constexpr uint32_t MACPLUS_INSTALL_800K_BYTES = 1600U * 512U;
