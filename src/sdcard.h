#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Mount the Cardputer microSD card (SDSPI on GPIO12/14/40/39) at /sd.
bool sdcardInit();
bool sdcardRetry();
bool sdcardMounted();

// Serialize all application-level FATFS operations across the HD backend.
// ESP-IDF's volume lock does not cover higher-level FILE buffering as one unit.
bool sdcardAcquire(uint32_t timeoutMs);
void sdcardRelease();

#ifdef __cplusplus
}
#endif
