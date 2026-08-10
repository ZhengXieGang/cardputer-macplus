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

// Serialize all application-level FATFS operations across the HD backend and
// the serial diagnostics. ESP-IDF's volume lock does not cover higher-level
// FILE buffering as one unit.
bool sdcardAcquire(uint32_t timeoutMs);
void sdcardRelease();

// Write deterministic temporary data, read it back, verify it, and remove it.
// Never touches macplus.rom or hd.img. Used by the serial 'sdwritetest'.
bool sdcardRunWriteReadTest(uint32_t bytes);

// Kept for interface compatibility; there are no pending updates anymore.
bool sdcardApplyPendingUpdates();

void sdcardPrintRoot();

#ifdef __cplusplus
}
#endif
