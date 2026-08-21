#pragma once

#include <stdint.h>

// Load/save the small set of user-adjustable MacPlus controls from the
// Launcher's shared NVS partition.  Loading is best-effort when that
// partition is unavailable; the compile-time defaults remain in effect.
void loadMacSettings(void);
bool saveMacSettings(uint16_t pointerSpeedPercent, uint8_t volume);
