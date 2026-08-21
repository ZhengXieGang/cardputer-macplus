#pragma once

// Runtime diagnostics are intentionally compiled out of the normal image.
// Enable them with -DMACPLUS_ENABLE_DEBUG_LOG=1 when a serial trace is needed.
#ifndef MACPLUS_ENABLE_DEBUG_LOG
#define MACPLUS_ENABLE_DEBUG_LOG 0
#endif

#if MACPLUS_ENABLE_DEBUG_LOG
#include <stdio.h>
#define MACPLUS_LOG(...) printf(__VA_ARGS__)
#else
#define MACPLUS_LOG(...) do { } while (0)
#endif
