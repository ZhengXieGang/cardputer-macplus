#ifndef TMECONFIG_H
#define TMECONFIG_H

// Cardputer-Adv / Stamp-S3A: ESP32-S3FN8, no PSRAM. The whole emulated Mac
// (including the 512x342 screen buffer) lives in the internal SRAM.  This
// matches the small-RAM configuration proven by pico-mac/umac on the RP2040:
// the Mac Plus ROM is patched (tools/patch_rom.py) to report a 256KB machine,
// and the video framebuffer sits at the top of that RAM.
#define TME_ROMSIZE (128*1024)
#define TME_CACHESIZE 0
#define TME_RAMSIZE (256*1024)     // 256KB emulated Mac RAM
// The 512x342 framebuffer (0x5580 bytes) sits at the top of the 256KB RAM:
// memtop - 0x5900 = 0x3A700.  The ROM's native 4MB-view address 0x3FA700
// wraps onto the same offset because 256KB divides 4MB evenly.
#define TME_SCREENBUF (TME_RAMSIZE - (512U*342U/8U) - 0x380U)
#define TME_SNDBUF (0x3FFD00 & (TME_RAMSIZE - 1))
#define TME_SNDBUF_ALT (TME_SNDBUF - 0x5C00)

#endif
