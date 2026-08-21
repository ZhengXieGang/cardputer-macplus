// Embedded patched Mac Plus v3 ROM (256KB RAM build).
//
// The firmware is self-contained so it can be installed by the M5 Launcher
// (which does not preserve custom data partitions).  Regenerate the blob
// with:  python3 tools/patch_rom.py <stock macplus.rom> roms/macplus-patched-256.rom
// The blob is regenerated whenever the ROM driver patch changes.
    .section .rodata
    .align 4
    .global macplus_rom_embedded
    .global macplus_rom_embedded_size
macplus_rom_embedded:
    .incbin "roms/macplus-patched-256.rom"
macplus_rom_embedded_size:
    .word 131072
