#!/usr/bin/env python3
"""Patch a Mac Plus v3 ROM for the Cardputer-Adv emulator (no PSRAM).

The emulator runs the Mac with 256KB of RAM in the chip's internal SRAM, the
same small-RAM technique proven by pico-mac/umac on the RP2040.  This tool
applies the two patches needed for that configuration to the stock Mac Plus
v3 ROM (checksum 0x4D1F8172):

  1. Disable the ROM checksum comparison (the patch changes the image).
  2. Force memtop to RAM_SIZE and skip the checksum/memory-test failure path,
     because the ROM only knows about 128/512/1024+KB machines.

The screen stays at the native 512x342 resolution (in RAM at memtop-0x5900,
exactly like a real Mac Plus), so no screen-size patches are needed.

Usage:
    python3 tools/patch_rom.py macplus.rom macplus-patched.rom [--ram-kb 256]
"""

import argparse
import struct
import sys

ROM_PLUS_V3_CHECKSUM = 0x4D1F8172
ROM_INITIAL_PC = 0x0040002A
ROM_SIZE = 128 * 1024


def rd16(rom, offset):
    return (rom[offset] << 8) | rom[offset + 1]


def wr16(rom, offset, value):
    rom[offset] = (value >> 8) & 0xFF
    rom[offset + 1] = value & 0xFF


def patch_plus_v3(rom, ram_size):
    # 1. Disable checksum check: turn "eor.l d3, d1" into "eor.l d1, d1"
    #    so the checksum comparison always reports success.
    wr16(rom, 0xD92, 0xB381)

    # 2. Small-RAM memtop hack (only for 128KB < RAM < 512KB).
    if 128 < (ram_size // 1024) < 512:
        # NOP out the probed memory-size computation, then force A5 = RAM_SIZE.
        for offset in range(0x376, 0x37E, 2):
            wr16(rom, offset, 0x4E71)  # NOP
        wr16(rom, 0x376, 0x2A7C)  # moveal #imm32, A5
        wr16(rom, 0x378, (ram_size >> 16) & 0xFFFF)
        wr16(rom, 0x37A, ram_size & 0xFFFF)
        # P_ChecksumRomAndTestMemory returns a failure code for sizes that
        # are not 128/512KB; branch over that failure path unconditionally.
        wr16(rom, 0x132, 0x6000)  # bra (was beq)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="stock macplus.rom (128KB, v3)")
    parser.add_argument("output", help="patched ROM image to write")
    parser.add_argument("--ram-kb", type=int, default=256,
                        help="emulated Mac RAM in KB (must match TME_RAMSIZE)")
    args = parser.parse_args()

    with open(args.input, "rb") as fh:
        data = bytearray(fh.read())

    if len(data) != ROM_SIZE:
        sys.exit(f"error: ROM must be exactly {ROM_SIZE} bytes, got {len(data)}")

    stored_checksum = struct.unpack(">I", data[0:4])[0]
    init_pc = struct.unpack(">I", data[4:8])[0]
    print(f"ROM: checksum=0x{stored_checksum:08X} pc=0x{init_pc:08X}")
    if stored_checksum != ROM_PLUS_V3_CHECKSUM:
        sys.exit("error: not a Mac Plus v3 ROM (checksum mismatch)")
    if init_pc != ROM_INITIAL_PC:
        sys.exit("error: unexpected initial PC")

    ram_size = args.ram_kb * 1024
    if not (128 * 1024 <= ram_size < 512 * 1024):
        sys.exit("error: --ram-kb must be between 128 and 512")

    patch_plus_v3(data, ram_size)
    with open(args.output, "wb") as fh:
        fh.write(data)
    print(f"ROM: patched for {args.ram_kb}KB RAM -> {args.output}")


if __name__ == "__main__":
    main()
