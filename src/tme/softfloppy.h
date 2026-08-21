#ifndef SOFTFLOPPY_H
#define SOFTFLOPPY_H

#include <stdint.h>

void softFloppyInit(void);
void softFloppyFrameTick(void);
int softFloppyPvHook(uint8_t opcode);

#endif
