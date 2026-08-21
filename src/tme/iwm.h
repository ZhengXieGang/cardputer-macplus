#include <stdint.h>

typedef int (*IwmSectorReader)(uint32_t sector, uint8_t *destination);

void iwmInit(void);
void iwmSetDiskReader(IwmSectorReader reader, uint32_t bytes, int inserted);
void iwmWrite(unsigned int addr, unsigned int val);
unsigned int iwmRead(unsigned int addr);
void iwmSetHeadSel(int s);
void iwmTick(unsigned int cycles);
