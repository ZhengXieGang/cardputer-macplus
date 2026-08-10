void iwmInit(void);
void iwmSetDisk(const uint8_t *data, uint32_t bytes, int inserted);
void iwmWrite(unsigned int addr, unsigned int val);
unsigned int iwmRead(unsigned int addr);
void iwmSetHeadSel(int s);
void iwmTick(unsigned int cycles);
uint32_t iwmGetFloppyReadCount(void);
