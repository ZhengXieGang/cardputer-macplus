

#include <stdint.h>

void tmeStartEmu(void *rom);
void tmeMouseMovement(int dx, int dy, int btn);
uint32_t tmeGetProgramCounter(void);
int tmeIsRunning(void);
uint16_t tmeGetMouseX(void);
uint16_t tmeGetMouseY(void);
uint8_t tmeGetScsiDeviceMask(void);
