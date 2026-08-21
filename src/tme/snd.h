#include <stdint.h>

void sndInit();
int sndPush(uint8_t *data, int volume);
void sndSetVolume(uint8_t volume);
uint8_t sndGetVolume(void);
