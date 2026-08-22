#include <stdint.h>

void sndInit();
int sndPush(uint8_t *data, int volume);
// Volume is a user-facing percentage (1..100), not M5Unified's raw value.
void sndSetVolume(uint8_t percent);
uint8_t sndGetVolume(void);
// Stop queued PCM and mute the codec before a software reset or power-down.
// This is also safe in WiFi transfer mode, where sndInit() is skipped.
void sndPrepareForRestart(void);
