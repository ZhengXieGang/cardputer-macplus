#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// The same firmware reboots into a low-memory WiFi transfer mode. That mode
// skips Mac RAM, emulator, display-task and audio initialization.
void webInstallRun();
int webInstallModeRequested();
void requestWebInstallMode();
void exitWebInstallMode();

#ifdef __cplusplus
}
#endif
