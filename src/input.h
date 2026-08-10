#pragma once

#include <stdint.h>

// Cardputer-Adv physical input: 56-key TCA8418 keyboard, BMI270 IMU mouse and
// the G0 (GO) button as the Mac mouse button.  All events are pushed into the
// emulator through the same paths the LAN/BLE clients used.
void cardputerInputInit();
void cardputerInputPoll();
void cardputerInputImuStatus();
void cardputerInputMouseStatus();
void cardputerInputKeysStatus();
bool cardputerInputAnyKeyPressed();

// IMU tuning (radial joystick tilt control). The accelerometer measures the
// gravity tilt vector. Motion engages above 5 degrees and releases below
// 3.5 degrees so hand tremor cannot repeatedly start/stop the cursor. Speed
// uses a gentle nonlinear response: small tilts stay controllable, while
// medium tilts need less effort and a large tilt still reaches
// imuSensitivity px/s. Runtime commands can change the sensitivity and axis
// orientation.
#define IMU_ENGAGE_DEADZONE_DEG 5.0f
#define IMU_RELEASE_DEADZONE_DEG 3.5f
#define IMU_MAX_TILT_DEG 45.0f
#define IMU_SENSITIVITY 320.0f
#define IMU_MAX_DELTA 10

extern float imuSensitivity;
extern int imuFlipX;
extern int imuFlipY;
extern int imuSwapXY;
