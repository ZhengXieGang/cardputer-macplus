#pragma once

#include <stdint.h>

// Cardputer-Adv physical input: 56-key TCA8418 keyboard, BMI270 IMU mouse and
// the G0 (GO) button as the Mac mouse button.  All events are pushed into the
// emulator through the same paths the LAN/BLE clients used.
void cardputerInputInit();
void cardputerInputPoll();
bool cardputerInputAnyKeyPressed();
uint8_t cardputerInputReadKeyPress();

// IMU tuning (radial joystick tilt control). The accelerometer measures the
// gravity tilt vector. Motion engages above 5 degrees and releases below
// 3.5 degrees so hand tremor cannot repeatedly start/stop the cursor. The
// pointer-speed percentage scales the entire response curve uniformly.
#define IMU_ENGAGE_DEADZONE_DEG 5.0f
#define IMU_RELEASE_DEADZONE_DEG 3.5f
#define IMU_MAX_TILT_DEG 40.0f
#define IMU_BASE_CURSOR_SPEED 320.0f
#define IMU_POINTER_SPEED_DEFAULT_PERCENT 100
#define IMU_POINTER_SPEED_MIN_PERCENT 10
#define IMU_POINTER_SPEED_MAX_PERCENT 300
#define IMU_POINTER_SPEED_STEP_PERCENT 10
#define IMU_MAX_DELTA 10

extern uint16_t imuPointerSpeedPercent;
extern int imuFlipX;
extern int imuFlipY;
extern int imuSwapXY;
