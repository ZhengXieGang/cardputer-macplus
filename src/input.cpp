/*
 * Cardputer-Adv input for the Mac Plus emulator.
 *
 * Keyboard: the 56-key TCA8418 FIFO is polled directly and translated to Mac
 * ADB scancodes. Modifiers follow the convention of the
 * emulator's web client: Ctrl = Command (0x37), Opt = Option (0x3A); the extra
 * Alt key becomes Mac Control (0x36).
 *
 * Mouse: the BMI270 accelerometer is used as a rate-control trackball. Tilting the
 * device moves the cursor; keeping it level stops it.  The G0/GO button is the
 * Mac mouse button (press and hold for dragging).
 */
#include <Arduino.h>
#include <M5Cardputer.h>

#include "hardware_config.h"
#include "input.h"
#include "webinstall.h"

extern "C" {
#include "tme/emu.h"
#include "tme/mouse.h"
#include "tme/via.h"
}

namespace {

constexpr uint8_t kNoKey = 0xFF;

// TCA8418 events are converted by the same 7x8-to-4x14 remap used in the
// official Cardputer library. Reading its FIFO directly avoids reliance on a
// GPIO11 interrupt edge that may be missed while another board service starts.
constexpr uint8_t kMacScanMatrix[4][14] = {
    {0x35, 0x12, 0x13, 0x14, 0x15, 0x17, 0x16, 0x1A, 0x1C, 0x19, 0x1D, 0x1B, 0x18, 0x33},
    {0x30, 0x0C, 0x0D, 0x0E, 0x0F, 0x11, 0x10, 0x20, 0x22, 0x1F, 0x23, 0x21, 0x1E, 0x2A},
    {kNoKey, 0x38, 0x00, 0x01, 0x02, 0x03, 0x05, 0x04, 0x26, 0x28, 0x25, 0x29, 0x27, 0x24},
    {0x37, 0x3A, 0x36, 0x06, 0x07, 0x08, 0x09, 0x0B, 0x2D, 0x2E, 0x2B, 0x2F, 0x2C, 0x31},
};

constexpr uint8_t kFnScanMatrix[4][14] = {
    {kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, 0x33},
    {kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey},
    {kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, 0x4D, kNoKey, kNoKey},
    {kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, kNoKey, 0x46, 0x48, 0x42, kNoKey},
};

constexpr uint8_t TCA8418_ADDR = 0x34;
constexpr uint8_t TCA8418_CFG = 0x01;
constexpr uint8_t TCA8418_INT_STAT = 0x02;
constexpr uint8_t TCA8418_KEY_LCK_EC = 0x03;
constexpr uint8_t TCA8418_KEY_EVENT_A = 0x04;
constexpr uint8_t TCA8418_KP_GPIO_1 = 0x1D;
constexpr uint8_t TCA8418_KP_GPIO_2 = 0x1E;
constexpr uint8_t TCA8418_GPIO_DIR_1 = 0x23;
constexpr uint8_t TCA8418_GPIO_INT_EN_1 = 0x1A;
constexpr uint8_t TCA8418_GPI_EM_1 = 0x20;
constexpr uint8_t TCA8418_GPIO_INT_LVL_1 = 0x26;
constexpr uint32_t TCA8418_I2C_HZ = 400000;

// Modifier -> Mac scancode.
constexpr uint8_t kMacCmd = 0x37;    // physical Ctrl
constexpr uint8_t kMacOption = 0x3A; // physical Opt
constexpr uint8_t kMacCtrl = 0x36;   // physical Alt
constexpr uint8_t kMacShift = 0x38;

bool keysDown[128] = {};
bool physicalKeysDown[70] = {};
bool mouseButton = false;
uint32_t lastMouseTick = 0;
float mouseAccX = 0.0f;
float mouseAccY = 0.0f;
float accSmX = 0.0f;
float accSmY = 0.0f;
float accSmZ = 0.0f;
float imuVelocityX = 0.0f;
float imuVelocityY = 0.0f;
bool imuFilterValid = false;
bool imuMoveActive = false;
uint32_t imuReadFailCount = 0;
uint32_t imuLastReinitMs = 0;
uint32_t webComboStartMs = 0;
uint32_t lastKeyboardPollMs = 0;
uint32_t lastImuPollMs = 0;
bool webComboActive = false;
bool webComboTriggered = false;
uint32_t mousePollCount = 0;
uint32_t mouseMoveCount = 0;
uint32_t lastMouseDt = 0;
uint8_t physicalMacScan[4][14];
bool fnDown = false;
bool tcaReady = false;
uint8_t pendingKeyPress = kNoKey;
uint32_t tcaEventCount = 0;
uint32_t tcaReadFailures = 0;
uint32_t tcaLastInitMs = 0;

constexpr uint32_t WEB_COMBO_HOLD_MS = 2000;
constexpr uint32_t KEYBOARD_POLL_MS = 8;
constexpr uint32_t IMU_POLL_MS = 10;
constexpr float IMU_FILTER_ALPHA = 0.15f;
constexpr float IMU_VELOCITY_ALPHA = 0.30f;
float imuEngageDeadzone = 0.0f;
float imuReleaseDeadzone = 0.0f;
float imuMaxTilt = 0.0f;

// BMI270 register access used to force the accelerometer/gyro power state.
// M5Unified's BMI270 driver normally enables them itself, but one of the
// writes can silently fail when the app is launched right after another app
// (e.g. the Launcher) also touched the IMU; the driver then reports the IMU
// as enabled while getAccel() never returns data.  Writing PWR_CTRL again
// recovers without a full board reset.
static constexpr uint8_t BMI270_ADDR = 0x69;
static constexpr uint8_t BMI270_CHIP_ID_REG = 0x00;
static constexpr uint8_t BMI270_ACC_X_LSB_REG = 0x0C;
static constexpr uint8_t BMI270_PWR_CTRL_REG = 0x7D;
static constexpr uint8_t BMI270_PWR_CTRL_ACC_GYR = 0x0E; // temp | accel | gyro
static constexpr float BMI270_ACCEL_SCALE = 8.0f / 32768.0f;

static bool readBmi270Accel(float *ax, float *ay, float *az) {
    uint8_t raw[6];
    if (!M5.In_I2C.readRegister(BMI270_ADDR, BMI270_ACC_X_LSB_REG,
                                raw, sizeof(raw), 400000)) {
        return false;
    }
    const int16_t x = static_cast<int16_t>(raw[0] | (raw[1] << 8));
    const int16_t y = static_cast<int16_t>(raw[2] | (raw[3] << 8));
    const int16_t z = static_cast<int16_t>(raw[4] | (raw[5] << 8));
    *ax = x * BMI270_ACCEL_SCALE;
    *ay = y * BMI270_ACCEL_SCALE;
    *az = z * BMI270_ACCEL_SCALE;
    return true;
}

static bool forceBmi270PowerOn() {
    const uint8_t chipId = M5.In_I2C.readRegister8(BMI270_ADDR, BMI270_CHIP_ID_REG, 400000);
    const bool writeOk =
        M5.In_I2C.writeRegister8(BMI270_ADDR, BMI270_PWR_CTRL_REG,
                                 BMI270_PWR_CTRL_ACC_GYR, 400000);
    printf("IMU: BMI270 chipid=0x%02X pwrctrl_write=%d\n", chipId, writeOk ? 1 : 0);
    return chipId == 0x24 && writeOk;
}

// Re-run the M5Unified IMU initialisation (with retries) and make sure the
// BMI270 is actually producing data.  Rate-limited so a flaky sensor cannot
// trigger a reset storm every poll.
static bool reinitImu() {
    const uint32_t now = millis();
    if (now - imuLastReinitMs < 1000) return false;
    imuLastReinitMs = now;
    printf("IMU: %lu consecutive read failures, reinitializing...\n",
           static_cast<unsigned long>(imuReadFailCount));
    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
        ok = M5.Imu.begin(&M5.In_I2C, M5.getBoard()) && M5.Imu.isEnabled();
        if (!ok) delay(20);
    }
    forceBmi270PowerOn();
    delay(5);
    float ax = 0, ay = 0, az = 0;
    const bool readable = ok && readBmi270Accel(&ax, &ay, &az);
    printf("IMU: reinit %s (enabled=%d readable=%d accel=%.2f,%.2f,%.2f)\n",
           readable ? "OK" : "FAILED",
           M5.Imu.isEnabled() ? 1 : 0, readable ? 1 : 0, ax, ay, az);
    return readable;
}

void pushKey(uint8_t scancode, bool down) {
    if (scancode >= 128 || scancode == 0xFF) return;
    if (keysDown[scancode] == down) return;
    keysDown[scancode] = down;
    kbdPushKey(scancode, down ? 0 : 1);
}

void setModifier(bool pressed, uint8_t scancode) {
    pushKey(scancode, pressed);
}

static bool tcaRead8(uint8_t reg, uint8_t *value) {
    return M5.In_I2C.readRegister(TCA8418_ADDR, reg, value, 1,
                                  TCA8418_I2C_HZ);
}

static bool tcaWrite8(uint8_t reg, uint8_t value) {
    return M5.In_I2C.writeRegister8(TCA8418_ADDR, reg, value,
                                    TCA8418_I2C_HZ);
}

static void releaseAllKeys() {
    for (uint8_t scancode = 0; scancode < 128; ++scancode) {
        if (keysDown[scancode]) pushKey(scancode, false);
    }
    memset(physicalMacScan, kNoKey, sizeof(physicalMacScan));
    memset(physicalKeysDown, 0, sizeof(physicalKeysDown));
    fnDown = false;
}

static bool initTca8418() {
    bool ok = true;
    for (uint8_t reg = TCA8418_GPIO_DIR_1; reg < TCA8418_GPIO_DIR_1 + 3; ++reg) {
        ok &= tcaWrite8(reg, 0x00);
    }
    for (uint8_t reg = TCA8418_GPI_EM_1; reg < TCA8418_GPI_EM_1 + 3; ++reg) {
        ok &= tcaWrite8(reg, 0xFF);
    }
    for (uint8_t reg = TCA8418_GPIO_INT_LVL_1;
         reg < TCA8418_GPIO_INT_LVL_1 + 3; ++reg) {
        ok &= tcaWrite8(reg, 0x00);
    }
    for (uint8_t reg = TCA8418_GPIO_INT_EN_1;
         reg < TCA8418_GPIO_INT_EN_1 + 3; ++reg) {
        ok &= tcaWrite8(reg, 0xFF);
    }
    ok &= tcaWrite8(TCA8418_KP_GPIO_1, 0x7F); // 7 rows
    ok &= tcaWrite8(TCA8418_KP_GPIO_2, 0xFF); // 8 columns

    // Flush at most the hardware FIFO depth before accepting new events.
    uint8_t event = 0;
    for (int i = 0; i < 10 && tcaRead8(TCA8418_KEY_EVENT_A, &event) &&
                    event != 0; ++i) {}
    ok &= tcaWrite8(TCA8418_INT_STAT, 0x1F);
    ok &= tcaWrite8(TCA8418_CFG, 0x01); // keypad events only
    tcaReady = ok;
    tcaLastInitMs = millis();
    if (!ok) ++tcaReadFailures;
    printf("KEYBOARD: TCA8418 %s (direct FIFO)\n", ok ? "ready" : "unavailable");
    return ok;
}

static void updateWebCombo() {
    if (keysDown[kMacCmd] && keysDown[kMacOption] && keysDown[kMacCtrl]) {
        const uint32_t now = millis();
        if (!webComboActive) {
            webComboActive = true;
            webComboTriggered = false;
            webComboStartMs = now;
        } else if (!webComboTriggered &&
                   now - webComboStartMs >= WEB_COMBO_HOLD_MS) {
            webComboTriggered = true;
            requestWebInstallMode();
        }
    } else {
        webComboActive = false;
        webComboTriggered = false;
    }
}

static void handleTcaEvent(uint8_t event, bool routeToMac) {
    // TCA8418 key-event bit 7 is set for a press and clear for a release.
    const bool down = (event & 0x80) != 0;
    const uint8_t code = event & 0x7F;
    if (code == 0 || code > 70) return;
    physicalKeysDown[code - 1] = down;
    const uint8_t raw = code - 1;
    const uint8_t rawRow = raw / 10;
    const uint8_t rawCol = raw % 10;
    if (rawRow >= 7 || rawCol >= 8) return;
    const uint8_t row = (rawCol + 4) % 4;
    const uint8_t col = rawRow * 2 + (rawCol > 3 ? 1 : 0);
    if (row >= 4 || col >= 14) return;

    // Fn is a layer selector, not a Macintosh key.
    if (row == 2 && col == 0) {
        fnDown = down;
        return;
    }

    uint8_t scancode = physicalMacScan[row][col];
    if (down) {
        scancode = fnDown && kFnScanMatrix[row][col] != kNoKey
            ? kFnScanMatrix[row][col] : kMacScanMatrix[row][col];
        physicalMacScan[row][col] = scancode;
        if (!routeToMac && pendingKeyPress == kNoKey) {
            pendingKeyPress = scancode;
        }
    } else {
        physicalMacScan[row][col] = kNoKey;
    }
    if (routeToMac && scancode != kNoKey) pushKey(scancode, down);
}

static void pollKeyboard(bool routeToMac) {
    if (!tcaReady) {
        if (millis() - tcaLastInitMs >= 1000) initTca8418();
        return;
    }
    uint8_t eventCount = 0;
    if (!tcaRead8(TCA8418_KEY_LCK_EC, &eventCount)) {
        if (++tcaReadFailures >= 5) {
            releaseAllKeys();
            tcaReady = false;
        }
        return;
    }
    tcaReadFailures = 0;
    eventCount &= 0x0F;
    for (uint8_t i = 0; i < eventCount; ++i) {
        uint8_t event = 0;
        if (!tcaRead8(TCA8418_KEY_EVENT_A, &event)) {
            ++tcaReadFailures;
            break;
        }
        if (event != 0) {
            handleTcaEvent(event, routeToMac);
            ++tcaEventCount;
        }
    }
    if (eventCount != 0) tcaWrite8(TCA8418_INT_STAT, 0x09);
    if (routeToMac) updateWebCombo();
}

void pollMouse() {
    float ax = 0, ay = 0, az = 0;
    if (!readBmi270Accel(&ax, &ay, &az)) {
        // Never let a stale fractional position turn a short I2C fault into
        // a visible cursor jump when the sensor recovers.
        mouseAccX = 0.0f;
        mouseAccY = 0.0f;
        imuVelocityX = 0.0f;
        imuVelocityY = 0.0f;
        imuMoveActive = false;
        imuFilterValid = false;
        if (++imuReadFailCount >= 50) {
            if (reinitImu()) imuReadFailCount = 0;
        }
        return;
    }
    imuReadFailCount = 0;

    const uint32_t now = millis();
    const uint32_t dt = now - lastMouseTick;
    lastMouseTick = now;
    lastMouseDt = dt;
    ++mousePollCount;
    if (dt == 0 || dt > 100) {
        mouseAccX = 0.0f;
        mouseAccY = 0.0f;
        imuVelocityX = 0.0f;
        imuVelocityY = 0.0f;
        imuMoveActive = false;
        imuFilterValid = false;
        return;
    }

    // Seed the filter from the first valid sample. Starting from zero would
    // make the cursor slowly drift for several hundred milliseconds after
    // boot. A slightly stronger filter than before removes hand tremor while
    // the velocity smoother below keeps larger tilts responsive.
    if (!imuFilterValid) {
        accSmX = ax;
        accSmY = ay;
        accSmZ = az;
        imuFilterValid = true;
    } else {
        accSmX = accSmX * (1.0f - IMU_FILTER_ALPHA) + ax * IMU_FILTER_ALPHA;
        accSmY = accSmY * (1.0f - IMU_FILTER_ALPHA) + ay * IMU_FILTER_ALPHA;
        accSmZ = accSmZ * (1.0f - IMU_FILTER_ALPHA) + az * IMU_FILTER_ALPHA;
    }
    if (accSmZ <= 0.1f) {
        // Edge-on or upside-down is not a useful pointing posture. Require a
        // fresh filter seed when the device is turned back over.
        mouseAccX = 0.0f;
        mouseAccY = 0.0f;
        imuVelocityX = 0.0f;
        imuVelocityY = 0.0f;
        imuMoveActive = false;
        imuFilterValid = false;
        return;
    }

    // True radial joystick: the gravity tilt vector is the stick position.
    // On the Cardputer-Adv the BMI270's X axis runs along the device's
    // left/right direction and its Y axis along forward/back.  The stick
    // deflection (ax/az, ay/az) is tan() of the tilt angle, so a radial
    // deadzone and a radial magnitude ramp give uniform response in every
    // direction -- push diagonally and the cursor moves diagonally.
    float stickX = accSmX / accSmZ * imuFlipX;
    float stickY = accSmY / accSmZ * imuFlipY;
    if (imuSwapXY) {
        const float tmp = stickX;
        stickX = stickY;
        stickY = tmp;
    }
    const float magnitude = sqrtf(stickX * stickX + stickY * stickY);
    if (imuMoveActive) {
        if (magnitude < imuReleaseDeadzone) imuMoveActive = false;
    } else if (magnitude > imuEngageDeadzone) {
        imuMoveActive = true;
    }
    if (!imuMoveActive) {
        mouseAccX = 0.0f;
        mouseAccY = 0.0f;
        imuVelocityX = 0.0f;
        imuVelocityY = 0.0f;
        return;
    }

    float velocity =
        (magnitude - imuEngageDeadzone) / (imuMaxTilt - imuEngageDeadzone);
    if (velocity < 0.0f) velocity = 0.0f;
    if (velocity > 1.0f) velocity = 1.0f;
    // Keep a useful response immediately after the deadzone while preserving
    // the full-speed endpoint. This is stronger than the old quadratic curve
    // at small and medium tilts without adding a powf() call.
    velocity *= 0.5f + 0.5f * velocity;
    const float nx = stickX / magnitude;
    const float ny = stickY / magnitude;
    const float targetX = nx * velocity;
    const float targetY = ny * velocity;
    imuVelocityX += (targetX - imuVelocityX) * IMU_VELOCITY_ALPHA;
    imuVelocityY += (targetY - imuVelocityY) * IMU_VELOCITY_ALPHA;
    // imuSensitivity is the cursor speed (px/s) at full tilt. dt is in
    // milliseconds; fractional pixels accumulate for smooth low-speed motion.
    mouseAccX += imuVelocityX * imuSensitivity * dt / 1000.0f;
    mouseAccY += imuVelocityY * imuSensitivity * dt / 1000.0f;
    if (mouseAccX > 200.0f) mouseAccX = 200.0f;
    if (mouseAccX < -200.0f) mouseAccX = -200.0f;
    if (mouseAccY > 200.0f) mouseAccY = 200.0f;
    if (mouseAccY < -200.0f) mouseAccY = -200.0f;
    int dx = static_cast<int>(mouseAccX);
    mouseAccX -= static_cast<float>(dx);
    int dy = static_cast<int>(mouseAccY);
    mouseAccY -= static_cast<float>(dy);
    dx = constrain(dx, -IMU_MAX_DELTA, IMU_MAX_DELTA);
    dy = constrain(dy, -IMU_MAX_DELTA, IMU_MAX_DELTA);
    if (dx != 0 || dy != 0) {
        ++mouseMoveCount;
        mouseMove(dx, dy, mouseButton ? 1 : 0);
    }
}

void pollButton() {
    // M5Unified does not register G0/GPIO0 as a button for the Cardputer-Adv,
    // so read the physical key directly (active low, pull-up held by the
    // board).  Two consecutive equal reads (~4ms) debounce it.
    const bool pressed = digitalRead(MOUSE_BUTTON_PIN) == LOW;
    static uint8_t stableReads = 0;
    if (pressed == mouseButton) {
        stableReads = 0;
        return;
    }
    if (++stableReads < 2) return;
    stableReads = 0;
    mouseButton = pressed;
    mouseMove(0, 0, pressed ? 1 : 0);
}

} // namespace

// Runtime-tunable IMU sensitivity (serial command "sens <value>").
float imuSensitivity = IMU_SENSITIVITY;
int imuFlipX = -1;
int imuFlipY = 1;
int imuSwapXY = 0;

void cardputerInputInit() {
    memset(keysDown, 0, sizeof(keysDown));
    memset(physicalMacScan, kNoKey, sizeof(physicalMacScan));
    mouseButton = false;
    lastMouseTick = millis();
    mouseAccX = 0.0f;
    mouseAccY = 0.0f;
    accSmX = 0.0f;
    accSmY = 0.0f;
    accSmZ = 0.0f;
    imuVelocityX = 0.0f;
    imuVelocityY = 0.0f;
    imuFilterValid = false;
    imuMoveActive = false;
    imuEngageDeadzone =
        tanf(IMU_ENGAGE_DEADZONE_DEG * 3.14159265f / 180.0f);
    imuReleaseDeadzone =
        tanf(IMU_RELEASE_DEADZONE_DEG * 3.14159265f / 180.0f);
    imuMaxTilt = tanf(IMU_MAX_TILT_DEG * 3.14159265f / 180.0f);
    imuReadFailCount = 0;
    imuLastReinitMs = 0;
    webComboStartMs = 0;
    webComboActive = false;
    webComboTriggered = false;
    fnDown = false;
    pendingKeyPress = kNoKey;
    tcaReady = false;
    tcaEventCount = 0;
    tcaReadFailures = 0;
    tcaLastInitMs = 0;
    const uint32_t now = millis();
    lastKeyboardPollMs = now - KEYBOARD_POLL_MS;
    lastImuPollMs = now - IMU_POLL_MS;
    pinMode(MOUSE_BUTTON_PIN, INPUT_PULLUP);
    initTca8418();
    float ax = 0, ay = 0, az = 0;
    forceBmi270PowerOn();
    delay(5);
    bool accOk = false;
    for (int attempt = 0; attempt < 4 && !accOk; ++attempt) {
        if (attempt > 0) {
            printf("IMU: init retry %d...\n", attempt);
            M5.Imu.begin(&M5.In_I2C, M5.getBoard());
            forceBmi270PowerOn();
            delay(20);
        }
        accOk = M5.Imu.isEnabled() && readBmi270Accel(&ax, &ay, &az);
    }
    printf("INPUT: IMU enabled=%d accel=%d (%.2f,%.2f,%.2f) g\n",
           M5.Imu.isEnabled() ? 1 : 0, accOk ? 1 : 0, ax, ay, az);
}

void cardputerInputPoll() {
    pollButton();
    const uint32_t now = millis();
    if (now - lastKeyboardPollMs >= KEYBOARD_POLL_MS) {
        lastKeyboardPollMs = now;
        pollKeyboard(true);
    }
    if (now - lastImuPollMs >= IMU_POLL_MS) {
        lastImuPollMs = now;
        pollMouse();
    }
}

bool cardputerInputAnyKeyPressed() {
    pollKeyboard(false);
    for (bool down : physicalKeysDown) {
        if (down) return true;
    }
    return false;
}

uint8_t cardputerInputReadKeyPress() {
    pollKeyboard(false);
    const uint8_t key = pendingKeyPress;
    pendingKeyPress = kNoKey;
    return key;
}

void cardputerInputImuStatus() {
    float ax = 0, ay = 0, az = 0;
    const bool ok = readBmi270Accel(&ax, &ay, &az);
    printf("IMU: enabled=%d read=%d accel=(%.3f,%.3f,%.3f) g fail=%lu last_reinit_ms=%lu\n",
           M5.Imu.isEnabled() ? 1 : 0, ok ? 1 : 0, ax, ay, az,
           static_cast<unsigned long>(imuReadFailCount),
           static_cast<unsigned long>(imuLastReinitMs));
}

void cardputerInputMouseStatus() {
    float ax = 0, ay = 0, az = 0;
    const bool ok = readBmi270Accel(&ax, &ay, &az);
    printf("MOUSE: polls=%lu moves=%lu dt=%lu active=%d acc=%.3f,%.3f,%.3f "
           "sm=%.3f,%.3f,%.3f vel=%.3f,%.3f accXY=%.2f,%.2f "
           "cursor=%u,%u read=%d\n",
           static_cast<unsigned long>(mousePollCount),
           static_cast<unsigned long>(mouseMoveCount),
           static_cast<unsigned long>(lastMouseDt),
           imuMoveActive ? 1 : 0,
           ax, ay, az, accSmX, accSmY, accSmZ, imuVelocityX, imuVelocityY,
           mouseAccX, mouseAccY,
           (unsigned)tmeGetMouseX(), (unsigned)tmeGetMouseY(), ok ? 1 : 0);
}

void cardputerInputKeysStatus() {
    uint8_t intStatus = 0;
    uint8_t eventCount = 0;
    const bool statusOk = tcaRead8(TCA8418_INT_STAT, &intStatus);
    const bool countOk = tcaRead8(TCA8418_KEY_LCK_EC, &eventCount);
    printf("KEYS: tca=%d i2c=%d/%d int=0x%02X fifo=%u events=%lu "
           "ctrl=%d opt=%d alt=%d shift=%d fn=%d\n",
           tcaReady ? 1 : 0, statusOk ? 1 : 0, countOk ? 1 : 0,
           intStatus, eventCount & 0x0F,
           static_cast<unsigned long>(tcaEventCount),
           keysDown[kMacCmd] ? 1 : 0, keysDown[kMacOption] ? 1 : 0,
           keysDown[kMacCtrl] ? 1 : 0, keysDown[kMacShift] ? 1 : 0,
           fnDown ? 1 : 0);
}
