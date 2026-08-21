#include <math.h>

#include "nvs.h"
#include "input.h"
#include "settings.h"
extern "C" {
#include "tme/snd.h"
}

namespace {

constexpr const char *kNamespace = "macplus";
constexpr const char *kPointerSpeedKey = "ptr_speed_pct";
constexpr const char *kLegacySensitivityKey = "imu_sens10";
constexpr const char *kVolumeKey = "volume";

static uint16_t clampPointerSpeedPercent(uint32_t value) {
    if (value < IMU_POINTER_SPEED_MIN_PERCENT) {
        return IMU_POINTER_SPEED_MIN_PERCENT;
    }
    if (value > IMU_POINTER_SPEED_MAX_PERCENT) {
        return IMU_POINTER_SPEED_MAX_PERCENT;
    }
    return static_cast<uint16_t>(value);
}

}  // namespace

void loadMacSettings(void) {
    nvs_handle_t handle;
    if (nvs_open(kNamespace, NVS_READONLY, &handle) != ESP_OK) return;

    uint16_t pointerSpeedPercent = 0;
    if (nvs_get_u16(handle, kPointerSpeedKey, &pointerSpeedPercent) == ESP_OK) {
        imuPointerSpeedPercent =
            clampPointerSpeedPercent(pointerSpeedPercent);
    } else {
        // Migrate the previous absolute full-tilt speed. 320 px/s was the
        // default and therefore maps to the new 100% whole-curve multiplier.
        uint32_t legacySensitivity10 = 0;
        if (nvs_get_u32(handle, kLegacySensitivityKey,
                        &legacySensitivity10) == ESP_OK) {
            const uint32_t percent = static_cast<uint32_t>(lroundf(
                (static_cast<float>(legacySensitivity10) / 10.0f) * 100.0f /
                IMU_BASE_CURSOR_SPEED));
            imuPointerSpeedPercent = clampPointerSpeedPercent(percent);
        }
    }

    uint8_t volume = 0;
    if (nvs_get_u8(handle, kVolumeKey, &volume) == ESP_OK) {
        sndSetVolume(volume);
    }
    nvs_close(handle);
}

bool saveMacSettings(uint16_t pointerSpeedPercent, uint8_t volume) {
    pointerSpeedPercent = clampPointerSpeedPercent(pointerSpeedPercent);

    nvs_handle_t handle;
    if (nvs_open(kNamespace, NVS_READWRITE, &handle) != ESP_OK) return false;
    esp_err_t error =
        nvs_set_u16(handle, kPointerSpeedKey, pointerSpeedPercent);
    if (error == ESP_OK) error = nvs_set_u8(handle, kVolumeKey, volume);
    if (error == ESP_OK) error = nvs_commit(handle);
    nvs_close(handle);
    if (error != ESP_OK) return false;

    imuPointerSpeedPercent = pointerSpeedPercent;
    sndSetVolume(volume);
    return true;
}
