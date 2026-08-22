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
constexpr const char *kVolumePercentKey = "volume_pct";
constexpr const char *kLegacyVolumeKey = "volume";

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

    uint8_t volumePercent = 0;
    if (nvs_get_u8(handle, kVolumePercentKey, &volumePercent) == ESP_OK) {
        sndSetVolume(volumePercent);
    } else {
        // Older builds stored M5Unified's raw 0..255 master value and the
        // web menu displayed it linearly. Convert that legacy value once.
        uint8_t legacyVolume = 0;
        if (nvs_get_u8(handle, kLegacyVolumeKey, &legacyVolume) == ESP_OK) {
            sndSetVolume(static_cast<uint8_t>(
                (static_cast<unsigned>(legacyVolume) * 100U + 127U) / 255U));
        }
    }
    nvs_close(handle);
}

bool saveMacSettings(uint16_t pointerSpeedPercent, uint8_t volumePercent) {
    pointerSpeedPercent = clampPointerSpeedPercent(pointerSpeedPercent);

    nvs_handle_t handle;
    if (nvs_open(kNamespace, NVS_READWRITE, &handle) != ESP_OK) return false;
    esp_err_t error =
        nvs_set_u16(handle, kPointerSpeedKey, pointerSpeedPercent);
    if (error == ESP_OK) {
        if (volumePercent < 1U) volumePercent = 1U;
        if (volumePercent > 100U) volumePercent = 100U;
        error = nvs_set_u8(handle, kVolumePercentKey, volumePercent);
    }
    if (error == ESP_OK) error = nvs_commit(handle);
    nvs_close(handle);
    if (error != ESP_OK) return false;

    imuPointerSpeedPercent = pointerSpeedPercent;
    sndSetVolume(volumePercent);
    return true;
}
