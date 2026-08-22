#include "debug_log.h"
#include <Arduino.h>
#include "esp_attr.h"

#include "webinstall.h"

extern "C" {
#include "tme/hd.h"
#include "tme/snd.h"
}

namespace {

constexpr uint32_t WEB_MODE_MAGIC = 0x57494649U; // "WIFI"
RTC_NOINIT_ATTR uint32_t webModeMagic;
RTC_NOINIT_ATTR uint32_t webModeMagicInverse;

void setWebMode(bool enabled) {
    webModeMagic = enabled ? WEB_MODE_MAGIC : 0;
    webModeMagicInverse = enabled ? ~WEB_MODE_MAGIC : 0;
}

} // namespace

extern "C" int webInstallModeRequested() {
    const bool requested = webModeMagic == WEB_MODE_MAGIC &&
                           webModeMagicInverse == ~WEB_MODE_MAGIC;
    if (requested) setWebMode(false);
    return requested ? 1 : 0;
}

extern "C" void requestWebInstallMode() {
    MACPLUS_LOG("WEB: rebooting into integrated transfer mode\n");
    sndPrepareForRestart();
    hdFlushNow();
    setWebMode(true);
    delay(100);
    ESP.restart();
}

extern "C" void exitWebInstallMode() {
    MACPLUS_LOG("WEB: leaving transfer mode\n");
    sndPrepareForRestart();
    setWebMode(false);
    delay(100);
    ESP.restart();
}
