#include <M5Cardputer.h>

#include <stdint.h>
#include <string.h>

extern "C" {
#include "tme/snd.h"
}

namespace {

constexpr size_t kSamplesPerFrame = 370;
constexpr uint32_t kSampleRate = 22200;
constexpr uint8_t kSpeakerVolume = 38; // 60% lower than the original default
constexpr uint8_t kAudioChannel = 0;

uint8_t audioBuffers[3][kSamplesPerFrame];
size_t nextBuffer = 0;
bool speakerReady = false;

}  // namespace

void sndInit() {
    memset(audioBuffers, 128, sizeof(audioBuffers));

    auto config = M5.Speaker.config();
    config.sample_rate = kSampleRate;
    config.dma_buf_len = 64;
    config.dma_buf_count = 2;
    // The emulator owns core 0. M5Unified's software mixer continuously
    // feeds I2S, so keeping it there can preempt the 68000 task below real
    // time. Core 1 already hosts the low-duty display/input service.
    config.task_priority = 1;
    config.task_pinned_core = 1;
    M5.Speaker.config(config);
    M5.Speaker.setVolume(kSpeakerVolume);
    speakerReady = M5.Speaker.begin();
    printf("AUDIO: Cardputer speaker %s (%lu Hz, 8-bit mono)\n",
           speakerReady ? "ready" : "unavailable",
           static_cast<unsigned long>(kSampleRate));
}

int sndPush(uint8_t *data, int volume) {
    if (!speakerReady || data == nullptr) return 0;

    uint8_t *output = audioBuffers[nextBuffer];
    if (volume <= 0) {
        memset(output, 128, kSamplesPerFrame);
    } else {
        if (volume > 7) volume = 7;
        const int shift = 7 - volume;
        for (size_t i = 0; i < kSamplesPerFrame; ++i) {
            const int sample = (static_cast<int>(data[i * 2]) - 128) >> shift;
            output[i] = static_cast<uint8_t>(sample + 128);
        }
    }

    // Mac supplies exactly one 370-sample block per 60 Hz video frame. Do not
    // wait for the previous block here: M5Unified otherwise blocks the 68000
    // task until its mixer finishes, which makes emulation run below 1x.
    if (!M5.Speaker.playRaw(output, kSamplesPerFrame, kSampleRate, false,
                            1, kAudioChannel, true)) {
        return 0;
    }

    nextBuffer = (nextBuffer + 1) % 3;
    return 1;
}
