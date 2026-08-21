#include "debug_log.h"
#include <M5Cardputer.h>

#include <stdint.h>
#include <string.h>

extern "C" {
#include "tme/snd.h"
}

namespace {

constexpr size_t kSamplesPerFrame = 370;
constexpr uint32_t kSampleRate = 22200;
// M5Unified's master volume is an 8-bit value (0..255).  90 is approximately
// 35% of full scale and is applied after begin(), when the board codec/I2S
// configuration is finalized.
constexpr uint8_t kSpeakerVolume = 90;
constexpr uint8_t kAudioChannel = 0;

// Cardputer Adv audio path: ESP32-S3 I2S1 -> ES8311 -> NS4150B.
// The board's NS4150B CTRL pin is hard-wired high, so the codec is the only
// software-controlled mute point available to the firmware.
constexpr uint8_t kEs8311Address = 0x18;
constexpr uint32_t kEs8311I2cFrequency = 100000;
constexpr uint8_t kEs8311DacMute = 0x60;  // DAC mute bits 6:5 in REG31.
constexpr uint8_t kEs8311DacVolume0dB = 0xBF;

uint8_t audioBuffers[3][kSamplesPerFrame];
size_t nextBuffer = 0;
bool speakerReady = false;
uint8_t speakerVolume = kSpeakerVolume;

static bool writeEs8311(uint8_t reg, uint8_t value) {
    if (!M5.In_I2C.isEnabled()) return false;
    // The keyboard and IMU share this bus. A short retry handles the rare
    // case where another peripheral just released the bus.
    for (int attempt = 0; attempt < 3; ++attempt) {
        if (M5.In_I2C.writeRegister8(kEs8311Address, reg, value,
                                     kEs8311I2cFrequency)) {
            return true;
        }
        delay(1);
    }
    return false;
}

static bool prepareEs8311Muted(void) {
    // Keep the output muted and the DAC at its lowest gain while the I2S
    // peripheral is brought up. This avoids the stock M5Unified callback's
    // unmuted analog-power transition at boot.
    static constexpr uint8_t init_data[][2] = {
        {0x00, 0x80},  // CSM power on
        {0x31, kEs8311DacMute},
        {0x32, 0x00},  // minimum DAC volume
        {0x01, 0xB5},  // MCLK from BCLK, enable codec clocks
        {0x02, 0x18},  // clock multiplier used by Cardputer
        {0x0D, 0x01},  // power up analog circuitry
        {0x12, 0x00},  // power up DAC
        {0x13, 0x00},  // keep headphone/output driver disabled for now
        {0x37, 0x08},  // bypass DAC equalizer (M5Unified configuration)
    };

    bool ok = true;
    for (const auto &entry : init_data) {
        ok = writeEs8311(entry[0], entry[1]) && ok;
    }
    return ok;
}

static bool enableEs8311Quietly(void) {
    bool ok = writeEs8311(0x13, 0x10);  // enable output driver while muted
    delay(5);

    // Ramp the codec gain while its DAC mute is still asserted. The I2S
    // task is already emitting zero PCM, so unmuting after this ramp leaves
    // the cone at its normal zero-signal bias instead of stepping it.
    for (uint16_t value = 0; value < kEs8311DacVolume0dB; value += 0x10) {
        ok = writeEs8311(0x32, static_cast<uint8_t>(value)) && ok;
        delay(1);
    }
    ok = writeEs8311(0x32, kEs8311DacVolume0dB) && ok;
    delay(2);
    ok = writeEs8311(0x31, 0x00) && ok;  // release DAC mute
    delay(2);
    return ok;
}

}  // namespace

void sndInit() {
    memset(audioBuffers, 128, sizeof(audioBuffers));

    auto config = M5.Speaker.config();
    // M5Cardputer.begin() is called with internal_spk=false. Configure the
    // known Cardputer Adv pins here so M5Unified never runs its abrupt codec
    // enable callback during Speaker.begin().
    config.pin_bck = GPIO_NUM_41;
    config.pin_ws = GPIO_NUM_43;
    config.pin_data_out = GPIO_NUM_42;
    config.i2s_port = I2S_NUM_1;
    config.stereo = false;
    config.buzzer = false;
    config.use_dac = false;
    config.dac_zero_level = 0;
    config.magnification = 16;
    config.sample_rate = kSampleRate;
    config.dma_buf_len = 64;
    config.dma_buf_count = 2;
    // The emulator owns core 0. M5Unified's software mixer continuously
    // feeds I2S, so keeping it there can preempt the 68000 task below real
    // time. Core 1 already hosts the low-duty display/input service.
    config.task_priority = 1;
    config.task_pinned_core = 1;
    M5.Speaker.config(config);
    const bool codecReady = prepareEs8311Muted();

    // Keep the software mixer silent until I2S has been running zero PCM and
    // the codec output has been enabled/muted in a controlled order.
    M5.Speaker.setVolume(0);
    const bool i2sReady = M5.Speaker.begin();
    bool outputReady = false;
    if (codecReady && i2sReady) {
        delay(5);
        outputReady = enableEs8311Quietly();
    }
    if (!outputReady) {
        // Fail closed if a late I2C write failed part-way through the ramp.
        writeEs8311(0x31, kEs8311DacMute);
        writeEs8311(0x13, 0x00);
    }
    speakerReady = i2sReady && codecReady && outputReady;
    if (speakerReady) {
        M5.Speaker.setVolume(speakerVolume);
    } else {
        // Leave the digital mixer silent if either the codec or I2S failed.
        M5.Speaker.setVolume(0);
    }
    MACPLUS_LOG("AUDIO: Cardputer speaker %s (%lu Hz, volume=%u, codec=%s)\n",
           speakerReady ? "ready" : "unavailable",
           static_cast<unsigned long>(kSampleRate),
           static_cast<unsigned>(speakerVolume),
           (codecReady && outputReady) ? "ready" : "unavailable");
}

void sndSetVolume(uint8_t volume) {
    speakerVolume = volume;
    if (speakerReady) M5.Speaker.setVolume(volume);
}

uint8_t sndGetVolume(void) {
    return speakerVolume;
}

int sndPush(uint8_t *data, int volume) {
    if (!speakerReady || data == nullptr) return 0;

    uint8_t *output = audioBuffers[nextBuffer];
    if (volume <= 0) {
        // Unsigned 8-bit PCM uses 0x80 as zero amplitude. Keep feeding this
        // level so the codec and amplifier remain at a defined idle bias.
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
