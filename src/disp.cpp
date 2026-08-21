#include "debug_log.h"
/*
 * Display driver for the Cardputer-Adv port of the Mac Plus emulator.
 *
 * The emulated Mac framebuffer is 512x342, 1 bit per pixel.  The panel is a
 * 240x135 ST7789 driven by M5GFX.  Instead of shrinking the whole desktop
 * (which made text unreadable), the panel shows a VIEW_W x VIEW_H crop of
 * the Mac screen at (near-)native resolution, and the crop pans to follow
 * the Mac mouse cursor so the area around the pointer is always visible.
 * Rendering runs on core 1 from the existing control task, so SPI traffic
 * never stalls the 68000 emulation on core 0 and no full RGB frame is needed.
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <Arduino.h>
#include <M5Cardputer.h>
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hardware_config.h"

extern "C" {
#include "tme/disp.h"
#include "tme/emu.h"
}

#define MAC_FB_W 512
#define MAC_FB_H 342
#define MAC_FB_STRIDE (MAC_FB_W / 8)
#define MAC_FB_BYTES (MAC_FB_STRIDE * MAC_FB_H)

#ifndef VIEW_SCALE
#define VIEW_SCALE 1
#endif
#define IMG_W CARD_DISP_W
#define IMG_H CARD_DISP_H
#define IMG_X 0
#define IMG_Y 0
// Crop size in Mac pixels; the panel shows it 1:1 (or VIEW_SCALE x).
#define VIEW_W (MAC_FB_W >= CARD_DISP_W * VIEW_SCALE ? CARD_DISP_W * VIEW_SCALE : MAC_FB_W)
#define VIEW_H (MAC_FB_H >= CARD_DISP_H * VIEW_SCALE ? CARD_DISP_H * VIEW_SCALE : MAC_FB_H)
static_assert(VIEW_SCALE == 1, "optimized display path requires VIEW_SCALE=1");

#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF

// 27 strips fit in the 32-bit dirty mask.
#define STRIP_ROWS 5
#define DISP_STRIP_COUNT ((CARD_DISP_H + STRIP_ROWS - 1) / STRIP_ROWS)
static_assert(DISP_STRIP_COUNT <= 32, "dirty mask too small");
static constexpr size_t CROP_ROW_BYTES = VIEW_W / 8;
static constexpr size_t CROP_SHADOW_BYTES = CROP_ROW_BYTES * VIEW_H;
static constexpr size_t DISPLAY_BUFFER_BYTES = CROP_SHADOW_BYTES;
static constexpr uint32_t PANEL_SPI_HZ = 80000000;
static uint8_t *displayBuffers = nullptr;

enum class CropFrameState : uint8_t {
    Free,
    Writing,
    Pending,
    Displaying,
};

struct CropFrame {
    uint8_t *pixels;
    uint32_t dirtyMask;
    CropFrameState state;
};

static CropFrame cropFrame = {};
static portMUX_TYPE displayStateMux = portMUX_INITIALIZER_UNLOCKED;
static volatile int viewX = 0;
static volatile int viewY = 0;
static volatile bool hasFrame = false;
static int lastViewX = 0;
static int lastViewY = 0;
static const lgfx::bgr888_t monoPalette[2] = {
    lgfx::bgr888_t(255, 255, 255),
    lgfx::bgr888_t(0, 0, 0),
};

enum class ConsoleMode : uint8_t {
    None,
    Message,
    Progress,
};

static ConsoleMode consoleMode = ConsoleMode::None;
static int consoleMessageLines = 0;

static void clearConsoleLine(int y) {
    M5.Display.fillRect(0, y, CARD_DISP_W, 14, COLOR_BLACK);
}

static void printConsoleLine(int y, uint16_t color, const char *prefix,
                             const char *text) {
    M5.Display.setTextColor(color, COLOR_BLACK);
    M5.Display.setCursor(6, y);
    M5.Display.print(prefix);
    M5.Display.print(text != nullptr ? text : "");
    // Erase any tail left by a longer previous status without clearing the
    // whole panel. The terminal lines fit within this padding at font size 1.
    M5.Display.print("                                        ");
}

// Copy VIEW_W bits starting at bit srcBit of srcRow into a byte-aligned
// crop row (MSB-first, same layout as the Mac framebuffer).
static void copyCropRow(uint8_t *dst, const uint8_t *srcRow, int srcBit) {
    const int byteOffset = srcBit >> 3;
    const int shift = srcBit & 7;
    if (shift == 0) {
        memcpy(dst, srcRow + byteOffset, CROP_ROW_BYTES);
        return;
    }
    for (size_t i = 0; i < CROP_ROW_BYTES; ++i) {
        const int currentByte = byteOffset + static_cast<int>(i);
        const int nextByte = byteOffset + static_cast<int>(i) + 1;
        const uint8_t current = currentByte < MAC_FB_STRIDE
            ? srcRow[currentByte] : 0;
        const uint8_t carry = nextByte < MAC_FB_STRIDE ? srcRow[nextByte] : 0;
        dst[i] = static_cast<uint8_t>(
            (current << shift) |
            (carry >> (8 - shift)));
    }
}

static bool takePendingFrame() {
    bool pending = false;
    portENTER_CRITICAL(&displayStateMux);
    if (cropFrame.state == CropFrameState::Pending) {
        cropFrame.state = CropFrameState::Displaying;
        pending = true;
    }
    portEXIT_CRITICAL(&displayStateMux);
    return pending;
}

static void releaseFrame() {
    portENTER_CRITICAL(&displayStateMux);
    cropFrame.state = CropFrameState::Free;
    portEXIT_CRITICAL(&displayStateMux);
}

static void renderFrame(const CropFrame &frame) {
    uint32_t dirty = frame.dirtyMask;
    int strip = 0;
    while (strip < DISP_STRIP_COUNT) {
        if ((dirty & (1U << strip)) == 0) {
            ++strip;
            continue;
        }
        const int firstStrip = strip;
        while (strip < DISP_STRIP_COUNT && (dirty & (1U << strip)) != 0) {
            ++strip;
        }
        const int y = firstStrip * STRIP_ROWS;
        int rows = (strip - firstStrip) * STRIP_ROWS;
        if (y + rows > CARD_DISP_H) rows = CARD_DISP_H - y;
        M5.Display.pushImageDMA(
            0, y, CARD_DISP_W, rows,
            frame.pixels + static_cast<size_t>(y) * CROP_ROW_BYTES,
            lgfx::palette_1bit, monoPalette);
    }
    auto *panel = M5.Display.getPanel();
    if (panel != nullptr && panel->getBus() != nullptr) {
        panel->getBus()->wait();
    }
}

void dispService() {
    if (displayBuffers == nullptr) return;
    if (!takePendingFrame()) return;
    renderFrame(cropFrame);
    releaseFrame();
}

void dispInit() {
    MACPLUS_LOG("DISP: native %dx%d, panel %dx%d, view %dx%d @%dx, pan follows mouse\n",
           MAC_FB_W, MAC_FB_H, CARD_DISP_W, CARD_DISP_H, VIEW_W, VIEW_H,
           VIEW_SCALE);
    if (displayBuffers == nullptr) {
        // The crop shadow is sent to the panel by DMA.  Allocate it while the
        // heap is still compact and prefer DMA-capable 8-bit RAM; the old
        // EXEC-only request could fail after the emulator reserved its RAM.
        displayBuffers = static_cast<uint8_t *>(heap_caps_malloc(
            DISPLAY_BUFFER_BYTES,
            MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
        if (displayBuffers == nullptr) {
            displayBuffers = static_cast<uint8_t *>(heap_caps_malloc(
                DISPLAY_BUFFER_BYTES, MALLOC_CAP_8BIT));
        }
        if (displayBuffers == nullptr) {
            displayBuffers = static_cast<uint8_t *>(malloc(DISPLAY_BUFFER_BYTES));
        }
        if (displayBuffers != nullptr) {
            memset(displayBuffers, 0, DISPLAY_BUFFER_BYTES);
            cropFrame.pixels = displayBuffers;
            cropFrame.dirtyMask = 0;
            cropFrame.state = CropFrameState::Free;
        }
    }
    MACPLUS_LOG("DISP: dynamic buffers %u bytes at %p\n",
           static_cast<unsigned int>(DISPLAY_BUFFER_BYTES), displayBuffers);
    M5.Display.fillScreen(COLOR_BLACK);
    if (displayBuffers == nullptr) return;
    auto *panel = M5.Display.getPanel();
    if (panel != nullptr && panel->getBus() != nullptr) {
        const uint32_t oldClock = panel->getBus()->getClock();
        panel->getBus()->setClock(PANEL_SPI_HZ);
        MACPLUS_LOG("DISP: SPI clock %lu -> %lu Hz\n",
               static_cast<unsigned long>(oldClock),
               static_cast<unsigned long>(panel->getBus()->getClock()));
    }
    MACPLUS_LOG("DISP: control-task service ready (free heap=%d)\n",
           (int)esp_get_free_heap_size());
    consoleMode = ConsoleMode::None;
    consoleMessageLines = 0;
}

void dispDraw(uint8_t *mem) {
    if (mem == nullptr || displayBuffers == nullptr) return;
    // Edge-follow panning: the crop stays still while the cursor is inside
    // the margin; when the cursor reaches the margin the crop eases after it,
    // so the view moves smoothly instead of jumping with every cursor step.
    int mx = tmeGetMouseX();
    int my = tmeGetMouseY();
    const int marginX = VIEW_W / 4;
    const int marginY = VIEW_H / 4;
    int desiredX = viewX;
    int desiredY = viewY;
    if (mx < viewX + marginX) desiredX = mx - marginX;
    else if (mx >= viewX + VIEW_W - marginX)
        desiredX = mx - (VIEW_W - marginX);
    if (my < viewY + marginY) desiredY = my - marginY;
    else if (my >= viewY + VIEW_H - marginY)
        desiredY = my - (VIEW_H - marginY);
    if (desiredX < 0) desiredX = 0;
    if (desiredY < 0) desiredY = 0;
    if (desiredX > MAC_FB_W - VIEW_W) desiredX = MAC_FB_W - VIEW_W;
    if (desiredY > MAC_FB_H - VIEW_H) desiredY = MAC_FB_H - VIEW_H;

    // Ease about half of the way per frame, always at least 1px in range.
    auto ease = [](int current, int desired) {
        const int delta = desired - current;
        if (delta > 0) return current + (delta + 1) / 2;
        if (delta < 0) return current - ((-delta + 1) / 2);
        return current;
    };
    viewX = ease(viewX, desiredX);
    viewY = ease(viewY, desiredY);

    bool writable = false;
    portENTER_CRITICAL(&displayStateMux);
    if (cropFrame.state == CropFrameState::Free) {
        cropFrame.state = CropFrameState::Writing;
        writable = true;
    }
    portEXIT_CRITICAL(&displayStateMux);
    if (!writable) {
        return;
    }

    // Assemble the visible crop, diff it against the previous snapshot and
    // mark only changed panel strips as dirty.
    const bool firstFrame = !hasFrame;
    const bool viewMoved = firstFrame || viewX != lastViewX || viewY != lastViewY;
    lastViewX = viewX;
    lastViewY = viewY;
    uint32_t dirty = viewMoved ? ((1U << DISP_STRIP_COUNT) - 1U) : 0U;
    uint8_t *writePixels = cropFrame.pixels;
    uint8_t newRow[CROP_ROW_BYTES];
    for (int py = 0; py < VIEW_H; ++py) {
        const uint8_t *srcRow = mem + (viewY + py) * MAC_FB_STRIDE;
        uint8_t *dstRow = writePixels + py * CROP_ROW_BYTES;
        copyCropRow(newRow, srcRow, viewX);
        if (!viewMoved &&
            memcmp(newRow, dstRow, CROP_ROW_BYTES) != 0) {
            dirty |= (1U << (py / STRIP_ROWS));
        }
        memcpy(dstRow, newRow, CROP_ROW_BYTES);
    }

    if (dirty == 0) {
        releaseFrame();
        return;
    }
    portENTER_CRITICAL(&displayStateMux);
    cropFrame.dirtyMask = dirty;
    cropFrame.state = CropFrameState::Pending;
    hasFrame = true;
    portEXIT_CRITICAL(&displayStateMux);
}

// Draw boot and transfer status as a compact terminal-style screen. M5GFX's
// built-in bitmap font follows the panel rotation and avoids custom bit-order
// handling, which previously mirrored every glyph.
void dispShowMessage(const char *lines[], int nlines) {
    if (nlines <= 0) return;
    if (consoleMode == ConsoleMode::Progress) {
        static constexpr int BAR_Y = 59;
        static constexpr int BAR_H = 16;
        M5.Display.fillRect(0, BAR_Y, CARD_DISP_W, BAR_H, COLOR_BLACK);
        clearConsoleLine(88);
    } else if (consoleMode == ConsoleMode::Message &&
               consoleMessageLines > nlines) {
        for (int i = nlines; i < consoleMessageLines; ++i) {
            clearConsoleLine(6 + i * 15);
        }
    }
    M5.Display.setTextDatum(textdatum_t::top_left);
    M5.Display.setTextSize(1);
    M5.Display.setTextWrap(false);
    for (int i = 0; i < nlines; ++i) {
        const char *text = lines[i] != nullptr ? lines[i] : "";
        uint16_t color = COLOR_WHITE;
        if (strstr(text, "OK") != nullptr || strstr(text, "READY") != nullptr) {
            color = 0x07E0;
        } else if (strstr(text, "FAIL") != nullptr ||
                   strstr(text, "ERROR") != nullptr) {
            color = 0xF800;
        } else if (i == 0) {
            color = 0x07FF;
        }
        printConsoleLine(6 + i * 15, color, i == 0 ? "$ " : "> ", text);
    }
    consoleMessageLines = nlines;
    consoleMode = ConsoleMode::Message;
}

void dispShowProgress(const char *title, const char *status,
                      const char *footer, uint32_t currentBytes,
                      uint32_t totalBytes) {
    const uint32_t percent = totalBytes == 0
        ? 0 : min<uint32_t>(100U, static_cast<uint32_t>(
              static_cast<uint64_t>(currentBytes) * 100U / totalBytes));
    char progress[24];
    snprintf(progress, sizeof(progress), "PROGRESS %3lu%%",
             static_cast<unsigned long>(percent));

    if (consoleMode == ConsoleMode::Message) {
        for (int i = 0; i < consoleMessageLines; ++i) {
            clearConsoleLine(6 + i * 15);
        }
    }
    M5.Display.setTextDatum(textdatum_t::top_left);
    M5.Display.setTextSize(1);
    M5.Display.setTextWrap(false);

    printConsoleLine(6, 0x07FF, "$ ", title);
    printConsoleLine(23, COLOR_WHITE, "> ", status);
    printConsoleLine(42, COLOR_WHITE, "> ", progress);

    static constexpr int BAR_X = 6;
    static constexpr int BAR_Y = 59;
    static constexpr int BAR_W = CARD_DISP_W - 12;
    static constexpr int BAR_H = 16;
    const int fillWidth = static_cast<int>(
        static_cast<uint32_t>(BAR_W - 6) * percent / 100U);
    M5.Display.fillRect(BAR_X, BAR_Y, BAR_W, BAR_H, COLOR_BLACK);
    M5.Display.drawRect(BAR_X, BAR_Y, BAR_W, BAR_H, COLOR_WHITE);
    M5.Display.drawRect(BAR_X + 2, BAR_Y + 2, BAR_W - 4, BAR_H - 4,
                        COLOR_WHITE);
    if (fillWidth > 0) {
        M5.Display.fillRect(BAR_X + 3, BAR_Y + 3, fillWidth, BAR_H - 6,
                            0x07E0);
    }

    printConsoleLine(88, COLOR_WHITE, "> ", footer);
    consoleMessageLines = 0;
    consoleMode = ConsoleMode::Progress;
}

void dispShowHdCacheProgress(uint32_t copiedBytes, uint32_t totalBytes) {
    if (totalBytes != 0 && copiedBytes >= totalBytes) {
        const char *lines[] = {
            "HD CACHE",
            "[OK] COPY COMPLETE",
            "STARTING MAC...",
        };
        dispShowMessage(lines, 3);
        return;
    }

    dispShowProgress("HD CACHE INIT", "[RUN] COPYING IMAGE",
                     "DO NOT POWER OFF", copiedBytes, totalBytes);
}
