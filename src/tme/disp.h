#include <stdint.h>

void dispInit();
void dispService();
void dispDraw(uint8_t *mem);
void dispShowMessage(const char *lines[], int nlines);
void dispShowProgress(const char *title, const char *status,
                      const char *footer, uint32_t currentBytes,
                      uint32_t totalBytes);
void dispShowHdCacheProgress(uint32_t copiedBytes, uint32_t totalBytes);
uint32_t dispGetFrameCount(void);
uint32_t dispGetRequestCount(void);
uint32_t dispGetAliveTicks(void);
uint32_t dispGetDroppedFrameCount(void);
uint32_t dispGetDirtyFrameCount(void);
uint32_t dispGetFullFrameCount(void);
uint32_t dispGetLastDirtyArea(void);
uint32_t dispGetLastRenderUs(void);
uint32_t dispGetLastAcquireUs(void);
uint32_t dispGetLastPresentUs(void);
