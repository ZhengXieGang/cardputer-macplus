#include <stdint.h>

void dispInit();
void dispService();
void dispDraw(uint8_t *mem);
void dispShowMessage(const char *lines[], int nlines);
void dispShowProgress(const char *title, const char *status,
                      const char *footer, uint32_t currentBytes,
                      uint32_t totalBytes);
void dispShowHdCacheProgress(uint32_t copiedBytes, uint32_t totalBytes);
