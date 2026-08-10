#ifndef NCR_H
#define NCR_H
#include <stdint.h>

// The Mac Plus SCSI Manager caps transfers at 64KB.  System 3.x drivers
// issue reads larger than this window when loading bigger applications (e.g.
// Font/DA Mover asks for 37 sectors = 18944 bytes); ncr.c splits such
// transfers into dataCapacity-sized chunks, so an 8KB window is enough.  It
// is allocated from IRAM (main.cpp) to keep the internal-DRAM budget for the
// 256KB Mac RAM and SD card.
#ifndef SCSI_DATA_BUFFER_BYTES
#define SCSI_DATA_BUFFER_BYTES (16U * 512U)
#endif

// Reserved very early by main.cpp (before SD/M5 fragment the heap), so the
// DMA window survives the no-PSRAM SRAM budget.  NULL means ncrInit()
// falls back to allocating late (which may fail once the heap is fragmented).
extern uint8_t *scsiDataBuffer;

typedef struct {
	uint8_t cmd[256];
	uint8_t *data;
	uint32_t dataCapacity;
	uint8_t msg[128];
	int cmdlen;
	int datalen;
	int msglen;
} SCSITransferData;

typedef struct {
	int (*scsiCmd)(SCSITransferData *data, unsigned int cmd, unsigned int len, unsigned int lba, void *arg);
	void *arg;
} SCSIDevice;


void ncrInit();
void ncrRegisterDevice(int id, SCSIDevice* dev);
uint8_t ncrGetDeviceMask(void);
unsigned int ncrRead(unsigned int addr, unsigned int dack);
void ncrWrite(unsigned int addr,unsigned int dack, unsigned int val);

#endif
