/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Jeroen Domburg <jeroen@spritesmods.com> wrote this file. As long as you retain 
 * this notice you can do whatever you want with this stuff. If we meet some day, 
 * and you think this stuff is worth it, you can buy me a beer in return. 
 * ----------------------------------------------------------------------------
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "ncr.h"
#include "m68k.h"

static const char* const regNamesR[]={
	"CURSCSIDATA","INITIATORCMD", "MODE", "TARGETCMD", "CURSCSISTATUS",
	"BUSANDSTATUS", "INPUTDATA", "RESETPARINT"
};

static const char* const regNamesW[]={
	"OUTDATA","INITIATORCMD", "MODE", "TARGETCMD", "SELECTENA",
	"STARTDMASEND", "STARTDMATARRECV", "STARTDMAINITRECV"
};


typedef struct {
	SCSIDevice *dev[8];
	uint8_t mode;
	uint8_t tcr;
	uint8_t dout;
	uint8_t din;
	uint8_t inicmd;
	int selected;
	int state;
	uint8_t tcrforbuf;
	SCSITransferData data;
	uint8_t *buf;
	int bufmax;
	int bufpos;
	int datalen;
	int chunkActive;
	int chunkRemaining;
	int chunkCmd;
	uint32_t chunkLba;
	uint8_t writeActive;
	uint8_t writeFailed;
	int writeCmd;
	uint32_t writeLba;
	uint32_t writeRemaining;
	uint8_t dmaActive;
} Ncr;

#define INIR_AIP (1<<6)
#define INIR_LA (1<<5)
#define INI_RST (1<<7)
#define INI_ACK (1<<4)
#define INI_BSY (1<<3)
#define INI_SEL (1<<2)
#define INI_ATN (1<<1)
#define INI_DBUS (1<<0)

#define SSR_RST (1<<7)
#define SSR_BSY (1<<6)
#define SSR_REQ (1<<5)
#define SSR_MSG (1<<4)
#define SSR_CD (1<<3)
#define SSR_IO (1<<2)
#define SSR_SEL (1<<1)
#define SSR_DBP (1<<0)

#define TCR_IO (1<<0)
#define TCR_CD (1<<1)
#define TCR_MSG (1<<2)
#define TCR_REQ (1<<3)

#define MODE_ARB (1<<0)
#define MODE_DMA (1<<1)
#define MODE_MONBSY (1<<2)
#define MODE_EIPINTEN (1<<3)
#define MODE_PARINTEN (1<<4)
#define MODE_PARCHK (1<<5)
#define MODE_TARGET (1<<6)
#define MODE_BDMA (1<<7)

#define BSR_ACK (1<<0)
#define BSR_ATN (1<<1)
#define BSR_BUSYERR (1<<2)
#define BSR_PHASEMATCH (1<<3)
#define BSR_IRQACT (1<<4)
#define BSR_PARERR (1<<5)
#define BSR_DMARQ (1<<6)
#define BSR_EODMA (1<<7)

#define ST_IDLE 0
#define ST_ARB 1
#define ST_ARBDONE 2
#define ST_SELECT 3
#define ST_SELDONE 4
#define ST_DATA 5

static const char* const stateNames[]={
	"IDLE", "ARB", "ARBDONE", "SELECT", "SELDONE", "DATA"
};

static Ncr ncr;
uint8_t *scsiDataBuffer = NULL;

uint8_t ncrGetDeviceMask(void) {
	uint8_t mask = 0;
	for (int id = 0; id < 8; ++id) {
		if (ncr.dev[id] != NULL) mask |= (uint8_t)(1U << id);
	}
	return mask;
}

void ncrInit(void) {
	memset(&ncr, 0, sizeof(ncr));
	if (scsiDataBuffer != NULL) {
		ncr.data.data = scsiDataBuffer;
	} else {
		ncr.data.data = (uint8_t *)heap_caps_malloc(
			SCSI_DATA_BUFFER_BYTES, MALLOC_CAP_EXEC | MALLOC_CAP_8BIT);
		if (ncr.data.data == NULL) {
			ncr.data.data = (uint8_t *)malloc(SCSI_DATA_BUFFER_BYTES);
		}
	}
	if (ncr.data.data != NULL) {
		ncr.data.dataCapacity = SCSI_DATA_BUFFER_BYTES;
		memset(ncr.data.data, 0, ncr.data.dataCapacity);
		printf("SCSI: data buffer=%u bytes at %p\n",
		       (unsigned int)ncr.data.dataCapacity, ncr.data.data);
	} else {
		ncr.data.dataCapacity = 0;
		printf("SCSI: data buffer allocation failed\n");
	}
}

static int ncrRefillReadChunk(void);
static int ncrCommitWriteChunk(void);

static unsigned int scsiCommandLba(const uint8_t *cmd) {
	const unsigned int group = cmd[0] >> 5;
	if (group == 0) {
		return cmd[3] | ((unsigned int)cmd[2] << 8) |
		       ((unsigned int)(cmd[1] & 0x1F) << 16);
	}
	return cmd[5] | ((unsigned int)cmd[4] << 8) |
	       ((unsigned int)cmd[3] << 16) | ((unsigned int)cmd[2] << 24);
}

static void parseScsiCmd(int isRead) {
	uint8_t *buf=ncr.data.cmd;
	int cmd=buf[0];
	int lba, len, ctrl;
	int group=(cmd>>5);
	if (group==0) { //6-byte command
		lba=buf[3]|(buf[2]<<8)|((buf[1]&0x1F)<<16);
		len=buf[4];
		if (len==0) len=256;
		ctrl=buf[5];
//		for (int x=0; x<6; x++) printf("%02X ", buf[x]);
//		printf("\n");
	} else if (group==1 || group==2) { //10-byte command
		lba=buf[5]|(buf[4]<<8)|(buf[3]<<16)|(buf[2]<<24);
		len=buf[8]|(buf[7]<<8);
		ctrl=buf[9];
//		for (int x=0; x<10; x++) printf("%02X ", buf[x]);
//		printf("\n");
	} else {
		printf("SCSI: UNSUPPORTED CMD %x\n", cmd);
		return;
	}
//	printf("SCSI: CMD %x LBA %x LEN %x CTRL %x %s\n", cmd, lba, len, ctrl, isRead?"*READ*":"*WRITE*");
	if (ncr.dev[ncr.selected]) {
		// Large reads are split into dataCapacity-sized chunks.  The Mac
		// keeps draining the data-in phase; ncrRefillReadChunk() is called
		// whenever the current chunk has been consumed, so a 16KB DMA window
		// can serve the 37-sector (18944-byte) reads that System 3.x issues
		// when loading bigger applications.
		if (isRead && (cmd == 0x08 || cmd == 0x28) &&
		    (size_t)len * 512U > ncr.data.dataCapacity) {
			ncr.chunkActive = 1;
			ncr.chunkRemaining = len * 512;
			ncr.chunkCmd = cmd;
			ncr.chunkLba = (uint32_t)lba;
			ncrRefillReadChunk();
		} else {
			ncr.chunkActive = 0;
			ncr.datalen = ncr.dev[ncr.selected]->scsiCmd(
				&ncr.data, cmd, len, lba, ncr.dev[ncr.selected]->arg);
		}
	}
}

static int ncrRefillReadChunk(void) {
	if (!ncr.chunkActive || ncr.chunkRemaining <= 0 ||
	    ncr.dev[ncr.selected] == NULL) {
		return 0;
	}
	uint32_t want = ncr.data.dataCapacity;
	if (want == 0) return 0;
	if ((uint32_t)ncr.chunkRemaining < want) want = ncr.chunkRemaining;
	want &= ~511U; // keep sector-aligned
	if (want == 0) return 0;

	const int bytes = ncr.dev[ncr.selected]->scsiCmd(
		&ncr.data, ncr.chunkCmd, want / 512U, ncr.chunkLba,
		ncr.dev[ncr.selected]->arg);
	if (bytes <= 0) {
		ncr.chunkActive = 0;
		return 0;
	}
	ncr.datalen = bytes;
	ncr.bufpos = 0;
	ncr.chunkRemaining -= bytes;
	ncr.chunkLba += (uint32_t)(bytes / 512);
	if (ncr.chunkRemaining <= 0) ncr.chunkActive = 0;
	return 1;
}

static void prepareDataOutPhase(void) {
	const uint8_t cmd = ncr.data.cmd[0];
	const unsigned int group = cmd >> 5;
	unsigned int sectors = 0;
	if (group == 0) {
		sectors = ncr.data.cmd[4];
		if (sectors == 0) sectors = 256;
	} else if (group == 1 || group == 2) {
		sectors = ((unsigned int)ncr.data.cmd[7] << 8) | ncr.data.cmd[8];
	}
	if (cmd == 0x0A || cmd == 0x2A) {
		const unsigned int maxSectors = ncr.data.dataCapacity / 512U;
		ncr.writeActive = maxSectors != 0;
		ncr.writeFailed = ncr.writeActive ? 0 : 1;
		ncr.writeCmd = cmd;
		ncr.writeLba = scsiCommandLba(ncr.data.cmd);
		ncr.writeRemaining = sectors;
		if (sectors > maxSectors) sectors = maxSectors;
		ncr.datalen = (int)(sectors * 512U);
	} else {
		ncr.writeActive = 0;
		ncr.datalen = 0;
	}
}

static int ncrCommitWriteChunk(void) {
	if (!ncr.writeActive || ncr.writeRemaining == 0 ||
	    ncr.dev[ncr.selected] == NULL || ncr.bufpos == 0 ||
	    (ncr.bufpos & 511) != 0) {
		return 0;
	}
	const unsigned int sectors = (unsigned int)ncr.bufpos / 512U;
	if (sectors > ncr.writeRemaining) return 0;
	const int bytes = ncr.dev[ncr.selected]->scsiCmd(
		&ncr.data, ncr.writeCmd, sectors, ncr.writeLba,
		ncr.dev[ncr.selected]->arg);
	if (bytes != (int)ncr.bufpos) {
		ncr.writeFailed = 1;
		return 0;
	}
	ncr.writeLba += sectors;
	ncr.writeRemaining -= sectors;
	ncr.bufpos = 0;
	const uint32_t maxBytes = ncr.data.dataCapacity & ~511U;
	uint32_t nextBytes = ncr.writeRemaining * 512U;
	if (nextBytes > maxBytes) nextBytes = maxBytes;
	ncr.datalen = (int)nextBytes;
	if (ncr.writeRemaining == 0) ncr.writeActive = 0;
	return 1;
}

static void ncrReceiveDataByte(uint8_t value) {
	if (ncr.buf == NULL || ncr.bufpos >= ncr.bufmax) return;
	ncr.buf[ncr.bufpos++] = value;
	if (ncr.writeActive && ncr.bufpos == ncr.datalen) {
		ncrCommitWriteChunk();
	}
}

unsigned int ncrRead(unsigned int addr, unsigned int dack) {
	unsigned int pc=m68k_get_reg(NULL, M68K_REG_PC);
	unsigned int ret=0;
	// The Mac ROM probes NCR registers while DMA mode remains enabled.  A
	// pseudo-DMA read advances the target data window only after the driver
	// has written a start-DMA register; otherwise the probe skips disk bytes.
	if ((ncr.mode & MODE_DMA) && ncr.dmaActive && dack) {
		if (ncr.tcr&TCR_IO) {
			if (ncr.bufpos >= ncr.datalen) ncrRefillReadChunk();
			if (ncr.bufpos < ncr.datalen && ncr.bufpos < ncr.bufmax) {
				ncr.din=ncr.buf[ncr.bufpos++];
			}
//			printf("Send next byte dma %d/%d\n", ncr.bufpos, ncr.datalen);
		}
	}
	if (addr==0) {
		ret=ncr.din;
//		printf("READ BYTE %02X dack=%d\n", ret, dack);
	} else if (addr==1) {
		// /rst s s /ack /bsy /sel /atn databus
		ret=ncr.inicmd;
		if (ncr.state==ST_ARB) {
			ret|=INIR_AIP;
			//We don't have a timer... just set arb to be done right now.
			if (ncr.dev[ncr.selected]) ncr.state=ST_ARBDONE;
		}
	} else if (addr==2) {
		ret=ncr.mode;
	} else if (addr==3) {
		ret=ncr.tcr;
	} else if (addr==4) {
		ret=0;
		if (ncr.inicmd&INI_RST) ret|=SSR_RST;
		if (ncr.inicmd&INI_BSY) ret|=SSR_BSY;
//		if (ncr.inicmd&INI_SEL) ret|=SSR_SEL;
		if (ncr.tcr&TCR_IO) ret|=SSR_IO;
		if (ncr.tcr&TCR_CD) ret|=SSR_CD;
		if (ncr.tcr&TCR_MSG) ret|=SSR_MSG;
		if (ncr.dev[ncr.selected] && (ncr.state==ST_SELDONE)) {
//			ret|=SSR_REQ;
			ret|=SSR_BSY;
		}
		if (ncr.state==ST_DATA) {
			if ((ncr.inicmd&INI_ACK)==0) {
				ret|=SSR_REQ;
			}
		}
		if (ncr.state==ST_ARB) return 0x40;
	} else if (addr==5) {
		ret=BSR_PHASEMATCH;
		if ((ncr.mode & MODE_DMA) && ncr.dmaActive) {
			if (ncr.bufpos >= ncr.datalen) ncrRefillReadChunk();
			ret|=BSR_DMARQ;
			if (ncr.bufpos>=ncr.datalen) {
//				printf("End of DMA reached: bufpos %d datalen %d\n", ncr.bufpos, ncr.datalen);
				ret|=BSR_EODMA;
			}
		}
	} else if (addr==6) {
		ret=ncr.din;
//		printf("READ BYTE (NCR addr6) %02X dack=%d\n", ret, dack);
	}
//	printf("%08X SCSI: (dack %d), cur st %s read %s (reg %d) = %x \n", 
//		pc, dack,  stateNames[ncr.state], regNamesR[addr], addr, ret);
	return ret;
}


void ncrWrite(unsigned int addr, unsigned int dack, unsigned int val) {
	unsigned int pc=m68k_get_reg(NULL, M68K_REG_PC);

	if (addr==0) {
		if (ncr.mode&MODE_DMA && dack) {
			if ((ncr.tcr&TCR_IO)==0) ncrReceiveDataByte((uint8_t)val);
		}
		ncr.dout=val;
		ncr.din=val;
	} else if (addr==1) {
		if ((val&INI_SEL) && (val&INI_DBUS) && (val&INI_BSY) && (ncr.state==ST_ARBDONE || ncr.state==ST_ARB)) {
			ncr.state=ST_SELECT;
			if (ncr.dout==0x81) ncr.selected=0;
			if (ncr.dout==0x82) ncr.selected=1;
			if (ncr.dout==0x84) ncr.selected=2;
			if (ncr.dout==0x88) ncr.selected=3;
			if (ncr.dout==0x90) ncr.selected=4;
			if (ncr.dout==0xA0) ncr.selected=5;
			if (ncr.dout==0xC0) ncr.selected=6;
//			printf("Selected dev: %d (val %x)\n", ncr.selected, ncr.dout);
		}
		if (((val&INI_BSY)==0) && ncr.state==ST_SELECT) {
			ncr.state=ST_SELDONE;
		}
		if (((val&INI_SEL)==0) && ncr.state==ST_SELDONE) {
			if (ncr.dev[ncr.selected]) {
				ncr.state=ST_DATA;
			} else {
				ncr.state=ST_IDLE;
			}
		}
		if (ncr.state==ST_DATA && ((ncr.inicmd&INI_ACK)==0) && (val&INI_ACK)) {
			//We have an ack.
			if (!(ncr.tcr&TCR_IO)) ncrReceiveDataByte(ncr.dout);
		}
		if (ncr.state==ST_DATA && (ncr.inicmd&INI_ACK) && ((val&INI_ACK)==0)) {
			//Ack line goes low..
			if (ncr.tcr&TCR_IO) {
				if (ncr.bufpos >= ncr.datalen) ncrRefillReadChunk();
				if (ncr.bufpos < ncr.datalen && ncr.bufpos < ncr.bufmax) {
					ncr.din=ncr.buf[ncr.bufpos++];
				}
//				printf("Send byte non-dma\n");
			}
		}
		if (val&INI_RST) {
			ncr.state=ST_IDLE;
			ncr.chunkActive=0;
			ncr.dmaActive=0;
		}
		ncr.inicmd&=~0x9F;
		ncr.inicmd|=val&0x9f;
	} else if (addr==2) {
		ncr.mode=val;
		if ((val & MODE_DMA) == 0) ncr.dmaActive=0;
		if (((val&1)==0) && ncr.state==ST_ARB) ncr.state=ST_IDLE;
		if (val&1) ncr.state=ST_ARB;
	} else if (addr==3) {
		if (ncr.tcr!=(val&0xf)) {
			int oldtcr=(ncr.tcr&7);
			int newtcr=(val&7);
			if (oldtcr==0 && ncr.writeActive) {
				if (ncr.bufpos != 0) ncrCommitWriteChunk();
				if (ncr.writeRemaining != 0) ncr.writeFailed=1;
				ncr.writeActive=0;
			} else if (oldtcr==0 && ncr.bufpos) {
				// End of non-write data out phase.
				parseScsiCmd(0);
			} else if ((oldtcr==TCR_CD) && (newtcr==TCR_IO)) {
				//Start of data in phase
				parseScsiCmd(1);
			} else if ((oldtcr==TCR_CD) && (newtcr==0)) {
				// A write CDB is followed by a DMA data-out phase. Keep REQ
				// asserted for the advertised transfer length until it completes.
				prepareDataOutPhase();
			}
			if ((ncr.tcr&0x7)==TCR_IO) {
//				printf("Data Out finished: Host read %d/%d bytes.\n", ncr.bufpos, ncr.datalen);
			}
			ncr.bufpos=0;
			ncr.dmaActive=0;
			int type=val&(TCR_MSG|TCR_CD);
			if (type==0) {
//				printf("Sel data buf %s.\n", (newtcr&TCR_IO)?"IN":"OUT");
				ncr.buf=ncr.data.data;
				ncr.bufmax=(int)ncr.data.dataCapacity;
			} else if (type==TCR_CD) {
//				printf("Sel cmd/status buf %s.\n", (newtcr&TCR_IO)?"IN":"OUT");
				ncr.buf=ncr.data.cmd;
				ncr.bufmax=sizeof(ncr.data.cmd);
				ncr.datalen=1;
			} else if (type==(TCR_CD|TCR_MSG)) {
//				printf("Sel msg buf %s.\n", (newtcr&TCR_IO)?"IN":"OUT");
				ncr.buf=ncr.data.msg;
				ncr.bufmax=sizeof(ncr.data.msg);
				ncr.datalen=1;
			}
			ncr.din=(ncr.buf!=NULL)?ncr.buf[0]:0;
		}
		ncr.tcr=val&0xf;
	} else if (addr==4) {
		// Select-enable is unused by the single-initiator model.
	} else if (addr==5) {
		// Start DMA send (initiator -> target).
		ncr.dmaActive=1;
	} else if (addr==6) {
		// Start DMA target receive.
		ncr.dmaActive=1;
	} else if (addr==7) {
		// Start DMA initiator receive (target -> initiator).
		ncr.dmaActive=1;
	}
//	printf("%08X SCSI: (dack %d), cur state %s %02x to %s (reg %d)\n", pc, dack, stateNames[ncr.state], val, regNamesW[addr], addr);
}

void ncrRegisterDevice(int id, SCSIDevice* dev){
	if (id < 0 || id >= 8) return;
	ncr.dev[id]=dev;
}
