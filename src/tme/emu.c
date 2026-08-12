/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Jeroen Domburg <jeroen@spritesmods.com> wrote this file. As long as you retain
 * this notice you can do whatever you want with this stuff. If we meet some day,
 * and you think this stuff is worth it, you can buy me a beer in return.
 * ----------------------------------------------------------------------------
 * Adapted for ESP32-S3 with OPI PSRAM (direct memory-mapped, no cache)
 */
#if defined(__GNUC__)
#pragma GCC optimize("O2")
#endif

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <stdint.h>
#include <errno.h>
#include "emu.h"
#include <string.h>
#include "tmeconfig.h"
#include "m68k.h"
#include "disp.h"
#include "iwm.h"
#include "via.h"
#include "scc.h"
#include "rtc.h"
#include "ncr.h"
#include <stdio.h>
#include <string.h>
#include "hd.h"
#include "snd.h"
#include "mouse.h"
#include "sdcard.h"
#include <stdbool.h>
#include <sys/time.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#define EMU_FRAME_RATE 60
#define EMU_CPU_CLOCK_HZ 7833600
#define EMU_CYCLES_PER_FRAME (EMU_CPU_CLOCK_HZ / EMU_FRAME_RATE)
#define EMU_FRAME_US (1000000 / EMU_FRAME_RATE)
#define EMU_FRAME_US_REMAINDER (1000000 % EMU_FRAME_RATE)
#define MOUSE_STEP_CYCLES 8000

static volatile uint32_t emuCyclesPerSecond = 0;
static volatile uint32_t emuWallFps = 0;
static volatile uint32_t emuWallSpeedPercent = 0;
static volatile uint32_t emuProgramCounter = 0;
static volatile int emuRunning = 0;
static int viaIrqRequested = 0;
static int sccIrqRequested = 0;

uint8_t tmeGetScsiDeviceMask(void) { return ncrGetDeviceMask(); }
extern unsigned char *macRam;
extern uint8_t *macFb[];

uint32_t tmeGetCyclesPerSecond(void) { return emuCyclesPerSecond; }
uint32_t tmeGetWallFps(void) { return emuWallFps; }
uint32_t tmeGetWallSpeedPercent(void) { return emuWallSpeedPercent; }
uint32_t tmeGetProgramCounter(void) { return emuProgramCounter; }
int tmeIsRunning(void) { return emuRunning; }

static void waitForFrameDeadline(int64_t deadlineUs) {
	for (;;) {
		const int64_t remainingUs = deadlineUs - esp_timer_get_time();
		if (remainingUs <= 0) return;
		if (remainingUs > 1500) {
			TickType_t ticks = pdMS_TO_TICKS((remainingUs - 500) / 1000);
			if (ticks == 0) ticks = 1;
			vTaskDelay(ticks);
		} else {
			esp_rom_delay_us((uint32_t)remainingUs);
		}
	}
}
uint16_t tmeGetMouseX(void) {
	if (macRam == NULL) return 0;
	return ((uint16_t)macRam[0x082E] << 8) | macRam[0x082F];
}
uint16_t tmeGetMouseY(void) {
	if (macRam == NULL) return 0;
	return ((uint16_t)macRam[0x082C] << 8) | macRam[0x082D];
}
unsigned char *macRom;
unsigned char *macRam;

#define MEMADDR_DUMMY_CACHE (void*)1

int rom_remap, video_remap=0, audio_remap=0, audio_volume=0, audio_en=0;

static int iwmSliceCycles;

static void iwmSyncToCpu(void) {
	const int elapsed = m68k_cycles_run() - iwmSliceCycles;
	if (elapsed > 0) iwmTick((unsigned int)elapsed);
	iwmSliceCycles += elapsed;
}

void m68k_instruction() {}

typedef uint8_t (*PeripAccessCb)(unsigned int address, int data, int isWrite);

uint8_t unhandledAccessCb(unsigned int address, int data, int isWrite) {
	unsigned int pc=m68k_get_reg(NULL, M68K_REG_PC);
	printf("Unhandled %s @ 0x%X! PC=0x%X\n", isWrite?"write":"read", address, pc);
	return 0xff;
}

uint8_t bogusReadCb(unsigned int address, int data, int isWrite) {
	if (isWrite) return 0;
	return address^(address>>8)^(address>>16);
}

uint8_t ncrAccessCb(unsigned int address, int data, int isWrite) {
	if (isWrite) {
		ncrWrite((address>>4)&0x7, (address>>9)&1, data);
		return 0;
	} else {
		return ncrRead((address>>4)&0x7, (address>>9)&1);
	}
}

uint8_t sscAccessCb(unsigned int address, int data, int isWrite) {
	if (isWrite) {
		sccWrite(address, data);
		return 0;
	} else {
		return sccRead(address);
	}
}

uint8_t iwmAccessCb(unsigned int address, int data, int isWrite) {
	// The Mac Plus IWM is wired only to the odd byte lane. A 16-bit CPU
	// access must therefore not let its even byte consume an IWM register.
	if ((address & 1U) == 0) return 0;
	iwmSyncToCpu();
	if (isWrite) {
		iwmWrite((address>>9)&0xf, data);
		return 0;
	} else {
		return iwmRead((address>>9)&0xf);;
	}
}

uint8_t sccIackCb(unsigned int address, int data, int isWrite) {
	// SCC interrupt acknowledge - read returns vector from SCC
	if (!isWrite) return sccRead(address);
	return 0;
}

uint8_t scsiDmaCb(unsigned int address, int data, int isWrite) {
	// SCSI pseudo-DMA area - return 0 for now
	(void)address; (void)data; (void)isWrite;
	return 0;
}

uint8_t viaAccessCb(unsigned int address, int data, int isWrite) {
	if (isWrite) {
		viaWrite((address>>9)&0xf, data);
		return 0;
	} else {
		return viaRead((address>>9)&0xf);
	}
}


#define FLAG_RO (1<<0);

typedef struct {
	uint8_t *memAddr;
	union {
		PeripAccessCb cb;
		int flags;
	};
} MemmapEnt;

#define MEMMAP_ES 0x20000 //entry size
#define MEMMAP_MAX_ADDR 0x1000000
MemmapEnt memmap[MEMMAP_MAX_ADDR/MEMMAP_ES];

#define MMAP_RAM_PTR(ent, addr) &ent->memAddr[addr&(MEMMAP_ES-1)]

static void regenMemmap(int remapRom) {
	int i;
	for (i=0; i<MEMMAP_MAX_ADDR/MEMMAP_ES; i++) {
		memmap[i].memAddr=0;
		memmap[i].cb=unhandledAccessCb;
	}

	if (remapRom) {
		memmap[0].memAddr=macRom;
		memmap[0].flags=FLAG_RO;
		for (i=1; i<0x400000/MEMMAP_ES; i++) {
			memmap[i].memAddr=NULL;
			memmap[i].cb=bogusReadCb;
		}
	} else {
		for (i=0; i<0x400000/MEMMAP_ES; i++) {
			memmap[i].memAddr=&macRam[(i*MEMMAP_ES)&(TME_RAMSIZE-1)];
			memmap[i].flags=0;
		}
	}

	memmap[0x400000/MEMMAP_ES].memAddr=macRom;
	memmap[0x400000/MEMMAP_ES].flags=FLAG_RO;
	for (i=0x400000/MEMMAP_ES+1; i<0x500000/MEMMAP_ES; i++) {
		memmap[i].memAddr=0;
		memmap[i].cb=bogusReadCb;
	}

	// 0x500000-0x580000: unassigned on a real Mac Plus.  The screen lives in
	// main RAM at the top of the 4MB view (see tmeconfig.h).  Return garbage
	// like other unmapped regions instead of faulting.
	for (i=0x500000/MEMMAP_ES; i<0x580000/MEMMAP_ES; i++) {
		memmap[i].memAddr=NULL;
		memmap[i].cb=bogusReadCb;
	}

	for (i=0x580000/MEMMAP_ES; i<0x600000/MEMMAP_ES; i++) {
		memmap[i].memAddr=NULL;
		memmap[i].cb=ncrAccessCb;
	}

	for (i=0x600000/MEMMAP_ES; i<0x700000/MEMMAP_ES; i++) {
		memmap[i].memAddr=&macRam[(i*MEMMAP_ES)&(TME_RAMSIZE-1)];
		memmap[i].flags=0;
	}

	for (i=0x800000/MEMMAP_ES; i<0xC00000/MEMMAP_ES; i++) {
		memmap[i].memAddr=NULL;
		memmap[i].cb=sscAccessCb;
	}

	for (i=0xc00000/MEMMAP_ES; i<0xe00000/MEMMAP_ES; i++) {
		memmap[i].memAddr=NULL;
		memmap[i].cb=iwmAccessCb;
	}
	for (i=0xE80000/MEMMAP_ES; i<0xF00000/MEMMAP_ES; i++) {
		memmap[i].memAddr=NULL;
		memmap[i].cb=viaAccessCb;
	}
	// 0xF00000-0xF80000: SCC interrupt ack (phase read)
	for (i=0xF00000/MEMMAP_ES; i<0xF80000/MEMMAP_ES; i++) {
		memmap[i].memAddr=NULL;
		memmap[i].cb=sccIackCb;
	}
	// 0xF80000-0x1000000: SCSI pseudo-DMA / phase-read area
	for (i=0xF80000/MEMMAP_ES; i<0x1000000/MEMMAP_ES; i++) {
		memmap[i].memAddr=NULL;
		memmap[i].cb=scsiDmaCb;
	}
}

uint8_t *macFb[2], *macSnd[2];


static void ramInit() {
	// main.cpp reserves the block early, while the heap is still contiguous.
	if (macRam == NULL) {
		macRam=(unsigned char*)malloc(TME_RAMSIZE);
	}
	assert(macRam);
	printf("Mac RAM allocated at %p (%d bytes)\n", macRam, TME_RAMSIZE);

	// Screen framebuffer at the top of RAM, exactly like a real Mac Plus:
	// 0x3FA700 in the 4MB view wraps onto TME_SCREENBUF in the 256KB buffer.
	macFb[0]=&macRam[TME_SCREENBUF];
	macFb[1]=macFb[0];
	macSnd[0]=&macRam[TME_SNDBUF];
	macSnd[1]=&macRam[TME_SNDBUF_ALT];
	printf("Clearing ram...\n");
	for (int x=0; x<TME_RAMSIZE; x++) macRam[x]=rand();
}


const inline static MemmapEnt *getMmmapEnt(const unsigned int address) {
	if (address>=MEMMAP_MAX_ADDR) return &memmap[127];
	return &memmap[address/MEMMAP_ES];
}

unsigned int m68k_read_memory_8(unsigned int address) {
	const MemmapEnt *mmEnt=getMmmapEnt(address);
	if (mmEnt->memAddr) {
		uint8_t *p;
		p=(uint8_t*)MMAP_RAM_PTR(mmEnt, address);
		return *p;
	} else {
		return mmEnt->cb(address, 0, 0);
	}
}

unsigned int m68k_read_memory_16(unsigned int address) {
	const MemmapEnt *mmEnt=getMmmapEnt(address);
	if (mmEnt->memAddr) {
		uint8_t *p=(uint8_t*)MMAP_RAM_PTR(mmEnt, address);
		// Some System 3 code performs odd-address word accesses. Avoid the
		// slow serial diagnostic and the unaligned native 16-bit dereference;
		// aligned accesses retain the existing fast path.
		if ((address&1)!=0) return ((unsigned int)p[0]<<8)|p[1];
		return __builtin_bswap16(*(uint16_t*)p);
	} else {
		unsigned int ret;
		ret=mmEnt->cb(address, 0, 0)<<8;
		ret|=mmEnt->cb(address+1, 0, 0);
		return ret;
	}
}

unsigned int m68k_read_memory_32(unsigned int address) {
	uint16_t a=m68k_read_memory_16(address);
	uint16_t b=m68k_read_memory_16(address+2);
	return (a<<16)|b;
}

void m68k_write_memory_8(unsigned int address, unsigned int value) {
	const MemmapEnt *mmEnt=getMmmapEnt(address);
	if (mmEnt->memAddr) {
		uint8_t *p;
		p=(uint8_t*)MMAP_RAM_PTR(mmEnt, address);
		*p=value;
	} else {
		mmEnt->cb(address, value, 1);
	}
}

void m68k_write_memory_16(unsigned int address, unsigned int value) {
	const MemmapEnt *mmEnt=getMmmapEnt(address);
	if (mmEnt->memAddr) {
		uint8_t *p=(uint8_t*)MMAP_RAM_PTR(mmEnt, address);
		if ((address&1)!=0) {
			p[0]=(value>>8)&0xff;
			p[1]=value&0xff;
		} else {
			*(uint16_t*)p=__builtin_bswap16(value);
		}
	} else {
		mmEnt->cb(address, (value>>8)&0xff, 1);
		mmEnt->cb(address+1, (value>>0)&0xff, 1);
	}
}

void m68k_write_memory_32(unsigned int address, unsigned int value) {
	m68k_write_memory_16(address, value>>16);
	m68k_write_memory_16(address+2, value);
}

unsigned char *m68k_pcbase=NULL;

void m68k_pc_changed_handler_function(unsigned int address) {
	const MemmapEnt *mmEnt=getMmmapEnt(address);
	if (mmEnt->memAddr) {
		uint8_t *p;
		p=(uint8_t*)MMAP_RAM_PTR(mmEnt, address);
		m68k_pcbase=p-address;
	} else {
		/* Keep the failing address and CPU state visible on the serial console.
		 * The callback receives the new PC before Musashi updates REG_PC, so
		 * report both values while diagnosing invalid control-flow targets. */
		const unsigned int d2=m68k_get_reg(NULL, M68K_REG_D2);
		const unsigned int tableOffset=(0x400U+(d2&0xffffU))&(TME_RAMSIZE-1);
		printf("PC not in mem! addr=0x%08X pc=0x%08X sr=0x%04X "
		       "ppc=0x%08X ir=0x%04X d0=0x%08X d2=0x%08X "
		       "a0=0x%08X a2=0x%08X a5=0x%08X sp=0x%08X "
		       "rom_remap=%d map=%u\n",
		       address,
		       m68k_get_reg(NULL, M68K_REG_PC),
		       m68k_get_reg(NULL, M68K_REG_SR),
		       m68k_get_reg(NULL, M68K_REG_PPC),
		       m68k_get_reg(NULL, M68K_REG_IR),
		       m68k_get_reg(NULL, M68K_REG_D0),
		       d2,
		       m68k_get_reg(NULL, M68K_REG_A0),
		       m68k_get_reg(NULL, M68K_REG_A2),
		       m68k_get_reg(NULL, M68K_REG_A5),
		       m68k_get_reg(NULL, M68K_REG_A7),
		       rom_remap,
		       address / MEMMAP_ES);
		if (macRam != NULL) {
			printf("LOWMEM: table+0x%04X=%02X%02X%02X%02X base=%02X%02X%02X%02X\n",
			       d2&0xffffU,
			       macRam[tableOffset], macRam[(tableOffset+1)&(TME_RAMSIZE-1)],
			       macRam[(tableOffset+2)&(TME_RAMSIZE-1)], macRam[(tableOffset+3)&(TME_RAMSIZE-1)],
			       macRam[0x400], macRam[0x401], macRam[0x402], macRam[0x403]);
			printf("BOOTPTR: b2a=%02X%02X%02X%02X b4c=%02X%02X%02X%02X\n",
				macRam[0xB2A], macRam[0xB2B], macRam[0xB2C], macRam[0xB2D],
				macRam[0xB4C], macRam[0xB4D], macRam[0xB4E], macRam[0xB4F]);
		}
		viaDebugPrint();
		abort();
	}
}


void printFps(int logOutput) {
	struct timeval tv;
	static struct timeval oldtv;
	gettimeofday(&tv, NULL);
	if (oldtv.tv_sec!=0) {
		long msec=(tv.tv_sec-oldtv.tv_sec)*1000;
		msec+=(tv.tv_usec-oldtv.tv_usec)/1000;
		if (msec > 0) {
			emuWallFps = (uint32_t)((60000ULL + (uint64_t)msec / 2) /
							(uint64_t)msec);
			emuWallSpeedPercent =
				(uint32_t)((100000ULL + (uint64_t)msec / 2) /
							(uint64_t)msec);
				if (logOutput) {
					printf("Speed: %lu%% (%lu fps)\n",
					       (unsigned long)emuWallSpeedPercent,
					       (unsigned long)emuWallFps);
				}
		}
	}
	oldtv.tv_sec=tv.tv_sec;
	oldtv.tv_usec=tv.tv_usec;
}

void tmeStartEmu(void *rom) {
	int ca1=0, ca2=0;
	int x, frame=0;
	int cyclesPerSec=0;
	int yieldDivider=0;
	int statsSeconds=0;
	int frameUsRemainder=0;
	int mouseCycles=0;
	int installInsertFrames=0;
	macRom=(unsigned char*)rom;
	ramInit();
	rom_remap=1;
	regenMemmap(1);
	printf("Display init before storage...\n");
	dispInit();
	printf("Creating HD and registering it...\n");
	ncrInit();
	SCSIDevice *hd=hdCreate();
	ncrRegisterDevice(6, hd);
	iwmInit();
	const uint32_t installDiskBytes = hdGetInstallVolumeBytes();
	if (installDiskBytes != 0) {
		// Keep the uploaded disk out of the boot drive scan.  System 3 polls
		// the Sony drive after Finder is up and will then mount the change.
		installInsertFrames = 600; // ten seconds at 60 Hz
		printf("INSTALL: delaying IWM insert (%lu bytes)\n",
		       (unsigned long)installDiskBytes);
	}
	viaClear(VIA_PORTA, 0x7F);
	viaSet(VIA_PORTA, 0x80);
	viaClear(VIA_PORTA, 0xFF);
	viaSet(VIA_PORTB, (1<<3));
	sccInit();
	printf("Initializing m68k...\n");
	m68k_pc_changed_handler_function(0x0);
	m68k_init();
	printf("Setting CPU type and resetting...");
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_pulse_reset();
	printf("Done! Running.\n");
	emuRunning=1;
	int64_t nextFrameUs=esp_timer_get_time();
	while(1) {
		nextFrameUs+=EMU_FRAME_US;
		frameUsRemainder+=EMU_FRAME_US_REMAINDER;
		if (frameUsRemainder>=EMU_FRAME_RATE) {
			nextFrameUs++;
			frameUsRemainder-=EMU_FRAME_RATE;
		}
		for (x=0; x<EMU_CYCLES_PER_FRAME;) {
			const int slice = (EMU_CYCLES_PER_FRAME - x) < 1000
				? (EMU_CYCLES_PER_FRAME - x) : 1000;
			iwmSliceCycles=0;
			const int executed=m68k_execute(slice);
			if (executed>iwmSliceCycles) {
				iwmTick((unsigned int)(executed-iwmSliceCycles));
			}
			viaStep(slice / 10);
			sccTick(slice / 10);
			x += slice;
			// The Mac mouse driver must observe each quadrature edge. Advancing
			// at every 1000 CPU cycles consumed a whole IMU movement burst before
			// the SCC external-status interrupt could acknowledge it.
			mouseCycles+=slice;
			if (mouseCycles>=MOUSE_STEP_CYCLES) {
				mouseCycles-=MOUSE_STEP_CYCLES;
				int r=mouseTick();
				if (r&MOUSE_BTN) viaClear(VIA_PORTB, (1<<3)); else viaSet(VIA_PORTB, (1<<3));
				if (r&MOUSE_QXB) viaClear(VIA_PORTB, (1<<4)); else viaSet(VIA_PORTB, (1<<4));
				if (r&MOUSE_QYB) viaClear(VIA_PORTB, (1<<5)); else viaSet(VIA_PORTB, (1<<5));
				sccSetDcd(SCC_CHANA, r&MOUSE_QXA);
				sccSetDcd(SCC_CHANB, r&MOUSE_QYA);
			}

		}
		cyclesPerSec+=x;
			emuProgramCounter=m68k_get_reg(NULL, M68K_REG_PC);
			dispDraw(macFb[0]);
			sndPush(macSnd[audio_remap?1:0], audio_en?audio_volume:0);
			if (++yieldDivider >= 8) {
				yieldDivider=0;
				vTaskDelay(1); // let the core-0 idle task service the watchdog
			}
		// Periodic SD write-back runs in a low-priority storage task. Keeping
		// it out of this loop prevents slow card writes from stalling emulation.
		hdFlushIfDue();
		waitForFrameDeadline(nextFrameUs);
		const int64_t frameNowUs=esp_timer_get_time();
		if (frameNowUs-nextFrameUs>EMU_FRAME_US) {
			nextFrameUs=frameNowUs;
			frameUsRemainder=0;
		}
		frame++;
		if (installInsertFrames > 0 && --installInsertFrames == 0) {
			iwmSetDiskReader(hdReadInstallSector, installDiskBytes, 1);
			printf("INSTALL: IWM disk inserted (%luKB %s)\n",
			       (unsigned long)(installDiskBytes / 1024U),
			       hdIsInstallVolumeMfs() ? "MFS" : "HFS");
		}
		ca1^=1;
		viaControlWrite(VIA_CA1, ca1);
		if (frame==59) {
			ca2^=1;
			viaControlWrite(VIA_CA2, ca2);
		}
		if (frame>=60) {
			ca2^=1;
			viaControlWrite(VIA_CA2, ca2);
				rtcTick();
				frame=0;
				statsSeconds++;
				const int logOutput=(statsSeconds%5)==0;
				printFps(logOutput);
			if (logOutput) printf("%d Hz\n", cyclesPerSec);
			emuCyclesPerSecond=cyclesPerSec;
			cyclesPerSec=0;
		}
	}
}

static void updateCpuIrq(void) {
	m68k_set_irq(sccIrqRequested ? 2 : (viaIrqRequested ? 1 : 0));
}

void viaIrq(int req) {
	viaIrqRequested=req?1:0;
	updateCpuIrq();
}

void sccIrq(int req) {
	sccIrqRequested=req?1:0;
	updateCpuIrq();
}


void viaCbPortAWrite(unsigned int val) {
	static int writes=0;
	if ((writes++)==0) val=0x67;
	video_remap=(val&(1<<6))?1:0;
	rom_remap=(val&(1<<4))?1:0;
	audio_remap=(val&(1<<3))?1:0;
	audio_volume=(val&7);
	iwmSetHeadSel(val&(1<<5));
	regenMemmap(rom_remap);
}

void viaCbPortBWrite(unsigned int val) {
	int b;
	b=rtcCom(val&4, val&1, val&2);
	if (b) viaSet(VIA_PORTB, 1); else viaClear(VIA_PORTB, 1);
	audio_en=!(val&(1<<7));
}
