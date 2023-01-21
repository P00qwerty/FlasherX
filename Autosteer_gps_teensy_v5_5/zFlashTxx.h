//******************************************************************************
// Flash write/erase functions (TLC/T3x/T4x/TMM), LMEM cache functions for T3.6
//******************************************************************************
// WARNING:  you can destroy your MCU with flash erase or write!
// This code may or may not protect you from that.
//
// Original by Niels A. Moseley, 2015.
// Modifications for OTA updates by Jon Zeeff, Deb Hollenback
// Paul Stoffregen's T4.x flash routines from Teensy4 core added by Jon Zeeff
// Frank Boesing's T3.x flash routines adapted for OTA by Joe Pasquariello
// This code is released into the public domain.
//******************************************************************************
#ifdef FlasherXOTA
#ifdef ARDUINO_TEENSY41
#ifndef _FLASHTXX_H_
#define _FLASHTXX_H_

#include <stdint.h>     // uint32_t, etc.

#if defined(__IMXRT1062__) && defined(ARDUINO_TEENSY41)
  #define FLASH_ID		"fw_teensy41"		// target ID (in code)
  #define FLASH_SIZE		(0x800000)		// 8MB
  #define FLASH_SECTOR_SIZE	(0x1000)		// 4KB sector size
  #define FLASH_WRITE_SIZE	(4)			// 4-byte/32-bit writes    
  #define FLASH_RESERVE		(4*FLASH_SECTOR_SIZE)	// reserve top of flash 
  #define FLASH_BASE_ADDR	(0x60000000)		// code starts here
#else
  #error MCU NOT SUPPORTED
#endif

#if defined(FLASH_ID)
  #define RAM_BUFFER_SIZE	(0 * 1024)
  #define IN_FLASH(a) ((a) >= FLASH_BASE_ADDR && (a) < FLASH_BASE_ADDR+FLASH_SIZE)
#endif

// reboot is the same for all ARM devices
#define CPU_RESTART_ADDR	((uint32_t *)0xE000ED0C)
#define CPU_RESTART_VAL		(0x5FA0004)
#define REBOOT			(*CPU_RESTART_ADDR = CPU_RESTART_VAL)

#define NO_BUFFER_TYPE		(0)
#define FLASH_BUFFER_TYPE	(1)
#define RAM_BUFFER_TYPE		(2)

// apparently better - thanks to Frank Boesing
#define RAMFUNC __attribute__ ((section(".fastrun"), noinline, noclone, optimize("Os") ))

#if defined(KINETISK) || defined(KINETISL)

// T3.x flash primitives (must be in RAM)
RAMFUNC int flash_word( uint32_t address, uint32_t value, int aFSEC, int oFSEC );
RAMFUNC int flash_phrase( uint32_t address, uint64_t value, int aFSEC, int oFSEC );
RAMFUNC int flash_erase_sector( uint32_t address, int aFSEC );
RAMFUNC int flash_sector_not_erased( uint32_t address );

#elif defined(__IMXRT1062__)

RAMFUNC int flash_sector_not_erased( uint32_t address );

// from cores\Teensy4\eeprom.c  --  use these functions at your own risk!!!
extern "C" void eepromemu_flash_write(void *addr, const void *data, uint32_t len);
extern "C" void eepromemu_flash_erase_sector(void *addr);
extern "C" void eepromemu_flash_erase_32K_block(void *addr);
extern "C" void eepromemu_flash_erase_64K_block(void *addr);

#endif // __IMXRT1062__

// functions used to move code from buffer to program flash (must be in RAM)
RAMFUNC void flash_move( uint32_t dst, uint32_t src, uint32_t size );

// functions that can be in flash
int  flash_write_block( uint32_t addr, uint8_t *data, uint32_t count );
int  flash_erase_block( uint32_t address, uint32_t size );

int  check_flash_id( uint32_t buffer, uint32_t size );
int  firmware_buffer_init( uint32_t *buffer_addr, uint32_t *buffer_size );
void firmware_buffer_free( uint32_t buffer_addr, uint32_t buffer_size );

#endif // _FLASHTXX_H_

#endif // ARDUINO_TEENSY41
#endif // ARDUINO_TEENSY41
