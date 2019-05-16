/*
 * memory_test.h
 *
 *  Created on: Aug 31, 2018
 *      Author: alvaro
 */

#ifndef MEMORY_TEST_H_
#define MEMORY_TEST_H_

#include <stddef.h>

#define MEM_23LCV1024       1
#define MEM_CY15B104Q       2

#define MEM_PART_NUM        MEM_CY15B104Q

// Chip select pin
#define MEM_SPI_CS_PIN      BRD_MEM_CS

// Instruction set for 23LCV1024
#define MEM_I_READ          0x03 // Read data from memory array beginning at selected address
#define MEM_I_WRITE         0x02 // Write data to memory array beginning at selected address
#define MEM_I_EDIO          0x3B // Enter Dual I/O access
#define MEM_I_RSTIO         0xFF // Reset Dual I/O access
#define MEM_I_RDMR          0x05 // Read Mode Register
#define MEM_I_WRMR          0x01 // Write Mode Register

// CY15B104Q opcodes
#define MEM_OC_WREN         0b00000110 // Set write enable latch        0x06
#define MEM_OC_WRDI         0b00000100 // Reset write enable latch      0x04
#define MEM_OC_RDSR         0b00000101 // Read Status Register          0x05
#define MEM_OC_WRSR         0b00000001 // Write Status Register         0x01
#define MEM_OC_READ         0b00000011 // Read SRAM memory data         0x03
#define MEM_OC_FSTRD        0b00001011 // Fast read memory data         0x0B (11)
#define MEM_OC_WRITE        0b00000010 // Write SRAM memory data        0x02
#define MEM_OC_SLEEP        0b10111001 // Enter sleep mode
#define MEM_OC_RDID         0b10011111 // Read device ID
#define MEM_OC_SNR          0b11000011 // Read 8-byte serial number (Reserved)
// CY15B104Q definitions
#define MEM_DEV_ID_LEN      9
#define BUFFER_SIZE         16
#define STRING_SIZE         50

// Modes of operation
#define MEM_OP_MODE_BYTE    (0b00 << 6)
#define MEM_OP_MODE_PAGE    (0b10 << 6)
#define MEM_OP_MODE_SEQ     (0b01 << 6)

void Mem_Init();
void Mem_Test();

void Mem_Write_Array(const uint32_t addr,
                     const uint8_t* src,
                     const size_t src_count);

void Mem_Write(uint32_t addr, uint8_t* data_ptr, uint32_t total_count);
void Mem_Read(uint32_t addr, uint8_t* data_ptr,uint32_t total_count);


#endif /* MEMORY_TEST_H_ */
