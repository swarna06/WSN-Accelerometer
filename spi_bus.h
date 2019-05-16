/*
 * spi_bus.h
 *
 *  Created on: Aug 31, 2018
 *      Author: alvaro
 */

#ifndef SPI_BUS_H_
#define SPI_BUS_H_

#include <stddef.h>

#define SPI_DATA_RATE       2000000
#define SPI_DATA_WIDTH      8

#define Spi_Assert_CS(pin)      GPIO_clearDio(pin) // CS low
#define Spi_Deassert_CS(pin)    GPIO_setDio(pin) // CS high

void Spi_Init();

void Spi_Init_CS_Pin(uint8_t pin);

uint32_t Spi_Flush_Fifo();

void Spi_Send(const uint8_t *src, const size_t src_count);

void Spi_Receive(uint8_t *dest, const size_t dest_cont);

void Spi_Transaction(const uint8_t *src,
                     const size_t src_count,
                     uint8_t *dest,
                     const size_t dest_count,
                     const uint32_t cs_pin);



#endif /* SPI_BUS_H_ */
