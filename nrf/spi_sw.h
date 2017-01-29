/*
 * spi_sw.h
 *
 *  Created on: 10 mar 2015
 *      Author: benjamin
 */

#ifndef SPI_SW_H_
#define SPI_SW_H_

#include "hw.h"

// Functions
void spi_sw_init(void);
void spi_sw_transfer(uint8_t *in_buf, const uint8_t *out_buf, uint32_t length);
void spi_sw_begin(void);
void spi_sw_end(void);

#endif /* SPI_SW_H_ */
