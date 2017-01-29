/*
	Copyright 2015 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "spi_sw.h"
#include "ch.h"
#include "hal.h"
#include <stdbool.h>
#include "stm32f4xx_spi.h"

#define SPI_MODE 2


// Private variables
static bool init_done = false;

// Private functions
static void spi_sw_delay(void);

uint8_t zerodata[128], trashdata[128];

#define SPIDIV_8    SPI_CR1_BR_1
#define SPIDIV_16   SPI_CR1_BR_1 | SPI_CR1_BR_0
#define SPIDIV_32   SPI_CR1_BR_2
#define SPIDIV_64   SPI_CR1_BR_2 | SPI_CR1_BR_0                 //666khz
#define SPIDIV_128  SPI_CR1_BR_2 | SPI_CR1_BR_1
#define SPIDIV_256  SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0  //138khz

static const SPIConfig spicfg = {
  NULL,
  NRF_PORT_CSN,
  NRF_PIN_CSN,
  SPIDIV_128 | SPI_CR1_MSTR | SPI_CR1_BIDIOE
};


void spi_sw_init(void)
{
	if(!init_done)
	{
		palSetPadMode(NRF_PORT_CSN, NRF_PIN_CSN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
		palSetPadMode(NRF_PORT_CE, NRF_PIN_CE, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

#if SPI_MODE == 0
		palSetPadMode(NRF_PORT_MISO, NRF_PIN_MISO, PAL_MODE_INPUT);

		palSetPadMode(NRF_PORT_SCK, NRF_PIN_SCK, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
		palSetPadMode(NRF_PORT_MOSI, NRF_PIN_MOSI, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

		palClearPad(NRF_PORT_SCK, NRF_PIN_SCK);
#endif
#if SPI_MODE == 1
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

		SPI_I2S_ClearITPendingBit(SPI3, SPI_I2S_IT_ERR | SPI_I2S_IT_OVR | SPI_I2S_IT_RXNE | SPI_I2S_IT_TIFRFE | SPI_I2S_IT_TXE);
		SPI_I2S_ClearFlag(SPI3, SPI_I2S_FLAG_BSY | SPI_I2S_FLAG_OVR | SPI_I2S_FLAG_RXNE | SPI_I2S_FLAG_TIFRFE | SPI_I2S_FLAG_TXE);

		palSetPadMode(GPIOC, 10, PAL_MODE_ALTERNATE(GPIO_AF_SPI3) | PAL_STM32_OTYPE_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
		palSetPadMode(GPIOC, 11, PAL_MODE_ALTERNATE(GPIO_AF_SPI3) | PAL_STM32_OTYPE_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
		palSetPadMode(GPIOC, 12, PAL_MODE_ALTERNATE(GPIO_AF_SPI3) | PAL_STM32_OTYPE_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

		SPI_InitTypeDef SPI_InitStructure;
		SPI_StructInit(&SPI_InitStructure);
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
		SPI_InitStructure.SPI_CRCPolynomial = 7;
		//SPI3 - SPI_BaudRatePrescaler_64  - 0.666 MHz
		//SPI3 - SPI_BaudRatePrescaler_32  - 1.333 MHz
		//SPI3 - SPI_BaudRatePrescaler_16  - 2.666 MHz
		//SPI3 - SPI_BaudRatePrescaler_8   - 5.333 MHz
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
		SPI_Init(SPI3, &SPI_InitStructure);
		SPI_Cmd(SPI3, ENABLE);

		SPI_NSSInternalSoftwareConfig(SPI3, SPI_NSSInternalSoft_Set);
#endif
#if SPI_MODE == 2
        palSetPadMode(NRF_PORT_SCK, NRF_PIN_SCK, PAL_MODE_ALTERNATE(6) | PAL_STM32_OSPEED_HIGHEST);
        palSetPadMode(NRF_PORT_MISO, NRF_PIN_MISO, PAL_MODE_ALTERNATE(6));
        palSetPadMode(NRF_PORT_MOSI, NRF_PIN_MOSI, PAL_MODE_ALTERNATE(6) | PAL_STM32_OSPEED_HIGHEST);

        spiStart(&SPID3, &spicfg);
#endif
		palSetPad(NRF_PORT_CSN, NRF_PIN_CSN);

		palSetPad(NRF_PORT_CE, NRF_PIN_CE);

		init_done = true;
	}
}

void spi_sw_transfer(uint8_t *in_buf, const uint8_t *out_buf, uint32_t length)
{
#if SPI_MODE == 2
	spiExchange(&SPID3, length, ((out_buf == 0) ? zerodata : out_buf), ((in_buf == 0) ? trashdata : in_buf));
#endif
#if SPI_MODE == 1
	for(uint32_t i=0; i<length; i++)
	{
		if(out_buf)
			SPI3->DR = out_buf[i];
		else
			SPI3->DR = 0;

		palClearPad(GPIOA, 4);


		while(!(SPI3->SR & SPI_I2S_FLAG_TXE));  // wait until transmit complete
		while(!(SPI3->SR & SPI_I2S_FLAG_RXNE));
		while(SPI3->SR & SPI_I2S_FLAG_BSY);  // wait until SPI is not busy anymore

		in_buf[i] = SPI3->DR;

		if(in_buf)
		{


			palSetPad(GPIOA, 4);
			spi_sw_delay();
		}
	}
#endif
#if SPI_MODE == 0
	palClearPad(NRF_PORT_SCK, NRF_PIN_SCK);
	spi_sw_delay();


	for (uint32_t i = 0;i < length;i++) {
		unsigned char send = out_buf ? out_buf[i] : 0;
		unsigned char recieve = 0;

		palSetPad(GPIOA, 4);
		for(int bit = 0; bit < 8; bit++)
		{
			palWritePad(NRF_PORT_MOSI, NRF_PIN_MOSI, send >> 7);
			send <<= 1;

			spi_sw_delay();

			palSetPad(NRF_PORT_SCK, NRF_PIN_SCK);

			recieve <<= 1;
			if(palReadPad(NRF_PORT_MISO, NRF_PIN_MISO))
				recieve |= 0x1;

			spi_sw_delay();
			palClearPad(NRF_PORT_SCK, NRF_PIN_SCK);
		}
		palClearPad(GPIOA, 4);

		if (in_buf) {
			in_buf[i] = recieve;
		}
	}
#endif
}

void spi_sw_begin(void)
{

#if SPI_MODE == 2
	spiSelect(&SPID3);
#else
	palClearPad(NRF_PORT_CSN, NRF_PIN_CSN);
#endif
	palSetPad(GPIOA, 3);
	spi_sw_delay();

}

void spi_sw_end(void)
{
	spi_sw_delay();
	palClearPad(GPIOA, 3);

#if SPI_MODE == 2
	spiUnselect(&SPID3);
#else
	palSetPad(NRF_PORT_CSN, NRF_PIN_CSN);
#endif
}

static void spi_sw_delay(void) {
	for (volatile int i = 0;i < 50;i++) {
		__NOP();
	}
}
