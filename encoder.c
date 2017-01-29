/*
	Copyright 2012-2016 Benjamin Vedder	benjamin@vedder.se

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

#include "encoder.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "hw.h"
#include "utils.h"
#include "debug.h"
#include "string.h"
#include "mc_interface.h"


#if 0
// Defines
#define AS5047P_READ_ANGLECOM		(0x3FFF | 0x4000 | 0x8000) // This is just ones
#define AS5047_SAMPLE_RATE_HZ		10000

#define SPI_SW_MISO_GPIO			HW_HALL_ENC_GPIO2
#define SPI_SW_MISO_PIN				HW_HALL_ENC_PIN2
#define SPI_SW_SCK_GPIO				HW_HALL_ENC_GPIO1
#define SPI_SW_SCK_PIN				HW_HALL_ENC_PIN1
#define SPI_SW_CS_GPIO				HW_HALL_ENC_GPIO3
#define SPI_SW_CS_PIN				HW_HALL_ENC_PIN3

// Private types
typedef enum {
	ENCODER_MODE_NONE = 0,
	ENCODER_MODE_ABI,
	ENCODER_MODE_AS5047P_SPI
} encoder_mode;

// Private variables
static bool index_found = false;
static uint32_t enc_counts = 10000;
static encoder_mode mode = ENCODER_MODE_NONE;
static float last_enc_angle = 0.0;

// Private functions
uint16_t spi_exchange(uint16_t x);
static void spi_transfer(uint16_t *in_buf, const uint16_t *out_buf, int length);
static void spi_begin(void);
static void spi_end(void);
static void spi_delay(void);

void encoder_deinit(void) {
	nvicDisableVector(HW_ENC_EXTI_CH);
	nvicDisableVector(HW_ENC_TIM_ISR_CH);

	TIM_DeInit(HW_ENC_TIM);

	palSetPadMode(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(SPI_SW_CS_GPIO, SPI_SW_CS_PIN, PAL_MODE_INPUT_PULLUP);

	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);

	index_found = false;
	mode = ENCODER_MODE_NONE;
	last_enc_angle = 0.0;
}

void encoder_init_abi(uint32_t counts) {
//	EXTI_InitTypeDef   EXTI_InitStructure;
//
//	// Initialize variables
//	index_found = false;
//	enc_counts = counts;
//
//	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1,
//			PAL_MODE_ALTERNATE(HW_ENC_TIM_AF) |
//			PAL_STM32_OSPEED_HIGHEST |
//			PAL_STM32_PUDR_FLOATING);
//
//	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2,
//			PAL_MODE_ALTERNATE(HW_ENC_TIM_AF) |
//			PAL_STM32_OSPEED_HIGHEST |
//			PAL_STM32_PUDR_FLOATING);
//
//	// Enable timer clock
//	HW_ENC_TIM_CLK_EN();
//
//	// Enable SYSCFG clock
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
//
//	TIM_EncoderInterfaceConfig (HW_ENC_TIM, TIM_EncoderMode_TI12,
//			TIM_ICPolarity_Rising,
//			TIM_ICPolarity_Rising);
//	TIM_SetAutoreload(HW_ENC_TIM, enc_counts - 1);
//
//	TIM_Cmd(HW_ENC_TIM, ENABLE);
//
//	// Interrupt on index pulse
//
//	// Connect EXTI Line to pin
//	SYSCFG_EXTILineConfig(HW_ENC_EXTI_PORTSRC, HW_ENC_EXTI_PINSRC);
//
//	// Configure EXTI Line
//	EXTI_InitStructure.EXTI_Line = HW_ENC_EXTI_LINE;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);
//
//	// Enable and set EXTI Line Interrupt to the highest priority
//	nvicEnableVector(HW_ENC_EXTI_CH, 0);

	mode = ENCODER_MODE_ABI;
}

void encoder_init_as5047p_spi(void) {
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	palSetPadMode(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN, PAL_MODE_INPUT);
	palSetPadMode(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(SPI_SW_CS_GPIO, SPI_SW_CS_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

	// Enable timer clock
	HW_ENC_TIM_CLK_EN();

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = ((168000000 / 2 / AS5047_SAMPLE_RATE_HZ) - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(HW_ENC_TIM, &TIM_TimeBaseStructure);

	// Enable overflow interrupt
	TIM_ITConfig(HW_ENC_TIM, TIM_IT_Update, ENABLE);

	// Enable timer
	TIM_Cmd(HW_ENC_TIM, ENABLE);

	nvicEnableVector(HW_ENC_TIM_ISR_CH, 6);

	mode = ENCODER_MODE_AS5047P_SPI;
	index_found = true;
}

bool encoder_is_configured(void) {
	return mode != ENCODER_MODE_NONE;
}

float encoder_read_deg(void) {
	static float angle = 0.0;

	switch (mode) {
	case ENCODER_MODE_ABI:
		angle = ((float)HW_ENC_TIM->CNT * 360.0) / (float)enc_counts;
		break;

	case ENCODER_MODE_AS5047P_SPI:
		angle = last_enc_angle;
		break;

	default:
		break;
	}

	return angle;
}

/**
 * Reset the encoder counter. Should be called from the index interrupt.
 */
void encoder_reset(void) {
	HW_ENC_TIM->CNT = 0;
	index_found = true;
}

/**
 * Timer interrupt
 */
void encoder_tim_isr(void) {
	uint16_t pos;

	spi_begin();
	spi_transfer(&pos, 0, 1);
	spi_end();

	pos &= 0x3FFF;
	last_enc_angle = ((float)pos * 360.0) / 16384.0;
}

/**
 * Set the number of encoder counts.
 *
 * @param counts
 * The number of encoder counts
 */
void encoder_set_counts(uint32_t counts) {
	if (counts != enc_counts) {
		enc_counts = counts;
		TIM_SetAutoreload(HW_ENC_TIM, enc_counts - 1);
		index_found = false;
	}
}

/**
 * Check if the index pulse is found.
 *
 * @return
 * True if the index is found, false otherwise.
 */
bool encoder_index_found(void) {
	return index_found;
}

// Software SPI
uint16_t spi_exchange(uint16_t x) {
	uint16_t rx;
	spi_transfer(&rx, &x, 1);
	return rx;
}

static void spi_transfer(uint16_t *in_buf, const uint16_t *out_buf, int length) {
	for (int i = 0;i < length;i++) {
		uint16_t send = out_buf ? out_buf[i] : 0xFFFF;
		uint16_t recieve = 0;

		for (int bit = 0;bit < 16;bit++) {
			//palWritePad(HW_SPI_PORT_MOSI, HW_SPI_PIN_MOSI, send >> 15);
			send <<= 1;

			spi_delay();
			palSetPad(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN);
			spi_delay();

			recieve <<= 1;
			if (palReadPad(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN)) {
				recieve |= 1;
			}

			palClearPad(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN);
			spi_delay();
		}

		if (in_buf) {
			in_buf[i] = recieve;
		}
	}
}

static void spi_begin(void) {
	palClearPad(SPI_SW_CS_GPIO, SPI_SW_CS_PIN);
}

static void spi_end(void) {
	palSetPad(SPI_SW_CS_GPIO, SPI_SW_CS_PIN);
}

static void spi_delay(void) {
	__NOP();
	__NOP();
	__NOP();
	__NOP();
}
#else
// Defines
#define AS5047_SAMPLE_RATE_HZ		5000

// Private types
typedef enum {
	ENCODER_MODE_NONE = 0,
	ENCODER_MODE_ABI,
	ENCODER_MODE_AS5047P_SPI
} encoder_mode;

#define CDM_0 	0
#define CDM_1 	1

// Private variables
static bool index_found = false;
static uint32_t enc_counts = 10000;
static encoder_mode mode = ENCODER_MODE_NONE;
static float last_enc_angle = 0.0;


/*******************************************************************************
 * Function Name : ICMU_delay_loop
 *******************************************************************************/
static void ICMU_delay_loop(__IO uint32_t nCount)
{
	nCount *= /*200*/100;
	for(; nCount != 0; nCount--);
}

/*******************************************************************************
 * Function Name : ICMU_delay_loop_reduced
 *******************************************************************************/
static void ICMU_delay_loop_reduced(__IO uint32_t nCount)
{
	nCount *= /*200*/35;
	for(; nCount != 0; nCount--);
}

/*******************************************************************************
 * Function Name : ICMU_MakeCRC_4
 * Description	 : Calculates CRC
 *******************************************************************************/
static uint8_t ICMU_MakeCRC_4(uint16_t data, uint8_t len)
{
	uint8_t CRCOut; // CRC Result
	uint16_t CRCData[4] = { 0, 0, 0, 0 };
	uint16_t doInvert;

	for (uint8_t i = 0; i < len; ++i) {
		doInvert = ((data >> ((len - 1) - i)) & 0x01) ^ CRCData[3];

		CRCData[3] = CRCData[2];
		CRCData[2] = CRCData[1];
		CRCData[1] = CRCData[0] ^ doInvert;
		CRCData[0] = doInvert;
	}

	CRCOut = (CRCData[3] << 3) | (CRCData[2] << 2) | (CRCData[1] << 1)
			| CRCData[0];
	CRCOut = (~CRCOut) & 0x0F;

	return CRCOut;
}


/*******************************************************************************
 * Function Name : ICMU_Frame
 * Description	 : Process one BISS frame (use to read/write position)
 *******************************************************************************/
uint32_t ICMU_Frame(uint8_t CDM)
{
#define SCK_S1_1 	palSetPad(GPIOB, 6)
#define SCK_S1_0 	palClearPad(GPIOB, 6)

	uint32_t Data = 0;

#define FRAME_LEN	26

	for(uint8_t k = 0; k < FRAME_LEN; k++) //26
	{
		SCK_S1_1;
		ICMU_delay_loop(1);
		SCK_S1_0;
		ICMU_delay_loop(1);

		if(palReadPad(GPIOB, 7) != 0) //SLO
			Data |= (1 << (FRAME_LEN - 1 - k)); //26
	}

	SCK_S1_1;
	ICMU_delay_loop(1);

	if(CDM == 0)
		SCK_S1_1;
	else
		SCK_S1_0;

	ICMU_delay_loop(20);
	SCK_S1_1;
	ICMU_delay_loop(30);

	return Data;
}

/*******************************************************************************
 * Function Name : ICMU_FrameReduced
 * Description	 : Use to get the current encoder position
 *******************************************************************************/
uint32_t ICMU_FrameReduced(void)
{
	uint32_t Data = 0;

	chSysLock();

//    palSetPad(GPIOC, 8);

	for(uint8_t k = 0; k < FRAME_LEN; k++) //26
	{
		SCK_S1_1;
		ICMU_delay_loop_reduced(1);
		SCK_S1_0;
		ICMU_delay_loop_reduced(1);

		if(palReadPad(GPIOB, 7) != 0) //SLO
			Data |= (1 << (FRAME_LEN - 1 - k)); //26
	}

//    palClearPad(GPIOC, 8);

	chSysUnlock();

//	SCK_S1_1;
//	ICMU_delay_loop_reduced(1);
//	SCK_S1_1;
//	ICMU_delay_loop_reduced(20);
//	SCK_S1_1;
//
//	ICMU_delay_loop_reduced(30);

	return Data;
}


/*******************************************************************************
 * Function Name : ICMU_ReadData
 * Description	 : Read 8bit values
 *******************************************************************************/
uint8_t ICMU_ReadData(uint16_t addr)
{
	unsigned char Result_CRC, k;
	unsigned int S, CTS, ID, ADR, R, W;
	unsigned int Data;
	long BufferTx;
	uint32_t CDS, DataReg;

	S = 1;
	CTS = 1;
	ID = 0x00;
	ADR = addr;
	R = 1;
	W = 0;
	Data = (CTS << 10) | (ID << 6) | (ADR << 0);
	Result_CRC = ICMU_MakeCRC_4(Data, 11); // Calculate CRC

	BufferTx = 0;
	BufferTx = (S << 17) | (CTS << 16) | (ID << 13) | (ADR << 6) | (Result_CRC << 2) | (R << 1) | W;

	chSysLock();

	k = 15;
	while(k != 0)
	{
		ICMU_Frame(CDM_0);
		k--;
	}

	uint8_t i = 18;

	while(i != 0)
	{
		if(((BufferTx >> (i - 1)) & 0x01) != 0)
			CDS = ICMU_Frame(CDM_1);
		else
			CDS = ICMU_Frame(CDM_0);

		i--;
	}

	S = 1;
	Data = 0x0;
	Result_CRC = ICMU_MakeCRC_4(Data, 8); // Calculate CRC

	BufferTx = 0;
	BufferTx = (S << 13) | (Data << 5) | (Result_CRC << 1) | 0;

	DataReg = 0;
	i = 14;
	while(i != 0)
	{
		if(((BufferTx >> (i - 1)) & 0x01) != 0)
			CDS = ICMU_Frame(CDM_1);
		else
			CDS = ICMU_Frame(CDM_0);

		DataReg = DataReg | (((CDS >> 21) & 0x01) << (i - 1));
		i--;
	}

	chSysUnlock();

	DataReg = (DataReg >> 4) & 0xff;

	return (DataReg);
}

/*******************************************************************************
 * Function Name : BISS_WriteData
 * Description	 : Write 8bit values
 *******************************************************************************/
void BISS_WriteData(uint16_t addr, uint16_t data)
{
	uint8_t CRCData;
	uint16_t S, CTS, ID, ADR, R, W;
	uint16_t Data;
	uint32_t BufferTx;
	uint32_t CDS, DataReg;

	S = 1;
	CTS = 1;
	ID = 0x00;
	ADR = addr;

	R = 0;
	W = 1;

	Data = (CTS << 10) | (ID << 6) | (ADR << 0);
	CRCData = ICMU_MakeCRC_4(Data, 11); // Calculate CRC

	BufferTx = 0;
	BufferTx = (S << 17) | (CTS << 16) | (ID << 13) | (ADR << 6) | (CRCData << 2) | (R << 1) | W;

	chSysLock();

	for(uint8_t i = 0; i < 15; i++)
		ICMU_Frame(CDM_0);

	for(uint8_t i = 18; i != 0; i--)
	{
		if(((BufferTx >> (i - 1)) & 0x01) != 0)
			ICMU_Frame(CDM_1);
		else
			ICMU_Frame(CDM_0);
	}

	S = 1;
	Data = data;
	CRCData = ICMU_MakeCRC_4(Data, 8); // Calculate CRC

	BufferTx = 0;
	BufferTx = (S << 13) | (Data << 5) | (CRCData << 1) | 0;

	DataReg = 0;

	for(uint8_t i = 14; i != 0; i--)
	{
		if(((BufferTx >> (i - 1)) & 0x01) != 0)
			CDS = ICMU_Frame(CDM_1);
		else
			CDS = ICMU_Frame(CDM_0);

		DataReg = DataReg | (((CDS >> 21) & 0x01) << (i - 1));
	}

	chSysUnlock();

	DataReg = (DataReg >> 4) & 0xff;
}


/*******************************************************************************
 * Function Name : ICMU_ReadPRES_POS
 * Description   :
 *******************************************************************************/
uint16_t ICMU_GetAbsPos()
{
	return (ICMU_FrameReduced() >> 9) & 0x0FFF;

//	uint32_t value = ICMU_FrameReduced(); 	//read data from IC-MU150
//
//
//	value = value >> 9; 					//2 LSB bits contain nothing (0)
//	value &= 0x000000000007FFFF; 			//mask 19 data-bits
//
//
//	int16_t signedValue = 0;
//
//	signedValue = (value & 0x0FFF);
////	if(BitIsSet(value, 18))
////		signedValue = (value & 0x0FFF);
////	else
////		signedValue = (value & 0x0003FFFF) | ~((1 << 18) - 1);
//
////	signedValue >>= (19-bitRate);
//
//	uint16_t absValue = signedValue;// + (int32_t)(1 << (bitRate - 1));
//
//	return absValue;
}

void encoder_deinit(void) {
	nvicDisableVector(HW_ENC_EXTI_CH);
	nvicDisableVector(HW_ENC_TIM_ISR_CH);

	TIM_DeInit(HW_ENC_TIM);

//	palSetPadMode(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN, PAL_MODE_INPUT_PULLUP);
//	palSetPadMode(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN, PAL_MODE_INPUT_PULLUP);
//	palSetPadMode(SPI_SW_CS_GPIO, SPI_SW_CS_PIN, PAL_MODE_INPUT_PULLUP);
//
//	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
//	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);

	index_found = false;
	mode = ENCODER_MODE_NONE;
	last_enc_angle = 0.0;
}

void encoder_init_abi(uint32_t counts) {
    (void)counts;
//	EXTI_InitTypeDef   EXTI_InitStructure;
//
//	// Initialize variables
//	index_found = false;
//	enc_counts = counts;
//
//	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1,
//			PAL_MODE_ALTERNATE(HW_ENC_TIM_AF) |
//			PAL_STM32_OSPEED_HIGHEST |
//			PAL_STM32_PUDR_FLOATING);
//
//	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2,
//			PAL_MODE_ALTERNATE(HW_ENC_TIM_AF) |
//			PAL_STM32_OSPEED_HIGHEST |
//			PAL_STM32_PUDR_FLOATING);
//
//	// Enable timer clock
//	HW_ENC_TIM_CLK_EN();
//
//	// Enable SYSCFG clock
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
//
//	TIM_EncoderInterfaceConfig (HW_ENC_TIM, TIM_EncoderMode_TI12,
//			TIM_ICPolarity_Rising,
//			TIM_ICPolarity_Rising);
//	TIM_SetAutoreload(HW_ENC_TIM, enc_counts - 1);
//
//	TIM_Cmd(HW_ENC_TIM, ENABLE);
//
//	// Interrupt on index pulse
//
//	// Connect EXTI Line to pin
//	SYSCFG_EXTILineConfig(HW_ENC_EXTI_PORTSRC, HW_ENC_EXTI_PINSRC);
//
//	// Configure EXTI Line
//	EXTI_InitStructure.EXTI_Line = HW_ENC_EXTI_LINE;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);
//
//	// Enable and set EXTI Line Interrupt to the highest priority
//	nvicEnableVector(HW_ENC_EXTI_CH, 0);

	mode = ENCODER_MODE_ABI;
}

void encoder_init_as5047p_spi(void) {
	return;

//	DebugSendString("ENC STARTED");
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//	palSetPadMode(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN, PAL_MODE_INPUT);
//	palSetPadMode(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
//	palSetPadMode(SPI_SW_CS_GPIO, SPI_SW_CS_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

	palSetPadMode(GPIOB, 6, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST); //b6-ma
	palSetPadMode(GPIOB, 7, PAL_MODE_INPUT); //b7 - slo


	// Enable timer clock
	HW_ENC_TIM_CLK_EN();

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = ((168000000 / 2 / AS5047_SAMPLE_RATE_HZ) - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(HW_ENC_TIM, &TIM_TimeBaseStructure);

	// Enable overflow interrupt
	TIM_ITConfig(HW_ENC_TIM, TIM_IT_Update, ENABLE);

	// Enable timer
	TIM_Cmd(HW_ENC_TIM, ENABLE);

	nvicEnableVector(HW_ENC_TIM_ISR_CH, 6);

	mode = ENCODER_MODE_AS5047P_SPI;
	index_found = true;

	BISS_WriteData(0x06, 0x82);
	BISS_WriteData(0x01, 0x80);
}

bool encoder_is_configured(void) {
	return mode != ENCODER_MODE_NONE;
}

float encoder_read_deg(void) {
	static float angle = 0.0;

	switch (mode) {
	case ENCODER_MODE_ABI:
		angle = ((float)HW_ENC_TIM->CNT * 360.0) / (float)enc_counts;
		break;

	case ENCODER_MODE_AS5047P_SPI:
		angle = last_enc_angle;
		break;

	default:
		break;
	}

	return angle;
}

/**
 * Reset the encoder counter. Should be called from the index interrupt.
 */
void encoder_reset(void) {
	HW_ENC_TIM->CNT = 0;
	index_found = true;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	long out = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	if(out>out_max)
		return out_max;
	if(out < out_min)
		return out_min;
	return out;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
    float out = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    if(out>out_max)
        return out_max;
    if(out < out_min)
        return out_min;
    return out;
}

/**
 * Timer interrupt
 */
void encoder_tim_isr(void)
{
//	uint16_t pos;

//	spi_begin();
//	spi_transfer(&pos, 0, 1);
//	spi_end();

//	pos &= 0x3FFF;
//	static uint8_t dir = 0;
//	static uint16_t prev = 0;
//	static uint16_t prevphase = 0;

//	uint16_t cur = phase();
//
//	static uint16_t phaserLast = 0;


	last_enc_angle = ((float) ICMU_GetAbsPos() * 360.0) / 4096.0;

//
////
//	static uint32_t cnt = 0;
//	if(++cnt >= 3000)
//	{
//		DebugSendChar('+');
//		cnt = 0;
////		char rrr[80] = { 0 }, ff[20];
////		itoa_(ADC_HALL_1, ff);
////		stpcpy(rrr, ff);
////		stpcpy(rrr, " - ");
////		itoa_(ADC_HALL_2, ff);
////		stpcpy(rrr, ff);
////		stpcpy(rrr, " - ");
////		itoa_(ADC_HALL_3, ff);
////		stpcpy(rrr, ff);
////		stpcpy(rrr, " - ");
////		DebugSendString(rrr);
//
////		DebugSendNumSpace(ADC_HALL_1);
////		DebugSendNumSpace(ADC_HALL_2);
////		DebugSendNumSpace(ADC_HALL_3);
//////		DebugSendNumSpace(phase());
//////		DebugSendNumSpace(phase7());
////
////		for(uint8_t i=0; i<last_enc_angle/3; i++)
////			DebugSendChar(/*dir==0?'-':'+'*/'*');
////		DebugSendNumSpace((uint16_t) last_enc_angle);
////		//DebugSendNumSpace(phaserLast);
//
//
//		//DebugSendNumWDescBin("x=", ICMU_GetAbsPos(), 26);
//		DebugSendNum(last_enc_angle*100);
//
//		DebugSendChar('\n');
//
//
//
////		DebugSendNumWDesc("enac 0x01 gain = ", ICMU_ReadData(0x01));
////		DebugSendNumWDesc("Stat 0x76 gain = ", ICMU_ReadData(0x76));
////
////		DebugSendNumWDesc("Stat 0x77 = ", ICMU_ReadData(0x77));
////		DebugSendNumWDesc("res = ", ICMU_ReadData(0x06));
////		DebugSendChar('\n');
//	}
}

/**
 * Set the number of encoder counts.
 *
 * @param counts
 * The number of encoder counts
 */
void encoder_set_counts(uint32_t counts) {
	if (counts != enc_counts) {
		enc_counts = counts;
		TIM_SetAutoreload(HW_ENC_TIM, enc_counts - 1);
		index_found = false;
	}
}

/**
 * Check if the index pulse is found.
 *
 * @return
 * True if the index is found, false otherwise.
 */
bool encoder_index_found(void) {
	return index_found;
}

#endif
