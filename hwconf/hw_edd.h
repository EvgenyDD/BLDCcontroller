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

#ifndef HW_EDD_H_
#define HW_EDD_H_

// Macros
#define ENABLE_GATE()				palSetPad(GPIOE, 14)
#define DISABLE_GATE()				palClearPad(GPIOE, 14)
#define DCCAL_ON()					palSetPad(GPIOE, 15)
#define DCCAL_OFF()					palClearPad(GPIOE, 15)
#define IS_DRV_FAULT()				(!palReadPad(GPIOE, 13))

#define LED_GREEN_ON()				palSetPad(GPIOE, 4)
#define LED_GREEN_OFF()				palClearPad(GPIOE, 4)
#define LED_RED_ON()				palSetPad(GPIOE, 5)
#define LED_RED_OFF()				palClearPad(GPIOE, 5)

#define RELAY_TURN_ON()				do{palSetPad(GPIOB, 2);	palClearPad(GPIOE, 7);}while(0);

#define RELAY_TURN_OFF()			do{palSetPad(GPIOE, 7);	palClearPad(GPIOB, 2);}while(0);

#define RELAY_RELEASE()				do{palClearPad(GPIOB, 2);palClearPad(GPIOE, 7);}while(0);

/*
 * ADC Vector
 */

#define HW_ADC_CHANNELS				18
#define HW_ADC_NBR_CONV				6

// ADC Indexes
#define ADC_IND_SENS1				2
#define ADC_IND_SENS2				1
#define ADC_IND_SENS3				0
#define ADC_IND_CURR1				4
#define ADC_IND_CURR2				3
#define ADC_IND_VIN_SENS			5

#define ADC_IND_CELL1				12
#define ADC_IND_CELL2				7
#define ADC_IND_CELL3				10
#define ADC_IND_CELL4				16
#define ADC_IND_CELL5				8
#define ADC_IND_CELL6				11

#define ADC_IND_TEMP_MOTOR			9
#define ADC_IND_TEMP_INVRT			13

#define ADC_IND_TEMP_PACK1			14
#define ADC_IND_TEMP_PACK2			15
//#define ADC_IND_VREFINT				6


// ADC macros and settings

// Component parameters (can be overridden)
#ifndef V_REG
//#define V_REG				3.3
#endif
#ifndef VIN_R1
#define VIN_R1				39000.0
#endif
#ifndef VIN_R2
#define VIN_R2				2200.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN	10.0
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES	0.001
#endif



// Input voltage
#define GET_INPUT_VOLTAGE()	((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// Voltage on ADC channel
#define ADC_VOLTS(ch)		((float)ADC_Value[ch] / 4095.0 * V_REG)

#define GET_CELL_POINT_1()  ((float)ADC_Value[ADC_IND_CELL1] / 727.754)
#define GET_CELL_POINT_2()  ((float)ADC_Value[ADC_IND_CELL2] / 342.91)
#define GET_CELL_POINT_3()  ((float)ADC_Value[ADC_IND_CELL3] / 192.32)
#define GET_CELL_POINT_4()  ((float)ADC_Value[ADC_IND_CELL4] / 163.69)
#define GET_CELL_POINT_5()  ((float)ADC_Value[ADC_IND_CELL5] / 118.763)
#define GET_CELL_POINT_6()  ((float)ADC_Value[ADC_IND_CELL6] / 124.616)

#define GET_CELL1()         (GET_CELL_POINT_1() - 0.0)
#define GET_CELL2()         (GET_CELL_POINT_2() - GET_CELL_POINT_1())
#define GET_CELL3()         (GET_CELL_POINT_3() - GET_CELL_POINT_2())
#define GET_CELL4()         (GET_CELL_POINT_4() - GET_CELL_POINT_3())
#define GET_CELL5()         (GET_CELL_POINT_5() - GET_CELL_POINT_4())
#define GET_CELL6()         (GET_CELL_POINT_6() - GET_CELL_POINT_5())

// NTC Termistors
//#define NTC_RES(adc_val)	(10000.0 / ((4096.0 / (float)adc_val) - 1.0))
//#define NTC_RES(adc_val)	((4095.0 * 47000.0) / adc_val - 51000.0) //NTC -> high side
#define NTC_RES(adc_val)	((adc_val * 47000.0) / (4095.0 - adc_val)) //NTC -> lo side
#define NTC_TEMP(adc_ind)	(1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 47000.0) / 3950.0) + (1.0 / (273.15 + 25.0))) - 273.15)

// Double samples in beginning and end for positive current measurement.
// Useful when the shunt sense traces have noise that causes offset.
#ifndef CURR1_DOUBLE_SAMPLE
#define CURR1_DOUBLE_SAMPLE	0
#endif
#ifndef CURR2_DOUBLE_SAMPLE
#define CURR2_DOUBLE_SAMPLE	0
#endif

// Number of servo outputs
#define HW_SERVO_NUM		0

// UART Peripheral - DEBUG
#define HW_UART_DEV			UARTD3
#define HW_UART_GPIO_AF		GPIO_AF_USART3
#define HW_UART_TX_PORT		GPIOB
#define HW_UART_TX_PIN		6
#define HW_UART_RX_PORT		GPIOB
#define HW_UART_RX_PIN		7

//// UART Peripheral - GPS
//#define HW_UART_DEV			UARTD3
//#define HW_UART_GPIO_AF		GPIO_AF_USART3
//#define HW_UART_TX_PORT		GPIOB
//#define HW_UART_TX_PIN		8
//#define HW_UART_RX_PORT		GPIOB
//#define HW_UART_RX_PIN		9


// ICU Peripheral for servo decoding
#define HW_ICU_DEV			ICUD3
#define HW_ICU_CHANNEL		ICU_CHANNEL_2
#define HW_ICU_GPIO_AF		GPIO_AF_TIM4
#define HW_ICU_GPIO			GPIOC
#define HW_ICU_PIN			7


// I2C Peripheral
#define HW_I2C_DEV			I2CD2
#define HW_I2C_GPIO_AF		GPIO_AF_I2C2
#define HW_I2C_SCL_PORT		GPIOB
#define HW_I2C_SCL_PIN		10
#define HW_I2C_SDA_PORT		GPIOB
#define HW_I2C_SDA_PIN		11

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOB
#define HW_HALL_ENC_PIN1		9
#define HW_HALL_ENC_GPIO2		GPIOE
#define HW_HALL_ENC_PIN2		3
#define HW_HALL_ENC_GPIO3		GPIOE
#define HW_HALL_ENC_PIN3		2

#define HW_ENC_TIM				TIM4
#define HW_ENC_TIM_AF			GPIO_AF_TIM4
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOE
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource0
#define HW_ENC_EXTI_CH			EXTI0_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line0
#define HW_ENC_EXTI_ISR_VEC		EXTI5_IRQHandler

#define HW_ENC_TIM_ISR_CH		TIM4_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM4_IRQHandler

// NRF pins
#define NRF_PORT_CSN			GPIOA
#define NRF_PIN_CSN				15
#define NRF_PORT_SCK			GPIOC
#define NRF_PIN_SCK				10
#define NRF_PORT_MOSI			GPIOC
#define NRF_PIN_MOSI			12
#define NRF_PORT_MISO			GPIOC
#define NRF_PIN_MISO			11
#define NRF_PORT_CE				GPIOD
#define NRF_PIN_CE				2
#define NRF_PORT_IRQ			GPIOD
#define NRF_PIN_IRQ				0
#define NRF_EXTI_LINE			EXTI_Line0

// SPI pins
#if 0
#define HW_SPI_DEV				SPID1
#define HW_SPI_GPIO_AF			GPIO_AF_SPI1
#define HW_SPI_PORT_NSS			GPIOA
#define HW_SPI_PIN_NSS			4
#define HW_SPI_PORT_SCK			GPIOA
#define HW_SPI_PIN_SCK			5
#define HW_SPI_PORT_MOSI		GPIOA
#define HW_SPI_PIN_MOSI			7
#define HW_SPI_PORT_MISO		GPIOA
#define HW_SPI_PIN_MISO			6
#endif

#define DRV8301_SPI_DEV			SPID1
#define DRV8301_SPI_GPIO_AF		GPIO_AF_SPI1
#define DRV8301_SPI_PORT_NSS	GPIOB
#define DRV8301_SPI_PIN_NSS		12
#define DRV8301_SPI_PORT_SCK	GPIOB
#define DRV8301_SPI_PIN_SCK		3
#define DRV8301_SPI_PORT_MOSI	GPIOB
#define DRV8301_SPI_PIN_MOSI	5
#define DRV8301_SPI_PORT_MISO	GPIOB
#define DRV8301_SPI_PIN_MISO	4

// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

#define ADC_HALL_1				ADC_Value[9]
#define ADC_HALL_2				ADC_Value[10]
#define ADC_HALL_3				ADC_Value[7]

// Macros
#if 1
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)
#endif

#endif /* HW_EDD_H_ */
