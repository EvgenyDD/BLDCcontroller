/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEBUG_H
#define DEBUG_H


/* Includes ------------------------------------------------------------------*/
#include <stm32f4xx.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void DebugInit(void);
void DebugSendString(char*);
void DebugSendStringSpace(char *pText);
void DebugSendChar(char);
void DebugSendNum(uint16_t Num);
void DebugSendNumSpace(uint16_t Num);
void DebugSendNumWDesc(char *string, uint32_t Num);

void ftoa_(float num, char str[], char precision);
void dtoa_(uint32_t n, char s[]);
void itoa_(int32_t n, char s[]);
void itoaChar_(uint8_t n, char s[]);
void DebugSendNumChar(uint8_t Num);
void reverse(char s[]);

#endif //DEBUG_H
