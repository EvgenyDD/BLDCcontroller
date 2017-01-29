/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t DebugDecrease;

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : reverse
* Description    : Reverses string
* Input          : pointer to string
*******************************************************************************/
void reverse(char s[])
{
	int c, i, j;
	for (i = 0, j = strlen(s)-1; i < j; i++, j--)
	{
		c = s[i];
		s[i] = s[j];
		s[j] = c;
	}
}



/*******************************************************************************
* Function Name  : itoa
* Description    : Convert int to char
* Input          : int number (signed/unsigned)
* Return         : pointer to text string
*******************************************************************************/
void itoa_(int32_t n, char s[])
{
	int sign;

	if ((sign = n) < 0)
		n = -n;

	int i = 0;

	do {
		s[i++] = n % 10 + '0';
	}
	while ((n /= 10) > 0);

	if (sign < 0) s[i++] = '-';

	s[i] = '\0';
	reverse(s);
}

/*******************************************************************************
* Function Name  : itoa
* Description    : Convert int to char
* Input          : int number (signed/unsigned)
* Return         : pointer to text string
*******************************************************************************/
void itoaChar_(uint8_t n, char s[])
{
	int i = 0;

	do {
		s[i++] = n % 10 + '0';
	}
	while ((n /= 10) > 0);

	s[i] = '\0';
	reverse(s);
}

/*******************************************************************************
* Function Name  : itoa
* Description    : Convert int to char
* Input          : int number (signed/unsigned)
* Return         : pointer to text string
*******************************************************************************/
void dtoa_(uint32_t n, char s[])
{
	int sign;

	if ((sign = n) < 0)
		n = -n;

	int i = 0;

	do {
		s[i++] = n % 10 + '0';
	}
	while ((n /= 10) > 0);

	if (sign < 0) s[i++] = '-';

	s[i] = '\0';
	reverse(s);
}


/*******************************************************************************
* Function Name  : ftoa_
* Description    : Convert float to char
* Input          : float number, char, output precision
* Return         : pointer to text string
*******************************************************************************/
//void ftoa_(float num, char str[], char precision)
//{
//	unsigned char zeroFlag=0;
//	int digit=0, reminder=0;
//	long wt=0;
//
//	if(num < 0)
//	{
//		num = -num;
//		zeroFlag = 1;
//	}
//
//	int whole_part = num;
//	int log_value=log10_(num), index=log_value;
//
//	if(zeroFlag) str[0]='-';
//
//	//Extract the whole part from float num
//	for(int i=1; i<log_value+2; i++)
//	{
//		wt = pow_(10.0, i);
//		reminder = whole_part % wt;
//		digit = (reminder - digit) / (wt/10);
//
//		//Store digit in string
//		str[index-- + zeroFlag] = digit + 48;              // ASCII value of digit  = digit + 48
//		if (index == -1)
//			break;
//	}
//
//	index = log_value + 1;
//	str[index+zeroFlag] = '.';
//
//	float fraction_part = num - whole_part;
//	float tmp1 = fraction_part, tmp=0;
//
//	//Extract the fraction part from  number
//	for( int i=1; i<=precision; i++)
//	{
//		wt = 10;
//		tmp  = tmp1 * wt;
//		digit = tmp;
//
//		//Store digit in string
//		str[++index + zeroFlag] = digit + 48;           // ASCII value of digit  = digit + 48
//		tmp1 = tmp - digit;
//	}
//	str[++index + zeroFlag] = '\0';
//}


/*******************************************************************************
* Function Name  : DebugInit
* Description    : Initialize debug (via USART2)
*******************************************************************************/

void DebugInit()
{
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	//NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

	USART_InitStruct.USART_BaudRate = 57600; //115200 real
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx/* | USART_Mode_Rx*/;
	USART_Init(USART2, &USART_InitStruct);
//		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
//
//		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//		NVIC_Init(&NVIC_InitStructure);

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART2, ENABLE);

}


/*******************************************************************************
* Function Name  : DebugSendString
* Description    : Sends debug information (via USART2)
* Input          : pointer to text massive
*******************************************************************************/
void DebugSendString(char *pText)
{
	for(; *pText != '\0'; pText++)
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}
		USART_SendData(USART2, *pText);
	}
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}
	USART_SendData(USART2, '\n');

}


/*******************************************************************************
* Function Name  : DebugSendString
* Description    : Sends debug information (via USART2)
* Input          : pointer to text massive
*******************************************************************************/
void DebugSendStringSpace(char *pText)
{
	for(; *pText != '\0'; pText++)
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}
		USART_SendData(USART2, *pText);
	}
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}
	USART_SendData(USART2, ' ');

}


/*******************************************************************************
* Function Name  : DebugSendChar
* Description    : Sends debug information (via USART2)
* Input          : char to send
*******************************************************************************/
void DebugSendChar(char charTx)
{
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, charTx);
}


void DebugSendNum(uint16_t Num)
{
	char str[50];
	itoa_(Num, str);
	DebugSendStringSpace(str);
}

void DebugSendNumSpace(uint16_t Num)
{
	char str[50];
	itoa_(Num, str);
	DebugSendStringSpace(str);
}

void DebugSendNumChar(uint8_t Num)
{
	char str[50];
	itoaChar_(Num, str);
	DebugSendString(str);
}

void DebugSendNumWDesc(char *string, uint32_t Num)
{
	char str[50]={'\0'}, number[20];
	strcat(str, string);
	itoa_(Num, number);
	strcat(str, number);
	DebugSendString(str);
}

/*******************************************************************************
 * Function Name  : itoa
 * Description    : Convert int to char
 * Input          : int number (signed/unsigned)
 * Return         : pointer to text string
 *******************************************************************************/
void itoaBinary_(uint32_t n, char s[], uint8_t len)
{

	for(uint8_t i = 0; i < len; i++)
	{
		if(n & (1<<i))
		{
			if(i < 10)
				s[i] = i + '0';
			else
				s[i] = i - 10 + 'A';
		}
		else
			s[i] = '_';
	}

	s[len] = '\0';
	reverse((char*) s);
}


void DebugSendNumWDescBin(char *string, uint32_t Num, uint8_t size)
{
	char str[110] = { '\0' }, number[100];
	strcat(str, string);
	itoaBinary_(Num, number, size);
	if(Num)
		strcat(str, number);
	DebugSendStringSpace(str);
}
