/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "math.h"
#include "BLE_USART.h"
#include <stdint.h>
#include "printf.h"
#include "Calculate_statistic.h"
#include "Delay.h"
#include <stdbool.h>
/* Private typedef -----------------------------------------------------------*/
extern USART_BLE USARTBLE;	//Wayne0905

/* Private variables ---------------------------------------------------------*/


void BLE_USART(UART_HandleTypeDef *huart, float *sendpData )
{

//	float *aa = sendpData;

//	snprintf_(tf_buff,10 ,"%.4f",&aa);
	__NOP();
//	USARTBLE.bufferSize = min_(APP_BUFFER_SIZE, strlen(USARTBLE.buffer));
	//USARTBLE.sendTimeout = 100 ;
//	if(HAL_UART_Transmit_DMA(huart, USARTBLE.buffer, USARTBLE.bufferSize)==HAL_OK)
//	{
//		__NOP();
//	}
		/*
		 HAL_UART_Receive(huart , &USARTBLE.Rbuffer, 14, 1000);

		 char C[20];
		 strcpy(C,  USARTBLE.Rbuffer );
		 */


}

_Bool checkBLECommandFromBLEGateway(char * BLEcommand,char * index, int len)
{
	if(strlen(BLEcommand) > 0)
	{
		   //Test 比對 function
	   char * pch;
	   /* 找尋 simple 字串 */
	   pch = strstr (BLEcommand,index);
	   if(strncmp(pch, index, len) == 0) {
		   return true;
	   }
	   else
	   {
		   return false;
	   }

	}
	else
	{

		return false;
	}

}



