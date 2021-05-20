/*
 * ADC_APP.h
 *
 *  Created on: Sep 8, 2020
 *      Author: USER
 */

#ifndef INC_ADC_APP_H_
#define INC_ADC_APP_H_



#endif /* INC_ADC_APP_H_ */


/* Exported types ------------------------------------------------------------*/
#define ADC_Buffer_Ch 1
typedef struct
{
	uint16_t BufferValue[ADC_Buffer_Ch];
	uint32_t BufferValue_Sum[ADC_Buffer_Ch];
	uint16_t BufferValue_Avg[ADC_Buffer_Ch];
	uint16_t Vol[ADC_Buffer_Ch];
	unsigned char __attribute__ ((aligned (32))) sendbuffer[128];
} ADC1_t;

ADC1_t ADC_1;
/* Exported functions prototypes ---------------------------------------------*/
void ADC1_Conv(ADC_HandleTypeDef *hadc);
