/*
 * ADS1256.c
 *
 *  Created on: Sep 5, 2019
 *      Author: Weitingee
 */


#include "ADS1256.h"
uint8_t  posChannels [4] = {AIN0, AIN2, AIN4, AIN6};
uint8_t  negChannels [4] = {AIN1, AIN3, AIN5, AIN7};
uint8_t SDATACcmduffer[1] = {CMD_RDATA};
uint8_t SYNCcmduffer[1] = {CMD_SYNC};
uint8_t WAKEUPcmduffer[1] = {CMD_WAKEUP};
uint8_t Databuffer[3] = {0x00,0x00,0x00};
int32_t read = 0;
float data;



/*
*********************************************************************************************************
*	name: setDIFFChannel
*	function: Write to MUX register - set channel to read from in single-ended mode
*   Bit 7,6,5,4 determine the positive input channel (AINp).
*   Bit 3,2,1,0 determine the negative input channel (AINn). e.g. (0-1, 2,3 - 4,5 - 6,7)
*	parameter:
*	The return value: val
*********************************************************************************************************
*/
void setDIFFChannel(uint8_t positiveCh, uint8_t NegativeCh)
{
	CS_0();
	writeByteToReg(REG_MUX, positiveCh <<4 | NegativeCh); //xxxx1000 - AINp = positiveCh, AINn = NegativeCh
	CS_1();
}

/*
*********************************************************************************************************
*	name: writeCMD
*	function: Send Standalone commands to register
*	parameter: command
*	The return value: None
*********************************************************************************************************
*/

void writeCMD(uint8_t command)
{
	uint8_t Txbuffer[1];
	Txbuffer[0] = command;
	CS_0();
	HAL_SPI_Transmit(&hspi1, Txbuffer ,1,50);
	CS_1();
}

/*
*********************************************************************************************************
*	name: setDataRate
*	function: sampling rate of collection
*	parameter: pga
*	The return value: None
*********************************************************************************************************
*/
void setDataRate(uint8_t drate)
{
	writeToReg(REG_DRATE,drate);
}

/**
*********************************************************************************************************
*	name: writeByteToReg
*	function: read 1 byte from register address registerID.
*	parameter: register ID
*	The return value:
*********************************************************************************************************
*/
void writeByteToReg(uint8_t registerID, uint8_t value)
{
	uint8_t Txbuffer[3];
	Txbuffer[0] = CMD_WREG | registerID;
	Txbuffer[1] = 0x00;
	Txbuffer[2] = value;
	HAL_SPI_Transmit_DMA(&hspi1, Txbuffer ,3);

	/*
	send8bit(CMD_WREG | registerID);		//1syt byte: address of the first register to write
	send8bit(0x00);							//2nd byte: number of byte to write = 1.
	send8bit(value);						//3rd byte: value to write to register
	*/
//	CS_1();

}


void writeToReg(uint8_t registerID, uint8_t value)
{
	CS_0();
	uint8_t Txbuffer[3];
	Txbuffer[0] = CMD_WREG | registerID;
	Txbuffer[1] = 0x00;
	Txbuffer[2] = value;
	HAL_SPI_Transmit(&hspi1, Txbuffer ,3,50);
	delay_us(10);
	CS_1();
	/*
	send8bit(CMD_WREG | registerID);		//1syt byte: address of the first register to write
	send8bit(0x00);							//2nd byte: number of byte to write = 1.
	send8bit(value);						//3rd byte: value to write to register
	*/
//	CS_1();

}
/*
*********************************************************************************************************
*	name: setPGA
*	function: Set gain of amplifier
*	parameter: pga
*	The return value: None
*********************************************************************************************************
*/
void setPGA(uint8_t pga)
{
	writeToReg(REG_ADCON,pga);
}
/*
*********************************************************************************************************
*	name: Send8bit
*	function: SPI send data to SPI slave
*	parameter: data
*	The return value: NULL
*********************************************************************************************************
*/
void send8bit(uint8_t data)
{
	HAL_SPI_Transmit(&hspi1, &data ,1,100);
}
/*
*********************************************************************************************************
*	name: waitDRDY
*	function: Wait for DRDY is Low
*	parameter: data
*	The return value: None
*********************************************************************************************************
*/
void waitDRDY(void)
{
	uint32_t i;
		for (i = 0; i < 40000000; i++){
			if (DRDY_IS_LOW()){
				break;
			}
		}
}

/*
*********************************************************************************************************
*	name: readChipID
*	function: Get data from Status register - chipID "check"
*	parameter:
*	The return value: val
*********************************************************************************************************
*/
uint8_t readChipID(void)
{
	waitDRDY();
	volatile uint8_t id = readByteFromReg(REG_STATUS);
	return (id >> 4);
}

/*
*********************************************************************************************************
*	name: receive8bit
*	function: receive data from SPI slave
*	parameter: data
*	The return value: NULL
*********************************************************************************************************
*/
uint8_t receive8bit(void)
{
	/*
	uint8_t TXbuffer[1];
	uint8_t RXbuffer[1];
	TXbuffer[0] = 0xff;

	HAL_SPI_Transmit(&hspi1, TXbuffer ,1,50);
	HAL_SPI_Receive(&hspi1, RXbuffer ,1,50);

	return RXbuffer[0];
	*/
	uint8_t send_data = 0xff;
	uint8_t read = 0;
	HAL_SPI_TransmitReceive(&hspi1,&send_data,&read,1,50);
	return read;

}
/*
*********************************************************************************************************
*	name: readByteFromReg
*	function: read 1 byte from register address registerID.
*	parameter: register ID
*	The return value:
*********************************************************************************************************
*/
uint8_t readByteFromReg(uint8_t registerID)
{
	uint8_t TXbuffer[2];
	TXbuffer[0] = CMD_RREG | registerID;
	TXbuffer[1] = 0x00;
	CS_0();
	HAL_SPI_Transmit(&hspi1, TXbuffer ,2,50);
	uint8_t read = receive8bit();
	delay_us(10);
	CS_1();

	return read;
}

/*
*********************************************************************************************************
*	name: setBuffer
*	function: Set the internal buffer (True-enable), (Fasle-disable)
*	parameter: bool val
*	The return value: val
*********************************************************************************************************
*/
void setBuffer(void)
{
	uint8_t val = 1;
	uint8_t Txbuffer[2];
	Txbuffer[0] = CMD_WREG | REG_STATUS;
	Txbuffer[1] = (0 <<3) | (1 << 2) | (val << 1);

	CS_0();
	HAL_SPI_Transmit(&hspi1, Txbuffer ,2,50);
	CS_1();
}


void ADS1256_SwitchChannalValue(int Channel)
{


	/**
	 * set channel
	 */
//	CS_0();

    setDIFFChannel(posChannels[Channel], negChannels[Channel]);
//    delay_us(5);
    TM_DelayMicros(5);
    CS_0();
    HAL_SPI_Transmit_DMA(&hspi1, SYNCcmduffer ,1);

//    delay_us(5);
    TM_DelayMicros(5);

    HAL_SPI_Transmit_DMA(&hspi1, WAKEUPcmduffer ,1);
//    TM_DelayMicros(25);
    CS_1();
//    TM_DelayMicros(250);

//    CS_1();

    /**
     * receive data
     */

}
float ADS1256_GetChannalValue()
{


	TM_DelayMicros(1);
	CS_0();

    HAL_SPI_Transmit_DMA(&hspi1, SDATACcmduffer ,1);
//    delay_us(7);
    TM_DelayMicros(25);
    HAL_SPI_Receive_DMA(&hspi1,Databuffer,3);
	TM_DelayMicros(25);
	CS_1();
	read  = ((int32_t)Databuffer[0] << 16) & 0x00FF0000;
	read |= ((int32_t)Databuffer[1] << 8);
	read |= Databuffer[2];

	if (read & 0x800000){
		read |= 0xFF000000;

	}

	data = read;
	data = data / 1670000;
//	delay_us(250);

	return data;
}


/*
float ADS1256_GetChannalValue(int Channel)
{

	//set channel

	CS_0();

    setDIFFChannel(posChannels[Channel], negChannels[Channel]);
//    delay_us(5);
    TM_DelayMicros(5);
    HAL_SPI_Transmit_DMA(&hspi1, SYNCcmduffer ,1);
//    delay_us(5);
    TM_DelayMicros(5);
    HAL_SPI_Transmit_DMA(&hspi1, WAKEUPcmduffer ,1);
//    delay_us(50);
    TM_DelayMicros(25);


    //receive data

    HAL_SPI_Transmit_DMA(&hspi1, SDATACcmduffer ,1);
//    delay_us(7);
    TM_DelayMicros(7);
    HAL_SPI_Receive_DMA(&hspi1,Databuffer,3);
	read  = ((int32_t)Databuffer[0] << 16) & 0x00FF0000;
	read |= ((int32_t)Databuffer[1] << 8);
	read |= Databuffer[2];
	if (read & 0x800000){
		read |= 0xFF000000;

	}

	data = read;
	data = data / 1670000;
//	delay_us(250);
	TM_DelayMicros(250);
	CS_1();

//	TM_DelayMicros(1000);
	return data;
}

*/


int ADS1256_init()
{
	   __disable_irq();
      int id;
	  //Reset ADS1256
	  waitDRDY();
	  writeCMD(CMD_RESET);
//	  delay_us(100);
	  TM_DelayMicros(100);

	  //Initialize ADS1256 parameter (Buffer, PGA, Sampling rate)
	  waitDRDY();
	  setBuffer();
//	  delay_us(10);
	  TM_DelayMicros(10);
	  waitDRDY();
	  setPGA(PGA_GAIN1);
//	  delay_us(10);
	  TM_DelayMicros(10);
	  waitDRDY();
	  setDataRate(DRATE_7500);
//	  delay_us(10);
	  TM_DelayMicros(10);
	  waitDRDY();
	  writeCMD(CMD_SELFCAL);
	  waitDRDY();

	  do {
		  id = readChipID();
	  } while (id!=3);

	  __enable_irq();

	  return id;


}


