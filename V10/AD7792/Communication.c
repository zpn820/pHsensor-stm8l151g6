/***************************************************************************//**
 *   @file   Communication.c
 *   @brief  Implementation of Communication Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 501
*******************************************************************************/

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/

#include "Communication.h"



//SPI_Handle      spi;
//SPI_Params      spiParams;
/***************************************************************************//**
 * @brief Initializes the SPI communication peripheral.
 *
 * @param lsbFirst - Transfer format (0 or 1).
 *                   Example: 0x0 - MSB first.
 *                            0x1 - LSB first.
 * @param clockFreq - SPI clock frequency (Hz).
 *                    Example: 1000 - SPI clock frequency is 1 kHz.
 * @param clockPol - SPI clock polarity (0 or 1).
 *                   Example: 0x0 - idle state for SPI clock is low.
 *	                          0x1 - idle state for SPI clock is high.
 * @param clockPha - SPI clock phase (0 or 1).
 *                   Example: 0x0 - data is latched on the leading edge of SPI
 *                                  clock and data changes on trailing edge.
 *                            0x1 - data is latched on the trailing edge of SPI
 *                                  clock and data changes on the leading edge.
 *
 * @return 0 - Initialization failed, 1 - Initialization succeeded.
*******************************************************************************/
unsigned char AD7792_SPI_Init(void)
{

    //uint8_t         transmitBuffer[MSGSIZE];
    //uint8_t         receiveBuffer[MSGSIZE];
 
//    SPI_init();  // Initialize the SPI driver
// 
//    SPI_Params_init(&spiParams);  // Initialize SPI parameters
//    spiParams.dataSize = 8;       // 8-bit data size
//		spiParams.frameFormat = SPI_POL1_PHA1;
//    
//    spi = SPI_open(Board_SPI0, &spiParams);
//    if (spi == NULL) {
//       while (1);  // SPI_open() failed
//   }
    return(1);
}

/***************************************************************************//**
 * @brief Writes data to SPI.
 *
 * @param data - Write data buffer:
 *               - first byte is the chip select number;
 *               - from the second byte onwards are located data bytes to write.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
unsigned char AD7792_Write(unsigned char* data,unsigned char bytesNumber)
{
		
		unsigned char chipSelect    = data[0];
	  unsigned char writeData[4]  = {0, 0, 0, 0};
    //unsigned char readData[4]	  = {0, 0, 0, 0};
    unsigned char byte          = 0;
    
    for(byte = 0;byte < bytesNumber;byte ++)
    {
        writeData[byte] = data[byte + 1];
    }
		
		if(chipSelect == 1)
    {
        ADI_PART_CS_LOW;
    } 
		
		for(byte = 0;byte < bytesNumber;byte ++)
    {
        SPI_SendData(SPI1,writeData[byte]);
        while (RESET == SPI_GetFlagStatus(SPI1,SPI_FLAG_TXE));                    
    }
    if(chipSelect == 1)
    {
        ADI_PART_CS_HIGH;
    }
	return(bytesNumber);
}

/***************************************************************************//**
 * @brief Reads data from SPI.
 *
 * @param data - As an input parameter, data represents the write buffer:
 *               - first byte is the chip select number;
 *               - from the second byte onwards are located data bytes to write.
 *               As an output parameter, data represents the read buffer:
 *               - from the first byte onwards are located the read data bytes. 
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
unsigned char AD7792_Read(unsigned char* data,
                       unsigned char bytesNumber)
{
		unsigned char chipSelect    = data[0];
	  unsigned char writeData[4]  = {0, 0, 0, 0};
    unsigned char readData[4]	  = {0, 0, 0, 0};
    unsigned char byte          = 0;
    
    for(byte = 0;byte < bytesNumber;byte ++)
    {
        writeData[byte] = data[byte + 1];
				data[byte + 1] = 0;
    }
		
		if(chipSelect == 1)
    {
        ADI_PART_CS_LOW;
    }
		
		for(byte = 0;byte < bytesNumber;byte ++)
    {
            SPI_SendData(SPI1,writeData[byte]);
						while (RESET == SPI_GetFlagStatus(SPI1,SPI_FLAG_TXE));   // 等待数据传输完成	
						while (RESET == SPI_GetFlagStatus(SPI1,SPI_FLAG_RXNE));  // 等待数据接收完成
						readData[byte]=SPI_ReceiveData(SPI1);
    }
   
	  
    if(chipSelect == 1)
    {
        ADI_PART_CS_HIGH;
    }
		
    for(byte = 0;byte < bytesNumber;byte ++)
    {
        data[byte] = readData[byte];
    }
	return(bytesNumber);
}
