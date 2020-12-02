/***************************************************************************//**
 *   @file   AD7792.c
 *   @brief  Implementation of AD7792 Driver.
 *   @author Bancisor MIhai
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
#include "AD7792.h"				// AD7792 definitions.
#include "Communication.h"		// Communication definitions.
#include "board.h"
uint32_t samplesAverage = 0x0;
/***************************************************************************//**
 * @brief Initializes the AD7792 and checks if the device is present.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful (ID is 0x0B).
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/
uint8_t AD7792_Init(void)
{ 
	uint8_t status = 0x0;
   status = (uint8_t)AD7792_GetRegisterValue(AD7792_REG_ID, 1, 1); 
//    //AD7792_SPI_Init();
//    if((AD7792_GetRegisterValue(AD7792_REG_ID, 1, 1) & 0x0F) != AD7792_ID)
//	{
//		status = 0x0;
//	}
    
	return(status);
}

/***************************************************************************//**
 * @brief Sends 32 consecutive 1's on SPI in order to reset the part.
 *
 * @return  None.    
*******************************************************************************/
void AD7792_Reset(void)
{
	uint8_t dataToSend[5] = {0x03, 0xff, 0xff, 0xff, 0xff};
	
  ADI_PART_CS_LOW;	    
	AD7792_Write(dataToSend,4);
	ADI_PART_CS_HIGH;	
}
/***************************************************************************//**
 * @brief Reads the value of the selected register
 *
 * @param regAddress - The address of the register to read.
 * @param size - The size of the register to read.
 *
 * @return data - The value of the selected register register.
*******************************************************************************/
uint16_t AD7792_GetRegisterValue(uint8_t regAddress, 
                                      uint8_t size,
                                      uint8_t modifyCS)
{
	uint8_t data[5]      = {0x00, 0x00, 0x00, 0x00, 0x00};
	uint16_t receivedData = 0x00;
    uint8_t i            = 0x00; 
    
	data[0] = 0x01 * modifyCS;
	data[1] = AD7792_COMM_READ |  AD7792_COMM_ADDR(regAddress); 
	AD7792_Read(data,(1 + size));
	for(i = 1;i < size + 1;i ++)
    {
        receivedData = (receivedData << 8) + data[i];
				data_ad7792[i] = data[i];
    }
    return (receivedData);
}
/***************************************************************************//**
 * @brief Writes the value to the register
 *
 * @param -  regAddress - The address of the register to write to.
 * @param -  regValue - The value to write to the register.
 * @param -  size - The size of the register to write.
 *
 * @return  None.    
*******************************************************************************/
void AD7792_SetRegisterValue(uint8_t regAddress,
                             uint16_t regValue, 
                             uint8_t size,
                             uint8_t modifyCS)
{
	uint8_t data[5]      = {0x00, 0x00, 0x00, 0x00, 0x00};	
	//uint8_t* dataPointer = (uint8_t*)&regValue;
    uint8_t bytesNr      = size + 1;
//   uint8_t i; 
    data[0] = 0x01 * modifyCS;
    data[1] = AD7792_COMM_WRITE |  AD7792_COMM_ADDR(regAddress);
//		for(i=2; i<bytesNr+1;i++)
//		{
//			  data[i] = *dataPointer;
//        dataPointer ++;
//		}
		
//    while(bytesNr > 1)
//    {
//        data[bytesNr] = *dataPointer;
//        dataPointer ++;
//        bytesNr --;
//    }	
		if(bytesNr==2)
		{
			data[2] = (uint8_t)regValue;
		}
		if(bytesNr==3)
		{data[2] = (uint8_t)(regValue>>8);
			data[3] = (uint8_t)regValue;
			
		}
	AD7792_Write(data,(1 + size));
}
/***************************************************************************//**
 * @brief  Waits for RDY pin to go low.
 *
 * @return None.
*******************************************************************************/
void AD7792_WaitRdyGoLow(void)
{
    while( GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) )
    {
        ;
    }
}

/***************************************************************************//**
 * @brief Sets the operating mode of AD7792.
 *
 * @param mode - Mode of operation.
 *
 * @return  None.    
*******************************************************************************/
uint16_t AD7792_SetMode(uint16_t mode)
{
    uint16_t command;
    
    command = AD7792_GetRegisterValue(AD7792_REG_MODE,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7792_MODE_SEL(0xFF);
    command |= AD7792_MODE_SEL(mode);
    AD7792_SetRegisterValue(
            AD7792_REG_MODE,
            command,
            2, 
            1); // CS is modified by SPI read/write functions.
		return (command);	
}
/***************************************************************************//**
 * @brief Selects the channel of AD7792.
 *
 * @param  channel - ADC channel selection.
 *
 * @return  None.    
*******************************************************************************/
uint16_t AD7792_SetChannel(uint16_t channel)
{
    uint16_t command;
    
    command = AD7792_GetRegisterValue(AD7792_REG_CONF,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7792_CONF_CHAN(0xFF);
    command |= AD7792_CONF_CHAN(channel);
    AD7792_SetRegisterValue(
            AD7792_REG_CONF,
            command,
            2,
            1); // CS is modified by SPI read/write functions.
//		    command = AD7792_GetRegisterValue(AD7792_REG_CONF,
//                                      2,
//                                      1); // CS is modified by SPI read/write functions.
	return (command);			
}

/***************************************************************************//**
 * @brief  Sets the gain of the In-Amp.
 *
 * @param  gain - Gain.
 *
 * @return  None.    
*******************************************************************************/
void AD7792_SetGain(uint16_t gain)
{
    uint16_t command;
    
    command = AD7792_GetRegisterValue(AD7792_REG_CONF,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7792_CONF_GAIN(0xFF);
    command |= AD7792_CONF_GAIN(gain);
    AD7792_SetRegisterValue(
            AD7792_REG_CONF,
            command,
            2,
            1); // CS is modified by SPI read/write functions.
}
/***************************************************************************//**
 * @brief  config the IEX.
 *
 * @param  iexcdir - AD7792_DIR_IEXC1_IOUT1_IEXC2_IOUT2	0   //IEXC1 connect to IOUT1, IEXC2 connect to IOUT2 
 *	               - AD7792_DIR_IEXC1_IOUT2_IEXC2_IOUT1	1   // IEXC1 connect to IOUT2, IEXC2 connect to IOUT1 
 *                 - AD7792_DIR_IEXC1_IEXC2_IOUT1		    2   // Both current sources IEXC1,2 connect to IOUT1  
 *                 - AD7792_DIR_IEXC1_IEXC2_IOUT2		    3   // Both current sources IEXC1,2 connect to IOUT2
 *
 * @param  iexcen  - AD7792_EN_IXCEN_10uA				  1  // Excitation Current 10uA 
 *								 - AD7792_EN_IXCEN_210uA				2  // Excitation Current 210uA 
 *							   - AD7792_EN_IXCEN_1mA					3  // Excitation Current 1mA 
 * 
 * @return  None.    
*******************************************************************************/
 
uint8_t AD7792_ConfigIO(uint8_t iexcdir,uint8_t iexcen)
{
    uint8_t command;
    
    command = AD7792_GetRegisterValue(AD7792_REG_IO,
                                      1,
                                      1); // CS is modified by SPI read/write functions.
    command |= AD7792_IEXCDIR(iexcdir);
    command |= AD7792_IEXCEN(iexcen);
    AD7792_SetRegisterValue(AD7792_REG_IO,
   													command,
            								1,
            								1); // CS is modified by SPI read/write functions.
		command = AD7792_GetRegisterValue(AD7792_REG_IO,
                                      1,
                                      1); // CS is modified by SPI read/write functions.
		return(command);
}
/***************************************************************************//**
 * @brief Sets the reference source for the ADC.
 *
 * @param type - Type of the reference.
 *               Example: AD7792_REFSEL_EXT	- External Reference Selected
 *                        AD7792_REFSEL_INT	- Internal Reference Selected.
 *
 * @return None.    
*******************************************************************************/
void AD7792_SetIntReference(uint8_t type)
{
    uint16_t command = 0;
    
    command = AD7792_GetRegisterValue(AD7792_REG_CONF,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7792_CONF_REFSEL(AD7792_REFSEL_INT);
    command |= AD7792_CONF_REFSEL(type);
    AD7792_SetRegisterValue(AD7792_REG_CONF,
														command,
														2,
                            1); // CS is modified by SPI read/write functions.
}

/***************************************************************************//**
 * @brief Performs the given calibration to the specified channel.
 *
 * @param mode - Calibration type.
 * @param channel - Channel to be calibrated.
 *oldRegValue &= AD7792_GetRegisterValue(AD7792_REG_MODE, 2, 1); -> |=
 * @return none.
*******************************************************************************/
void AD7792_Calibrate(uint8_t mode, uint8_t channel)
{
    unsigned short oldRegValue = 0x0;
    unsigned short newRegValue = 0x0;
    
    AD7792_SetChannel(channel);
    oldRegValue |= AD7792_GetRegisterValue(AD7792_REG_MODE, 2, 1); // CS is modified by SPI read/write functions.
    oldRegValue &= ~AD7792_MODE_SEL(0x7);
    newRegValue = oldRegValue | AD7792_MODE_SEL(mode);
    ADI_PART_CS_LOW; 
    AD7792_SetRegisterValue(AD7792_REG_MODE, newRegValue, 2, 0); // CS is not modified by SPI read/write functions.
    AD7792_WaitRdyGoLow();
    ADI_PART_CS_HIGH;
    
}

/***************************************************************************//**
 * @brief Performs the given calibration to the specified channel.
 *
 * @param mode - Calibration type.
 * @param channel - Channel to be calibrated.
 *
 * @return none.
*******************************************************************************/
void AD7792_SysZeroCalibrate(uint8_t mode, uint8_t channel)
{
    unsigned short oldRegValue = 0x0;
    unsigned short newRegValue = 0x0;
    
    AD7792_SetChannel(channel);
		oldRegValue |= AD7792_GetRegisterValue(AD7792_REG_MODE, 2, 1); // CS is modified by SPI read/write functions.
    oldRegValue &= ~AD7792_MODE_SEL(0x7);
    newRegValue = oldRegValue | AD7792_MODE_SEL(mode);
    ADI_PART_CS_LOW; 
    AD7792_SetRegisterValue(AD7792_REG_MODE, newRegValue, 2, 0); // CS is not modified by SPI read/write functions.
    AD7792_WaitRdyGoLow();
    ADI_PART_CS_HIGH;
    
}
/***************************************************************************//**
 * @brief Returns the result of a single conversion.
 *
 * @return regData - Result of a single analog-to-digital conversion.
*******************************************************************************/
uint16_t AD7792_SingleConversion(void)
{
    uint16_t command = 0x0;
    uint16_t regData = 0x0;
    
    command  = AD7792_MODE_SEL(AD7792_MODE_SINGLE);
    ADI_PART_CS_LOW;
    AD7792_SetRegisterValue(AD7792_REG_MODE, 
                            command,
                            2,
                            0);// CS is not modified by SPI read/write functions.
    AD7792_WaitRdyGoLow();
    regData = AD7792_GetRegisterValue(AD7792_REG_DATA, 2, 0); // CS is not modified by SPI read/write functions.
    ADI_PART_CS_HIGH;

    return(regData);
}

/***************************************************************************//**
 * @brief Returns the average of several conversion results.
 *
 * @return samplesAverage - The average of the conversion results.
*******************************************************************************/
uint16_t AD7792_ContinuousReadAvg(uint8_t sampleNumber)
{
    //uint32_t samplesAverage = 0x0;
		 samplesAverage = 0x0;
    uint16_t command        = 0x0;
    uint8_t count          = 0x0;
		uint16_t oldRegValue = 0x0;
    //unsigned short newRegValue = 0x0;
    ADI_PART_CS_LOW;
				
    oldRegValue |= AD7792_GetRegisterValue(AD7792_REG_MODE, 2, 0); // CS is modified by SPI read/write functions.
    oldRegValue &= ~AD7792_MODE_SEL(0x7);
    command = oldRegValue | AD7792_MODE_SEL(AD7792_MODE_CONT);

    AD7792_SetRegisterValue(AD7792_REG_MODE,
                            command, 
                            2,
                            0);// CS is not modified by SPI read/write functions.
    for(count = 0;count < sampleNumber;count ++)
    {
        AD7792_WaitRdyGoLow();
        samplesAverage += AD7792_GetRegisterValue(AD7792_REG_DATA, 2, 0);  // CS is not modified by SPI read/write functions.
    }
    ADI_PART_CS_HIGH;
    samplesAverage = samplesAverage / sampleNumber;
    
    return((uint16_t)samplesAverage);
}