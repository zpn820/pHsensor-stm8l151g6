#ifndef _CC1101_H_
#define _CC1101_H_

#include <stdlib.h>
#include "stm8l15x_conf.h"
#include "CC1101_REG.h"
#include "board.h"

/*===========================================================================
------------------------------Internal IMPORT functions----------------------
you must offer the following functions for this module
1. uint8_t SPI_ExchangeByte(uint8_t input); // SPI Send and Receive function
2. CC_CSN_LOW();                        // Pull down the CSN line
3. CC_CSN_HIGH();                       // Pull up the CSN Line
===========================================================================*/
// CC1101相关控制引脚定义， CSN(PB4), IRQ(PA2), GDO2(PA3) 

#define CC_CSN_LOW()    GPIO_ResetBits(PORT_CC1101, PIN_CC1101);\
                        while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)!=0);
#define CC_CSN_HIGH()   GPIO_SetBits(PORT_CC1101, PIN_CC1101)

#define CC_IRQ_READ()   GPIO_ReadInputDataBit(PORT_GDO0, PIN_GDO0)

/*===========================================================================
----------------------------------macro definitions--------------------------
============================================================================*/
typedef enum { TX_MODE, RX_MODE } TRMODE;
typedef enum { BROAD_ALL, BROAD_NO, BROAD_0, BROAD_0AND255 } ADDR_MODE;
typedef enum { BROADCAST, ADDRESS_CHECK} TX_DATA_MODE;

/*===========================================================================
-------------------------------------exported APIs---------------------------
============================================================================*/

/*read a byte from the specified register*/
uint8_t CC1101ReadReg(uint8_t addr);

/*Read a status register*/
uint8_t CC1101ReadStatus(uint8_t addr);

/*Set the device as TX mode or RX mode*/
void CC1101SetTRMode(TRMODE mode);

/*Write a command byte to the device*/
void CC1101WriteCmd(uint8_t command);

/*Set the CC1101 into IDLE mode*/
void CC1101SetIdle(void);

/*Send a packet*/
void CC1101SendPacket(uint8_t *txbuffer, uint8_t size, TX_DATA_MODE mode);

/*Set the address and address mode of the CC1101*/
void CC1101SetAddress(uint8_t address, ADDR_MODE AddressMode);

/*Set the SYNC bytes of the CC1101*/
void CC1101SetSYNC(uint16_t sync);

/*Receive a packet*/
uint8_t CC1101RecPacket(uint8_t *rxBuffer);

/*Initialize the WOR function of CC1101*/
void  CC1101WORInit(void);

/*Initialize the CC1101, User can modify it*/
void CC1101Init(void);

#endif // _CC1101_H_

/*===========================================================================
-----------------------------------文件结束----------------------------------
===========================================================================*/
