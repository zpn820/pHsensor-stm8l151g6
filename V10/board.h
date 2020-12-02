/*
================================================================================
File Name    : MyTypeDef.H
Description  : universal Data type definations
Ahthor       : LiYong
================================================================================
*/
#ifndef _BOARD_H_
#define _BOARD_H_

#include <stdlib.h>
#include "stm8l15x.h"
#include "stm8l15x_conf.h"
#include "AD7792.h"
extern	uint8_t state;
extern uint8_t data_ad7792[5];
typedef enum 
{
  SET_CC1101   = (uint8_t)0x00, /*!< Set CC1101*/
  SET_AD7792   = (uint8_t)0x01, /*!< Set AD7792 */

} SPISlave_TypeDef;


/* PROT CSN  */
#define PORT_CC1101     GPIOB
#define PORT_AD7792			GPIOA

#define PIN_CC1101		  GPIO_Pin_3
#define PIN_AD7792			GPIO_Pin_3

/****************************************/

/*CC1101 PROT DEFINE  */
#define PORT_SPI        GPIOB


#define PIN_SCLK        GPIO_Pin_5        
#define PIN_MOSI        GPIO_Pin_6
#define PIN_MISO        GPIO_Pin_7

#define PORT_GDO0       GPIOB
#define PIN_GDO0 				GPIO_Pin_4

#define PORT_GDO2       GPIOC

#define PIN_GDO2        GPIO_Pin_4
/****************************************/
/*AD7792 PORT DEFINE   */

/****************************************/

/*LED PORT DEFINE   */
#define PORT_LED 				GPIOA

#define PIN_LED					GPIO_Pin_2
/****************************************/

/*EN PORT DEFINE   */
#define PORT_EN 				GPIOB
#define PIN_EN					GPIO_Pin_2
/****************************************/

/*KEY PORT DEFINE    */
#define PORT_KEY				GPIOD
#define PIN_KEY					GPIO_Pin_0

/**
  * @brief  I2C EEPROM Interface pins
  */
#define sEE_I2C                          I2C1
#define sEE_I2C_CLK                      CLK_Peripheral_I2C1
#define sEE_I2C_SCL_PIN                  GPIO_Pin_1                  /* PC.01 */
#define sEE_I2C_SCL_GPIO_PORT            GPIOC                       /* GPIOC */
#define sEE_I2C_SDA_PIN                  GPIO_Pin_0                  /* PC.00 */
#define sEE_I2C_SDA_GPIO_PORT            GPIOC                       /* GPIOC */
//#define sEE_M24C64_32

#define sEE_I2C_DMA                      DMA1
#define sEE_I2C_DMA_CHANNEL_TX           DMA1_Channel3
#define sEE_I2C_DMA_CHANNEL_RX           DMA1_Channel0
#define sEE_I2C_DMA_FLAG_TX_TC           DMA1_FLAG_TC3
#define sEE_I2C_DMA_FLAG_RX_TC           DMA1_FLAG_TC0
#define sEE_I2C_DR_Address               ((uint16_t)0x005216)
#define sEE_USE_DMA
		
#define sEE_DIRECTION_TX                 0
#define sEE_DIRECTION_RX                 1
		
typedef enum
{
  IdleState 		= 0,
  CalibratState = 1,
  pHtestState 	= 2,
	EpreadState		= 3
} Model_TypeDef;

//extern uint8_t r_10ms;

typedef struct bitdef
{
	uint8_t bit0:1;
	uint8_t bit1:1;
	uint8_t bit2:1;
	uint8_t bit3:1;
	uint8_t bit4:1;
	uint8_t bit5:1;
	uint8_t bit6:1;
	uint8_t bit7:1;
}flag;
extern flag timerflag;
extern flag keyflag;
extern flag calibratflag;
	
typedef struct pH
{
  uint16_t AN1; /*!<  */
  uint16_t AN2; /*!<  */
	uint16_t pH7; /*!<  */
	uint16_t pH4; /*!<  */
	uint16_t pH10; /*!<  */
	uint16_t temp;
}pHCalib;
#define f10msComes  timerflag.bit0
#define f200msComes timerflag.bit1
#define f1sComes 		timerflag.bit2
#define f20sComes   timerflag.bit3	
#define f120sComes   timerflag.bit4
#define f1sUsed     timerflag.bit5
#define fTimer4cnt3en     timerflag.bit6

#define	fKeyShort		keyflag.bit0
#define fKeyLong3		keyflag.bit1
#define fKeyLong10	keyflag.bit2

#define fcalibrated   	calibratflag.bit0
#define fcalibratpH7  	calibratflag.bit1
#define fcalibratpH4  	calibratflag.bit2
#define fcalibratpH10 	calibratflag.bit3
#define fcalibratedtemp calibratflag.bit4
#define fcalibratedpH7  	calibratflag.bit5
#define fcalibratedpH4  	calibratflag.bit6
#define fcalibratedpH10 	calibratflag.bit7

/*===========================================================================
-------------------------------------exported APIs---------------------------
============================================================================*/
void RTC_Config(void);
void LSE_StabTime(void);
void Delay(uint32_t nCount);
void ADC1_Config(void);
void SPI_Config(void);
void SPI_ReConfig(SPISlave_TypeDef model);
void GPIO_Config(void);
void GO_Sleep(void);
void OUT_Sleep(void);
void Delay1s(void);











#endif
/*
================================================================================
====================================End of file=================================
================================================================================
*/
