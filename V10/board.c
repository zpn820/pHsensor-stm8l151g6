/**
  ******************************************************************************
  * @file    board.c
  * @author  zpn
  * @version v12
  * @date    
  * @brief   IO define and peripheral initial
  ******************************************************************************
  */ 

#include "board.h"


/**
  * @brief  Configure RTC peripheral 
  * @param  None
  * @retval None
  */
void RTC_Config(void)
{
  /* Enable RTC clock */
  CLK_RTCClockConfig(CLK_RTCCLKSource_LSE, CLK_RTCCLKDiv_1);
	
  /* Wait for LSE clock to be ready */
  while (CLK_GetFlagStatus(CLK_FLAG_LSERDY) == RESET);
  /* wait for 1 second for the LSE Stabilisation */
  LSE_StabTime();
  CLK_PeripheralClockConfig(CLK_Peripheral_RTC, ENABLE);

  /* Configures the RTC wakeup timer_step = RTCCLK/16 = LSE/16 = 488.28125 us */
  RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div16);

  /* Enable wake up unit Interrupt */
  RTC_ITConfig(RTC_IT_WUT, ENABLE);

  /* Enable general Interrupt*/
  enableInterrupts();
}
/**
  * @brief  Wait 1 sec for LSE stabilisation .
  * @param  None.
  * @retval None.
  * Note : TIM4 is configured for a system clock = 2MHz
  */
void LSE_StabTime(void)
{

  CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);

  /* Configure TIM4 to generate an update event each 1 s */
  TIM4_TimeBaseInit(TIM4_Prescaler_32768, 246);
  /* Clear update flag */
  TIM4_ClearFlag(TIM4_FLAG_Update);

  /* Enable TIM4 */
  TIM4_Cmd(ENABLE);

  /* Wait 1 sec */
  while ( TIM4_GetFlagStatus(TIM4_FLAG_Update) == RESET );

  TIM4_ClearFlag(TIM4_FLAG_Update);

  /* Disable TIM4 */
  TIM4_Cmd(DISABLE);

  CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, DISABLE);
}
/**
  * @brief Delay.
  * @param[in] nCount
  * @retval None
  */
void Delay(uint32_t nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
}
/**
  * @brief ADC1_Config.
  * @param  None.
  * @retval None
  * Note :
  */
void ADC1_Config(void)
{
  CLK_PeripheralClockConfig( CLK_Peripheral_ADC1, ENABLE);
  ADC_DeInit(ADC1);
  ADC_Init(ADC1, ADC_ConversionMode_Continuous, ADC_Resolution_12Bit, ADC_Prescaler_2);
  ADC_SamplingTimeConfig(ADC1, ADC_Group_SlowChannels, ADC_SamplingTime_384Cycles);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 Channel 3 */
  ADC_ChannelCmd(ADC1, ADC_Channel_22, ENABLE);

  /* Enable End of conversion ADC1 Interrupt */
 // ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

  /* Start ADC1 Conversion using Software trigger*/
  //ADC_SoftwareStartConv(ADC1);

}


/**
  * @brief SPI_Config.
  * @param  None.
  * @retval None
  * Note :
  */
void SPI_Config(void)
{
  
	CLK_PeripheralClockConfig( CLK_Peripheral_SPI1, ENABLE);
	SPI_DeInit(SPI1);
	
	// SPI相关IO口配置

}


/**
  * @brief SPI_Config.
  * @param  None.
  * @retval None
  * Note :
  */
void SPI_ReConfig(SPISlave_TypeDef model)
{
  SPI_Cmd(SPI1, DISABLE);
 	if(model==SET_CC1101)
	{
	// 配置SPI相关参数,8分频（500KHZ）	
	SPI_Init(SPI1, SPI_FirstBit_MSB,SPI_BaudRatePrescaler_16,
           SPI_Mode_Master, SPI_CPOL_Low,SPI_CPHA_1Edge, 
	 				 SPI_Direction_2Lines_FullDuplex,SPI_NSS_Soft,(uint8_t)0x07);
	SPI_Cmd(SPI1, ENABLE);
	Delay(10);
	}
	if (model==SET_AD7792)
	{
			// 配置SPI相关参数,4分频（1MHZ）	
	SPI_Init(SPI1, SPI_FirstBit_MSB,SPI_BaudRatePrescaler_8,
           SPI_Mode_Master, SPI_CPOL_Low,SPI_CPHA_2Edge, 
	 				 SPI_Direction_2Lines_FullDuplex,SPI_NSS_Soft,(uint8_t)0x07);
	SPI_Cmd(SPI1, ENABLE);
	Delay(10);
	}
}


/**
  * @brief GPIO_Config().
  * @param  None.
  * @retval None
  * Note :
  */
void GPIO_Config(void)
{
    /* Configures the LCD GLASS relative GPIO port IOs and LCD peripheral */
    GPIO_Init(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2,GPIO_Mode_Out_PP_Low_Slow);
		GPIO_Init(GPIOA,GPIO_Pin_3,GPIO_Mode_Out_PP_High_Slow);

    GPIO_Init(GPIOB,GPIO_Pin_3,GPIO_Mode_Out_PP_High_Slow);
		GPIO_Init(GPIOB,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2,GPIO_Mode_Out_PP_Low_Slow);
		GPIO_ResetBits(GPIOB,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2);
		
		GPIO_Init(GPIOC,GPIO_Pin_0|GPIO_Pin_1,GPIO_Mode_Out_OD_Low_Slow);

    GPIO_Init(GPIOD,GPIO_Pin_0,GPIO_Mode_Out_PP_Low_Slow);
		
	
    GPIO_ResetBits(GPIOA,GPIO_Pin_2);
			
    GPIO_Init(PORT_GDO0, PIN_GDO0, GPIO_Mode_In_PU_No_IT);
		GPIO_Init(PORT_GDO2, PIN_GDO2, GPIO_Mode_In_PU_No_IT);
    GPIO_Init(PORT_CC1101, PIN_CC1101, GPIO_Mode_Out_PP_High_Slow);
    GPIO_SetBits(PORT_CC1101, PIN_CC1101); 
	  
		GPIO_Init(PORT_SPI, PIN_MISO, GPIO_Mode_In_PU_No_IT);       // MISO (PB7)
	  GPIO_Init(PORT_SPI, PIN_SCLK, GPIO_Mode_Out_PP_Low_Fast);  // SCLK (PB5)
	  GPIO_Init(PORT_SPI, PIN_MOSI, GPIO_Mode_Out_PP_Low_Fast);  // MOSI (PB6)
}


/**
  * @brief GPIO_Config().
  * @param  None.
  * @retval None
  * Note :
  */
void GO_Sleep(void)
{
    /* Configures the LCD GLASS relative GPIO port IOs and LCD peripheral */
    GPIO_SetBits(PORT_EN, PIN_EN);
		//GPIO_Init(GPIOC,GPIO_Pin_0,GPIO_Mode_Out_PP_Low_Slow);
		GPIO_Init(PORT_SPI, PIN_MISO, GPIO_Mode_Out_PP_Low_Slow);       // MISO (PB7)
		GPIO_Init(PORT_GDO0, PIN_GDO0, GPIO_Mode_Out_PP_Low_Slow);
		GPIO_Init(PORT_GDO2, PIN_GDO2, GPIO_Mode_Out_PP_Low_Slow);
		
		GPIO_ResetBits(PORT_SPI,PIN_MISO);
		GPIO_ResetBits(PORT_GDO0,PIN_GDO0);
		GPIO_ResetBits(PORT_GDO2,PIN_GDO2);
		GPIO_ResetBits(PORT_CC1101,PIN_CC1101);
		GPIO_ResetBits(PORT_AD7792,PIN_AD7792);
    //GPIO_ResetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
    
		//GPIO_SetBits(PORT_EN, PIN_EN);
    
}


		