/**
  ******************************************************************************
  * @file    LCD/LCD_SegmentsDrive/main.c
  * @author  MCD Application Team
  * @version V1.5.2
  * @date    30-September-2014
  * @brief   Main program body
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include "stm8l15x.h"
#include "board.h"
#include "CC1101.h"
#include "stm8l15x_conf.h"		

#include "AD7792.h"				     // AD7792 definitions.
#include "Communication.h"		// Communication definitions.

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SCROLL_SPEED  100 /* Low value gives higher speed */
#define SCROLL_NUM    5

#define TX              1       // 发送模式
#define RX              0       // 接收模式

#define SEND_GAP        5000    // 每间隔1s发送一次数据
#define RECV_TIMEOUT    800     // 接收超时

#define ACK_LENGTH      10      // 应答信号长度        
#define SEND_LENGTH     5   // 发送数据每包的长度


#define ADC_RATIO              ((uint16_t) 806) /*ADC_RATIO = ( 3.3 * 1000 * 1000)/4095 */
#define ASCII_NUM_0            ((uint8_t)   48)
#define F_SAMPLING 1000        //1kHz sampling
#define AVERAGE_LENGTH 50      //how many samples to average in one set
#define ADC_BUFFER_SIZE (AVERAGE_LENGTH) //20 averaged set of samples

/* Private macro -------------------------------------------------------------*/

uint16_t  SendTime = 1;           // 计数数据发送间隔时间
uint32_t  RecvWaitTime = 0;       // 接收等待时间                
uint16_t  SendCnt = 0;            // 计数发送的数据包数  

/* Private variables ---------------------------------------------------------*/
uint8_t     regesit_value[48] ={0};
uint8_t     i_t	= 0;
uint8_t     data_ad7792[5]={0};
uint8_t     SendBuffer[SEND_LENGTH] = {1,0,0,0,0}; 
uint16_t    ADCdata = 0;
uint16_t    PotVoltage = 0;
uint8_t     AD7792ID = 0;

uint16_t    AD7792_Avg = 0;
uint16_t    value_test = 0;


unsigned int * ADCBuffer;
unsigned long AverageSum;


/* Private function prototypes -----------------------------------------------*/

uint8_t RF_SendPacket(uint8_t *Sendbuffer, uint8_t length);
static void ADCCollectDataAvrg(unsigned int * ADCBuffer);
void Average(unsigned int * ADCBuffer);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  At startup the system clock source is automatically selected as HSI RC clock output divided 
  by 8 (HSI/8).
  */
void main(void)
{
	CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_2);
  GPIO_Config();
  SPI_Config();
//    ADC1_Config();  
//    allocate buffer space
//    if ((ADCBuffer = malloc(ADC_BUFFER_SIZE * sizeof(ADCBuffer[0]))) == NULL)
//    trap();
  /* Enable RTC clock */
  RTC_Config();
  GPIO_SetBits(PORT_LED, PIN_LED);
  /* Configures the RTC */
  RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
  RTC_ITConfig(RTC_IT_WUT, ENABLE);
	
	SPI_ReConfig(SET_CC1101);
  CC1101Init();
	for(i_t=0;i_t<0x2f;i_t++)
	{
		regesit_value[i_t] = CC1101ReadReg(i_t);
	}
  /* Enable general Interrupt*/
  enableInterrupts();
  RTC_SetWakeUpCounter(10);
  RTC_WakeUpCmd(ENABLE);
	
	SPI_ReConfig(SET_AD7792);
	AD7792_Reset();
	value_test = AD7792_Init();
	value_test = AD7792_ConfigIO(AD7792_DIR_IEXC1_IOUT1_IEXC2_IOUT2,AD7792_EN_IXCEN_210uA);
	
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
	AD7792_SetGain(AD7792_GAIN_1);                // set gain to 1
		
  AD7792_SetIntReference(AD7792_REFSEL_EXT);    // select internal 1.17V reference
  AD7792_Calibrate(AD7792_MODE_CAL_INT_ZERO,
                     AD7792_CH_AIN1P_AIN1M);      // Internal Zero-Scale Calibration
	AD7792_SysZeroCalibrate(AD7792_MODE_CAL_SYS_ZERO,
                     AD7792_CH_AIN1P_AIN1M);      // Internal Zero-Scale Calibration
	AD7792_Calibrate(AD7792_MODE_CAL_INT_ZERO,
                     AD7792_CH_AIN2P_AIN2M);      // Internal Zero-Scale Calibration
	AD7792_SysZeroCalibrate(AD7792_MODE_CAL_SYS_ZERO,
                     AD7792_CH_AIN2P_AIN2M); 
  while (1)
  {
		
    CLK_PeripheralClockConfig(CLK_Peripheral_ADC1, DISABLE);
	  SPI_ReConfig(SET_AD7792);
		
		value_test=AD7792_SetChannel(AD7792_CH_AIN1P_AIN1M);     // use AIN2(+) - AIN2(-)
		AD7792_Avg = AD7792_ContinuousReadAvg(10);
		SendBuffer[1] = /*(uint8_t)(AverageSum/1000)*/(uint8_t)(AD7792_Avg>>8);
    SendBuffer[2] = /*(uint8_t)((AverageSum%1000)/100)*/(uint8_t)AD7792_Avg;
		value_test=AD7792_GetRegisterValue(AD7792_REG_MODE, 2, 1); // CS is modified by SPI read/write functions.
		Delay(20);
		value_test=AD7792_SetChannel(AD7792_CH_AIN2P_AIN2M);     // use AIN2(+) - AIN2(-)
	 	AD7792_Avg = AD7792_ContinuousReadAvg(10);
		SendBuffer[3] = /*(uint8_t)(AverageSum/1000)*/(uint8_t)(AD7792_Avg>>8);
    SendBuffer[4] = /*(uint8_t)((AverageSum%1000)/100)*/(uint8_t)AD7792_Avg;
		
		
		SPI_ReConfig(SET_CC1101);
    if (RF_SendPacket(SendBuffer, SEND_LENGTH))
    {
     GPIO_ToggleBits(PORT_LED,PIN_LED);
    }
		CC1101WriteCmd( CC1101_SPWD);
		
		SPI_ReConfig(SET_AD7792);

		//AD7792_SetMode(AD7792_MODE_PWRDN);
		
		SPI_Cmd(SPI1, DISABLE);
		CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, DISABLE);
		GPIO_Init(PORT_SPI, PIN_MISO, GPIO_Mode_Out_PP_High_Slow);       // MISO (PB7)
		GPIO_Init(PORT_GDO0, PIN_GDO0, GPIO_Mode_Out_PP_High_Slow);
		GPIO_Init(PORT_GDO2, PIN_GDO2, GPIO_Mode_Out_PP_High_Slow);
		GPIO_SetBits(PORT_SPI, PIN_MISO);
		GPIO_SetBits(PORT_SPI, PIN_MISO);
		GPIO_SetBits(PORT_SPI, PIN_MISO);
		//GO_Sleep();
    halt();
    RTC_WakeUpCmd(DISABLE);
    RTC_SetWakeUpCounter(1);
    RTC_WakeUpCmd(ENABLE);
		GPIO_Init(PORT_SPI, PIN_MISO, GPIO_Mode_In_PU_No_IT);       // MISO (PB7)
		GPIO_Init(PORT_GDO0, PIN_GDO0, GPIO_Mode_In_PU_No_IT);
		GPIO_Init(PORT_GDO2, PIN_GDO2, GPIO_Mode_In_PU_No_IT);		
		CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, ENABLE);
		//OUT_Sleep();
		Delay(10);
//    
		
  }
}
	

/**
  * @brief RF_SendPacket.
  * @param  None.
  * @retval None
  * Note :
  */
uint8_t RF_SendPacket(uint8_t *Sendbuffer, uint8_t length)
{
    uint8_t ack_flag = 0;         // =1,接收到应答信号，=0不处理
    uint8_t error = 0, i=0, ack_len=0, ack_buffer[65]={ 0 }, TxBuffer[100];
    (void)TxBuffer;
    CC1101SendPacket(SendBuffer, length, ADDRESS_CHECK);    // 发送数据   
    CC1101SetTRMode(RX_MODE);           // 进入接收模式，等待应答
    RecvWaitTime = 50000;        // 等待应答超时限制为800ms
    
    ack_flag = 1;
	//LSE_StabTime
   while (CC_IRQ_READ() != 0)
	{
		RecvWaitTime--;
		if (0 == RecvWaitTime)      
		{ ack_flag = 0; break; }
	}
    
    if (0 != ack_flag)                          // 检测是否收到数据包
    {
        while (CC_IRQ_READ() == 0);
		ack_len = CC1101RecPacket(ack_buffer);  // 读取收到的数据

        // 判断数据是否有误，应答信号应该为10-19
        for (i=0, error=0; i<10; i++ )
        {
            if (ack_buffer[i] != (i+10))    { error=1; break; }
        }
        
        if ((ack_len==10) && (error==0))    { return (1); } // 数据无误  
    }
    
    return (0);  
}

//collecting data from ADC into buffer
static void ADCCollectDataAvrg(unsigned int * ADCBuffer)
{
    unsigned int i;    
    //sample data
    i=0;
    ADC_SoftwareStartConv(ADC1);
    while (i<ADC_BUFFER_SIZE)
    {
      //wait for EOC
      while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
      //collect ADC value
      ADC_ClearFlag(ADC1,ADC_FLAG_EOC);
      ADCBuffer [i] = ADC_GetConversionValue(ADC1);
      i++;
    }

}//ADCCollectData

//perform averaging on whole collected ADC data (floating average)
void Average(unsigned int * ADCBuffer)
{
  unsigned int i;
  AverageSum = 0;

  //averaging
  for (i=0; i<(ADC_BUFFER_SIZE); i++)
  {
    
    //sum set of samples (can be optimized)
    AverageSum += ADCBuffer[i];
    
    //write back the averaged sample
  }
  AverageSum= AverageSum/AVERAGE_LENGTH;
}//Average

/**
  * @brief RF_SendPacket.
  * @param  None.
  * @retval None
  * Note :
  */
void OUT_Sleep(void)

{		GPIO_ResetBits(PORT_EN, PIN_EN);
	  GPIO_Init(PORT_SPI, PIN_MISO, GPIO_Mode_In_PU_No_IT);       // MISO (PB7)
		GPIO_Init(PORT_GDO0, PIN_GDO0, GPIO_Mode_In_PU_No_IT);
		GPIO_Init(PORT_GDO2, PIN_GDO2, GPIO_Mode_In_PU_No_IT);
	   
		//GPIO_Init(GPIOC,GPIO_Pin_0,GPIO_Mode_Out_PP_Low_Slow);

		
		GPIO_SetBits(PORT_CC1101,PIN_CC1101);
		GPIO_SetBits(PORT_AD7792,PIN_AD7792);
		
	  SPI_ReConfig(SET_CC1101);
    CC1101Init();
		
		SPI_ReConfig(SET_AD7792);
		AD7792_Reset();
		AD7792_Init();
		AD7792_ConfigIO(AD7792_DIR_IEXC1_IOUT1_IEXC2_IOUT2,AD7792_EN_IXCEN_210uA);
	  AD7792_SetGain(AD7792_GAIN_1);                // set gain to 1
		AD7792_SetIntReference(AD7792_REFSEL_EXT);    // select internal 1.17V reference
		
		AD7792_Calibrate(AD7792_MODE_CAL_INT_ZERO,
                     AD7792_CH_AIN1P_AIN1M);      // Internal Zero-Scale Calibration
	  AD7792_Calibrate(AD7792_MODE_CAL_INT_ZERO,
                     AD7792_CH_AIN2P_AIN2M);      // Internal Zero-Scale Calibration

}










#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
