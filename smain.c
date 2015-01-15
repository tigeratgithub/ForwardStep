/**
  ******************************************************************************
  * @file    TIM/DMA/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "encoder.h"
#include "modmaster.h"
#include "plcsys.h"
#include <string.h>


/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);

#define MOD_DI_COUNT	100
#define MOD_DO_COUNT	100
#define MOD_M_COUNT		100
#define MOD_V_COUNT		100
/* Private functions ---------------------------------------------------------*/
extern uint8_t MOD_DI[MOD_DI_COUNT], MOD_DO[MOD_DO_COUNT];
extern uint8_t	MOD_M[MOD_M_COUNT], MOD_V[MOD_V_COUNT];
extern uint16_t capture ;

Mod_Master_Frame_TypeDef modu3;
/** 
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	u8 index = 0;
	u16 oldticks = 0;
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */
       
	/* System Clocks Configuration */
	RCC_Configuration();

	/* GPIO Configuration */
	GPIO_Configuration();

	/* DMA Configuration */
	NVIC_Configuration();


	modu3.uart = USART3;
	modu3.tim = TIM6;
	modu3.baud = 115200;
	modu3.parity = 2;
	
	modbusRTUInit(&modu3);
	

	

	while(1)
	{
		if (oldticks != capture)
		{
			if (modu3.modState == Mod_State_Idle && modu3.request == FALSE)
			{
				if (index == 0) {
					mod_master_send(&modu3, 1, ReadHoldRegs, 0, 4);
					index = 1;
				} else {
					RAM16(&(modu3.data[0])) = RAM16(&(MOD_V[0]));
					RAM16(&(modu3.data[2])) = RAM16(&(MOD_V[2]));
					RAM16(&(modu3.data[4])) = RAM16(&(MOD_V[4]));
					RAM16(&(modu3.data[6])) = capture;
					mod_master_send(&modu3, 1, WriteMultiRegs, 4, 8);
					index = 0;
				}
			
			}
			
			cycleWork(&modu3);
			oldticks = capture;
			

		}
		
		runCyc();
		runSys();
	}
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
	/* TIM1, GPIOA and GPIOB clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	/* DMA clock enable */
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

/**
  * @brief  Configure the TIM1 Pins.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOA Configuration: Channel 3 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//USART3 TX = PB10 RX PB11 Pin
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}


/**
  * @brief  Configures the DMA.配置自动发送和接收
  * @param  None
  * @retval None
  */
//void DMA_Configuration(void)
//{
//	DMA_InitTypeDef DMA_InitStructure;

//	DMA_DeInit(UART3_TX_DMA); 
//	DMA_InitStructure.DMA_PeripheralBaseAddr = UART3_DR_Base;
//	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)U3TX_Buffer;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
//	DMA_InitStructure.DMA_BufferSize = 0;//实际运行时设定长度
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; //normal or cycle
//	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //优先级
//	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 	
//	DMA_Init(UART3_TX_DMA, &DMA_InitStructure);
//	
//}

/**
* 我需要几个中断呢？
* a. u3_tx_dma 
* b. u3_recv_time_idle 定时器中断，或者帧中断
* c. send pulse count 计数器中断
* d. systick 最高优先级
*/
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef  NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	
	//systick 9MHz HCLK/8 
	
	if (SysTick_Config(	SystemCoreClock / 5000))
	{
		while(1);
	}
	
	NVIC_InitStructure.NVIC_IRQChannel = (u8)SysTick_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

//void UART_Config(void)
//{
//	USART_InitTypeDef USART_InitStructure;
//	USART_InitStructure.USART_BaudRate = 256000;
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;
//	USART_InitStructure.USART_Parity = USART_Parity_No;
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

//	USART_Init(USART3, &USART_InitStructure);
//	USART_DMACmd(USART3, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
//	//DMA_Cmd(UART3_TX_DMA, ENABLE);
//	//DMA_Cmd(UART3_RX_DMA, ENABLE);
//	
//	DMA_ITConfig(UART3_TX_DMA, DMA_IT_TC, ENABLE);
//	
//	
//	USART_Cmd(USART3, ENABLE);
//	
//}



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

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

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
