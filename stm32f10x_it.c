/**
  ******************************************************************************
  * @file    TIM/OCToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
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
#include "stm32f10x_it.h"

#include "encoder.h"
#include "modmaster.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup TIM_OCToggle
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t capture = 0;
extern vu32	curTick;	//plc ÏµÍ³µÎ´ð
//extern __IO uint16_t CCR1_Val;
//extern __IO uint16_t CCR2_Val;
//extern __IO uint16_t CCR3_Val;
//extern __IO uint16_t CCR4_Val;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	//todo by tiger 2015.1.10 add pulse time check 
	capture ++;
	curTick ++;
}

extern Mod_Master_Frame_TypeDef modu3;
void USART3_IRQHandler(void)
{
	//for tc/rxne
	Mod_Master_Frame_TypeDef* frame = &modu3;

	//tc
	if (USART_GetITStatus(frame->uart, USART_IT_TC) == SET)
		mod_int_tc(frame);
	//rx
	if (USART_GetITStatus(frame->uart, USART_IT_RXNE) == SET)
		mod_int_rx(frame);
}

void DMA1_Channel2_IRQHandler(void)
{
	Mod_Master_Frame_TypeDef* frame = &modu3;
	if (DMA_GetITStatus(DMA1_FLAG_TC2) == SET)
	{
		__disable_irq();
		mod_int_dma_tc(frame);
		__enable_irq();
	}
	DMA_ClearFlag(DMA1_FLAG_GL2 | DMA1_FLAG_TC2 | DMA1_FLAG_HT2 | DMA1_FLAG_TE2);
	DMA_ClearITPendingBit(DMA1_IT_GL2 | DMA1_IT_TC2 | DMA1_IT_HT2 | DMA1_IT_TE2);
}
	
//uint16_t x_count = 0;
void TIM1_UP_IRQHandler(void)
{
//	TIM_Cmd(TIM1, DISABLE);
}

//extern T_Encoder_TypeDef enc1, enc2;
/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/
void TIM4_IRQHandler(void)
{
//	if ((TIM4->SR & 0x1) == 1)
//	{
//		enc1.curCounter += enc1.TIM->ARR * (enc1.TIM->RCR + 1);
//	} 
//	
}

//modbus time check
void TIM6_IRQHandler(void)
{
	Mod_Master_Frame_TypeDef* frame = &modu3;
	if (frame->modState == Mod_State_Recving)
		mod_int_frame_timeout(frame);
	if (frame->modState == Mod_State_WaitForReply)
		mod_int_timeout(frame);
	TIM_IT_UPDATE = FALSE;
}



/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None TIM3_IRQn
  * @retval None
  */
void TIM3_IRQHandler(void)
{
//  /* TIM3_CH1 toggling with frequency = 183.1 Hz */
//  if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
//  {
//    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1 );
//    capture = TIM_GetCapture1(TIM3);
//    TIM_SetCompare1(TIM3, capture + CCR1_Val );
//  }

//  /* TIM3_CH2 toggling with frequency = 366.2 Hz */
//  if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
//  {
//    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
//    capture = TIM_GetCapture2(TIM3);
//    TIM_SetCompare2(TIM3, capture + CCR2_Val);
//  }

//  /* TIM3_CH3 toggling with frequency = 732.4 Hz */
//  if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
//  {
//    TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
//    capture = TIM_GetCapture3(TIM3);
//    TIM_SetCompare3(TIM3, capture + CCR3_Val);
//  }

//  /* TIM3_CH4 toggling with frequency = 1464.8 Hz */
//  if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)
//  {
//    TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
//    capture = TIM_GetCapture4(TIM3);
//    TIM_SetCompare4(TIM3, capture + CCR4_Val);
//  }
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
