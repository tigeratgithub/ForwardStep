#include "modmaster.h"
#include "utility.h"

#ifdef USE_FULL_ASSERT
void t_assert_failed(u8* file, u32 line);
#endif

#define MOD_15_TICKS	10	//1MS counter
#define MOD_35_TICKS	10	//

//BOOL  TX_Pin __attribute__((at(0x20000004)));  //此处地址并不对
#define TX_Pin BITVAL(&(GPIOC->ODR), 5)

#define MOD_DI_COUNT	100
#define MOD_DO_COUNT	100
#define MOD_M_COUNT		100
#define MOD_V_COUNT		100

uint8_t MOD_DI[MOD_DI_COUNT], MOD_DO[MOD_DO_COUNT];
uint8_t	MOD_M[MOD_M_COUNT], MOD_V[MOD_V_COUNT];

/* Table of CRC values for high杘rder byte */ 
const unsigned char auchCRCHi[] = { 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
} ;

/* Table of CRC values for low杘rder byte */ 
const char auchCRCLo[] = { 
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
}; 

Mod_Int_Status_TypeDef modInts;

void modbusRTUInit(Mod_Master_Frame_TypeDef* frame)
{
	
	//frame->baud = baud;
	frame->retryCount = 0;
	frame->timeout = 2000; //60ms
	frame->modRole = Mod_Rol_Master;
	frame->modState = Mod_State_Init;
	frame->modEvent = Mod_Event_No;
	frame->dataLen = 0;
	frame->txCursor = 0;
	frame->rxCursor = 0;
	frame->txLen = 0;
	frame->rxLen = 0;
	frame->txOK = FALSE;
	frame->rxOK = FALSE;
	frame->rxBufOver = FALSE;
	frame->rxOver = FALSE;
	frame->errCode = 0;
	
	/*
	ITC_SetSoftwarePriority(ITC_IRQ_TIM4_OVF, ITC_PRIORITYLEVEL_1);
	ITC_SetSoftwarePriority(ITC_IRQ_UART2_TX, ITC_PRIORITYLEVEL_1);
	ITC_SetSoftwarePriority(ITC_IRQ_UART2_RX, ITC_PRIORITYLEVEL_1);
	ITC_SetSoftwarePriority(ITC_IRQ_UART2_TX, ITC_PRIORITYLEVEL_1);
	*/
	
	setINTPri();//set int priority 
	
	MOD_NVIC_Config(frame);
	MOD_UART_Config(frame);
	MOD_TIM_Config(frame->tim);
	frame->modState = Mod_State_Idle;
	
}

void MOD_NVIC_Config(Mod_Master_Frame_TypeDef* frame)
{
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	
	//INT: TC, RXNE, TIMcheck
	
	//UART TC/RX INT
	NVIC_InitStructure.NVIC_IRQChannel = (u8)USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x2;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = (u8)DMA1_Channel2_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	//TIMcheck
	if (frame->tim == TIM6) {
		NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
	} else if (frame->tim == TIM7) {
		NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	} else {
		while(1) {}
	}
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x3;
	NVIC_Init(&NVIC_InitStructure);	
}

void MOD_UART_Config(Mod_Master_Frame_TypeDef* frame)
{
	DMA_InitTypeDef DMA_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	
	DMA_Channel_TypeDef *dma_tx, *dma_rx;
	

	USART_DeInit(frame->uart);
	
	USART_InitStructure.USART_BaudRate = frame->baud;
	if (frame->parity == 1 || frame->parity == 2)
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;
	else
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	if (frame->parity == 0)
		USART_InitStructure.USART_Parity = USART_Parity_No;
	else if (frame->parity == 1)
		USART_InitStructure.USART_Parity = USART_Parity_Odd;
	else if (frame->parity == 2)
		USART_InitStructure.USART_Parity = USART_Parity_Even;
	else {
		USART_InitStructure.USART_Parity = USART_Parity_No;
	}
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(frame->uart, &USART_InitStructure);
	
	if (frame->uart == USART1) { 
		dma_tx = DMA1_Channel4;
		dma_rx = DMA1_Channel5;
	} else if (frame->uart == USART2) {
		dma_tx = DMA1_Channel7;
		dma_rx = DMA1_Channel6;
	} else if (frame->uart == USART3) {
		dma_tx = DMA1_Channel2;
		dma_rx = DMA1_Channel3;
	} else if (frame->uart == UART4) {
		dma_tx = DMA2_Channel5;
		dma_rx = DMA2_Channel3;		
	} else {
		while(1){}
	}


	
	//tx
	DMA_DeInit(dma_tx); 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(frame->uart->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)frame->txframe;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 0;//实际运行时设定长度
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; //normal or cycle
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 	
	DMA_Init(dma_tx, &DMA_InitStructure);
	
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)frame->rxframe;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = MOD_MAX_BUF_LEN;
	DMA_Init(dma_rx, &DMA_InitStructure);

	DMA_ITConfig(dma_tx, DMA_IT_TC, ENABLE);
	//DMA_Cmd(dma_tx, DISABLE);
	//DMA_Cmd(dma_rx, DISABLE);

	
	
	// 启用接收中断 用于1.5char 检测
	//否则可以启用IDLE中断，兼容性差一点
	USART_ClearFlag(frame->uart, USART_FLAG_TC | USART_FLAG_RXNE);
	USART_ClearITPendingBit(frame->uart, USART_IT_TC);
	USART_ClearITPendingBit(frame->uart, USART_IT_RXNE);
	
	//USART_ITConfig(frame->uart, USART_IT_TC, ENABLE); 
	USART_ITConfig(frame->uart, USART_IT_RXNE, ENABLE); 
	
	USART_DMACmd(frame->uart, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
	 
	USART_Cmd(frame->uart, ENABLE);
}

/**
* fix time check is 1ms
*
*/
void MOD_TIM_Config(TIM_TypeDef* TIM)
{
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	uint16_t PrescalerValue = (uint16_t) (SystemCoreClock / 10000) - 1;
	
	TIM_DeInit(TIM);
	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 10;	//fix 1ms
	TIM_TimeBaseInit(TIM, &TIM_TimeBaseStructure);
	
	TIM_Cmd(TIM, DISABLE);
	TIM_ClearFlag(TIM, TIM_FLAG_Update);
	TIM_ClearITPendingBit(TIM, TIM_IT_Update);
	TIM_ITConfig(TIM, TIM_IT_Update, ENABLE);

}

void setTimeoutCheck(Mod_Master_Frame_TypeDef* frame, Mod_Timer_Check_TypeDef act)
{
	if (act == MOD_TIMER_START)
	{
		TIM_Cmd(frame->tim, DISABLE);
		TIM_SetAutoreload(frame->tim, frame->timeout * 10); //1ms 单位
		TIM_SetCounter(frame->tim, 0);
		TIM_Cmd(frame->tim, ENABLE);
		
//		TIM2_Cmd(DISABLE);
//		TIM2_SetCounter(0);
//		TIM2_Cmd(ENABLE);
		
		modInts.timeout_int_en = TRUE;
	} else if (act == MOD_TIMER_STOP) {
		TIM_Cmd(frame->tim, DISABLE);
		//TIM2_Cmd(DISABLE);
		modInts.timeout_int_en = FALSE;
	}
}

void setFrameCheck(Mod_Master_Frame_TypeDef* frame, Mod_Timer_Check_TypeDef act)
{
	if (act == MOD_TIMER_START)
	{
		TIM_Cmd(frame->tim, DISABLE);
		TIM_SetAutoreload(frame->tim, 10); //1ms 单位
		TIM_SetCounter(frame->tim, 0);
		TIM_Cmd(frame->tim, ENABLE);
		
		//TIM4_Cmd(DISABLE);
		//TIM4_SetCounter(0);
		//IM4_Cmd(ENABLE);
		
		modInts.frame_int_en = TRUE;
	} else if (act == MOD_TIMER_STOP) {
		TIM_Cmd(frame->tim, DISABLE);
		//TIM4_Cmd(DISABLE);
		modInts.frame_int_en = FALSE;
	}	

}

		
void startRX(Mod_Master_Frame_TypeDef* frame)
{
	//tien tcien rien 7, 6, 5
	
	

	UART_TIEN = FALSE;
	UART_TCIEN = FALSE;
	
	//rxne bit5 tc bit6
	UART_TXE = FALSE;
	UART_TC = FALSE;
	UART_RXNE = FALSE;

	UART_RIEN = TRUE;
	UART_REN = TRUE;
	
	TX_Pin = FALSE;
}

void startTX(Mod_Master_Frame_TypeDef* frame)
{
	//rxne bit5 tc bit6
	UART_RIEN = FALSE;
	UART_TXE = FALSE;
	UART_TC = FALSE;
	UART_RXNE = FALSE;

	UART_TCIEN = FALSE;
	UART_TIEN = FALSE;
	UART_TEN = TRUE;

	TX_Pin = TRUE;
}

void stopUART(Mod_Master_Frame_TypeDef* frame)
{
	UART_TCIEN = FALSE;
	UART_TIEN = FALSE;
	UART_RIEN = FALSE;
	
	UART_TC = FALSE;
	UART_TXE = FALSE;
	UART_RXNE = FALSE;
	
	UART_REN = FALSE;
	UART_TEN = FALSE;

}

void sendFrame(Mod_Master_Frame_TypeDef* frame)
{

	
	frame->respOK = FALSE;
	
	setTimeoutCheck(frame, MOD_TIMER_STOP);
	
	stopUART(frame);
	
	frame->request = FALSE;
	frame->txCursor = 0;
	frame->txOK = FALSE;



	
	frame->modState = Mod_State_Sending;

	
	startTX(frame);
	/* BY TIGER 此处需要特别注意 */
	//UART2_SendData8(frame->txframe[0]);
	if (frame->uart == USART3)
	{
		DMA_Cmd(DMA1_Channel2, DISABLE);
		DMA_SetCurrDataCounter(DMA1_Channel2, frame->txLen);
		DMA_Cmd(DMA1_Channel2, ENABLE);
	}
	//UART_TCIEN = TRUE;

	
	return;
}

void cycleWork(Mod_Master_Frame_TypeDef* frame)
{
	//for ProcessErr, Idle, waitforreply
	if (frame->modState == Mod_State_Idle)
	{
		if (frame->request == TRUE)
		{
			sendFrame(frame);
		}
	} else if (frame->modState == Mod_State_ProcessReply) {
		frameProcessData(frame);
	}
	
}

void frameProcessData(Mod_Master_Frame_TypeDef* frame)
{
	//uint16_t 	*wp;
	uint8_t 	*target;
	uint16_t	i;
	
	frame->fromAddr = frame->rxframe[0];
	//todo process
	if ((frame->rxframe[1] & 0x80) == 0x80) 
	{
		frame->errCode = frame->rxframe[2];
		frame->respOK = FALSE;
	} else {
		if (frame->cmdCode != frame->rxframe[1]) 
		{
			frame->errCode = Mod_Err_Unknow;
			//add err process for cmdCode err
			frame->respOK = FALSE;
		} else {
			//start process 
			switch (frame->cmdCode)
			{
				case ReadHoldRegs:
				{
					frame->dataLen = frame->rxframe[2];
					target = (uint8_t*)((uint32_t)(MOD_V));
					//target = M; //
					for (i = 0; i < frame->dataLen; i += 2)
					{
						//----------------TIGER 此处需要定制 开始--------------------//
						frame->data[i] = frame->rxframe[i + 3];
						frame->data[i + 1] = frame->rxframe[i + 4];
						if (i < (MOD_V_COUNT - 1))
						{
							*target = frame->data[i + 1];
							target ++;
							*target = frame->data[i];
							target ++;
						}
						//----------------TIGER 此处需要定制 结束--------------------//
					}
						
					frame->errCode = Mod_Err_No;
					break;
				}
				case WriteMultiRegs:
				{
					frame->dataAddr = READ_WORD(frame->rxframe[2]);
					frame->dataLen = READ_WORD(frame->rxframe[4]);
					frame->errCode = Mod_Err_No;
					break;
				}
				default :
				{
					//todo add cmd function process
				}
			}
			frame->errCode = 0;
			frame->linkFail = FALSE;
			frame->respOK = TRUE;
		}
	}
	
	frame->modState = Mod_State_Idle;
	frame->request = FALSE;

}



/* The function returns the CRC as a unsigned short type  */
/* message to calculate CRC upon  */ 
/* quantity of bytes in message   */ 
unsigned short CRC16 (uint8_t *puchMsg, uint8_t usDataLen )  	 
{ 
	uint8_t uchCRCHi = 0xFF ;  					/* high byte of CRC initialized   */ 
	uint8_t uchCRCLo = 0xFF ;  					/* low byte of CRC initialized   */ 
	uint8_t uIndex ;  							/* will index into CRC lookup table   */ 
	 
	while (usDataLen--)  						/* pass through message buffer   */ 
	{ 
		uIndex = uchCRCLo ^ *puchMsg++ ;   			/* calculate the CRC   */ 
		uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex] ; 
		uchCRCHi = auchCRCLo[uIndex] ; 
	} 
	return (uchCRCHi << 8 | uchCRCLo) ; 
}



//????????
//void mod_machine{	
//}


void mod_master_send(Mod_Master_Frame_TypeDef* frame, uint8_t wsAddr, Mod_Cmd_Code_TypeDef cmdCode, 
		uint16_t dataAddr, uint8_t dataLen)
{
	uint8_t i, j, len;
	uint16_t crc;
	
	setFrameCheck(frame, MOD_TIMER_STOP);
	
	frame->toAddr = wsAddr;
	frame->cmdCode = cmdCode;
	frame->dataAddr = dataAddr;
	frame->dataLen = dataLen;
	switch (cmdCode)
	{
		case	ReadCoils:
		case	ReadDInput:
		case 	ReadHoldRegs:
		case 	ReadInputRegs:
		{
			
			frame->txframe[0] = wsAddr;
			frame->txframe[1] = cmdCode;
			frame->txframe[2] = (uint8_t)(frame->dataAddr >> 8);
			frame->txframe[3] = (uint8_t)(frame->dataAddr & 0x00ff);
			frame->txframe[4] = (uint8_t)(frame->dataLen >> 8);
			frame->txframe[5] = (uint8_t)(frame->dataLen & 0x00ff);
			len = 5;
			
			crc = CRC16(frame->txframe, 6);
			frame->txframe[6] = (uint8_t)(crc & 0x00ff);
			frame->txframe[7] = (uint8_t)(crc >> 8);
			frame->txLen = 8;
			
			frame->request = TRUE;
			
			break;
		}
		case 	WriteSingleCoil:
		{
			frame->txframe[0] = wsAddr;
			frame->txframe[1] = cmdCode;
			frame->txframe[2] = (uint8_t)(frame->dataAddr >> 8);
			frame->txframe[3] = (uint8_t)(frame->dataAddr & 0x00ff);
			if (frame->data[0] == 0)
			{
				frame->txframe[4] = 0xff;
				frame->txframe[5] = 0x00;	
			} else {
				frame->txframe[4] = 0x00;
				frame->txframe[5] = 0x00;
			}
			crc = CRC16(frame->txframe, 6);
			frame->txframe[6] = (uint8_t)(crc & 0x00ff);
			frame->txframe[7] = (uint8_t)(crc >> 8);
			frame->txLen = 8;
			
			frame->request = TRUE;
			break;
		}
		case 	WriteSingleReg:
		{
			frame->txframe[0] = wsAddr;
			frame->txframe[1] = cmdCode;
			frame->txframe[2] = (uint8_t)(frame->dataAddr >> 8);
			frame->txframe[3] = (uint8_t)(frame->dataAddr & 0x00ff);
			frame->txframe[4] = frame->data[0];//?????
			frame->txframe[5] = frame->data[1];	

			crc = CRC16(frame->txframe, 6);
			frame->txframe[6] = (uint8_t)(crc & 0x00ff);
			frame->txframe[7] = (uint8_t)(crc >> 8);
			frame->txLen = 8;

			frame->request = TRUE;
			break;
			
		}
		case	WriteMultiCoils:
		{
			frame->txframe[0] = wsAddr;
			frame->txframe[1] = cmdCode;
			frame->txframe[2] = (uint8_t)(frame->dataAddr >> 8);
			frame->txframe[3] = (uint8_t)(frame->dataAddr & 0x00ff);
			frame->txframe[4] = (uint8_t)(frame->dataLen >> 8);
			frame->txframe[5] = (uint8_t)(frame->dataLen & 0x00ff);
			len = frame->dataLen;
			frame->txframe[6] = ((len % 8) == 0) ? (len / 8) : (len / 8) + 1;
			j = 7;

			for (i = 0; i < frame->txframe[6]; i ++)
			{
				frame->txframe[j] = frame->data[i];
				j ++;
			}

			crc = CRC16(frame->txframe, j);
			
			frame->txframe[j] = (uint8_t)(crc & 0x00ff);
			frame->txframe[j + 1] = (uint8_t)(crc >> 8);	
			frame->txLen = j + 2;

			frame->request = TRUE;
			break;
		}
		case 	WriteMultiRegs:
		{
			frame->txframe[0] = wsAddr;
			frame->txframe[1] = cmdCode;
			frame->txframe[2] = (uint8_t)(frame->dataAddr >> 8);
			frame->txframe[3] = (uint8_t)(frame->dataAddr & 0x00ff);
			frame->txframe[4] = (uint8_t)(frame->dataLen >> 8);
			frame->txframe[5] = (uint8_t)((frame->dataLen >> 1) & 0x00ff);
			frame->txframe[6] = (frame->dataLen);
			
			j = 7;
			for (i = 0; i < frame->dataLen; i += 2)
			{
				frame->txframe[j] = frame->data[i + 1];
				frame->txframe[j + 1] = frame->data[i];
				j += 2;
			}
			crc = CRC16(frame->txframe, j);
			
			frame->txframe[j ++] = (uint8_t)(crc & 0x00ff);
			frame->txframe[j ++] = (uint8_t)(crc >> 8);
			frame->txLen = j ++;

			frame->request = TRUE;
			
			break;
		}
		default:
		{
			break;
		}
	}
}
//////////////////////////////////////////////////
///////all interrupt function/////////////////////
//////////////////////////////////////////////////
void setmembuf(uint8_t *di, uint8_t *coils, uint8_t *m, uint8_t *v)
{
//
}

void setINTPri(void)
{
	//disableInterrupts();
//	ITC_SetSoftwarePriority(ITC_IRQ_TIM1_OVF, ITC_PRIORITYLEVEL_2);
//	ITC_SetSoftwarePriority(ITC_IRQ_ADC1, ITC_PRIORITYLEVEL_1);
//	ITC_SetSoftwarePriority(ITC_IRQ_TIM2_OVF, ITC_PRIORITYLEVEL_2);
//	ITC_SetSoftwarePriority(ITC_IRQ_TIM4_OVF, ITC_PRIORITYLEVEL_2);
//	ITC_SetSoftwarePriority(ITC_IRQ_UART2_TX, ITC_PRIORITYLEVEL_3);
//	ITC_SetSoftwarePriority(ITC_IRQ_UART2_RX, ITC_PRIORITYLEVEL_3);
	//enableInterrupts();
}

void mod_int_rx(Mod_Master_Frame_TypeDef* frame)
{
	uint32_t xval;
	
	if (UART_RXNE == TRUE)
	{
		UART_RXNE = FALSE;
		if (frame->rxCursor < (sizeof(frame->rxframe) - 1))
		{
			frame->rxframe[frame->rxCursor] = frame->uart->DR;
			frame->rxCursor ++;
			frame->rxLen = frame->rxCursor;
			frame->modState = Mod_State_Recving;
			setFrameCheck(frame, MOD_TIMER_START);
		}
		else 
		{
			//buffer is over
			setFrameCheck(frame, MOD_TIMER_STOP);
			frame->modState = Mod_State_ProcessReply;

	
			frame->rxBufOver = TRUE;
		}

	}
	if (UART_ORE == SET)
	{
		//sr = UART2->SR;
		//sr = UART2->DR;
		UART_RXNE = FALSE;

		xval = frame->uart->SR;
		if (xval != 9999)
			xval = frame->uart->DR;
		frame->modState = Mod_State_ProcessReply;

		setFrameCheck(frame, MOD_TIMER_STOP);

		frame->rxOver = TRUE;
	}
}

void mod_int_dma_tc(Mod_Master_Frame_TypeDef* frame)
{
	UART_TC = FALSE;
	UART_TCIEN = TRUE;
}

void mod_int_tc(Mod_Master_Frame_TypeDef* frame)
{
	if (UART_TC == TRUE)
	{
		stopUART(frame);

		frame->txOK = TRUE;

		frame->rxOK = FALSE;
		frame->rxCursor = 0;
		frame->rxLen = 0;
		frame->rxBufOver = FALSE;
		frame->rxOver = FALSE;

		setTimeoutCheck(frame, MOD_TIMER_START);

		frame->modState = Mod_State_WaitForReply;
		frame->modEvent = Mod_Event_No;	//reset Event
		startRX(frame);
	}
}



void mod_int_timeout(Mod_Master_Frame_TypeDef* frame)
{
	stopUART(frame);
	
	setTimeoutCheck(frame, MOD_TIMER_STOP);
	
	if (TIM_IT_UPDATE == TRUE)
	{
		TIM_IT_UPDATE = FALSE;
		//Do something for response timeout
		//events & states think for overload
		
		frame->modState = Mod_State_ProcessErr;
		frame->modEvent = Mod_Event_RespTimeout;

		if (frame->retryCount < MOD_MAX_RETRYS) //exceed max retry count
		{
			//resend , count ++
			frame->retryCount ++;
			sendFrame(frame);
			return;
		} else {
			//commucation error
			frame->retryCount = 0;
			frame->linkFail = TRUE;
			frame->modState = Mod_State_Idle;
			
			return;
		}
	} else {
		//process exception code
		//todo
		while(1);
	}
}

void mod_int_frame_timeout(Mod_Master_Frame_TypeDef* frame)
{
	uint16_t crc, val;

	stopUART(frame);

	setFrameCheck(frame, MOD_TIMER_STOP);
	
	//exception Slave or unexception Slave
	TIM_IT_UPDATE = FALSE;


	if (frame->rxframe[0] != frame->toAddr)
	{
		if (frame->retryCount < MOD_MAX_RETRYS) //exceed max retry count
		{
			//resend , count ++
			frame->retryCount ++;
			sendFrame(frame);
			return;
		} else {
			//commucation error
			frame->retryCount = 0;
			frame->linkFail = TRUE;
			frame->modState = Mod_State_Idle;
			frame->request = FALSE;

			return;
		}			
	} else {

		frame->rxOK = TRUE;

		
		if (frame->rxCursor < 2) 
			frame->rxCursor = 10;
		crc = CRC16(frame->rxframe, frame->rxCursor - 2);
		val = ((uint16_t)(frame->rxframe[frame->rxCursor - 1]) << 8) + 
			frame->rxframe[frame->rxCursor - 2];
		if (crc != val)
		{
			if (frame->retryCount < MOD_MAX_RETRYS) //exceed max retry count
			{
				//resend , count ++
				frame->retryCount ++;
				sendFrame(frame);
				return;
			} else {
				//commucation error
				frame->retryCount = 0;
				frame->linkFail = TRUE;
				frame->modState = Mod_State_Idle;
				frame->request = FALSE;
				return;
			}
		} else {
			//process reply
			frame->modState = Mod_State_ProcessReply;
			frame->modEvent = Mod_Event_No;			
			
			frame->retryCount = 0;
			frame->linkFail = FALSE;

			return;
		}
		
	}


}


///////////////////////////////////////////////////////////////
///////////////hardware interrupts Monit///////////////////////
///////////////////////////////////////////////////////////////



/**
* 读写内存,各自实现
*/
void mod_rw_03(Mod_Master_Frame_TypeDef *frame, uint8_t* mod_di, 
		uint8_t* mod_do, uint8_t* mod_m, uint8_t* mod_v)
{

	uint8_t *target;
	uint16_t i;

	if (frame->dataLen > MOD_V_COUNT)
		return;
	target = (uint8_t*)((uint32_t)(MOD_V));

	for (i = 0; i < frame->dataLen; i = i + 2)
	{
		if (i < (MOD_V_COUNT - 2))
		{
			*target = frame->data[i + 1];
			target ++;
			*target = frame->data[i];
			target ++;
		}
	}

}

void mod_rw_10(Mod_Master_Frame_TypeDef *frame, uint8_t* mod_di, 
		uint8_t* mod_do, uint8_t* mod_m, uint8_t* mod_v)
{
	
}
