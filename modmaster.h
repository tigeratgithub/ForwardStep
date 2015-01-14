#include "stm32f10x.h"
#include "utility.h"

#ifndef __MODSTM8
#define __MODSTM8

#define UART_TIEN	BITVAL(&(frame->uart->CR1), 7)
#define UART_TCIEN	BITVAL(&(frame->uart->CR1), 6)
#define UART_TXE	BITVAL(&(frame->uart->SR), 7)
#define UART_TC		BITVAL(&(frame->uart->SR), 6)
#define UART_RXNE	BITVAL(&(frame->uart->SR), 5)
#define UART_RIEN	BITVAL(&(frame->uart->CR1), 5)
#define UART_REN	BITVAL(&(frame->uart->CR1), 2)
#define UART_TEN	BITVAL(&(frame->uart->CR1), 3)
#define UART_ORE	BITVAL(&(frame->uart->SR), 3)

#define TIM_IT_UPDATE BITVAL(&(frame->tim->SR), 0)

#define READ_WORD(X) *(uint16_t*)(X)
#define WRITE_WORD(X, word) (*(uint16_t*)(X) = word)

typedef enum MOD_ROL
{
	Mod_Rol_Slave = 0x00,
	Mod_Rol_Master = 0x01
} Mod_Rol_TypeDef;

typedef enum mod_timer_check
{
	MOD_TIMER_START = 0x01,
	//MOD_TIMER_PAUSE = 0x02,
	//MOD_TIMER_RESUM = 0x03,
	MOD_TIMER_STOP  = 0x04
} Mod_Timer_Check_TypeDef;

typedef enum mod_master_state
{
	Mod_State_Init				= 0x00,
	Mod_State_Idle				= 0x01,
	Mod_State_BroadcastDelay 	= 0x02,
	Mod_State_WaitForReply		= 0x03,
	Mod_State_ProcessReply		= 0x04,
	Mod_State_ProcessErr		= 0x05,
	Mod_State_Sending			= 0x06,
	Mod_State_Recving			= 0x07
} Mod_Master_State_TypeDef;

//events source is 
//1. from timer (timeout)
//2. TX.RX INT
//3. user request
typedef enum mod_master_event
{
	Mod_Event_No				= 0x0000,
	Mod_Event_bc_Req			= 0x1000,
	Mod_Event_bc_Timeout		= 0x2000,
	Mod_Event_Send2Slave		= 0x0001,
	Mod_Event_UnSlave			= 0x0010,
	Mod_Event_OKSlave			= 0x0020,
	Mod_Event_RespTimeout		= 0x0040,
	Mod_Event_FrameError		= 0x0100,
	Mod_Event_EndReply			= 0x0200,
	Mod_Event_EndErrProcess		= 0x4000
} Mod_Master_Event_TypeDef; 

typedef enum mod_errcode
{
	Mod_Err_No				= 0x0,
	Mod_Err_IllFun			= 0x01,
	Mod_Err_IllDAddr		= 0x02,
	Mod_Err_IllDVal			= 0x03,
	Mod_Err_Unknow			= 0x04
} Mod_Exception_TypeDef;

typedef enum 
{
	FALSE = 0,
	TRUE = 1
} BOOL;

typedef struct mod_int_status
{
	BOOL	timeout_int_en;
	BOOL	frame_int_en;
	BOOL	tx_int_en;
	BOOL	rx_int_en;
	BOOL	cycle_int_en;
	
	BOOL	timeout_int_act;
	BOOL	frame_int_act;
	BOOL	tx_int_act;
	BOOL	rx_int_act;
	BOOL	cycle_int_act;
} Mod_Int_Status_TypeDef;



#define MOD_MAX_DATA_LEN	32
#define MOD_MAX_BUF_LEN		50
#define MOD_MAX_RETRYS		3
//modbus master
typedef struct mod_master_frame
{
	uint32_t			baud;		//9=9.6k 19=19.2k 38=38.4 56= 56k 115 = 115200
	uint16_t			parity;		//校验方式 0 = none 1 = odd 2 = even
	uint8_t				fromAddr;			//addr of workstation
	uint8_t				toAddr;
	uint8_t				retryCount;		//
	uint16_t				timeout;		//unit ms
	Mod_Rol_TypeDef		modRole;
	Mod_Master_State_TypeDef	modState;
	Mod_Master_Event_TypeDef	modEvent;
	uint8_t				cmdCode;
	uint16_t			dataAddr;
	uint16_t			dataLen;
	uint8_t				data[MOD_MAX_DATA_LEN];
	uint8_t				txframe[MOD_MAX_BUF_LEN];
	uint8_t				rxframe[MOD_MAX_BUF_LEN];
	uint8_t				txCursor;
	uint8_t				txLen;
	uint8_t				rxCursor;
	uint8_t				rxLen;
	BOOL				request;
	BOOL				txOK;
	BOOL				rxOK;
	BOOL				rxBufOver;
	BOOL				rxOver;	
	BOOL				linkFail;
	BOOL				heartbeat;
	BOOL				respOK;
	uint8_t				errCode;
	TIM_TypeDef*		tim;
	USART_TypeDef*		uart;
} Mod_Master_Frame_TypeDef;


/*
typedef struct mod_mem_addr
{
	
} Mod_Mem_Addr_TypeDef;
*/

typedef enum mod_cmd_code
{
	ReadDInput		= 0x02,
	ReadCoils		= 0x01,
	ReadHoldRegs	= 0x03,
	ReadInputRegs	= 0x04,
	WriteSingleCoil	= 0x05,
	WriteSingleReg	= 0x06,
	ReadExcepStatus	= 0x07,
	Diagnostic		= 0x08,
	WriteMultiCoils	= 0x0f,
	WriteMultiRegs	= 0x10
} Mod_Cmd_Code_TypeDef;

//void modCore(Mod_Master_Frame_TypeDef* aFrame);
void setTimeoutCheck(Mod_Master_Frame_TypeDef* frame, Mod_Timer_Check_TypeDef act);
void setFrameCheck(Mod_Master_Frame_TypeDef* frame, Mod_Timer_Check_TypeDef act);
void modbusRTUInit(Mod_Master_Frame_TypeDef* frame);
void MOD_NVIC_Config(Mod_Master_Frame_TypeDef* frame);
void MOD_UART_Config(Mod_Master_Frame_TypeDef* frame);
void MOD_TIM_Config(TIM_TypeDef* tim);
void mod_master_send(Mod_Master_Frame_TypeDef* frame, uint8_t wsAddr, Mod_Cmd_Code_TypeDef cmdCode, 
		uint16_t dataAddr, uint8_t dataLen);
void cycleWork(Mod_Master_Frame_TypeDef* aFrame);
unsigned short CRC16 (uint8_t *puchMsg, uint8_t usDataLen );
void frameProcessData(Mod_Master_Frame_TypeDef* aFrame);
void sendFrame(Mod_Master_Frame_TypeDef* aFrame);

void mod_int_dma_tc(Mod_Master_Frame_TypeDef* frame);


void mod_int_frame_timeout(Mod_Master_Frame_TypeDef* frame);
void setINTPri(void);
void mod_int_rx(Mod_Master_Frame_TypeDef* frame);
void mod_int_tc(Mod_Master_Frame_TypeDef* frame);
void mod_int_timeout(Mod_Master_Frame_TypeDef* frame);

void startRX(Mod_Master_Frame_TypeDef* frame);
void startTX(Mod_Master_Frame_TypeDef* frame);
void stopUART(Mod_Master_Frame_TypeDef* frame);

void mod_rw_03(Mod_Master_Frame_TypeDef *frame, uint8_t* mod_di, 
		uint8_t* mod_do, uint8_t* mod_m, uint8_t* mod_v);

void mod_rw_16(Mod_Master_Frame_TypeDef *frame, uint8_t* mod_di, 
		uint8_t* mod_do, uint8_t* mod_m, uint8_t* mod_v);
#endif

/*
void modCore(Mod_Master_Frame_TypeDef* aFrame)
{
	int16_t crc, val;
	
	switch (aFrame->modState)
	{
		case Mod_State_Init:
		{
			//enable;
			
			break;
		}
		case Mod_State_Sending:
		{
			break;
		}
		case Mod_State_Idle:
		{
			switch (aFrame->modEvent)
			{
				case Mod_Event_Send2Slave:
				{
					sendFrame(2, aFrame);
					break;
				}
				case Mod_Event_bc_Req:
				{
					while(1);
					break;
				}
				default:
				{
					//do some err flag 
					while(1)
					{}
				}
			}
			break;
		}
		case Mod_State_WaitForReply:
		{
			switch (aFrame->modEvent)
			{
				case Mod_Event_UnSlave:
				{
					disableInterrupts();
					aFrame->rxCursor = 0;
					aFrame->rxBufOver = FALSE;
					aFrame->rxOver = FALSE;
					aFrame->rxOK = FALSE;
					
					enableInterrupts();
					break;
				}
				case Mod_Event_OKSlave:
				{
					//check crc, and adjust ok or fail 
					//crc = CRC16(aFrame->frame, aFrame->rxCursor - 2);
					if ( crc == (uint16_t)aFrame->rxframe[aFrame->rxCursor - 2])
					{
						//do for process;
					}
					break;
				}
				case Mod_Event_RespTimeout:
				{
					disableInterrupts();
					aFrame->rxCursor = 0;
					aFrame->rxBufOver = FALSE;
					aFrame->rxOver = FALSE;
					aFrame->rxOK = FALSE;
					UART2_Cmd(DISABLE);
					aFrame->modState = Mod_State_ProcessErr;
					enableInterrupts();
					break;
				}
				default:
				{
					//do some err flag
				}
			}
			break;
		}
		case Mod_State_ProcessReply:
		{
			switch (aFrame->modEvent)
			{
				case Mod_Event_EndReply:
				{
					
					break;
				}
				case Mod_Event_FrameError:
				{
					
					break;
				}
				default:			//noevent
				{
					
					//do some err flag
				}
			}
			break;
		}
		case Mod_State_ProcessErr:
		{
			switch (aFrame->modEvent)
			{
				case Mod_Event_EndErrProcess:
				{
					//add process
					aFrame->modState = Mod_State_Idle;
					break;
				}
				default:
				{
					//do some err flag
				}
			}
			break;
		}
	}
	
	


}
*/
