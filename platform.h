#include "stm32f10x.h"


#define UART_DMA_BUF_SIZE	256

typedef struct uart_dma_tx
{
	uint32_t	DRBase;
	uint8_t		txbuf[UART_DMA_BUF_SIZE];
	
	
} Uart_DMA_Typedef;
