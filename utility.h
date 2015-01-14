#include "stm32f10x.h"

#define BITBAND(a, i) (((u32)(a) & 0xF0000000) + 0x2000000 + (((u32)(a) & 0xFFFFF) << 5) + (i<<2))

#define BITVAL(a, i) (*(__IO uint32_t*)(((u32)(a) & 0xF0000000) + 0x2000000 + (((u32)(a) & 0xFFFFF) << 5) + (i<<2)))

#define MEM_ADDR(addr) *((volatile unsigned long *) (adr))

#define RAM8(x)   	(*(__IO u8*)(u32)(x))
#define RAM16(x)   	(*(__IO u16*)(u32)(x))
#define RAM32(x)   	(*(__IO u32*)(u32)(x))

