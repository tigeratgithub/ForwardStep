#include "stm32f10x.h"

#define BITBAND(a, i) (0x22000000 + (((uint32_t)a - 0x20000000) << 5) + (i << 2))
#define BITVAL(a, i) (*(__IO uint8_t*)(0x22000000 + (((uint32_t)(a) - 0x20000000) << 5) + ((i) << 2)))
