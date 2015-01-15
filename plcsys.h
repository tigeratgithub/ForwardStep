#include "utility.h"

#ifndef __PLCSYS__
#define __PLCSYS__

//#define INum	16
//#define IBitNum	((INum) << 3)
//#define QNum	16
//#define AIWNum	64
//#define AQWNum 	64
//#define VNum	10240
//#define LNum	64
#define MB_NUM	32
//#define SMNum	256
//#define TNum	256
//#define TvNum	256
//#define CNum	256
//#define CvNum	256
//#define SNum	32
//#define ACNum	5
#define BYTE_STACK_SIZE		64
#define BIT_STACK_SIZE		64
#define EDGE_STACK_SIZE		64

void runFirst(void);
void runLast(void);
void runCyc(void);
void runSys(void);
//typedef struct {
//	vu8	bit0: 1;
//	vu8	bit1: 1;
//	vu8	bit2: 1;
//	vu8	bit3: 1;
//	vu8	bit4: 1;
//	vu8	bit5: 1;
//	vu8	bit6: 1;
//	vu8	bit7: 1;
//} PLC_Bits_TypeDef;

//typedef union 
//{
//	PLC_Bits_TypeDef	bits;
//	vu8					byte;
//} PLC_BB_TypeDef;

/**
#define M(a, b)	MBIT_ARRY[(a)][(b)]
#define MW(X)	&(RAM16(MBYTE_ARRAY[X])
	__LD(M(10,0));
	__A(SM(10,0));
	__ADD_I(10, MW(10))



*/

#endif
