#include "plcsys.h"


extern void plc_readIO(void);
extern void plc_writeIO(void);

#define IB_COUNT	2
#define QB_COUNT	1
#define MB_COUNT	32
#define VB_COUNT	1024
#define AIW_COUNT	0
#define AQW_COUNT	0
#define	T_COUNT		32		//256个定时器
#define C_COUNT		32		//256个计数器
#define SMB_COUNT	32
#define AC_COUNT	5
#define BYTE_STACK_SIZE		64
#define BIT_STACK_SIZE		64
#define EDGE_STACK_SIZE		64

vu8		rI[IB_COUNT];
vu8		rQ[QB_COUNT];
vu8		rM[MB_COUNT];
vu8		rV[VB_COUNT];
vu8		rT[T_COUNT];
vu8		rC[C_COUNT];
vu16	rTpv[T_COUNT << 3];				
vu16	rCpv[T_COUNT << 3];
vu8		rSM[SMB_COUNT];					//系统位变量
vu8		rAC[AC_COUNT];					//系统寄存器

vu32	*rSM_ARRAY;						//指向位段区域的指针

vu8		rBitStack[BIT_STACK_SIZE];		//bit stack
vu8		rByteStack[BYTE_STACK_SIZE];	//byte stack
vu8		rEdgeStack[BIT_STACK_SIZE];		//edge stack
u16		edgeIndex;						//边沿堆栈索引

vu32		bitSP;
vu32		byteSP;
vu32		edgeSP;

vu32		cycTick;	
vu32		curTick;
vu32		oldTick;

#define AC	(rAC[0])
#define AC0 (rAC[1])
#define AC1 (rAC[2])
#define AC2 (rAC[3])
#define AC3 (rAC[4])	

/**
#define M(a, b)	MBIT_ARRY[(a)][(b)]
#define MW(X)	&(RAM16(MBYTE_ARRAY[X])
	__LD(M(10,0));
	__A(SM(10,0));
	__ADD_I(10, MW(10))



*/
#define SM(a, b) 	rSM_ARRAY[((a) << 3) + (b)] 
#define SMB(a)		rSM[(a)]
void runFirst(void)
{
	


//	rSM[0].bits.bit0 = 1;
//	rSM[0].bits.bit1 = 1;
//	rSM[0].bits.bit4 = 0;
//	rSM[0].bits.bit5 = 0;
//	rSM[0].bits.bit6 = 0;
	
	rSM_ARRAY = (vu32*)BITBAND(rSM, 0);

	
	curTick = 0;
	oldTick = curTick;
	cycTick = 0;
	//============start fit code============//
	plc_readIO();
	plc_writeIO();
	

	//============end fit code============//
}

void runCyc(void)
{
	if (curTick < oldTick) 
		cycTick = 65535 - oldTick + curTick;
	else
		cycTick = curTick - oldTick;

	if (oldTick != curTick)
		oldTick = curTick; 

	if (curTick % 60000 < 30000)
		SM(0, 4) = 0;
	else
		SM(0, 4) = 1;

	if (curTick % 1000 < 500)
		SM(0, 5) = 0;
	else
		SM(0, 5) = 1;

	if (SM(0, 6) == 1)
		SM(0, 6) = 0;
	else
		SM(0, 6) = 1;

	plc_readIO();

}

void runLast(void)
{
	SM(0, 0) = 1;
	SM(0, 1) = 0;
	//writeIO();
	//runComm();
}
