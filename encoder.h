#include "stm32f10x.h"

#ifndef __ENC_H__
#define __ENC_H__


/*
	need function
	1、encoder_startCount
	2、encoder_pauseCount
	3、encoder_resetCount
	4、encoder_setCounter
	5、encoder_init 单相计数，AB相计数，增计数，减计数
	6、encoder_config  Hardware config 

	global variable 
	1、channel no -> TIMER no
	2、saveCounter
	3、reload count
	4、curCount
	5、countMode
	6、outPin GPIO_TypeDef* + Pin_x
	7、enablePin
	8、resetPin
	9、从中间点位开始计数 避免计数抖动 ，好方法啊
*/
typedef struct t_gpio_pin
{
	GPIO_TypeDef* GPIOX;	
	uint16_t	  PinX;		//0,1,2,4,8,16,32,......
} T_GPIO_Pin_TypeDef;

typedef enum t_countmode 
{
	AB			= 0,	//默认4倍频
	Single		= 1   	
} T_Countmode_Enum;

typedef enum t_triggelevel
{
	Raising	= 0,
	Falling	= 1
} T_TrigLevel_Enum;

typedef void (*funTrigger)(void);	//trige int event
typedef void (*funEnable)(void);	//enable int event
typedef void (*funReset)(void);	//reset int event 

typedef struct t_encoder_str
{
	uint8_t				ch;
	TIM_TypeDef 		*TIM;
	int32_t				savedCounter;
	uint32_t			saveAddr;	//ROM, EEPROM address
	int32_t				reloadCounter;
	volatile int64_t	curCounter;
	T_Countmode_Enum 	countMode;
	T_GPIO_Pin_TypeDef	outPin;
	T_GPIO_Pin_TypeDef	enablePin;
	T_GPIO_Pin_TypeDef	resetPin;
	funTrigger			ftrigger;	//output event 
	funEnable			fenable;	//input event
	funReset			freset;		//input event
} T_Encoder_TypeDef;

/*
	need function
	1、encoder_startCount
	2、encoder_pauseCount
	3、encoder_resetCount
	4、encoder_setCounter
	5、encoder_init 单相计数，AB相计数，增计数，减计数
	6、encoder_config  Hardware config 

	global variable 
	1、channel no -> TIMER no
	2、saveCounter
	3、reload count
	4、curCount
	5、countMode
	6、outPin GPIO_TypeDef* + Pin_x
	7、enablePin
	8、resetPin
	9、从中间点位开始计数 避免计数抖动 ，好方法啊
*/

void encoder_Deinit(T_Encoder_TypeDef *enc);
TIM_TypeDef* getTIMx(uint8_t ch);

void encoder_config(T_Encoder_TypeDef *enc, uint8_t ch, int32_t reload, int32_t count, 
		T_Countmode_Enum mode, T_GPIO_Pin_TypeDef outpin, 
		T_GPIO_Pin_TypeDef enablepin, T_GPIO_Pin_TypeDef resetpin);
int64_t getEncCounter(T_Encoder_TypeDef *enc);
/**
*	this function change with hardwear, customer write it self;
*
*
**/
void encoder_bindStartPin(GPIO_TypeDef *GPIOx, uint16_t pin, EXTITrigger_TypeDef tl);

void encoder_startCounter(T_Encoder_TypeDef *enc);

void encoder_pauseCounter(T_Encoder_TypeDef *enc);

void encoder_resetCounter(T_Encoder_TypeDef *enc);

void encoder_setCounter(T_Encoder_TypeDef *enc, int32_t c);

#endif
