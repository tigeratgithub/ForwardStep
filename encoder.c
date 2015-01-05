#include "stm32f10x.h"




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


T_Encoder_TypeDef enc1, enc2;
T_GPIO_Pin_TypeDef spin, ppin, rpin;
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
#define TIMx_PRE_EMPTION_PRIORITY 1
#define TIMx_SUB_PRIORITY 0
void encoder_Deinit(T_Encoder_TypeDef *enc)
{
	enc->ch = 1;
	enc->TIM = TIM1;
	//enc->savedCounter = 0;
	enc->saveAddr = 0;
	enc->reloadCounter = 0;
	enc->curCounter = 0;
	enc->countMode = AB;
	enc->outPin.GPIOX = 0;
	enc->outPin.PinX = 0;
	enc->enablePin.GPIOX = 0;
	enc->enablePin.PinX = 0;
	enc->resetPin.GPIOX = 0;
	enc->resetPin.PinX = 0;
}

TIM_TypeDef* getTIMx(uint8_t ch)
{
	switch (ch)
	{
		case 1:
			return TIM1;
		case 2:
			return TIM2;
		case 3:
			return TIM3;
		case 4:
			return TIM4;
		case 5:
			return TIM5;
		case 8:
			return TIM8;
		default :
			return 0;
	}
}

void encoder_config(T_Encoder_TypeDef *enc, uint8_t ch, int32_t reload, int64_t count, 
		T_Countmode_Enum mode, T_GPIO_Pin_TypeDef outpin, 
		T_GPIO_Pin_TypeDef enablepin, T_GPIO_Pin_TypeDef resetpin)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
  
	/* Encoder unit connected to TIM3, 4X mode */
	//GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	enc->ch = ch;
	enc->TIM = getTIMx(enc->ch);
	enc->reloadCounter = reload;
	enc->curCounter = count;
	enc->countMode = mode;
	enc->outPin = outpin;
	enc->enablePin = enablepin;
	enc->resetPin = resetpin;
	
	TIM_DeInit(enc->TIM);
	
	TIM_TimeBaseStructure.TIM_Period = 49999;		//50000 overflow 
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 200;	//200 * 50000 = 10000000 1kk
	TIM_TimeBaseInit(enc->TIM, &TIM_TimeBaseStructure);

	if (enc->countMode == AB)
		TIM_EncoderInterfaceConfig(enc->TIM, TIM_EncoderMode_TI12, 
			TIM_ICPolarity_BothEdge, TIM_ICPolarity_BothEdge);
	else
		TIM_EncoderInterfaceConfig(enc->TIM, TIM_EncoderMode_TI1, 
			TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 4;
	TIM_ICInit(enc->TIM, &TIM_ICInitStructure); 
	
	TIM_ClearFlag(enc->TIM, TIM_FLAG_Update);
	TIM_ClearITPendingBit(enc->TIM, TIM_IT_Update);
	
	/* Enable the TIM3 Update Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMx_PRE_EMPTION_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIMx_SUB_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	
	enc->TIM->CNT = 0;
	
	TIM_Cmd(enc->TIM, ENABLE);
}

int64_t getEncCounter(T_Encoder_TypeDef *enc)
{
	return (enc->curCounter + enc->TIM->CNT);
}
/**
*	this function change with hardwear, customer write it self;
*
*
**/
void encoder_bindStartPin(GPIO_TypeDef *GPIOx, uint16_t pin, EXTITrigger_TypeDef tl)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	/* Configure PA.00 pin as input floating */
	GPIO_InitStructure.GPIO_Pin = pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOx, &GPIO_InitStructure);

	/* Enable AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Connect EXTI0 Line to PA.00 pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

	/* Configure EXTI0 line */
	EXTI_InitStructure.EXTI_Line = (uint32_t)(1 << pin);
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = tl; 
	//EXTI_Trigger_Rising;  EXTI_Trigger_Falling  EXTI_Trigger_Rising_Falling
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}



void encoder_startCounter(T_Encoder_TypeDef *enc)
{
	TIM_Cmd(enc->TIM, ENABLE);
}

void encoder_pauseCounter(T_Encoder_TypeDef *enc)
{
	TIM_Cmd(enc->TIM, DISABLE);
}

void encoder_resetCounter(T_Encoder_TypeDef *enc)
{
	enc->TIM->CNT = 0;
}

void encoder_setCounter(T_Encoder_TypeDef *enc, int32_t c)
{
	enc->TIM->CNT = c;
}
