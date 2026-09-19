#ifndef __TIMER_H
#define __TIMER_H
#include "stm32f10x.h"

extern volatile uint8_t RxFlag;		//一帧接收完成的标志位
extern volatile uint16_t RxLen;		//该帧的长度，含 CRC
void Timer_Init(void);

#endif
