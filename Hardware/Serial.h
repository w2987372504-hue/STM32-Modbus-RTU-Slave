#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>
#include "stm32f10x.h"
#include "Modbus.h"		//用到 MODBUS_BUF_SIZE

extern uint8_t Serial_RxData[MODBUS_BUF_SIZE];	//定义串口接收的数据数组
extern volatile uint16_t RNum;					//定义串口接收的数据数组的索引变量
extern volatile uint8_t Count;					//帧尾静默计时，单位 ms
extern volatile uint8_t TmFlag;					//本帧已收到过字节的标志位

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);
uint8_t Serial_GetRxFlag(void);
void Serial_ClearRx(void);
void ModeI(void);
void ModeO(void);


#endif
