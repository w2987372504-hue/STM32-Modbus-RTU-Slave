#ifndef __MODBUS_H
#define __MODBUS_H

#include "stm32f10x.h"

/* ==================== 协议常量 ==================== */

#define MODBUS_BUF_SIZE		256		//Modbus RTU 单帧最大长度（含 CRC），收发缓冲区都按此分配

#define MODBUS_SLAVE_ADDR	0x01	//本机从机地址

#define MODBUS_REG_NUM		64		//寄存器数量，覆盖地址 0x0000~0x003F

/* 帧尾判定：Modbus RTU 规定帧与帧之间至少要有 3.5 个字符时间的静默。
   9600 波特率下 1 个字符 = 1 起始位 + 8 数据位 + 1 停止位 = 10 位 ≈ 1.04ms，
   3.5 个字符 ≈ 3.65ms，取整为 4ms。TIM2 是 1ms 中断一次，故单位是 ms。
   注意：若把波特率改成 115200，T3.5 只有约 0.3ms，这个值必须跟着改小，
   否则从机要等 4ms 才应答，主机会先超时，表现为"从机不响应" */
#define MODBUS_T35_MS		4

/* 寄存器地址分配（整片寄存器都可读可写，没有只读区） */
#define REG_TEMPERATURE		0x0000	//温度，单位 0.1℃，编码方式见 Modbus_Slave_Init
#define REG_HUMIDITY		0x0001	//湿度，单位 0.1%RH

/* 功能码 */
#define MODBUS_FC_READ_HOLDING		0x03	//读保持寄存器
#define MODBUS_FC_READ_INPUT		0x04	//读输入寄存器（本从机与 0x03 共用同一片寄存器区）
#define MODBUS_FC_WRITE_SINGLE		0x06	//写单个寄存器
#define MODBUS_FC_WRITE_MULTIPLE	0x10	//写多个寄存器

/* 异常码 */
#define MODBUS_EXC_ILLEGAL_FUNC		0x01	//非法功能码
#define MODBUS_EXC_ILLEGAL_ADDR		0x02	//非法数据地址
#define MODBUS_EXC_ILLEGAL_VALUE	0x03	//非法数据值

/* ==================== 对外接口 ==================== */

extern uint16_t Modbus_Reg[MODBUS_REG_NUM];		//寄存器区，0x03/0x04 读，0x06/0x10 写

uint16_t Modbus_CRC16(uint8_t *puchMsg, uint16_t usDataLen);
void Modbus_Slave_Init(void);
void Modbus_Slave_Process(uint8_t *frame, uint16_t len);

#endif
