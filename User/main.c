/**
  * 文    件：main.c
  * 功    能：Modbus RTU 从机主程序
  * 说    明：本机地址 0x01，对外提供 64 个可读可写寄存器，其中
  *           0x0000 = 温度、0x0001 = 湿度，模拟原来的温湿度传感器。
  *           收帧由串口中断 + 定时器中断完成，主循环只做两件事：
  *           收到整帧就交给 Modbus_Slave_Process 处理，然后刷新 OLED。
  *
  * 本文件中的函数：
  *   main                       初始化各模块，然后循环等待并处理主机请求
  *   Modbus_Display             （内部）把寄存器区里的温湿度显示到 OLED
  */
#include "stm32f10x.h"                  // Device header
#include "Serial.h"
#include "Timer.h"
#include "OLED.h"
#include "Modbus.h"

/**
  * 函    数：把从机当前对外提供的温湿度刷新到 OLED
  * 参    数：无
  * 返 回 值：无
  * 注意事项：显示的是寄存器区里的值，也就是主机通过 0x03/0x04 能读到的值。
  *           主机用 0x06/0x10 写进来多少，这里就显示多少
  */
static void Modbus_Display(void)
{
	int16_t temVal = (int16_t)Modbus_Reg[REG_TEMPERATURE];
	int16_t humVal = (int16_t)Modbus_Reg[REG_HUMIDITY];

	/* 温度编码沿用原传感器约定：数值 ≥10000 表示负温度，实际值为 -(数值-10000) */
	if (temVal >= 10000)
	{
		temVal = -(temVal - 10000);
	}

	OLED_ShowHexNum(1, 7, MODBUS_SLAVE_ADDR, 2);					//从机地址，占第 7~8 列

	OLED_ShowSignedNum(2, 5, temVal / 10, 2);						//温度整数部分，占第 5~7 列
	OLED_ShowChar(2, 8, '.');										//小数点，第 8 列
	OLED_ShowNum(2, 9, (temVal >= 0 ? temVal : -temVal) % 10, 1);	//温度小数部分，第 9 列

	OLED_ShowSignedNum(3, 5, humVal / 10, 2);						//湿度整数部分
	OLED_ShowChar(3, 8, '.');										//小数点
	OLED_ShowNum(3, 9, humVal % 10, 1);								//湿度小数部分
}

int main(void)
{
	Timer_Init();			//初始化定时器
	Serial_Init();			//初始化串口
	OLED_Init();			//初始化OLED
	Modbus_Slave_Init();	//初始化从机寄存器区

	/*显示静态字符串*/
	OLED_ShowString(1, 1, "Slave:");
	OLED_ShowString(2, 1, "Tem:");
	OLED_ShowString(3, 1, "Hum:");

	Modbus_Display();

	ModeI();				//初始化为接收模式

	while (1)
	{
		if (Serial_GetRxFlag())		//判断是否收到了一帧完整请求
		{
			Modbus_Slave_Process(Serial_RxData, RxLen);		//校验、执行并应答
			Modbus_Display();								//把寄存器最新值刷到OLED
		}
	}
}
