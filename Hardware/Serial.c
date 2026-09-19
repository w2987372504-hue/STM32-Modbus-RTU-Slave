/**
  * 文    件：Serial.c
  * 功    能：USART1 串口驱动 + RS485 收发方向控制
  * 说    明：RS485 是半双工，同一时刻只能收或只能发，发完必须切回接收，
  *           否则主机的下一帧收不到。方向由 PB14 / PB15 两个脚控制（见 ModeO / ModeI）。
  *           接收采用"中断逐字节收 + 定时器判帧尾"的方式，缓冲区是 Serial_RxData。
  *
  * 对外接口：
  *   Serial_Init                串口初始化：PA9 复用推挽输出、PA10 上拉输入、PB14/PB15 方向脚
  *   Serial_SendByte            发送一个字节，等待发送完成
  *   Serial_SendArray           发送一个数组，按字节依次发出
  *   Serial_SendString          发送一个以 '\0' 结尾的字符串
  *   Serial_SendNumber          发送一个十进制数字，可指定显示位数
  *   Serial_Printf              格式化打印，用法同 printf
  *   ModeO                      切换为发送模式（PB14 / PB15 拉高，驱动器使能）
  *   ModeI                      切换为接收模式（PB14 / PB15 拉低，接收器使能）
  *   Serial_GetRxFlag           查询并清除"一帧收完"标志，主循环用它判断该不该处理
  *   Serial_ClearRx             清空接收状态，丢弃半截帧（收发的方向切换后调用）
  *
  * 内部辅助函数：
  *   Serial_Pow                 整数次方，供 Serial_SendNumber 拆位使用
  *   fputc                      printf 的底层重定向函数，由 C 库调用
  *
  * 中断服务函数：
  *   USART1_IRQHandler          串口接收中断，每收到一字节存入缓冲区并把帧尾计时清零
  */
#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
#include "Serial.h"
#include "Timer.h"
#include "Modbus.h"


/**
  * 函    数：串口初始化
  * 参    数：无
  * 返 回 值：无
  */
void Serial_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//开启USART1的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//开启GPIOA的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//开启GPIOB的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA9引脚初始化为复用推挽输出
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA10引脚初始化为上拉输入
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//将PB14引脚初始化为推挽输出，用作RS485方向控制

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//将PB15引脚初始化为推挽输出，用作RS485方向控制
	
	/*USART初始化*/
	USART_InitTypeDef USART_InitStructure;					//定义结构体变量
	USART_InitStructure.USART_BaudRate = 9600;				//波特率
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制，不需要
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//模式，发送模式和接收模式均选择
	USART_InitStructure.USART_Parity = USART_Parity_No;		//奇偶校验，不需要
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	//停止位，选择1位
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//字长，选择8位
	USART_Init(USART1, &USART_InitStructure);				//将结构体变量交给USART_Init，配置USART1
	
	/*中断输出配置*/
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);			//开启串口接收数据的中断
	
	/*NVIC中断分组*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//配置NVIC为分组2
	
	/*NVIC配置*/
	NVIC_InitTypeDef NVIC_InitStructure;					//定义结构体变量
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		//选择配置NVIC的USART1线
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//指定NVIC线路使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		//指定NVIC线路的抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//指定NVIC线路的响应优先级为1
	NVIC_Init(&NVIC_InitStructure);							//将结构体变量交给NVIC_Init，配置NVIC外设
	
	/*USART使能*/
	USART_Cmd(USART1, ENABLE);								//使能USART1，串口开始运行
}

/**
  * 函    数：串口发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);		//将字节数据写入数据寄存器，写入后USART自动生成时序波形
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	//等待发送完成
	/*下次写入数据寄存器会自动清除发送完成标志位，故此循环后，无需清除标志位*/
}

/**
  * 函    数：串口发送一个数组
  * 参    数：Array 要发送数组的首地址
  * 参    数：Length 要发送数组的长度
  * 返 回 值：无
  */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)		//遍历数组
	{
		Serial_SendByte(Array[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}

/**
  * 函    数：串口发送一个字符串
  * 参    数：String 要发送字符串的首地址
  * 返 回 值：无
  */
void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)//遍历字符数组（字符串），遇到字符串结束标志位后停止
	{
		Serial_SendByte(String[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}

/**
  * 函    数：次方函数（内部使用）
  * 返 回 值：返回值等于X的Y次方
  */
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;	//设置结果初值为1
	while (Y --)			//执行Y次
	{
		Result *= X;		//将X累乘到结果
	}
	return Result;
}

/**
  * 函    数：串口发送数字
  * 参    数：Number 要发送的数字，范围：0~4294967295
  * 参    数：Length 要发送数字的长度，范围：0~10
  * 返 回 值：无
  */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)		//根据数字长度遍历数字的每一位
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');	//依次调用Serial_SendByte发送每位数字
	}
}

/**
  * 函    数：使用printf需要重定向的底层函数
  * 参    数：保持原始格式即可，无需变动
  * 返 回 值：保持原始格式即可，无需变动
  */
int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);			//将printf的底层重定向到自己的发送字节函数
	return ch;
}

/**
  * 函    数：自己封装的prinf函数
  * 参    数：format 格式化字符串
  * 参    数：... 可变的参数列表
  * 返 回 值：无
  */
void Serial_Printf(char *format, ...)
{
	char String[100];				//定义字符数组
	va_list arg;					//定义可变参数列表数据类型的变量arg
	va_start(arg, format);			//从format开始，接收参数列表到arg变量
	vsprintf(String, format, arg);	//使用vsprintf打印格式化字符串和参数列表到字符数组中
	va_end(arg);					//结束变量arg
	Serial_SendString(String);		//串口发送字符数组（字符串）
}

void ModeO(void)
{
   GPIO_SetBits(GPIOB, GPIO_Pin_14);
   GPIO_SetBits(GPIOB, GPIO_Pin_15);
}

void ModeI(void)
{
   GPIO_ResetBits(GPIOB, GPIO_Pin_14);
   GPIO_ResetBits(GPIOB, GPIO_Pin_15);
}

uint8_t Serial_GetRxFlag(void)
{
	if(RxFlag == 1)	//判断串口接收的标志位变量是否为1
	{
		RxFlag = 0;	//将串口接收的标志位变量清零
		return 1;	//返回串口接收的标志位变量
	}
	else
	{
		return 0;	//返回串口接收的标志位变量
	}
}

/**
  * 函    数：清空接收状态，丢弃当前正在接收的半截帧
  * 参    数：无
  * 返 回 值：无
  * 注意事项：先读 SR 再读 DR 是 STM32 手册给出的标准清标志序列，
  *           一次能把 RXNE / ORE / NE / FE / PE 全部清掉。
  *           不能用 USART_ClearFlag 代替：该函数在 SPL 里只能传 CTS / LBD / TC / RXNE，
  *           传 ORE 会触发 assert_param 失败。
  *           485 收发方向切换的瞬间总线上常有毛刺，不清的话下一次接收
  *           可能立刻误触发一次 RXNE，把噪声当成帧头
  */
void Serial_ClearRx(void)
{
	volatile uint16_t tmp;

	tmp = USART1->SR;		//读 SR，锁存标志
	tmp = USART1->DR;		//再读 DR，把上面几个标志一并清掉
	(void)tmp;				//tmp 只用于产生这两次读操作，读完就没用了

	RNum   = 0;				//丢弃半截帧，重新开始收
	TmFlag = 0;
	Count  = 0;
}

uint8_t Serial_RxData[MODBUS_BUF_SIZE];		//定义串口接收的数据数组

/* RNum 必须是 16 位：Modbus RTU 单帧最长 256 字节，8 位变量最多表示到 255，
   既存不下"已收满 256 字节"这个状态，下面 RNum < MODBUS_BUF_SIZE 的守卫也会恒为真，
   索引到 255 后直接回绕到 0，把帧头覆盖掉 */
volatile uint16_t RNum = 0;					//接收索引
volatile uint8_t Count = 0;					//帧尾静默计时，单位 ms
volatile uint8_t TmFlag = 0;				//本帧已收到过字节的标志位

/**
  * 函    数：USART1中断函数
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数为中断函数，无需调用，中断触发后自动执行
  *           函数名为预留的指定名称，可以从启动文件复制
  *           请确保函数名正确，不能有任何差异，否则中断函数将不能进入
  */
void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)		//判断是否是USART1的接收事件触发的中断
	{
		TmFlag = 1;						//收到字节了，帧尾计时从此开始有意义

		if (RNum < MODBUS_BUF_SIZE)		//缓冲区没满才存
		{
			Serial_RxData[RNum ++] = USART_ReceiveData(USART1);		//读取数据寄存器，存放在接收的数据变量
		}
		else
		{
			USART_ReceiveData(USART1);	//缓冲区满了，读走丢弃，避免触发溢出，但不停止接收
		}

		Count = 0;						//每收到一个字节，帧尾静默计时就清零重来
	}
}
