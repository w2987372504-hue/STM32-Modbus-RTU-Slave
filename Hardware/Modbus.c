/**
  * 文    件：Modbus.c
  * 功    能：Modbus RTU 从机协议层
  * 说    明：负责校验主机的请求帧、执行寄存器读写、组装应答帧或异常帧。
  *           收字节和判帧尾由串口中断、定时器中断完成（见 Serial.c、Timer.c），
  *           本文件只处理"已经收完整的一帧"。
  *
  * 对外接口：
  *   Modbus_CRC16               计算 Modbus RTU 的 CRC16 校验值
  *   Modbus_Slave_Init          从机初始化，设置寄存器区的上电初值
  *   Modbus_Slave_Process       从机主处理入口，每收到一帧完整请求调用一次
  *
  * 内部函数（static，仅供本文件使用）：
  *   Modbus_Reply               给应答帧补 CRC 并发送，内含 485 方向切换
  *   Modbus_SendException       组装并发送异常应答帧
  *   Modbus_HandleReadRegs      处理 0x03 / 0x04 读寄存器请求
  *   Modbus_HandleWriteSingle   处理 0x06 写单个寄存器请求
  *   Modbus_HandleWriteMultiple 处理 0x10 写多个寄存器请求
  */
#include "Modbus.h"
#include "Serial.h"

uint16_t Modbus_Reg[MODBUS_REG_NUM];			//寄存器区，主机通过 0x03/0x04 读、0x06/0x10 写

static uint8_t Modbus_TxBuf[MODBUS_BUF_SIZE];	//应答帧组帧缓冲区，不含 CRC
static uint8_t Modbus_Broadcast;				//本帧是否为广播帧（地址 0x00），广播帧只执行不应答

/**
  * 函    数：计算 Modbus RTU 的 CRC16 校验值
  * 参    数：puchMsg  要计算的数据首地址
  * 参    数：usDataLen 数据长度，单位字节
  * 返 回 值：CRC16 校验值
  * 算    法：CRC-16/MODBUS，多项式 0x8005（反转后为 0xA001），初值 0xFFFF
  * 注意事项：返回的是 16 位数值，上线时低字节在前、高字节在后，
  *           即先发 (crc & 0xFF)，再发 (crc >> 8)
  *           变量名必须用小写 crc：stm32f10x.h 里已把 CRC 定义成外设指针宏，
  *           写成大写会被宏替换，整个函数编译不过
  */
uint16_t Modbus_CRC16(uint8_t *puchMsg, uint16_t usDataLen)
{
	uint16_t crc = 0xFFFF;		//CRC 初值，Modbus 规定固定为 0xFFFF
	uint8_t i;

	while (usDataLen --)		//遍历每一个字节
	{
		crc ^= *puchMsg ++;		//把当前字节异或到 crc 的低字节

		for (i = 0; i < 8; i ++)	//对该字节的 8 位逐位处理
		{
			if (crc & 0x0001)			//如果最低位是 1
			{
				crc = (crc >> 1) ^ 0xA001;	//右移一位后异或多项式
			}
			else						//如果最低位是 0
			{
				crc >>= 1;					//只右移一位
			}
		}
	}
	return crc;
}

/**
  * 函    数：把 Modbus_TxBuf 里的前 len 个字节补上 CRC 后发送出去
  * 参    数：len  待发送数据的长度，不含 CRC 本身
  * 返 回 值：无
  * 注意事项：本函数内部完成 RS485 的方向切换，调用者只管把数据填进 Modbus_TxBuf
  *           广播帧（地址 0x00）在这里被拦下，只执行不应答
  */
static void Modbus_Reply(uint16_t len)
{
	uint16_t crc;

	if (Modbus_Broadcast)
	{
		return;				//广播帧不应答，直接返回
	}

	crc = Modbus_CRC16(Modbus_TxBuf, len);
	Modbus_TxBuf[len]     = (uint8_t)(crc & 0xFF);		//CRC 低字节在前
	Modbus_TxBuf[len + 1] = (uint8_t)(crc >> 8);		//CRC 高字节在后

	ModeO();					//切到发送：DE 拉高，驱动器使能
	Serial_SendArray(Modbus_TxBuf, len + 2);

	/* 必须等 TC 而不是 TXE：TXE 只说明发送数据寄存器空了，
	   最后一个字节还停在移位寄存器里往外发，此时拉低 DE 会把它截断，
	   主机侧表现为"偶尔最后一个字节丢失、老是报 CRC 错" */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

	ModeI();					//切回接收

	Serial_ClearRx();			//清掉方向切换瞬间可能收进来的毛刺
}

/**
  * 函    数：组装并发送一个异常应答帧
  * 参    数：addr  从机地址，取自请求帧
  * 参    数：func  请求帧里的功能码（本函数负责置上 0x80 标志位）
  * 参    数：code  异常码，取值 MODBUS_EXC_xxx
  * 返 回 值：无
  */
static void Modbus_SendException(uint8_t addr, uint8_t func, uint8_t code)
{
	Modbus_TxBuf[0] = addr;				//地址沿用请求帧里的，保证与请求对得上
	Modbus_TxBuf[1] = func | 0x80;		//功能码最高位置 1 表示这是异常应答
	Modbus_TxBuf[2] = code;

	Modbus_Reply(3);					//异常帧固定 5 字节，此处 3 字节加 CRC
}

/**
  * 函    数：处理 0x03 / 0x04 读寄存器请求
  * 参    数：frame  指向完整的请求帧
  * 参    数：len    请求帧长度，含 CRC
  * 返 回 值：无
  * 帧格式：请求 [addr][fc][起始Hi][起始Lo][数量Hi][数量Lo][CRCLo][CRCHi]  固定 8 字节
  *         应答 [addr][fc][字节数=2N][数据Hi][数据Lo]...[CRCLo][CRCHi]
  */
static void Modbus_HandleReadRegs(uint8_t *frame, uint16_t len)
{
	uint16_t startAddr, regNum, i;

	if (len != 8)
	{
		return;			//读请求必然是 8 字节，长度不对说明帧本身就是坏的，连异常都不回
	}

	startAddr = ((uint16_t)frame[2] << 8) | frame[3];	//起始寄存器地址
	regNum    = ((uint16_t)frame[4] << 8) | frame[5];	//要读的寄存器个数

	if (regNum < 1 || regNum > 125)		//协议规定的合法范围是 1~125
	{
		Modbus_SendException(frame[0], frame[1], MODBUS_EXC_ILLEGAL_VALUE);
		return;
	}

	if (startAddr + regNum > MODBUS_REG_NUM)	//起始地址 + 个数不得越过寄存器区末尾
	{
		Modbus_SendException(frame[0], frame[1], MODBUS_EXC_ILLEGAL_ADDR);
		return;
	}

	Modbus_TxBuf[0] = frame[0];
	Modbus_TxBuf[1] = frame[1];
	Modbus_TxBuf[2] = (uint8_t)(regNum * 2);		//字节数域 = 寄存器个数 × 2

	for (i = 0; i < regNum; i ++)
	{
		Modbus_TxBuf[3 + i * 2] = (uint8_t)(Modbus_Reg[startAddr + i] >> 8);		//高字节在前
		Modbus_TxBuf[4 + i * 2] = (uint8_t)(Modbus_Reg[startAddr + i] & 0xFF);		//低字节在后
	}

	Modbus_Reply(3 + regNum * 2);
}

/**
  * 函    数：处理 0x06 写单个寄存器请求
  * 参    数：frame  指向完整的请求帧
  * 参    数：len    请求帧长度，含 CRC
  * 返 回 值：无
  * 帧格式：请求 [addr][06][地址Hi][地址Lo][数据Hi][数据Lo][CRCLo][CRCHi]  固定 8 字节
  *         应答与请求完全相同，即把请求帧原样发回
  */
static void Modbus_HandleWriteSingle(uint8_t *frame, uint16_t len)
{
	uint16_t addr, i;

	if (len != 8)
	{
		return;
	}

	addr = ((uint16_t)frame[2] << 8) | frame[3];

	if (addr >= MODBUS_REG_NUM)
	{
		Modbus_SendException(frame[0], frame[1], MODBUS_EXC_ILLEGAL_ADDR);
		return;
	}

	Modbus_Reg[addr] = ((uint16_t)frame[4] << 8) | frame[5];

	for (i = 0; i < 6; i ++)		//0x06 的应答就是请求帧本身
	{
		Modbus_TxBuf[i] = frame[i];
	}

	Modbus_Reply(6);
}

/**
  * 函    数：处理 0x10 写多个寄存器请求
  * 参    数：frame  指向完整的请求帧
  * 参    数：len    请求帧长度，含 CRC
  * 返 回 值：无
  * 帧格式：请求 [addr][10][起始Hi][起始Lo][数量Hi][数量Lo][字节数=2N][数据...][CRCLo][CRCHi]
  *         应答 [addr][10][起始Hi][起始Lo][数量Hi][数量Lo][CRCLo][CRCHi]  只有 8 字节，没有数据
  */
static void Modbus_HandleWriteMultiple(uint8_t *frame, uint16_t len)
{
	uint16_t startAddr, regNum, byteNum, i;

	if (len < 11)		//最短的 0x10 请求是写 1 个寄存器，9 + 2 = 11 字节
	{
		return;
	}

	startAddr = ((uint16_t)frame[2] << 8) | frame[3];
	regNum    = ((uint16_t)frame[4] << 8) | frame[5];
	byteNum   = frame[6];

	if (regNum < 1 || regNum > 123)		//0x10 最多写 123 个寄存器（再大装不下 9 字节帧头）
	{
		Modbus_SendException(frame[0], frame[1], MODBUS_EXC_ILLEGAL_VALUE);
		return;
	}

	/* 字节数域必须等于 个数×2，整帧长度必须等于 9 + 个数×2，两者对不上说明帧是坏的 */
	if (byteNum != regNum * 2 || len != 9 + regNum * 2)
	{
		Modbus_SendException(frame[0], frame[1], MODBUS_EXC_ILLEGAL_VALUE);
		return;
	}

	if (startAddr + regNum > MODBUS_REG_NUM)
	{
		Modbus_SendException(frame[0], frame[1], MODBUS_EXC_ILLEGAL_ADDR);
		return;
	}

	for (i = 0; i < regNum; i ++)
	{
		Modbus_Reg[startAddr + i] = ((uint16_t)frame[7 + i * 2] << 8) | frame[8 + i * 2];
	}

	/* 应答帧 8 字节，前 6 字节与请求帧一致，且不含任何数据 */
	for (i = 0; i < 6; i ++)
	{
		Modbus_TxBuf[i] = frame[i];
	}

	Modbus_Reply(6);
}

/**
  * 函    数：从机初始化，设置寄存器区的上电初值
  * 参    数：无
  * 返 回 值：无
  */
void Modbus_Slave_Init(void)
{
	uint16_t i;

	for (i = 0; i < MODBUS_REG_NUM; i ++)
	{
		Modbus_Reg[i] = 0;
	}

	/* 模拟原温湿度传感器。上电后这两个值不再自动变化，
	   主机用 0x06 / 0x10 写进来多少，读回去就是多少 */
	Modbus_Reg[REG_TEMPERATURE] = 253;		//25.3℃
	Modbus_Reg[REG_HUMIDITY]    = 605;		//60.5%RH

	/* 温度编码沿用原传感器约定，以兼容主机侧原有的解码逻辑：
	   非负温度直接存数值（253 表示 25.3℃），
	   负温度存 10000 + |T|（-5.3℃ 存 10053） */
}

/**
  * 函    数：从机主处理入口，收到一帧完整请求后调用
  * 参    数：frame  指向接收缓冲区里的完整帧
  * 参    数：len    帧长度，含 CRC
  * 返 回 值：无
  * 注意事项：前四步中的"静默返回"是从机的关键。地址不匹配或 CRC 错误时
  *           绝不能回异常帧：总线上可能还有别的从机在应答，本机再发就撞车了；
  *           CRC 错误则说明帧本身就是坏的，应答只会让总线更乱。
  *           异常帧只在"地址匹配 + CRC 正确，但功能码或数据非法"时才发
  */
void Modbus_Slave_Process(uint8_t *frame, uint16_t len)
{
	uint16_t crc;

	/* 1. 长度检查：最短的合法帧 4 字节，最长 256 字节 */
	if (len < 4 || len > MODBUS_BUF_SIZE)
	{
		return;
	}

	/* 2. 地址过滤：不是本机地址、也不是广播地址的帧，静默丢弃 */
	if (frame[0] != MODBUS_SLAVE_ADDR && frame[0] != 0x00)
	{
		return;
	}

	/* 3. CRC 校验：不通过说明这帧是坏的（噪声或者半个帧），静默丢弃 */
	crc = Modbus_CRC16(frame, len - 2);
	if ((crc & 0xFF) != frame[len - 2] || (crc >> 8) != frame[len - 1])
	{
		return;
	}

	/* 4. 记住是不是广播帧，Modbus_Reply 会据此决定发不发 */
	Modbus_Broadcast = (frame[0] == 0x00) ? 1 : 0;

	/* 5. 功能码分发 */
	switch (frame[1])
	{
		case MODBUS_FC_READ_HOLDING:
		case MODBUS_FC_READ_INPUT:
			Modbus_HandleReadRegs(frame, len);
			break;

		case MODBUS_FC_WRITE_SINGLE:
			Modbus_HandleWriteSingle(frame, len);
			break;

		case MODBUS_FC_WRITE_MULTIPLE:
			Modbus_HandleWriteMultiple(frame, len);
			break;

		default:
			Modbus_SendException(frame[0], frame[1], MODBUS_EXC_ILLEGAL_FUNC);
			break;
	}
}
