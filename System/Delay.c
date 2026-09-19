/**
  * 文    件：Delay.c
  * 功    能：基于 SysTick 的阻塞式延时
  * 说    明：延时期间 CPU 一直空转等待，不做别的事。本工程里暂时没有地方调用，
  *           属于备用模块。注意 Delay_ms(1000) 会占满一秒钟的 CPU 时间。
  *
  * 对外接口：
  *   Delay_us                   微秒级延时，范围 0~233015（受 SysTick 24 位计数上限约束）
  *   Delay_ms                   毫秒级延时，内部循环调用 Delay_us(1000)
  *   Delay_s                    秒级延时，内部循环调用 Delay_ms(1000)
  */
#include "stm32f10x.h"

/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~233015
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
	SysTick->LOAD = 72 * xus;				//设置定时器重装值
	SysTick->VAL = 0x00;					//清空当前计数值
	SysTick->CTRL = 0x00000005;				//设置时钟源为HCLK，启动定时器
	while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
	SysTick->CTRL = 0x00000004;				//关闭定时器
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
} 
