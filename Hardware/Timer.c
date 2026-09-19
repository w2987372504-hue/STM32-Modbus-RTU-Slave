/**
  * 文    件：Timer.c
  * 功    能：TIM2 定时中断，用于判定 Modbus RTU 的帧尾
  * 说    明：TIM2 配置成每 1ms 中断一次。Modbus RTU 没有帧头帧尾字节，
  *           靠总线上 3.5 个字符时间的静默来划分帧边界，所以串口每收到一个字节，
  *           本文件里的 Count 就清零，连续 MODBUS_T35_MS 毫秒没有新字节就认为一帧收完了。
  *           帧收完后置 RxFlag=1，并同时把帧长存进 RxLen、索引 RNum 清零，供主循环取用。
  *
  * 对外接口：
  *   Timer_Init                 定时器初始化：TIM2 配置为 1ms 周期中断并开 NVIC
  *
  * 中断服务函数：
  *   TIM2_IRQHandler            定时器更新中断，帧尾静默计时到点则判定一帧接收完成
  */
#include "stm32f10x.h"                  // Device header
#include "Serial.h"
#include "Timer.h"
#include "Modbus.h"
/**
  * 函    数：定时中断初始化
  * 参    数：无
  * 返 回 值：无
  */
void Timer_Init(void)
{
	/*开启时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);			//开启TIM2的时钟
	
	/*配置时钟源*/
	TIM_InternalClockConfig(TIM2);		//选择TIM2为内部时钟，若不调用此函数，TIM默认也为内部时钟
	
	/*时基单元初始化*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//定义结构体变量
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;				//计数周期，即ARR的值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 720 - 1;				//预分频器，即PSC的值
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;			//重复计数器，高级定时器才会用到
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);				//将结构体变量交给TIM_TimeBaseInit，配置TIM2的时基单元	
	
	/*中断输出配置*/
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);						//清除定时器更新标志位
																//TIM_TimeBaseInit函数末尾，手动产生了更新事件
																//若不清除此标志位，则开启中断后，会立刻进入一次中断
																//如果不介意此问题，则不清除此标志位也可
	
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);					//开启TIM2的更新中断
	
	/*NVIC中断分组*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//配置NVIC为分组2
																//即抢占优先级范围：0~3，响应优先级范围：0~3
																//此分组配置在整个工程中仅需调用一次
																//若有多个中断，可以把此代码放在main函数内，while循环之前
																//若调用多次配置分组的代码，则后执行的配置会覆盖先执行的配置
	
	/*NVIC配置*/
	NVIC_InitTypeDef NVIC_InitStructure;						//定义结构体变量
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;				//选择配置NVIC的TIM2线
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//指定NVIC线路使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//指定NVIC线路的抢占优先级为2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			//指定NVIC线路的响应优先级为1
	NVIC_Init(&NVIC_InitStructure);								//将结构体变量交给NVIC_Init，配置NVIC外设
	
	/*TIM使能*/
	TIM_Cmd(TIM2, ENABLE);			//使能TIM2，定时器开始运行
}

volatile uint8_t  RxFlag = 0;	//定义接收标志位变量
volatile uint16_t RxLen  = 0;	//定义接收数据长度变量（最长 256，必须 16 位）

/**
  * 函    数：定时器中断函数，可以复制到使用它的地方
  * 参    数：无
  * 返 回 值：无
  * 注意事项：本中断只负责一件事——用空闲时间判定 Modbus RTU 的帧尾。
  *           RTU 模式没有帧头帧尾字节，全靠总线上 3.5 个字符时间的静默来划分帧边界，
  *           所以每收到一个字节（串口中断里 Count 清零）就开始重新计时，
  *           连续 MODBUS_T35_MS 毫秒没有新字节，就认为这一帧收完了
  */
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		if (TmFlag == 1)		//收到过字节，帧尾计时才有意义；线路一直空闲时 TmFlag 为 0，不会误判
		{
			Count ++;

			if (Count >= MODBUS_T35_MS)
			{
				Count  = 0;
				TmFlag = 0;		//本帧结束，等下一个字节重新开始计时

				RxLen  = RNum;	//先把帧长存下来，
				RNum   = 0;		//再清索引，为下一帧腾出缓冲区
				RxFlag = 1;		//通知主循环：一帧收完了
			}
		}
	}
}

