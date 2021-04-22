#include "timer.h"
#include "led.h"
#include "sys.h"



// void TIM_PWMmotor_Init(u16 arr,u32 psc)
// {
// 	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue;
// 	GPIO_InitTypeDef GPIO_Initstructure;
// 	GPIO_InitTypeDef GPIO_Initstrue;
// 	TIM_OCInitTypeDef TIM_OCInitTypeDefStrue;
	
//   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//时钟使能TIM4
	
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOD,ENABLE);//时钟使能
// 	GPIO_PinRemapConfig(GPIO_Remap_TIM4,ENABLE);
	
// 	GPIO_Initstructure.GPIO_Mode=GPIO_Mode_AF_PP;//推挽复用输出
// 	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_12;
// 	GPIO_Initstructure.GPIO_Speed=GPIO_Speed_50MHz;
// 	GPIO_Init(GPIOD,&GPIO_Initstructure); //
	

	
// 	TIM_TimeBaseInitStrue.TIM_Period=arr;//设置重装值
// 	TIM_TimeBaseInitStrue.TIM_Prescaler=psc;//设置预分频数
// 	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
// 	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1;//设置时钟分割
// 	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStrue);//根据指定参数初始化
	
// 	TIM_OCInitTypeDefStrue.TIM_OCMode=TIM_OCMode_PWM1;//选择定时器模式：脉冲宽度调试模式1
// 	TIM_OCInitTypeDefStrue.TIM_OCPolarity=TIM_OCPolarity_High;//输出比较极性高,当电平小于CCR时为高电平
// 	TIM_OCInitTypeDefStrue.TIM_OutputState=TIM_OutputState_Enable;//比较输出使能
	

	
// 	TIM_OC1Init(TIM4,&TIM_OCInitTypeDefStrue);
// 	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);
	

	
// 	TIM_Cmd(TIM4,ENABLE);
// 	//TIM_SetComparex();来设置电机PWM的占空比
	
	
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
// 	GPIO_Initstrue.GPIO_Mode=GPIO_Mode_Out_PP;
// 	GPIO_Initstrue.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
// 	GPIO_Initstrue.GPIO_Speed=GPIO_Speed_50MHz;
// 	GPIO_Init(GPIOC,&GPIO_Initstrue);
// }

// //TIM2CH1和CH2代表电机左前轮和左后轮！TIM2CH3和CH4代表电机右前轮和右后轮！

// void motorspeed(float speed)
// {
// 	TIM_SetCompare1(TIM4,speed);
	
// }



// //PC0-7对应着四个电机的正反转
// void motordirect(u8 direct)//控制电机方向,0为反转，1为正转
// {

// 	if(direct==1) //设置正转
// 	{
// 		GPIO_SetBits(GPIOC,GPIO_Pin_0);
// 		GPIO_ResetBits(GPIOC,GPIO_Pin_1);
	
// 	}
// 	else
// 	{
// 		GPIO_SetBits(GPIOC,GPIO_Pin_1);
// 		GPIO_ResetBits(GPIOC,GPIO_Pin_0);
// 	}
// }

// void motorenable(void)
// {
// 	GPIO_SetBits(GPIOC,GPIO_Pin_2);
// }














void TIM3_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设
							 
}
//定时器3中断服务程序
void TIM3_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源 
		}
}
//TIM3 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM3_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//使能定时器3时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3部分重映射  TIM3_CH2->PB5    
 
   //设置该引脚为复用输出功能,输出TIM3 CH2的PWM脉冲波形	GPIOB.5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
 
   //初始化TIM3
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM3 Channel2 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
    TIM_Cmd(TIM3, ENABLE);  //使能TIM3
}








