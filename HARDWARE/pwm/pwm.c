#include "pwm.h"

void TIM3_MOTOR_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(TIM3_MOTOR_PWM_CLK	,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = MOTOR_1;           //GPIOA6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_Init(TIM3_MOTOR_PWM_PORT,&GPIO_InitStructure);              //初始化PA6
	
	GPIO_InitStructure.GPIO_Pin = MOTOR_2;           //GPIOA7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_Init(TIM3_MOTOR_PWM_PORT,&GPIO_InitStructure);              //初始化PA7
	
	TIM_TimeBaseStructure.TIM_Period=arr; //自动重装载值
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//初始化定时器1
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
	
	TIM_CtrlPWMOutputs(TIM3,ENABLE);	//MOE 主输出使能
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM1在CCR1上的预装载寄存器
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM1在CCR1上的预装载寄存器
  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPE使能 
	TIM_Cmd(TIM3, ENABLE);  //使能TIM1
}

void TIM2_MOTOR_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(TIM2_MOTOR_PWM_CLK	,ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);   //重映射必须要开AFIO时钟
	GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = MOTOR_3;           //GPIOB10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_Init(TIM2_MOTOR_PWM_PORT,&GPIO_InitStructure);              //初始化PB10
	
	GPIO_InitStructure.GPIO_Pin = MOTOR_4;           //GPIOB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_Init(TIM2_MOTOR_PWM_PORT,&GPIO_InitStructure);              //初始化PB11
	
	TIM_TimeBaseStructure.TIM_Period=arr; //自动重装载值
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);//初始化定时器1
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
	
	TIM_CtrlPWMOutputs(TIM2,ENABLE);	//MOE 主输出使能
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM1在CCR1上的预装载寄存器
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM1在CCR1上的预装载寄存器
  TIM_ARRPreloadConfig(TIM2,ENABLE);//ARPE使能 
	TIM_Cmd(TIM2, ENABLE);  //使能TIM1
}


void TIM1_PWM_COMPARE(u8 ch, u16 TIM_PWM_VAL)
{
    switch (ch)
    {
    case 1:
        TIM_SetCompare1(TIM3, TIM_PWM_VAL );//通道1占空比
        break;
    case 2:
        TIM_SetCompare2(TIM3, TIM_PWM_VAL );//通道2占空比
        break;
    case 3:
        TIM_SetCompare3(TIM4, TIM_PWM_VAL );//通道3占空比
        break;
    case 4:
        TIM_SetCompare4(TIM4, TIM_PWM_VAL );//通道4占空比
        break;	
    }
}

void POWER_M(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(MOTOR_IN1_CLK	,ENABLE);
	GPIO_InitStruct.GPIO_Pin = MOTOR1_IN1 | MOTOR1_IN2 | MOTOR2_IN1 | MOTOR2_IN2 | MOTOR3_IN1 | MOTOR3_IN2 | MOTOR4_IN1 | MOTOR4_IN2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MOTOR_IN1_PORT	, &GPIO_InitStruct);//e

}

