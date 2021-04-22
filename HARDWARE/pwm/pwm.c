#include "pwm.h"

void TIM3_MOTOR_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(TIM3_MOTOR_PWM_CLK	,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = MOTOR_1;           //GPIOA6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_Init(TIM3_MOTOR_PWM_PORT,&GPIO_InitStructure);              //��ʼ��PA6
	
	GPIO_InitStructure.GPIO_Pin = MOTOR_2;           //GPIOA7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_Init(TIM3_MOTOR_PWM_PORT,&GPIO_InitStructure);              //��ʼ��PA7
	
	TIM_TimeBaseStructure.TIM_Period=arr; //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//��ʼ����ʱ��1
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1
	
	TIM_CtrlPWMOutputs(TIM3,ENABLE);	//MOE �����ʹ��
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���
  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPEʹ�� 
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM1
}

void TIM2_MOTOR_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(TIM2_MOTOR_PWM_CLK	,ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);   //��ӳ�����Ҫ��AFIOʱ��
	GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = MOTOR_3;           //GPIOB10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_Init(TIM2_MOTOR_PWM_PORT,&GPIO_InitStructure);              //��ʼ��PB10
	
	GPIO_InitStructure.GPIO_Pin = MOTOR_4;           //GPIOB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_Init(TIM2_MOTOR_PWM_PORT,&GPIO_InitStructure);              //��ʼ��PB11
	
	TIM_TimeBaseStructure.TIM_Period=arr; //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);//��ʼ����ʱ��1
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1
	
	TIM_CtrlPWMOutputs(TIM2,ENABLE);	//MOE �����ʹ��
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���
  TIM_ARRPreloadConfig(TIM2,ENABLE);//ARPEʹ�� 
	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIM1
}


void TIM1_PWM_COMPARE(u8 ch, u16 TIM_PWM_VAL)
{
    switch (ch)
    {
    case 1:
        TIM_SetCompare1(TIM3, TIM_PWM_VAL );//ͨ��1ռ�ձ�
        break;
    case 2:
        TIM_SetCompare2(TIM3, TIM_PWM_VAL );//ͨ��2ռ�ձ�
        break;
    case 3:
        TIM_SetCompare3(TIM4, TIM_PWM_VAL );//ͨ��3ռ�ձ�
        break;
    case 4:
        TIM_SetCompare4(TIM4, TIM_PWM_VAL );//ͨ��4ռ�ձ�
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

