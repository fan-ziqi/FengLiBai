#include "timer.h"
#include "led.h"
#include "sys.h"



// void TIM_PWMmotor_Init(u16 arr,u32 psc)
// {
// 	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue;
// 	GPIO_InitTypeDef GPIO_Initstructure;
// 	GPIO_InitTypeDef GPIO_Initstrue;
// 	TIM_OCInitTypeDef TIM_OCInitTypeDefStrue;
	
//   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//ʱ��ʹ��TIM4
	
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOD,ENABLE);//ʱ��ʹ��
// 	GPIO_PinRemapConfig(GPIO_Remap_TIM4,ENABLE);
	
// 	GPIO_Initstructure.GPIO_Mode=GPIO_Mode_AF_PP;//���츴�����
// 	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_12;
// 	GPIO_Initstructure.GPIO_Speed=GPIO_Speed_50MHz;
// 	GPIO_Init(GPIOD,&GPIO_Initstructure); //
	

	
// 	TIM_TimeBaseInitStrue.TIM_Period=arr;//������װֵ
// 	TIM_TimeBaseInitStrue.TIM_Prescaler=psc;//����Ԥ��Ƶ��
// 	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up;//���ϼ���
// 	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1;//����ʱ�ӷָ�
// 	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStrue);//����ָ��������ʼ��
	
// 	TIM_OCInitTypeDefStrue.TIM_OCMode=TIM_OCMode_PWM1;//ѡ��ʱ��ģʽ�������ȵ���ģʽ1
// 	TIM_OCInitTypeDefStrue.TIM_OCPolarity=TIM_OCPolarity_High;//����Ƚϼ��Ը�,����ƽС��CCRʱΪ�ߵ�ƽ
// 	TIM_OCInitTypeDefStrue.TIM_OutputState=TIM_OutputState_Enable;//�Ƚ����ʹ��
	

	
// 	TIM_OC1Init(TIM4,&TIM_OCInitTypeDefStrue);
// 	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);
	

	
// 	TIM_Cmd(TIM4,ENABLE);
// 	//TIM_SetComparex();�����õ��PWM��ռ�ձ�
	
	
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
// 	GPIO_Initstrue.GPIO_Mode=GPIO_Mode_Out_PP;
// 	GPIO_Initstrue.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
// 	GPIO_Initstrue.GPIO_Speed=GPIO_Speed_50MHz;
// 	GPIO_Init(GPIOC,&GPIO_Initstrue);
// }

// //TIM2CH1��CH2��������ǰ�ֺ�����֣�TIM2CH3��CH4��������ǰ�ֺ��Һ��֣�

// void motorspeed(float speed)
// {
// 	TIM_SetCompare1(TIM4,speed);
	
// }



// //PC0-7��Ӧ���ĸ����������ת
// void motordirect(u8 direct)//���Ƶ������,0Ϊ��ת��1Ϊ��ת
// {

// 	if(direct==1) //������ת
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

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��

	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx����
							 
}
//��ʱ��3�жϷ������
void TIM3_IRQHandler(void)   //TIM3�ж�
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
		{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ 
		}
}
//TIM3 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM3_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//ʹ�ܶ�ʱ��3ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3������ӳ��  TIM3_CH2->PB5    
 
   //���ø�����Ϊ�����������,���TIM3 CH2��PWM���岨��	GPIOB.5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO
 
   //��ʼ��TIM3
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM3 Channel2 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC2

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
    TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3
}








