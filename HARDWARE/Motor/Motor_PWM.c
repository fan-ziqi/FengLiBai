/*-------------------------------------------------------------------------------------------
        		   			�����ڿ���ϵͳ(2015-8-12)

 Ӳ��ƽ̨:
 			������: STM32F103VET6 64K RAM 512K ROM
			������: LMD18200T 
		    ��Դ:   DC +12V

 ���ƽ̨:
 			��������: RealView MDK-ARM uVision4.10
			C������ : ARMCC
			ASM������:ARMASM
			������:   ARMLINK
			�ײ�����: ����������������       
 
 ʱ��: 2015��8��12��       
 
 ����: BoX
-------------------------------------------------------------------------------------------*/
#include "motor_pwm.h"
#include "motor_control.h"
#include "stdio.h"
/*------------------------------------------
 				ȫ�ֱ���				
------------------------------------------*/ 
extern M1TypeDef M1;
extern M2TypeDef M2;
/*------------------------------------------
 ��������:����TIM2�������PWMʱ�õ���I/O
 ����˵��:PA0 - TIM2_CH1 - M4_PWM
	   	  PA1 - TIM2_CH2 - M1_PWM
	   	  PA2 - TIM2_CH3 - M2_PWM
		  PA3 - TIM2_CH4 - M3_PWM
		  
		  - M1_DIR  -> PD14
 		  - M1_STOP -> PD12

		  - M2_DIR  -> PD15
 		  - M2_STOP -> PD13
		  
		  - M3_DIR  -> PD10
 		  - M3_STOP -> PD11
		  
		  - M4_DIR  -> PD8
 		  - M4_STOP -> PD9				
------------------------------------------*/								   
void PWM_GPIO_Config(void) 	  
{
	 GPIO_InitTypeDef GPIO_InitStructure;		  
	 
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,  ENABLE); 	
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	  
	 GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/*------------------------------------------
 ��������:����TIM2�����PWM�źŵ�ģʽ
 ����˵��:- TIM2ͨ��4���PWM
 		  - PWMģʽ1
 		  - ���Ե͵�ƽ
		  - PWMƵ�� = 24kHz				
------------------------------------------*/
void PWM_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
		  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 	

	TIM_TimeBaseStructure.TIM_Prescaler =  0; 			        //ʱ��Ԥ��Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseStructure.TIM_Period = 3000;				    //�Զ���װֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	    //ʱ�ӷ�Ƶ1
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode =  TIM_OCMode_PWM2;         
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_Pulse = 0;     
	TIM_OC1Init(TIM2,&TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode =  TIM_OCMode_PWM2;         
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_Pulse = 0;     
	TIM_OC2Init(TIM2,&TIM_OCInitStructure);				        
	
	TIM_OCInitStructure.TIM_OCMode =  TIM_OCMode_PWM2;           
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_Pulse = 0;     
	TIM_OC3Init(TIM2,&TIM_OCInitStructure);	
	
	TIM_OCInitStructure.TIM_OCMode =  TIM_OCMode_PWM2;           
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_Pulse = 0;     
	TIM_OC4Init(TIM2,&TIM_OCInitStructure);			         
	
	TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable); //ʹ��TIM2��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable); //ʹ��TIM2��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM2,TIM_OCPreload_Enable); //ʹ��TIM2��CCR3�ϵ�Ԥװ�ؼĴ���
	TIM_OC4PreloadConfig(TIM2,TIM_OCPreload_Enable); //ʹ��TIM2��CCR4�ϵ�Ԥװ�ؼĴ���

	TIM_SetCompare1(TIM2,0);
	TIM_SetCompare2(TIM2,0);
	TIM_SetCompare3(TIM2,0);
	TIM_SetCompare4(TIM2,0);

	TIM_Cmd(TIM2,ENABLE);	//ʹ��TIM2
}
/*------------------------------------------
 ��������:����˶�����ܽ�����
 ����˵��:- M1_DIR  -> PD14
 		  - M1_STOP -> PD12

		  - M2_DIR  -> PD15
 		  - M2_STOP -> PD13
		  
		  - M3_DIR  -> PD10
 		  - M3_STOP -> PD11
		  
		  - M4_DIR  -> PD8
 		  - M4_STOP -> PD9					
------------------------------------------*/ 
void MOTOR_DIR_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   // �������    
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	M1_Forward;
	M2_Forward;
	M3_Forward;
	M4_Forward;
	
	M1_STOP;
	M2_STOP;
	M3_STOP;
	M4_STOP;	 
}
/*------------------------------------------
 ��������:PWM�����ʼ��				
------------------------------------------*/
void PWM_Init(void) 
{
	PWM_GPIO_Config();
	PWM_Mode_Config();
	MOTOR_DIR_GPIO_Config();		 
}
/*------------------------------------------
 ��������:���1�������˶�
 ��������:CCR2_VALռ�ձȼ���ֵ
 ����˵��:CCR2_VALԽ��ת��Խ�� 				
------------------------------------------*/
void PWM_M1_Forward(uint16_t val)
{   	
	M1_Backward;
	M1_STOP;
	TIM_SetCompare2(TIM2,val);     //ֵԽ��ת��Խ��
}
/*------------------------------------------
 ��������:���1�������˶�
 ��������:CCR2_VALռ�ձȼ���ֵ
 ����˵��:CCR2_VALԽ��ת��Խ�� 				
------------------------------------------*/
void PWM_M1_Backward(uint16_t val)
{   	
	M1_Forward;
	M1_RELEASE;
	TIM_SetCompare2(TIM2,val);     //ֵԽ��ת��Խ��
}
/*------------------------------------------
 ��������:���2�������˶�
 ��������:CCR3_VALռ�ձȼ���ֵ
 ����˵��:CCR3_VALԽ��ת��Խ�� 				
------------------------------------------*/
void PWM_M2_Forward(uint16_t val)
{   
	M2_Backward;
	M2_STOP;
	TIM_SetCompare3(TIM2,val);     //ֵԽ��ת��Խ��
}
/*------------------------------------------
 ��������:���2�������˶�
 ��������:CCR3_VALռ�ձȼ���ֵ
 ����˵��:CCR3_VALԽ��ת��Խ�� 				
------------------------------------------*/
void PWM_M2_Backward(uint16_t val)
{   	
	M2_Forward;
	M2_RELEASE;
	TIM_SetCompare3(TIM2,val);     //ֵԽ��ת��Խ��
}
/*------------------------------------------
 ��������:���3�������˶�
 ��������:CCR4_VALռ�ձȼ���ֵ
 ����˵��:CCR4_VALԽ��ת��Խ�� 				
------------------------------------------*/
void PWM_M3_Forward(uint16_t val)
{   	
	M3_Forward;
	M3_RELEASE;
	TIM_SetCompare4(TIM2,val);     //ֵԽ��ת��Խ��
}
/*------------------------------------------
 ��������:���3�������˶�
 ��������:CCR4_VALռ�ձȼ���ֵ
 ����˵��:CCR4_VALԽ��ת��Խ�� 				
------------------------------------------*/
void PWM_M3_Backward(uint16_t val)
{   
	M3_Backward;
	M3_STOP;
	TIM_SetCompare4(TIM2,val);     //ֵԽ��ת��Խ��
}
/*------------------------------------------
 ��������:���4�������˶�
 ��������:CCR1_VALռ�ձȼ���ֵ
 ����˵��:CCR1_VALԽ��ת��Խ�� 				
------------------------------------------*/
void PWM_M4_Forward(uint16_t val)
{   
	M4_Forward;
	M4_RELEASE;
	TIM_SetCompare1(TIM2,val);     //ֵԽ��ת��Խ��
}
/*------------------------------------------
 ��������:���4�������˶�
 ��������:CCR1_VALռ�ձȼ���ֵ
 ����˵��:CCR1_VALԽ��ת��Խ�� 				
------------------------------------------*/
void PWM_M4_Backward(uint16_t val)
{   	
	M4_Backward;
	M4_STOP;
	TIM_SetCompare1(TIM2,val);     //ֵԽ��ת��Խ��
}


