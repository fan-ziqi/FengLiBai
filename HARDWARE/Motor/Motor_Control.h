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
#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include "stm32f10x.h"
/*------------------------------------------
 			  �����������				
------------------------------------------*/
#define STROKE_LIMIT 	 (90000)   //�������г�<0-80000>
#define SPEED_LIMIT  	 (65535)   //�������ٶ�<65535>
#define POWER_LIMIT  	 (3000)    //�����޹���<0-3000>

#define STROKE_MAX   	 (80000)   //��������г�<0-80000>
#define SPEED_MAX	 	 (300)	   //��������ٶ�<0-300>
#define POWER_MAX	 	 (2400)	   //������޹���<0-3000>

#define TIMEOUT_CNT  	 (5000)	   //���㳬ʱ������ms<0-10000>
#define ZERO_POINT	 	 (10000)   //���òο����Ϊ<0-80000>
/*------------------------------------------
 			  �������Ĭ��ֵ				
------------------------------------------*/
#define DEFAULT_SPEED	 (10)	   
#define DEFAULT_POSITION (11000)
#define DEFAULT_POWER    (200)
/*------------------------------------------
 				����ṹ��				
------------------------------------------*/
typedef struct
{
	        float Offset;	  //����ƫ����
			float CurPos;
			float PrevPos;
			float CurAcc;
			float PrevSpeed;

	volatile float SetXPos;	  //�趨λ��
	volatile float SetYPos;	  //�趨λ��
	volatile float SetSpeed;  //�趨�ٶ�
	
	volatile float CurXPos;	  //��ǰλ��
	volatile float CurYPos;	  //��ǰλ��
	volatile float CurSpeed;  //��ǰ�ٶ�ʸ��

	volatile int32_t  PWM;	      //PWM
	volatile uint8_t  ShootFlag;
	volatile uint8_t  AdjustFlag;
	volatile uint8_t  ErrFlag;

	volatile uint32_t SetMaxPos;	  //����趨���λ��
	volatile uint32_t SetMaxPower;	  //����趨�������
	volatile int32_t  SetMaxSpeed;	  //����趨����ٶ�
		
}M1TypeDef,M2TypeDef;

void MCU_Reset(void);
void M1TypeDef_Init(void);
void M2TypeDef_Init(void);
void Mode_0(void);
void Mode_1(void);
void Mode_2(void);
void Mode_3(void);
void Mode_4(void);
void Mode_5(void);
void Mode_6(void);
void MotorMove(int32_t pwm1,int32_t pwm2);
#endif

