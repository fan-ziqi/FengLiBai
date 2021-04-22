#ifndef _PWM_H
#define _PWM_H

#include "stm32f10x.h"



#define MOTOR_IN1_PORT				GPIOF
#define MOTOR_IN1_CLK	 				RCC_APB2Periph_GPIOF
#define MOTOR1_IN1						GPIO_Pin_0
#define MOTOR1_IN2						GPIO_Pin_1
#define MOTOR2_IN1						GPIO_Pin_2
#define MOTOR2_IN2						GPIO_Pin_3
#define MOTOR3_IN1						GPIO_Pin_4
#define MOTOR3_IN2						GPIO_Pin_5
#define MOTOR4_IN1						GPIO_Pin_6
#define MOTOR4_IN2						GPIO_Pin_7

#define TIM3_MOTOR_PWM_PORT				GPIOA
#define TIM3_MOTOR_PWM_CLK	 				RCC_APB2Periph_GPIOA
#define MOTOR_1								GPIO_Pin_6
#define MOTOR_2								GPIO_Pin_7

#define TIM2_MOTOR_PWM_PORT				GPIOB
#define TIM2_MOTOR_PWM_CLK	 				RCC_APB2Periph_GPIOB
#define MOTOR_3								GPIO_Pin_10
#define MOTOR_4								GPIO_Pin_11


//#define ENA_TOGGLE(a)      if(a%2 == 0)\
//													 GPIO_SetBits(POWER_M_PORT,ENA_PIN);\
//													 else\
//													 GPIO_ResetBits(POWER_M_PORT,ENA_PIN);

//void POWER_M(void);
void MOTOR_PWM_Init(u16 arr,u16 psc);
void TIM2_MOTOR_PWM_Init(u16 arr,u16 psc);
void TIM3_MOTOR_PWM_Init(u16 arr,u16 psc);
void TIM3_PWM_COMPARE(u8 ch, u16 TIM_PWM_VAL);
void TIM2_PWM_COMPARE(u8 ch, u16 TIM_PWM_VAL);
void POWER_M(void);
#endif

