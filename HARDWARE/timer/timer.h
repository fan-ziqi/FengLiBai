#ifndef __TIMER_H
#define __TIMER_H

#include "sys.h"

// void TIM3_Int_Init(u16 arr,u16 psc);

// void TIM_PWMsteer_Init(u16 arr,u32 psc);
// void TIM_PWMmotor_Init(u16 arr,u32 psc);
// void motorspeed(float speed);
// void motordirect(u8 direct);
// void motorenable(void);

void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_IRQHandler(void);
void TIM3_PWM_Init(u16 arr,u16 psc);


#endif
