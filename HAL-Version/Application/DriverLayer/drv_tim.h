#ifndef __DRV_TIM_H
#define __DRV_TIM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "rp_math.h"
#include "drv_haltick.h"
/* Exported macro ------------------------------------------------------------*/
#define FRIC_PWM_L	TIM3->CCR1
#define FRIC_PWM_R	TIM3->CCR2

#define FCCW_PWM	TIM4->CCR1
#define LCW_PWM	    TIM4->CCR2
#define LCCW_PWM	TIM4->CCR3
#define FCW_PWM	    TIM4->CCR4

#define UNLOCK_PWM  1000

/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void PWM_Init(void);
void ENCODER_Init(void);
void FRICTION_PwmOut(int16_t pwm1, int16_t pwm2);
void FLY_PwmOut(int16_t fccw, int16_t lcw,int16_t lccw, int16_t fcw);

#endif
