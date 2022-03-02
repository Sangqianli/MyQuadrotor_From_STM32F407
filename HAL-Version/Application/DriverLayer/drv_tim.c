/**
 * @file        drv_tim.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        23-August-2020
 * @brief       TIMER Driver Package(Based on HAL).
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_tim.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void FRICTION_PwmOut(int16_t pwm1, int16_t pwm2);
void COVER_PwmOut(int16_t pwm);
void FLY_PwmOut(int16_t fccw, int16_t lcw,int16_t lccw, int16_t fcw);

/* Exported functions --------------------------------------------------------*/
void PWM_Init(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);	
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);	
	
	FRICTION_PwmOut(0, 0);
	
	FLY_PwmOut(0,0,0,0);
}

void ENCODER_Init(void)
{
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_2);	
}

void FRICTION_PwmOut(int16_t pwm1, int16_t pwm2)
{
	FRIC_PWM_L = pwm1 + 1000;
	FRIC_PWM_R = pwm2 + 1000;
}

void FLY_PwmOut(int16_t fccw, int16_t lcw,int16_t lccw, int16_t fcw)
{
	fccw = constrain(fccw,0,1200);
	lcw = constrain(fccw,0,1200);
	lccw = constrain(fccw,0,1200);
	fcw = constrain(fccw,0,1200);
	
	FCCW_PWM = fccw + UNLOCK_PWM;
	LCW_PWM = lcw + UNLOCK_PWM + 200 ;
	LCCW_PWM = lccw + UNLOCK_PWM;
	FCW_PWM  = fcw + UNLOCK_PWM;
	
}


