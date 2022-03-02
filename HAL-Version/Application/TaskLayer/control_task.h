#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "fire_task.h"

/* Exported macro ------------------------------------------------------------*/
typedef struct Fly
{
	float yaw_target;
	float pitch_target;
	float roll_target;
	
	pid_ctrl_t YawAngle_Pid;
	pid_ctrl_t PitchAngle_Pid;
	pid_ctrl_t RollAngle_Pid;
	pid_ctrl_t YawRate_Pid;
	pid_ctrl_t PitchRate_Pid;
	pid_ctrl_t RollRate_Pid;
	float gun_target;
	
}Fly_t;

typedef enum
{
	FCCW = 0,
	LCW,
	LCCW,
	FCW
}motorID_t;
/* Exported types ------------------------------------------------------------*/
extern int16_t NormalPwm[2];
/* Exported functions --------------------------------------------------------*/
void Control_Init(void);
void Fly_clear(void);
#endif
