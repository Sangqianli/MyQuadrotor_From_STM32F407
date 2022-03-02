#ifndef __SYSTEM_TASK_H
#define __SYSTEM_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "control_task.h"
#include "monitor_task.h"
//#include "chassis_task.h"
//#include "gimbal_task.h"
//#include "fire_task.h"
//#include "vision_task.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
 typedef enum {
	STATE_NORMAL,	// 系统正常
	STATE_RCLOST,	// 遥控失联
	STATE_RCERR,	// 遥控出错
	STATE_WRONG,	// 其它系统错误
} state_t;

typedef enum{
	RC_NOMAL =0,
	RC_OFF =1,
	RC_ERR=2,
	RC_SOMEWRONG=3,
}event_t;


typedef struct {
	state_t curstate;
	event_t event;
	state_t nextstate;
	void (*transaction)(void);
	void (*action)(void);
}statelogic_t;

typedef struct{
	state_t curstate;
	void (*action)(void);
	int transnum;
	statelogic_t* logictabal;
}statemachine_t;
/* Exported functions --------------------------------------------------------*/
void StartSystemTask(void const * argument);
void Application_Init(void);
#endif
