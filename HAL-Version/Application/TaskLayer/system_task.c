/**
 * @file        system_task.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        27-October-2020
 * @brief       Decision Center.
 */

/* Includes ------------------------------------------------------------------*/
#include "system_task.h"

#include "cmsis_os.h"
#include "device.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void action_nomal(void);
static void action_lost(void);
static void action_error(void);
static void action_wrong(void);
static void Data_clear(void);
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

system_t sys = {
    .remote_mode = ATTITUDE,
    .state = SYS_STATE_RCLOST,
    .event.SYS_RESET = false,
    .event.REMOTE_SWITCH = false,
    .event.AUTO_MODE_SWITCH = false,
    .event.ALL_READY	= false,
};

statelogic_t syslogic[]= {
    {STATE_RCLOST,RC_NOMAL,STATE_NORMAL,NULL,action_lost},
    {STATE_NORMAL,RC_OFF,STATE_RCLOST,Data_clear,action_nomal},
    {STATE_RCLOST,RC_ERR,STATE_RCERR,action_error,NULL},
    {STATE_RCLOST,RC_SOMEWRONG,STATE_WRONG,action_wrong,NULL},
};
statemachine_t sysmachine = {
    .curstate = STATE_RCLOST,
    .action = action_lost,
    .transnum = 4,
    .logictabal = syslogic,
};

statelogic_t* findTranss(statemachine_t *psm,const event_t evt) {
    int i;
    for(i=0; i<psm->transnum; i++)
    {
        if((psm->logictabal[i].curstate == psm->curstate)&&(psm->logictabal[i].event == evt))
        {
            return &psm->logictabal[i];
        }

    }
    return NULL;
}

void * findAction(statemachine_t *psm)
{
    int i;
    for(i=0; i<psm->transnum; i++)
    {
        if(psm->logictabal[i].curstate == psm->curstate)
        {
            return psm->logictabal[i].action;
        }
    }
    return NULL;
}

void transStatemachine(statemachine_t *psm,event_t evt)
{
    statelogic_t *pTranss;

    pTranss = findTranss(psm,evt);

    if(pTranss != NULL)
    {
        psm->curstate = pTranss->nextstate;//转换状态
        if(pTranss->transaction!= NULL)
            pTranss->transaction();//执行转换函数
    }

    psm->action = findAction(psm);//状态常态执行函数修改
    if(psm->action != NULL)
        psm->action();
}

/* Private functions ---------------------------------------------------------*/
static event_t sys_event = RC_OFF;

static void getEvent()
{

    if(sys_event ==RC_OFF)
    {
        /* 遥控错误 */
        if(rc_sensor.errno == DEV_DATA_ERR) {
            sys_event = RC_ERR;
        } else {
            sys_event = RC_SOMEWRONG;
        }
    }
    /* 遥控在线 */
    if ((rc_sensor.work_state == DEV_ONLINE)&&(rc_sensor.errno == NONE_ERR))
    {
        sys_event =RC_NOMAL;
    }
    else
    {
        sys_event =RC_OFF;
    }


}


static void Data_clear()
{
	Fly_clear();
}
/**
 *	@brief	通过遥控器更新系统信息(非正常状态下重置遥控信息)
 */
static void rc_update_info(void)
{
    if(sys.state != SYS_STATE_NORMAL) {
        sys.remote_mode = ATTITUDE;
    }
    else {
        if( rc_sensor.info->s2 == RC_SW_MID )
        {
            sys.remote_mode = ATTITUDE;
        }
        else if(rc_sensor.info->s2 == RC_SW_UP)
        {
            sys.remote_mode = STEADY;
        } else if(rc_sensor.info->s2 == RC_SW_DOWN)
        {
            sys.remote_mode = AUTO;
        }
    }
}

/**
 *	@brief	根据遥控器切换控制方式
 */
static void system_ctrl_mode_switch(void)
{
    static uint16_t tum_cnt = 0;
    if( (rc_sensor.info->s2_switch_uptomid)||(rc_sensor.info->s2_siwtch_up)||(rc_sensor.info->s2_switch_downtomid)||(rc_sensor.info->s2_siwtch_down) )
    {
        sys.event.REMOTE_SWITCH = true;
        sys.event.RESET_CAL = true;
        sys.event.ALL_READY = false;
        rc_sensor.info->s2_switch_uptomid = false;
        rc_sensor.info->s2_siwtch_up = false;
        rc_sensor.info->s2_switch_downtomid = false;
        rc_sensor.info->s2_siwtch_down = false;

        Data_clear();
    }
}


static void system_state_machine(void)
{
//    if( (sys.switch_state.REMOTE_SWITCH == false)&&(sys.switch_state.SYS_RESET == false) )
    sys.event.ALL_READY = true;
//    if(sys.switch_state.ALL_READY)//系统正常且复位完成后允许切换
//    {
    system_ctrl_mode_switch();
//    }
}

static void action_nomal()
{
    /* 失联恢复 */
    if(sys.state == SYS_STATE_RCLOST)
    {
        // 可在此处同步云台复位标志位
        // 系统参数复位
        sys.event.SYS_RESET = true;//失联复位标志位
        sys.event.RESET_CAL = true;
        sys.event.ALL_READY = false;//未复位好
        sys.remote_mode = ATTITUDE;
//					sys.state = SYS_STATE_NORMAL;
    }
    sys.state = SYS_STATE_NORMAL;
    // 可在此处等待云台复位后才允许切换状态
    system_state_machine();


}
static void action_lost()
{
    sys.state = SYS_STATE_RCLOST;
    RC_ResetData(&rc_sensor);
    Data_clear();//清除任务信息
}

static void action_error()
{
    sys.state = SYS_STATE_RCERR;
    //reset CPU
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}
static void action_wrong()
{
    sys.state = SYS_STATE_WRONG;
    //reset CPU
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}
/* Exported functions --------------------------------------------------------*/
void Application_Init()  //任务层初始化
{
//    Chassis_Init();
//    Gimbal_Init();
//    Fire_Init();
//    Vision_Init();
    Control_Init();
}
/**
 *	@brief	系统决策任务
 */
void StartSystemTask(void const * argument)
{
    for(;;)
    {
        portENTER_CRITICAL();

        // 更新遥控信息
        rc_update_info();

        getEvent();

        transStatemachine(&sysmachine, sys_event);

//        /* 遥控离线 */
//        if(rc_sensor.work_state == DEV_OFFLINE)
//        {
//            sys.state = SYS_STATE_RCLOST;
//            RC_ResetData(&rc_sensor);
//            Data_clear();//清除任务信息
//        }
//        /* 遥控在线 */
//        else if(rc_sensor.work_state == DEV_ONLINE)
//        {
//            /* 遥控正常 */
//            if(rc_sensor.errno == NONE_ERR)
//            {
//                /* 失联恢复 */
//                if(sys.state == SYS_STATE_RCLOST)
//                {
//                    // 可在此处同步云台复位标志位
//                    // 系统参数复位
//                    sys.event.SYS_RESET = true;//失联复位标志位
//                    sys.event.RESET_CAL = true;
//                    sys.event.ALL_READY = false;//未复位好
//                    sys.remote_mode = ATTITUDE;
////					sys.state = SYS_STATE_NORMAL;
//                }
//                sys.state = SYS_STATE_NORMAL;
//                // 可在此处等待云台复位后才允许切换状态
//                system_state_machine();
//            }
//            /* 遥控错误 */
//            else if(rc_sensor.errno == DEV_DATA_ERR) {
//                sys.state = SYS_STATE_RCERR;
//                //reset CPU
//                __set_FAULTMASK(1);
//                NVIC_SystemReset();
//            } else {
//                sys.state = SYS_STATE_WRONG;
//                //reset CPU
//                __set_FAULTMASK(1);
//                NVIC_SystemReset();
//            }
//        }

        portEXIT_CRITICAL();

        osDelay(2);
    }
}
