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
        psm->curstate = pTranss->nextstate;//ת��״̬
        if(pTranss->transaction!= NULL)
            pTranss->transaction();//ִ��ת������
    }

    psm->action = findAction(psm);//״̬��ִ̬�к����޸�
    if(psm->action != NULL)
        psm->action();
}

/* Private functions ---------------------------------------------------------*/
static event_t sys_event = RC_OFF;

static void getEvent()
{

    if(sys_event ==RC_OFF)
    {
        /* ң�ش��� */
        if(rc_sensor.errno == DEV_DATA_ERR) {
            sys_event = RC_ERR;
        } else {
            sys_event = RC_SOMEWRONG;
        }
    }
    /* ң������ */
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
 *	@brief	ͨ��ң��������ϵͳ��Ϣ(������״̬������ң����Ϣ)
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
 *	@brief	����ң�����л����Ʒ�ʽ
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
//    if(sys.switch_state.ALL_READY)//ϵͳ�����Ҹ�λ��ɺ������л�
//    {
    system_ctrl_mode_switch();
//    }
}

static void action_nomal()
{
    /* ʧ���ָ� */
    if(sys.state == SYS_STATE_RCLOST)
    {
        // ���ڴ˴�ͬ����̨��λ��־λ
        // ϵͳ������λ
        sys.event.SYS_RESET = true;//ʧ����λ��־λ
        sys.event.RESET_CAL = true;
        sys.event.ALL_READY = false;//δ��λ��
        sys.remote_mode = ATTITUDE;
//					sys.state = SYS_STATE_NORMAL;
    }
    sys.state = SYS_STATE_NORMAL;
    // ���ڴ˴��ȴ���̨��λ��������л�״̬
    system_state_machine();


}
static void action_lost()
{
    sys.state = SYS_STATE_RCLOST;
    RC_ResetData(&rc_sensor);
    Data_clear();//���������Ϣ
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
void Application_Init()  //������ʼ��
{
//    Chassis_Init();
//    Gimbal_Init();
//    Fire_Init();
//    Vision_Init();
    Control_Init();
}
/**
 *	@brief	ϵͳ��������
 */
void StartSystemTask(void const * argument)
{
    for(;;)
    {
        portENTER_CRITICAL();

        // ����ң����Ϣ
        rc_update_info();

        getEvent();

        transStatemachine(&sysmachine, sys_event);

//        /* ң������ */
//        if(rc_sensor.work_state == DEV_OFFLINE)
//        {
//            sys.state = SYS_STATE_RCLOST;
//            RC_ResetData(&rc_sensor);
//            Data_clear();//���������Ϣ
//        }
//        /* ң������ */
//        else if(rc_sensor.work_state == DEV_ONLINE)
//        {
//            /* ң������ */
//            if(rc_sensor.errno == NONE_ERR)
//            {
//                /* ʧ���ָ� */
//                if(sys.state == SYS_STATE_RCLOST)
//                {
//                    // ���ڴ˴�ͬ����̨��λ��־λ
//                    // ϵͳ������λ
//                    sys.event.SYS_RESET = true;//ʧ����λ��־λ
//                    sys.event.RESET_CAL = true;
//                    sys.event.ALL_READY = false;//δ��λ��
//                    sys.remote_mode = ATTITUDE;
////					sys.state = SYS_STATE_NORMAL;
//                }
//                sys.state = SYS_STATE_NORMAL;
//                // ���ڴ˴��ȴ���̨��λ��������л�״̬
//                system_state_machine();
//            }
//            /* ң�ش��� */
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
