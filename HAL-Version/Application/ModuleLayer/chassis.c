/**
 * @file        chassis.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        23-October-2020
 * @brief       Chassis Module.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "chassis.h"

#include "can_potocol.h"
#include "rp_math.h"
#include "kalman.h"
#include "kalman_filter.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void CHASSIS_Init(void);
void CHASSIS_Ctrl(void);
void CHASSIS_Test(void);
void CHASSIS_SelfProtect(void);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// ���̵����������
drv_can_t				*chas_drv[CHAS_MOTOR_CNT];
chassis_motor_t			*chas_motor[CHAS_MOTOR_CNT];
chassis_motor_info_t	*chas_motor_info[CHAS_MOTOR_CNT];

/* Exported variables --------------------------------------------------------*/
// ���̵��PID������
chassis_motor_pid_t 	chas_motor_pid[CHAS_MOTOR_CNT] = {
	[CHAS_LF] = {
		.speed.kp = 11.5,
		.speed.ki = 0.472,
		.speed.kd = 0,
		.speed.integral_max = 18000,
		.speed.out_max = 10000,
		.angle.kp = 0.55,
		.angle.ki = 0,
		.angle.kd = 0.008,
		.angle.integral_max = 0,
		.angle.out_max = 9000,
	},
	[CHAS_RF] = {
		.speed.kp = 11.5,
		.speed.ki = 0.472,
		.speed.kd = 0,
		.speed.integral_max = 18000,
		.speed.out_max = 10000,
		.angle.kp = 0.55,
		.angle.ki = 0,
		.angle.kd = 0.008,
		.angle.integral_max = 0,
		.angle.out_max = 9000,
	},
	[CHAS_LB] = {
		.speed.kp = 11.5,
		.speed.ki = 0.472,
		.speed.kd = 0,
		.speed.integral_max = 18000,
		.speed.out_max = 10000,
		.angle.kp = 0.55,
		.angle.ki = 0,
		.angle.kd = 0.008,
		.angle.integral_max = 0,
		.angle.out_max = 9000,
	},
	[CHAS_RB] = {
		.speed.kp = 11.5,
		.speed.ki = 0.472,
		.speed.kd = 0,
		.speed.integral_max = 18000,
		.speed.out_max = 10000,
		.angle.kp = 0.55,
		.angle.ki = 0,
		.angle.kd = 0.008,
		.angle.integral_max = 0,
		.angle.out_max = 9000,
	},
};

// ����Z����Ƕȿ�����
chassis_z_pid_t		chas_z_pid = {
	.angle = {
		.target = CHAS_MECH_ANGLE_POS_MID,
		.measure = CHAS_MECH_ANGLE_POS_MID,
		.kp = 0.48,
		.ki = 0,
		.kd = 0,
		.integral_max = 0,
		.out_max = 9000,
	},
	.out = 0,
};

// ����ģ�������
chassis_ctrl_t		chas_ctrl = {
	.motor = &chas_motor_pid,
	.z_atti = &chas_z_pid,
};

// ����ģ�鴫����
chassis_dev_t		chas_dev = {
	.chas_motor[CHAS_LF] = &chassis_motor[CHAS_LF],
	.chas_motor[CHAS_RF] = &chassis_motor[CHAS_RF],
	.chas_motor[CHAS_LB] = &chassis_motor[CHAS_LB],
	.chas_motor[CHAS_RB] = &chassis_motor[CHAS_RB],
	.yaw_motor = &gimbal_motor[YAW],
	.imu_sensor = &imu_sensor,
	.rc_sensor = &rc_sensor,
};

// ����ģ����Ϣ
chassis_info_t 	chas_info = {
	.remote_mode = RC,
	.co_pid_mode = MECH,
	.local_mode = CHASSIS_MODE_NORMAL,
	.co_angle_logic = POS_LOGIC,
};

chassis_t chassis = {
	.controller = &chas_ctrl,
	.dev = &chas_dev,
	.info = &chas_info,
	.init = CHASSIS_Init,
	.test = CHASSIS_Test,
	.ctrl = CHASSIS_Ctrl,
	.self_protect = CHASSIS_SelfProtect,
};

/* Private functions ---------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* ��Ϣ�� --------------------------------------------------------------------*/
/**
 *	@brief	���̻�ȡϵͳ��Ϣ
 */
void CHASSIS_GetSysInfo(void)
{
	/*----���Ʒ�ʽ�޸�----*/
	if(sys.remote_mode == RC) {
		chas_info.remote_mode = RC;
	}
	else if(sys.remote_mode == KEY) {
		chas_info.remote_mode = KEY;
	}
	
	/*----����ģʽ�޸�----*/
}

void CHASSIS_GetJudgeInfo(void)
{
}

void CHASSIS_GetRcInfo(void)
{
}

void CHASSIS_GetTopGyroInfo(void)
{
}

void CHASSIS_GetSelfAttitude(void)
{
}

void CHASSIS_UpdateController(void)
{
	chas_motor_pid[CHAS_LF].speed.measure = chas_motor_info[CHAS_LF]->speed;
	chas_motor_pid[CHAS_RF].speed.measure = chas_motor_info[CHAS_RF]->speed;
	chas_motor_pid[CHAS_LB].speed.measure = chas_motor_info[CHAS_LB]->speed;
	chas_motor_pid[CHAS_RB].speed.measure = chas_motor_info[CHAS_RB]->speed;	
}

/* Ӧ�ò� --------------------------------------------------------------------*/
/* ����� --------------------------------------------------------------------*/
void CHASSIS_Init(void)
{
	chas_drv[CHAS_LF] = chas_dev.chas_motor[CHAS_LF]->driver;
	chas_drv[CHAS_RF] = chas_dev.chas_motor[CHAS_RF]->driver;
	chas_drv[CHAS_LB] = chas_dev.chas_motor[CHAS_LB]->driver;
	chas_drv[CHAS_RB] = chas_dev.chas_motor[CHAS_RB]->driver;

	chas_motor[CHAS_LF] = chas_dev.chas_motor[CHAS_LF];
	chas_motor[CHAS_RF] = chas_dev.chas_motor[CHAS_RF];
	chas_motor[CHAS_LB] = chas_dev.chas_motor[CHAS_LB];
	chas_motor[CHAS_RB] = chas_dev.chas_motor[CHAS_RB];	
	
	chas_motor_info[CHAS_LF] = chas_dev.chas_motor[CHAS_LF]->info;
	chas_motor_info[CHAS_RF] = chas_dev.chas_motor[CHAS_RF]->info;
	chas_motor_info[CHAS_LB] = chas_dev.chas_motor[CHAS_LB]->info;
	chas_motor_info[CHAS_RB] = chas_dev.chas_motor[CHAS_RB]->info;
	
}

void CHASSIS_GetInfo(void)
{
	CHASSIS_GetSysInfo();
	CHASSIS_GetJudgeInfo();
	CHASSIS_GetRcInfo();
	CHASSIS_GetTopGyroInfo();
	CHASSIS_GetSelfAttitude();
	CHASSIS_UpdateController();
}

void CHASSIS_SelfProtect(void)
{
	CHASSIS_Stop(chas_motor_pid);
	CHASSIS_PidParamsInit(chas_motor_pid, CHAS_MOTOR_CNT);
	CHASSIS_Z_PidParamsInit(&chas_z_pid);
	CHASSIS_GetInfo();
}

void CHASSIS_PidCtrl(void)
{
	// ���̵���ٶȻ�
	CHASSIS_Speed_PidCalc(chas_motor_pid, CHAS_LF);
	CHASSIS_Speed_PidCalc(chas_motor_pid, CHAS_LB);
	CHASSIS_Speed_PidCalc(chas_motor_pid, CHAS_RF);
	CHASSIS_Speed_PidCalc(chas_motor_pid, CHAS_RB);
	
	// ���̵�������Ӧ
	CHASSIS_PidOut(chas_motor_pid);
}

void CHASSIS_NormalCtrl(void)
{
	
}

void CHASSIS_ReloadBulletCtrl(void)
{
}

void CHASSIS_SzuPupCtrl(void)
{
}

void CHASSIS_BuffCtrl(void)
{
}

void CHASSIS_RcCtrl(void)
{
	
}

void CHASSIS_KeyCtrl(void)
{

}

void CHASSIS_Ctrl(void)
{
	/*----��Ϣ����----*/
	CHASSIS_GetInfo();
	/*----�����޸�----*/ 
	if(chas_info.remote_mode == RC) {
		CHASSIS_RcCtrl();
	}
	else if(chas_info.remote_mode == KEY) {
		CHASSIS_KeyCtrl();
	}
	/*----�������----*/
	CHASSIS_PidCtrl();	
}

void CHASSIS_Test(void)
{
}