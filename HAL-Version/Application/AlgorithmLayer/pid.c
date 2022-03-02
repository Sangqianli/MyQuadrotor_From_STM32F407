/**
 * @file        pid.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        11-September-2020
 * @brief       Pid Algorithm.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "rp_math.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void pid_calculate(pid_ctrl_t *pid)
{
	pid->err = pid->target-pid->measure;
	// p i d ��������
	pid->pout = pid->kp * pid->err;
	pid->iout += (pid->ki * pid->err);
	pid->iout = constrain(pid->iout, -pid->integral_max, +pid->integral_max);
    pid->dout = pid->kd * (pid->err - pid->last_err);
	// �ۼ�pid���ֵ
	pid->out = pid->pout + pid->iout + pid->dout;
	pid->out = constrain(pid->out, -pid->out_max, pid->out_max);
	// ��¼�ϴ����ֵ
	pid->last_err = pid->err;
}

void pid2_calculate(pid2_ctrl_t *pid)
{
	// p i d ��������
	pid->pout = pid->kp * pid->err;
	pid->iout += (pid->ki * pid->err);
	pid->iout = constrain(pid->iout, -pid->integral_max, +pid->integral_max);
    pid->dout = pid->kd * (pid->err - pid->last_err);
	// �ۼ�pid���ֵ
	pid->out = pid->pout + pid->iout + pid->dout;
	pid->out = constrain(pid->out, -pid->out_max, pid->out_max);
	// ��¼�ϴ����ֵ
	pid->last_err = pid->err;
}
void pid_clear(pid_ctrl_t *pid)
{
	pid->err = 0;
	pid->last_err = 0;
	pid->integral = 0;
	pid->pout = 0;
	pid->iout = 0;
	pid->dout = 0;
	pid->out = 0;
}

void pid2_clear(pid2_ctrl_t *pid)
{
	pid->err = 0;
	pid->last_err = 0;
	pid->integral = 0;
	pid->pout = 0;
	pid->iout = 0;
	pid->dout = 0;
	pid->out = 0;
}
