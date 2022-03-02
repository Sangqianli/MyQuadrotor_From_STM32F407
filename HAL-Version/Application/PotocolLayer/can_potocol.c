/**
 * @file        can_potocol.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-September-2020
 * @brief       CAN Potocol.
 */

/* Includes ------------------------------------------------------------------*/
#include "can_potocol.h"

#include "drv_can.h"
#include "motor.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	��CAN�����ж�ȡ�����λ�÷���
 */
static uint16_t CAN_GetMotorAngle(uint8_t *rxData)
{
    uint16_t angle;
    angle = ((uint16_t)rxData[0] << 8| rxData[1]);
    return angle;
}

/**
 *	@brief	��CAN�����ж�ȡ�����ת��ת�ٷ���
 */
static int16_t CAN_GetMotorSpeed(uint8_t *rxData)
{
    int16_t speed;
    speed = ((uint16_t)rxData[2] << 8| rxData[3]);
    return speed;
}

/**
 *	@brief	��CAN�����ж�ȡ�����ʵ��ת�ص�������
 */
static int16_t CAN_GetMotorCurrent(uint8_t *rxData)
{
    int16_t current;
    current = ((int16_t)rxData[4] << 8 | rxData[5]);
    return current;
}

/**
 *	@brief	RM3508 CAN��ʶ��
 */
static uint32_t RM3508_GetStdId(drv_can_t *drv)
{
    if((drv->can_id - 0x201U) < 4)
        return 0x200;
    else
        return 0x1FF;
}

/**
 *	@brief	RM3508 CAN�����±�
 */
static uint8_t RM3508_GetDrvId(drv_can_t *drv)
{
    return (drv->can_id - 0x201U)%4;
}

/**
 *	@brief	GM6020 CAN��ʶ��
 */
static uint32_t GM6020_GetStdId(drv_can_t *drv)
{
    if((drv->can_id - 0x205U) < 4)
        return 0x1FF;
    else
        return 0x2FF;
}

/**
 *	@brief	GM6020 CAN�����±�
 */
static uint8_t GM6020_GetDrvId(drv_can_t *drv)
{
    return (drv->can_id - 0x205U)%4;
}

/**
 *	@brief	RM2006 CAN��ʶ��
 */
static uint32_t RM2006_GetStdId(drv_can_t *drv)
{
    if((drv->can_id - 0x201U) < 4)
        return 0x200;
    else
        return 0x1FF;
}

/**
 *	@brief	RM2006 CAN�����±�
 */
static uint8_t RM2006_GetDrvId(drv_can_t *drv)
{
    return (drv->can_id - 0x201U)%4;
}

/* Exported functions --------------------------------------------------------*/
void motor_update(motor_t *motor, uint8_t *rxBuf)
{
    motor_info_t *motor_info = motor->info;

    motor_info->angle = CAN_GetMotorAngle(rxBuf);
    motor_info->speed = CAN_GetMotorSpeed(rxBuf);
    motor_info->current = CAN_GetMotorCurrent(rxBuf);

    motor_info->offline_cnt = 0;
}

void motor_init(motor_t *motor)
{
    drv_can_t *drv_can = motor->driver;

    motor->info->offline_cnt = motor->info->offline_max_cnt+1;
    motor->work_state = DEV_OFFLINE;

    motor->errno = NONE_ERR;
    if(motor->id == DEV_ID_CHASSIS) {
        drv_can->drv_id = RM3508_GetDrvId(drv_can);
        drv_can->std_id = RM3508_GetStdId(drv_can);
    }
    else if(motor->id == DIAL) {
        drv_can->drv_id = RM2006_GetDrvId(drv_can);
        drv_can->std_id = RM2006_GetStdId(drv_can);
    }
    else if(motor->id == DEV_ID_GIMBAL_PITCH) {
        drv_can->drv_id = GM6020_GetDrvId(drv_can);
        drv_can->std_id = GM6020_GetStdId(drv_can);
    }
    else if(motor->id == DEV_ID_GIMBAL_YAW) {
        drv_can->drv_id = GM6020_GetDrvId(drv_can);
        drv_can->std_id = GM6020_GetStdId(drv_can);
    }
    else {
        motor->errno = DEV_ID_ERR;
    }
}

void leader_init(leader_t *leader)
{
    leader->info->offline_cnt = leader->info->offline_max_cnt+1;
    leader->work_state = DEV_OFFLINE;
}

void leader_update(leader_t *leader, uint8_t *rxBuf)
{
    leader_info_t *leader_info = leader->info;
    leader_info->temporary_data = rxBuf[8];
    leader_info->offline_cnt = 0;
}

/**
 *	@brief	CAN ���͵�������
 */
void CAN_SendSingleData(drv_can_t *drv, int16_t txData)
{
    int16_t txArr[4] = {0, 0, 0, 0};

    txArr[drv->drv_id] = txData;
    if(drv->type == DRV_CAN1)
        CAN1_SendData(drv->std_id, txArr);
    else if(drv->type == DRV_CAN2)
        CAN2_SendData(drv->std_id, txArr);
}

/**
 *	@brief	CAN �������ݴ�
 */
void CAN1_SendDataBuff( uint32_t std_id, int16_t *txBuff)
{
    CAN1_SendData(std_id, txBuff);
}
void CAN2_SendDataBuff( uint32_t std_id, int16_t *txBuff)
{
    CAN2_SendData(std_id, txBuff);
}

int16_t CAN1ID;
/**
 *	@brief	CAN1 ��������
 */
void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
    CAN1ID = canId;
    /* ���� */
    if(canId == DIAL_CAN_ID)
    {
        motor[DIAL].update(&motor[DIAL], rxBuf);
        motor[DIAL].check(&motor[DIAL]);
    }
    /* Pitch��6020 */
    else if(canId == GIMBAL_CAN_ID_PITCH)
    {
        motor[GIMBAL_PITCH].update(&motor[GIMBAL_PITCH], rxBuf);
        motor[GIMBAL_PITCH].check(&motor[GIMBAL_PITCH]);

    }
}

/**
 *	@brief	CAN2 ��������
 */
int16_t CAN2ID;
//uint32_t cnt;
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
    CAN2ID = canId;
//	cnt++;
    /* ����3508 */
    if(canId == CHASSIS_CAN_ID)
    {
        motor[CHASSIS].update(&motor[CHASSIS], rxBuf);
        motor[CHASSIS].check(&motor[CHASSIS]);
    }
    /* Yaw��6020 */
    else if(canId == GIMBAL_CAN_ID_YAW)
    {
        motor[GIMBAL_YAW].update(&motor[GIMBAL_YAW], rxBuf);
        motor[GIMBAL_YAW].check(&motor[GIMBAL_YAW]);
    }	/* ����̨ģʽ���� */
    else if(canId == LEADER_DATA_ID)
    {
        leader_sensor.info->temporary_data = rxBuf[7];
        memcpy(&leader_sensor.info->data, &leader_sensor.info->temporary_data, 1);
        leader_sensor.info->offline_cnt = 0;
    }
}

