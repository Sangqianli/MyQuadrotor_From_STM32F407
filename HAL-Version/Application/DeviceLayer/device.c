/**
 * @file        device.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        15-September-2020
 * @brief       Devices' Manager.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "device.h"

#include "drv_haltick.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
dev_list_t dev_list = {
	.rc_sen = &rc_sensor,
	.imu_sen = &imu_sensor,
};

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DEV_Init(void)
{
	dev_list.rc_sen->init(dev_list.rc_sen);
//	dev_list.imu_sen->init(dev_list.imu_sen);
}
