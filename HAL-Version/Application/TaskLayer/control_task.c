/**
 * @file        control_task.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-November-2020
 * @brief       Control Center
 */

/* Includes ------------------------------------------------------------------*/
#include "control_task.h"
#include "driver.h"
#include "device.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void Attitude_Fly(void);
void Fly_clear(void);
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int16_t StopPwm[2] = {0,0};
int16_t DowmPwm[4] = {0,0,0,0};

static bool test_fly = false;

static int16_t FLY_HIGH = 400;
/* Exported variables --------------------------------------------------------*/
int16_t NormalPwm[2] = {0,0};
int16_t FlyPwm[4] = {0,0,0,0};

Fly_t Fly_process;
/* Private functions ---------------------------------------------------------*/
 void Fly_clear()
{
	Fly_process.gun_target = 0;
	Fly_process.pitch_target = 0;
	Fly_process.roll_target =0;	
	Fly_process.yaw_target = 0;

	
    Fly_process.PitchAngle_Pid.target=0;	
	Fly_process.PitchAngle_Pid.iout = 0;	
    Fly_process.PitchAngle_Pid.out=0;//

    Fly_process.PitchRate_Pid.target=0;	
	Fly_process.PitchRate_Pid.iout = 0;
    Fly_process.PitchRate_Pid.out=0;//

    Fly_process.RollAngle_Pid.target=0;	
	Fly_process.RollAngle_Pid.iout = 0;	
    Fly_process.RollAngle_Pid.out=0;//

    Fly_process.RollRate_Pid.target=0;	
	Fly_process.RollRate_Pid.iout = 0;
    Fly_process.RollRate_Pid.out=0;//
	
    Fly_process.YawAngle_Pid.target=0;	
	Fly_process.YawAngle_Pid.iout = 0;	
    Fly_process.YawAngle_Pid.out=0;//

    Fly_process.YawRate_Pid.target=0;	
	Fly_process.YawRate_Pid.iout = 0;
    Fly_process.YawRate_Pid.out=0;//		
	
	FlyPwm[FCCW] = 0; 
	FlyPwm[LCW] = 0;
	FlyPwm[LCCW] = 0;
	FlyPwm[FCW] = 0;	
}
static void Quadrotor_Check(rc_sensor_t *rc_sen,imu_sensor_t *imu_sen)
{
	rc_sensor_info_t *rc_info = rc_sen->info;
    imu_sensor_info_t *imu_info = imu_sen->info;
	
	static bool check_flag;
	if(rc_info->s1_siwtch_up == true)
	{
		check_flag = true;
		rc_info->s1_siwtch_up = false;
	}
	if(check_flag == true)
	{
		imu_info->angle_pitch_offset = imu_info->pitch;
		imu_info->angle_yaw_offset = imu_info->yaw;
		imu_info->angle_roll_offset = imu_info->roll;
		check_flag = false;
	}
}

static void Attitude_Fly()
{
	Fly_process.pitch_target =(float) (rc_sensor.info->ch1/33.f);
	Fly_process.pitch_target = constrain(Fly_process.pitch_target,-20,20);
//	
//	Fly_process.roll_target +=(float) (-rc_sensor.info->ch0/6000.f);
//	Fly_process.roll_target = constrain(Fly_process.roll_target,-180,180);
//	
//	Fly_process.yaw_target += (float) (-rc_sensor.info->ch2/6000.f);
/***************************************************************************/	
	Fly_process.gun_target += (float) (rc_sensor.info->ch3/600.f);
	Fly_process.gun_target = constrain(Fly_process.gun_target,0,1000);
	

//	FlyPwm[FCCW] = Fly_process.gun_target;
//	FlyPwm[LCW] = Fly_process.gun_target;
//	FlyPwm[LCCW] = Fly_process.gun_target;
//	FlyPwm[FCW] = Fly_process.gun_target;
/***************************************************************************/	
	Fly_process.PitchAngle_Pid.target = Fly_process.pitch_target;
//	Fly_process.YawAngle_Pid.target = Fly_process.yaw_target;
//	Fly_process.RollAngle_Pid.target = Fly_process.roll_target;
	
	Fly_process.PitchAngle_Pid.measure = imu_sensor.info->pitch;
//	Fly_process.YawAngle_Pid.measure = imu_sensor.info->yaw;
//	Fly_process.RollAngle_Pid.measure = imu_sensor.info->roll;
//	
	pid_calculate(&Fly_process.PitchAngle_Pid);
//	pid_calculate(&Fly_process.YawAngle_Pid);
//	pid_calculate(&Fly_process.RollAngle_Pid);	
//	
	Fly_process.PitchRate_Pid.target = Fly_process.PitchAngle_Pid.out;
//	Fly_process.YawRate_Pid.target = Fly_process.YawAngle_Pid.out;
//	Fly_process.RollRate_Pid.target = Fly_process.RollAngle_Pid.out;	
//	

	Fly_process.PitchRate_Pid.measure = imu_sensor.info->rate_pitch;
//	Fly_process.YawRate_Pid.measure = imu_sensor.info->rate_yaw;
//	Fly_process.RollRate_Pid.measure = imu_sensor.info->rate_roll;


	pid_calculate(&Fly_process.PitchRate_Pid);
//	pid_calculate(&Fly_process.YawRate_Pid);
//	pid_calculate(&Fly_process.RollRate_Pid);	
//	
	FlyPwm[FCCW] = Fly_process.gun_target - Fly_process.PitchRate_Pid.out - Fly_process.RollRate_Pid.out + Fly_process.YawRate_Pid.out; 
	FlyPwm[LCW] = Fly_process.gun_target + Fly_process.PitchRate_Pid.out - Fly_process.RollRate_Pid.out - Fly_process.YawRate_Pid.out;
	FlyPwm[LCCW] = Fly_process.gun_target  + Fly_process.PitchRate_Pid.out + Fly_process.RollRate_Pid.out + Fly_process.YawRate_Pid.out;
	FlyPwm[FCW] = Fly_process.gun_target - Fly_process.PitchRate_Pid.out + Fly_process.RollRate_Pid.out - Fly_process.YawRate_Pid.out;
}


static void Control_Stop()
{

    FRICTION_PwmOut(0, 0);
    FLY_PwmOut(0, 0, 0, 0);
    LED_RED_ON();
    LED_GREEN_OFF();
    LASER_OFF();

}

static void Control_Normal()
{
	if(sys.remote_mode == ATTITUDE)
		Attitude_Fly();
	
	
    FRICTION_PwmOut(NormalPwm[0], NormalPwm[1]);
	FLY_PwmOut(FlyPwm[0], FlyPwm[1], FlyPwm[2], FlyPwm[3]);
    LED_RED_OFF();
    LED_GREEN_ON();
    LASER_ON();
}
/* Exported functions --------------------------------------------------------*/

void Control_Init()
{
	Fly_process.gun_target = 0;
/******PitchPID参数*******************************************/	
    Fly_process.PitchAngle_Pid.target=0;	
	Fly_process.PitchAngle_Pid.kp = 1;
	Fly_process.PitchAngle_Pid.ki = 0;
	Fly_process.PitchAngle_Pid.kd = 0;	
    Fly_process.PitchAngle_Pid.integral_max=600;
    Fly_process.PitchAngle_Pid.out_max=1200;
    Fly_process.PitchAngle_Pid.out=0;//

    Fly_process.PitchRate_Pid.target=0;	
	Fly_process.PitchRate_Pid.kp = 1;
	Fly_process.PitchRate_Pid.ki = 0;
	Fly_process.PitchRate_Pid.kd = 0;	
    Fly_process.PitchRate_Pid.integral_max=600;
    Fly_process.PitchRate_Pid.out_max=1200;
    Fly_process.PitchRate_Pid.out=0;//

/******RollPID参数*******************************************/		
    Fly_process.RollAngle_Pid.target=0;	
	Fly_process.RollAngle_Pid.kp = 0;
	Fly_process.RollAngle_Pid.ki = 0;
	Fly_process.RollAngle_Pid.kd = 0;	
    Fly_process.RollAngle_Pid.integral_max=600;
    Fly_process.RollAngle_Pid.out_max=1200;
    Fly_process.RollAngle_Pid.out=0;//

    Fly_process.RollRate_Pid.target=0;	
	Fly_process.RollRate_Pid.kp = 0;
	Fly_process.RollRate_Pid.ki = 0;
	Fly_process.RollRate_Pid.kd = 0;	
    Fly_process.RollRate_Pid.integral_max=600;
    Fly_process.RollRate_Pid.out_max=1200;
    Fly_process.RollRate_Pid.out=0;//	
	
/******YawPID参数*******************************************/		
    Fly_process.YawAngle_Pid.target=0;	
	Fly_process.YawAngle_Pid.kp = 0;
	Fly_process.YawAngle_Pid.ki = 0;
	Fly_process.YawAngle_Pid.kd = 0;	
    Fly_process.YawAngle_Pid.integral_max=600;
    Fly_process.YawAngle_Pid.out_max=1200;
    Fly_process.YawAngle_Pid.out=0;//

    Fly_process.YawRate_Pid.target=0;	
	Fly_process.YawRate_Pid.kp = 0;
	Fly_process.YawRate_Pid.ki = 0;
	Fly_process.YawRate_Pid.kd = 0;	
    Fly_process.YawRate_Pid.integral_max=600;
    Fly_process.YawRate_Pid.out_max=1200;
    Fly_process.YawRate_Pid.out=0;//	
}

/**
 *	@brief	控制任务
 */
void StartControlTask(void const * argument)
{
    for(;;)
    {
        if(sys.state == SYS_STATE_NORMAL) {
			Quadrotor_Check(&rc_sensor,&imu_sensor);
            Control_Normal();
        } else {
            Control_Stop();
        }
        osDelay(2);
    }
}
