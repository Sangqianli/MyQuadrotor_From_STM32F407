/**
 * @file        monitor_task.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-November-2020
 * @brief       Monitor&Test Center
 */

/* Includes ------------------------------------------------------------------*/
#include "gimbal_task.h"
#include "device.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
#define SCOUT_PITCH_SPEED  3    /*6*/
#define SCOUT_YAW_SPEED    3	/*侦察速度4,1:1*/
#define SCOUT_YAW_TRUN_TIMES 4U //前后侦察切换次数（往返）
#define SCOUT_FRONT   1U   //修改前后侦察时用到，作用于修改往返次数，跟上面切换次数有关
#define SCOUT_BACK    4U

#define SCOUT_YAW_RIGHT_ANGLE  -1600        //  +-1600
#define SCOUT_YAW_LEFT_ANGLE   1600 	/*侦察yaw轴角度左右边界*/

#define SCOUT_YAW_RIGHT_FRONT_ANGLE   -1200        //  +-1600
#define SCOUT_YAW_LEFT_FRONT_ANGLE    1200 	/*侦察yaw轴角度左右边界*/
#define SCOUT_YAW_RIGHT_BACK_ANGLE   -5800
#define SCOUT_YAW_LEFT_BACK_ANGLE    -2800


#define SCOUT_PITCH_UP_ANGLE     500   //500
#define SCOUT_PITCH_DOWN_ANGLE  -650	/*侦察pitch轴角度上下边界*/

#define ATTACK_RAMP              30    //切换到打击模式下的斜坡参数30

/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t  Yaw_RCratio = 1;
uint16_t  Pitch_RCratio = 1;
float  Gimbal_yaw_Q = 1,Gimbal_yaw_R = 2,Gimbal_pitch_Q = 1,Gimbal_pitch_R = 2,Pitch_PPM_Q=1,Pitch_PPM_R=10;
extKalman_t RC_yaw_p,RC_pitch_p,Pitch_PPM_p;
uint16_t attack_switch_ramp=0;//记录斜坡步数
/* Exported variables --------------------------------------------------------*/
Gimbal_t  Gimbal_process = {
    . Gimbal_queue = {
        .nowLength = 0,
        .queue = {0},
        .is_queue_full = false
    }
};
/* Private functions ---------------------------------------------------------*/
static void Gimbal_RCdata()
{
    static float yaw_target_last;
    static float yaw_target_now;
    static float pitch_target_last;
    static float pitch_target_now;//记录遥控值
    yaw_target_now = -rc_sensor.info->ch0 * Yaw_RCratio;
    yaw_target_now=KalmanFilter(&RC_yaw_p,yaw_target_now);
    pitch_target_now = rc_sensor.info->ch1  * Pitch_RCratio;
    pitch_target_now = KalmanFilter(&RC_pitch_p,pitch_target_now);

    if(yaw_target_now>0)
    {
        if((yaw_target_now-yaw_target_last)>0)
        {
            Gimbal_process.Yaw_taget = yaw_target_now+motor[GIMBAL_YAW].info->angle_sum;
        }
    }
    if(yaw_target_now<0)
    {
        if((yaw_target_now-yaw_target_last)<0)
        {
            Gimbal_process.Yaw_taget = yaw_target_now+motor[GIMBAL_YAW].info->angle_sum;
        }
    }

    if(pitch_target_now>0)
    {
        if((pitch_target_now-pitch_target_last)>0)
        {
            Gimbal_process.Pitch_taget = pitch_target_now-motor[GIMBAL_PITCH].info->angle_sum;
        }
    }
    if(pitch_target_now<0)
    {
        if((pitch_target_now-pitch_target_last)<0)
        {
            Gimbal_process.Pitch_taget = pitch_target_now-motor[GIMBAL_PITCH].info->angle_sum;
        }
    }

    yaw_target_last   = yaw_target_now;
    pitch_target_last = pitch_target_now;
}

static void Gimbal_Measure_Data()
{ 
    static float gg;
    static float roll_cos,roll_sin;
    gg= (imu_sensor.info->roll*3.1415f)/180.0f;
    /*世界Yaw轴速度合成*/
    roll_cos = arm_cos_f32(gg);
    roll_sin = arm_sin_f32(gg);
    Gimbal_process.RealYaw_speed=imu_sensor.info->rate_yaw*roll_cos+imu_sensor.info->rate_roll*roll_sin;
    /*..........................................*/
    Gimbal_process.YAW_PPM.measure = motor[GIMBAL_YAW].info->angle_sum;
    Gimbal_process.PITCH_PPM.measure = -motor[GIMBAL_PITCH].info->angle_sum;
	
/*...........陀螺仪速度*/	
    Gimbal_process.YAW_PVM.measure = Gimbal_process.RealYaw_speed;
    Gimbal_process.PITCH_PVM.measure = -imu_sensor.info->rate_pitch;
/*......................*/
	
/*...........电机速度*/
//    Gimbal_process.YAW_PVM.measure = motor[GIMBAL_YAW].info->speed;
//    Gimbal_process.PITCH_PVM.measure = -motor[GIMBAL_PITCH].info->speed;		
/*......................*/

	
	
//    Gimbal_process.PITCH_PVM.measure = KalmanFilter(&Pitch_PPM_p,Gimbal_process.PITCH_PVM.measure);
}

static void Gimbal_control()
{
    Gimbal_process.YAW_PPM.target = Gimbal_process.Yaw_taget;
    Gimbal_process.PITCH_PPM.target = Gimbal_process.Pitch_taget;
    pid_calculate(&Gimbal_process.YAW_PPM);
    pid_calculate(&Gimbal_process.PITCH_PPM);

    Gimbal_process.YAW_PVM.target = Gimbal_process.YAW_PPM.out;
    Gimbal_process.PITCH_PVM.target = Gimbal_process.PITCH_PPM.out;
    pid_calculate(&Gimbal_process.YAW_PVM);
    pid_calculate(&Gimbal_process.PITCH_PVM);

    NormalData_0x2FF[0] =  (int16_t)(-Gimbal_process.PITCH_PVM.out);
    NormalData_0x2FF[1] =  (int16_t)(Gimbal_process.YAW_PVM.out);
}

static void Gimbal2_control()
{
    Gimbal_process.YAW_PPM.target = Gimbal_process.Yaw_taget;
    Gimbal_process.PITCH_PPM.target = Gimbal_process.Pitch_taget;
    pid_calculate(&Gimbal_process.YAW_PPM)                                                                                                                                                                                              ;
    pid_calculate(&Gimbal_process.PITCH_PPM);

    Gimbal_process.YAW_PVM.target = Gimbal_process. YAW_PPM.out;
    Gimbal_process.PITCH_PVM.target = Gimbal_process.PITCH_PPM.out;

    pid_calculate(&Gimbal_process.PITCH_PVM);

    Gimbal_process.PITCH2_PVM.err = Gimbal_process.PITCH_PVM.err;
    Gimbal_process.PITCH_PVM.err = KalmanFilter(&Pitch_PPM_p,Gimbal_process.PITCH_PVM.err);

    pid2_calculate(&Gimbal_process.PITCH2_PVM);
    pid_calculate(&Gimbal_process.YAW_PVM);

    NormalData_0x2FF[0] =  (int16_t)(-Gimbal_process.PITCH2_PVM.out);
    NormalData_0x2FF[1] =  (int16_t)(Gimbal_process.YAW_PVM.out);
}

/**
* @brief 检测背后伤害
* @param void
* @return void
*检测到后作用一段时间，再恢复.
*/
static bool  Hurt_back()
{
    static int32_t safe_cnt;
    static bool result = false;
    if( (judge_sensor.info->hurt_data_update)&&( (judge_sensor.info->RobotHurt.armor_id == 1)&&( judge_sensor.info->RobotHurt.hurt_type == 0) ) )//伤害数据更新
    {
//            judge_sensor.info->hurt_data_update = false;
        safe_cnt = 0;
        result = true;
    }
    else
    {
        safe_cnt++;
        if (safe_cnt>3000)//6s
        {
            safe_cnt = 0;
            result = false;
        }
    }
    return result;
}

/**
* @brief 云台手向后扫描信息
* @param void
* @return void
*提供检测发送按键来判定是否向后扫描
*返回true表示向后，返回flase表示不干涉
*/
static bool Scan_Key_info()
{
    static	bool	result = false;
    if(judge_sensor.info->AerialData.cmd & back_scan)
        result	= true;
    else
        result	= false;
    return	result;
}

/**
* @brief 获取侦察模式的侦察方向
* @param void
* @return void
*作用在YAW轴修改边界上
*/
static void  Get_scout_direction()
{
//    if( (Chassis_process.Trip_times % SCOUT_YAW_TRUN_TIMES == 0)&&(Chassis_process.Trip_times!=0)  )
//    {
//        Gimbal_process.Scout_direction = -1;
//    }//向后巡航
//    else
//    {
//        Gimbal_process.Scout_direction = 1;
//    }//向前侦察

    if( Hurt_back()|| Scan_Key_info()  )//
    {
        Gimbal_process.Scout_direction = -1;
    }//向后巡航
    else
    {
        Gimbal_process.Scout_direction = 1;
    }//向前侦察
}
/**
* @brief 侦察模式下的侦察函数
* @param void
* @return void
*	该函数作用在期望值上.
*/
static  void Scout()
{
    static uint8_t yaw_dir = 1,pitch_dir = 1;	/*yaw轴和pitch轴的两个方向变量*/
    if(yaw_dir == 0)//向右侦察
    {
        Gimbal_process.Yaw_taget -= SCOUT_YAW_SPEED;
        if(Gimbal_process.YAW_PPM.target <= (SCOUT_YAW_RIGHT_ANGLE) )
        {
            Gimbal_process.Yaw_taget = SCOUT_YAW_RIGHT_ANGLE;
            yaw_dir=1;
        }
    } else if(yaw_dir == 1) //向左侦察
    {
        Gimbal_process.Yaw_taget += SCOUT_YAW_SPEED;
        if(Gimbal_process.YAW_PPM.target >= (SCOUT_YAW_LEFT_ANGLE) )
        {
            Gimbal_process.Yaw_taget = SCOUT_YAW_LEFT_ANGLE;
            yaw_dir=0;
        }
    }
    /*	判断边界  */


    /*		↑以上是yaw轴的侦察处理↑			*/

    if(pitch_dir == 0)//向下侦察
    {
        Gimbal_process.Pitch_taget -= SCOUT_PITCH_SPEED;
        if(Gimbal_process.PITCH_PPM.target <= (SCOUT_PITCH_DOWN_ANGLE) )
        {
            Gimbal_process.Pitch_taget = SCOUT_PITCH_DOWN_ANGLE;
//            if( abs(Gimbal_process.PITCH_PPM.err <= 30) )
//            {
            pitch_dir=1;
//            }
        }

    } else if(pitch_dir == 1) //向上侦察
    {
        Gimbal_process.Pitch_taget += SCOUT_PITCH_SPEED;
        if(Gimbal_process.PITCH_PPM.target >= (SCOUT_PITCH_UP_ANGLE) )
        {
            Gimbal_process.Pitch_taget = SCOUT_PITCH_UP_ANGLE;
//            if( abs(Gimbal_process.PITCH_PPM.err <= 20) )
//            {
            pitch_dir=0;
//            }
        }
    }
}

static  void Scout_2_1()
{
    static uint8_t yaw_dir = 1,pitch_dir = 1;	/*yaw轴和pitch轴的两个方向变量*/
    if(yaw_dir == 0)//向右侦察
    {
        Gimbal_process.Yaw_taget -= SCOUT_YAW_SPEED;
    } else if(yaw_dir == 1) //向左侦察
    {
        Gimbal_process.Yaw_taget += SCOUT_YAW_SPEED;
    }
    /*	判断边界  */
    if(Chassis_process.Derection_flag == 1)  //向右运动时
    {
        if(Gimbal_process.Yaw_taget <= SCOUT_YAW_RIGHT_BACK_ANGLE)
        {
            Gimbal_process.Yaw_taget = SCOUT_YAW_RIGHT_BACK_ANGLE;
            yaw_dir=1;
        } else if(Gimbal_process.Yaw_taget >= SCOUT_YAW_LEFT_BACK_ANGLE)
        {
            Gimbal_process.Yaw_taget = SCOUT_YAW_LEFT_BACK_ANGLE;
            yaw_dir=0;
        }
    }
    if(Chassis_process.Derection_flag == -1)  //向左运动时
    {
        if(Gimbal_process.Yaw_taget <= SCOUT_YAW_RIGHT_FRONT_ANGLE)
        {
            Gimbal_process.Yaw_taget = SCOUT_YAW_RIGHT_FRONT_ANGLE;
            yaw_dir=1;
        } else if(Gimbal_process.Yaw_taget >= SCOUT_YAW_LEFT_FRONT_ANGLE)
        {
            Gimbal_process.Yaw_taget = SCOUT_YAW_LEFT_FRONT_ANGLE;
            yaw_dir=0;
        }
    }

    /*		↑以上是yaw轴的侦察处理↑			*/

    if(pitch_dir == 0)//向下侦察
    {
        Gimbal_process.Pitch_taget -= SCOUT_PITCH_SPEED;
    } else if(pitch_dir == 1) //向上侦察
    {
        Gimbal_process.Pitch_taget += SCOUT_PITCH_SPEED;
    }
    /*	判断边界  */
    if(Gimbal_process.Pitch_taget >= SCOUT_PITCH_UP_ANGLE)
    {
        Gimbal_process.Pitch_taget = SCOUT_PITCH_UP_ANGLE;
        pitch_dir=0;
    } else if(Gimbal_process.Pitch_taget <= SCOUT_PITCH_DOWN_ANGLE)
    {
        Gimbal_process.Pitch_taget = SCOUT_PITCH_DOWN_ANGLE;
        pitch_dir=1;
    }
}


static  void Scout_2_2()
{
    static uint8_t yaw_dir = 1,pitch_dir = 1;	/*yaw轴和pitch轴的两个方向变量*/

    Get_scout_direction();//确定往前还是往后侦察

    if(yaw_dir == 0)//向右侦察
    {
        Gimbal_process.Yaw_taget -= SCOUT_YAW_SPEED;
        if( Gimbal_process.Scout_direction == -1 )
        {
            if(Gimbal_process.YAW_PPM.target <= (SCOUT_YAW_RIGHT_BACK_ANGLE) )
            {
                Gimbal_process.Yaw_taget = SCOUT_YAW_RIGHT_BACK_ANGLE;
                yaw_dir = 1;
            }
            if(Gimbal_process.YAW_PPM.target >= SCOUT_YAW_RIGHT_FRONT_ANGLE )
            {
                Gimbal_process.Yaw_taget = SCOUT_YAW_LEFT_BACK_ANGLE; //直接转
            }
        } else
        {
            if(Gimbal_process.YAW_PPM.target <= SCOUT_YAW_RIGHT_FRONT_ANGLE)
            {
                Gimbal_process.Yaw_taget = SCOUT_YAW_RIGHT_FRONT_ANGLE;
                yaw_dir = 1;
            }
        }
    } else if(yaw_dir == 1) //向左侦察
    {
        Gimbal_process.Yaw_taget += SCOUT_YAW_SPEED;
        if( Gimbal_process.Scout_direction == -1 )
        {
            if(Gimbal_process.YAW_PPM.target >= (SCOUT_YAW_LEFT_BACK_ANGLE) )
            {
                Gimbal_process.Yaw_taget = SCOUT_YAW_LEFT_BACK_ANGLE;
                yaw_dir = 0;
            }
        } else
        {
            if(Gimbal_process.YAW_PPM.target >= SCOUT_YAW_LEFT_FRONT_ANGLE)
            {
                Gimbal_process.Yaw_taget = SCOUT_YAW_LEFT_FRONT_ANGLE;
                yaw_dir = 0;
            }

            if(Gimbal_process.YAW_PPM.target <= SCOUT_YAW_LEFT_BACK_ANGLE)
            {
                Gimbal_process.Yaw_taget = SCOUT_YAW_RIGHT_FRONT_ANGLE;  //直接转
            }
        }
    }

//    /*	判断边界  */
//    if( Gimbal_process.Scout_direction == -1 )
//    {
//        if(Gimbal_process.YAW_PPM.target <= SCOUT_YAW_RIGHT_BACK_ANGLE)
//        {
//            Gimbal_process.Yaw_taget = SCOUT_YAW_RIGHT_BACK_ANGLE;
//            yaw_dir=1;
//        } else if(Gimbal_process.YAW_PPM.target >= SCOUT_YAW_LEFT_BACK_ANGLE)
//        {
//            Gimbal_process.Yaw_taget = SCOUT_YAW_LEFT_BACK_ANGLE;
//            yaw_dir=0;
//        }
//    }//向后侦察
//    else
//    {
//        if(Gimbal_process.YAW_PPM.target <= SCOUT_YAW_RIGHT_FRONT_ANGLE)
//        {
//            Gimbal_process.Yaw_taget = SCOUT_YAW_RIGHT_FRONT_ANGLE;
//            yaw_dir=1;
//        } else if(Gimbal_process.YAW_PPM.target >= SCOUT_YAW_LEFT_FRONT_ANGLE)
//        {
//            Gimbal_process.Yaw_taget = SCOUT_YAW_LEFT_FRONT_ANGLE;
//            yaw_dir=0;
//        }

//    }//向前侦察

    /*		↑以上是yaw轴的侦察处理↑			*/
    if(pitch_dir == 0)//向下侦察
    { 
        Gimbal_process.Pitch_taget -= SCOUT_PITCH_SPEED;
        if(Gimbal_process.PITCH_PPM.target <= (SCOUT_PITCH_DOWN_ANGLE) )
        {
            Gimbal_process.Pitch_taget = SCOUT_PITCH_DOWN_ANGLE;
            pitch_dir=1;
        }
    } else if(pitch_dir == 1) //向上侦察
    {
        Gimbal_process.Pitch_taget += SCOUT_PITCH_SPEED;
        if(Gimbal_process.PITCH_PPM.target >= (SCOUT_PITCH_UP_ANGLE) )
        {
            Gimbal_process.Pitch_taget = SCOUT_PITCH_UP_ANGLE;
            pitch_dir=0;
        }
    }
}



static void Gimbal_line()
{
//    Gimbal_process.Yaw_taget=constrain(Gimbal_process.Yaw_taget,-1650.f,1650.f);	 //联盟赛版本YAW轴软件限位
    Gimbal_process.Pitch_taget=constrain(Gimbal_process.Pitch_taget,PITCH_DOWN_LINE,PITCH_UP_LINE);
}

static void AUTOMode_switch()
{
    if(sys.auto_mode == AUTO_MODE_SCOUT)
    {
        Gimbal_process.Yaw_taget = Gimbal_process.YAW_PPM.measure;
        Gimbal_process.Pitch_taget = Gimbal_process.PITCH_PPM.measure;
        sys.fire_state.FIRE_OPEN = false;
        attack_switch_ramp = ATTACK_RAMP;
    }
    if(sys.auto_mode == AUTO_MODE_ATTACK)
    {
        Gimbal_process.Yaw_taget = Gimbal_process.YAW_PPM.measure;
        Gimbal_process.Pitch_taget = Gimbal_process.PITCH_PPM.measure;
        sys.fire_state.FIRE_OPEN = false;
        attack_switch_ramp = ATTACK_RAMP;
    }
    sys.switch_state.AUTO_MODE_SWITCH = false;
}

static void Gimbal_reset()
{
    static float yaw_ramp,pitch_ramp;
    static float delta_yaw,delta_pitch;
    if(sys.switch_state.RESET_CAL)
    {
        delta_yaw = 480.f - motor[GIMBAL_YAW].info->angle;
        delta_pitch = 2695.f - motor[GIMBAL_PITCH].info->angle;
        if(abs(delta_yaw)<4096)
        {
            delta_yaw = delta_yaw;
        } else if(delta_yaw>=4096)
        {
            delta_yaw -=8192;
        } else if(delta_yaw<=-4096)
        {
            delta_yaw +=8192;
        }
        yaw_ramp = delta_yaw/500.f;
        pitch_ramp = delta_pitch/500.f;
        sys.switch_state.RESET_CAL = false;
    }
    Gimbal_process.Yaw_taget =  RampFloat( abs(yaw_ramp),delta_yaw,Gimbal_process.Yaw_taget);
    Gimbal_process.Pitch_taget =  RampFloat( abs(pitch_ramp), - delta_pitch,Gimbal_process.Pitch_taget);
    if( (abs(motor[GIMBAL_YAW].info->angle-480)<=10)&&(abs(motor[GIMBAL_PITCH].info->angle-2695)<=10) )
    {
        if(sys.switch_state.SYS_RESET)
        {
            sys.switch_state.SYS_RESET = false;
        }
        else if(sys.switch_state.REMOTE_SWITCH)
        {
            sys.switch_state.REMOTE_SWITCH = false;
            sys.auto_mode = AUTO_MODE_SCOUT;
        }
        motor[GIMBAL_YAW].info->angle_sum = 0;
        motor[GIMBAL_PITCH].info->angle_sum = 0;
        Gimbal_process.Yaw_taget = motor[GIMBAL_YAW].info->angle_sum;
        Gimbal_process.Pitch_taget = - motor[GIMBAL_PITCH].info->angle_sum;
    }
    sys.fire_state.FIRE_OPEN = false;
    NormalData_0x200[1] = 0;
}

/**
* @brief 云台闪躲
* @param void
* @return void
*	用于云台闪避柱子，防止弹簧压缩的时候撞到
*/
static void Gimbal_Evasion()
{


}
static void Attack()
{
    if(sys.predict_state.PREDICT_ACTION)
    {


    } else if(sys.predict_state.PREDICT_OPEN)
    {
        Gimbal_process.Yaw_taget = Vision_process.data_kal.YawTarget_KF;
        Gimbal_process.Pitch_taget = Vision_process.data_kal.PitchTarget_KF;
    } else if(attack_switch_ramp != 0)
    {
        Gimbal_process.Yaw_taget =  RampFloat( abs(Vision_process.data_kal.YawTarget_KF-Gimbal_process.Yaw_taget) / attack_switch_ramp, Vision_process.data_kal.YawTarget_KF, Gimbal_process.Yaw_taget);
        Gimbal_process.Pitch_taget =  RampFloat( abs(Vision_process.data_kal.PitchTarget_KF-Gimbal_process.Pitch_taget)/attack_switch_ramp, Vision_process.data_kal.PitchTarget_KF, Gimbal_process.Pitch_taget);
        attack_switch_ramp--;
        sys.predict_state.PREDICT_OPEN = false;
        if(attack_switch_ramp == 0 )
            sys.predict_state.PREDICT_OPEN = true;
    }//只是跟随
}

static void Attack_2_1()
{
    if(sys.predict_state.PREDICT_ACTION)
    {


    } else if(sys.predict_state.PREDICT_OPEN)
    {
        Gimbal_process.Yaw_taget = Vision_process.data_kal.YawTarget_KF;
        Gimbal_process.Pitch_taget = Vision_process.data_kal.PitchTarget_KF;
    }
    else {
        Gimbal_process.Yaw_taget =  Vision_process.data_kal.YawTarget_KF;
        Gimbal_process.Pitch_taget =  Vision_process.data_kal.PitchTarget_KF;
        sys.predict_state.PREDICT_OPEN = false;
        attack_switch_ramp--;
//        if( (abs(Vision_process.data_kal.PitchGet_KF)<=5)&&(abs(Vision_process.data_kal.YawGet_KF)<=20) )
//            sys.predict_state.PREDICT_OPEN = true;
        if(attack_switch_ramp == 0 )
            sys.predict_state.PREDICT_OPEN = true;
    }//只是跟随
}

/**
* @brief 自动模式自瞄
* @param void
* @return void
*	自动模式下的云台运动
*/
static void Auto_Aimed()
{
    if(sys.switch_state.AUTO_MODE_SWITCH)
    {
        AUTOMode_switch();
    }
    else if(sys.auto_mode == AUTO_MODE_SCOUT)
    {
//        Scout();
//		Scout_2_1();
        Scout_2_2();
    } else if(sys.auto_mode == AUTO_MODE_ATTACK)
    {
//      Attack();
        Attack_2_1();
    }
}
/**
* @brief 自检自瞄
* @param void
* @return void
*	自检模式下的云台运动
*/
static void Inpect_Aimed()
{
    if(sys.switch_state.AUTO_MODE_SWITCH)
    {
        AUTOMode_switch();
    }
    else if(sys.auto_mode == AUTO_MODE_SCOUT)
    {
//                        Scout();
    } else if(sys.auto_mode == AUTO_MODE_ATTACK)
    {
        Attack_2_1();
    }
}

/* Exported functions --------------------------------------------------------*/
float Get_Queue_Angle(uint8_t time_dis)
{
    if(time_dis<=Gimbal_process.Gimbal_queue.nowLength)
        return Gimbal_process.Gimbal_queue.queue[Gimbal_process.Gimbal_queue.nowLength-time_dis];
    else
        return Gimbal_process.Gimbal_queue.queue[Gimbal_process.Gimbal_queue.nowLength+200-time_dis];
}

void Update_Gimbal_Angle_Queue(int16_t now_angle)
{
    if( Gimbal_process.Gimbal_queue.nowLength<199)
        Gimbal_process.Gimbal_queue.is_queue_full=false;
    else
        Gimbal_process.Gimbal_queue.is_queue_full=true;

    if(Gimbal_process.Gimbal_queue.is_queue_full==false)
    {
        Gimbal_process.Gimbal_queue.queue[Gimbal_process.Gimbal_queue.nowLength]=now_angle;
        Gimbal_process.Gimbal_queue.nowLength++;
    }
    if(Gimbal_process.Gimbal_queue.is_queue_full==true)
    {
        Gimbal_process.Gimbal_queue.nowLength=0;
    }
}

void Gimbal_Init()
{
    KalmanCreate(&RC_yaw_p,Gimbal_yaw_Q,Gimbal_yaw_R);
    KalmanCreate(&RC_pitch_p,Gimbal_pitch_Q,Gimbal_pitch_R);
    KalmanCreate(&Pitch_PPM_p,Pitch_PPM_Q,Pitch_PPM_R);

    Gimbal_process.YAW_PPM.target=0;
    Gimbal_process.YAW_PPM.kp=10;//10
    Gimbal_process.YAW_PPM.ki=0;
    Gimbal_process.YAW_PPM.kd=0;
    Gimbal_process.YAW_PPM.integral_max=8000;
    Gimbal_process.YAW_PPM.out_max=16000;
    Gimbal_process.YAW_PPM.out=0;//yaw位置环

    Gimbal_process.YAW_PVM.target=0;
    Gimbal_process.YAW_PVM.kp=16;//25
    Gimbal_process.YAW_PVM.ki=0.2;//0.4
    Gimbal_process.YAW_PVM.kd=0;
    Gimbal_process.YAW_PVM.integral_max=20000;
    Gimbal_process.YAW_PVM.out_max=28000;
    Gimbal_process.YAW_PVM.out=0;//yaw速度环


    Gimbal_process.PITCH_PPM.target=0;
    Gimbal_process.PITCH_PPM.kp=10;//10
    Gimbal_process.PITCH_PPM.ki=0;
    Gimbal_process.PITCH_PPM.kd=0;
    Gimbal_process.PITCH_PPM.integral_max=8000;
    Gimbal_process.PITCH_PPM.out_max=16000;
    Gimbal_process.PITCH_PPM.out=0;//pitch位置环

    Gimbal_process.PITCH_PVM.target=0;
    Gimbal_process.PITCH_PVM.kp=12;//12  
    Gimbal_process.PITCH_PVM.ki=0.1;//0.3
    Gimbal_process.PITCH_PVM.kd=0;
    Gimbal_process.PITCH_PVM.integral_max=26000;
    Gimbal_process.PITCH_PVM.out_max=28000;
    Gimbal_process.PITCH_PVM.out=0;//pitch速度环

    Gimbal_process.PITCH2_PVM.kp=12;//12  
    Gimbal_process.PITCH2_PVM.ki=0.6;//0.6
    Gimbal_process.PITCH2_PVM.kd=0;
    Gimbal_process.PITCH2_PVM.integral_max=26000;
    Gimbal_process.PITCH2_PVM.out_max=28000;
    Gimbal_process.PITCH2_PVM.out=0;//pitch速度环
}

void StartGimbalTask(void const * argument)
{
    for(;;)
    {
        if(sys.state == SYS_STATE_NORMAL)
        {
            Gimbal_Measure_Data();
            if(sys.switch_state.ALL_READY)
            {
                if(sys.remote_mode == RC)
                {
                    Gimbal_RCdata();
                }
                else if( sys.remote_mode == AUTO )
                {
                    Auto_Aimed();
                } else if(sys.remote_mode == INSPECTION)
                {
                    Inpect_Aimed();
                }
                Gimbal_line();
            }
            else
            {
                Gimbal_reset();
            }
            Gimbal_control();
//            Gimbal2_control();
        }
        osDelay(2);
    }
}


