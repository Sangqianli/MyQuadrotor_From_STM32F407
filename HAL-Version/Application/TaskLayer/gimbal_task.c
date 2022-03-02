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
#define SCOUT_YAW_SPEED    3	/*����ٶ�4,1:1*/
#define SCOUT_YAW_TRUN_TIMES 4U //ǰ������л�������������
#define SCOUT_FRONT   1U   //�޸�ǰ�����ʱ�õ����������޸������������������л������й�
#define SCOUT_BACK    4U

#define SCOUT_YAW_RIGHT_ANGLE  -1600        //  +-1600
#define SCOUT_YAW_LEFT_ANGLE   1600 	/*���yaw��Ƕ����ұ߽�*/

#define SCOUT_YAW_RIGHT_FRONT_ANGLE   -1200        //  +-1600
#define SCOUT_YAW_LEFT_FRONT_ANGLE    1200 	/*���yaw��Ƕ����ұ߽�*/
#define SCOUT_YAW_RIGHT_BACK_ANGLE   -5800
#define SCOUT_YAW_LEFT_BACK_ANGLE    -2800


#define SCOUT_PITCH_UP_ANGLE     500   //500
#define SCOUT_PITCH_DOWN_ANGLE  -650	/*���pitch��Ƕ����±߽�*/

#define ATTACK_RAMP              30    //�л������ģʽ�µ�б�²���30

/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t  Yaw_RCratio = 1;
uint16_t  Pitch_RCratio = 1;
float  Gimbal_yaw_Q = 1,Gimbal_yaw_R = 2,Gimbal_pitch_Q = 1,Gimbal_pitch_R = 2,Pitch_PPM_Q=1,Pitch_PPM_R=10;
extKalman_t RC_yaw_p,RC_pitch_p,Pitch_PPM_p;
uint16_t attack_switch_ramp=0;//��¼б�²���
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
    static float pitch_target_now;//��¼ң��ֵ
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
    /*����Yaw���ٶȺϳ�*/
    roll_cos = arm_cos_f32(gg);
    roll_sin = arm_sin_f32(gg);
    Gimbal_process.RealYaw_speed=imu_sensor.info->rate_yaw*roll_cos+imu_sensor.info->rate_roll*roll_sin;
    /*..........................................*/
    Gimbal_process.YAW_PPM.measure = motor[GIMBAL_YAW].info->angle_sum;
    Gimbal_process.PITCH_PPM.measure = -motor[GIMBAL_PITCH].info->angle_sum;
	
/*...........�������ٶ�*/	
    Gimbal_process.YAW_PVM.measure = Gimbal_process.RealYaw_speed;
    Gimbal_process.PITCH_PVM.measure = -imu_sensor.info->rate_pitch;
/*......................*/
	
/*...........����ٶ�*/
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
* @brief ��ⱳ���˺�
* @param void
* @return void
*��⵽������һ��ʱ�䣬�ٻָ�.
*/
static bool  Hurt_back()
{
    static int32_t safe_cnt;
    static bool result = false;
    if( (judge_sensor.info->hurt_data_update)&&( (judge_sensor.info->RobotHurt.armor_id == 1)&&( judge_sensor.info->RobotHurt.hurt_type == 0) ) )//�˺����ݸ���
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
* @brief ��̨�����ɨ����Ϣ
* @param void
* @return void
*�ṩ��ⷢ�Ͱ������ж��Ƿ����ɨ��
*����true��ʾ��󣬷���flase��ʾ������
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
* @brief ��ȡ���ģʽ����췽��
* @param void
* @return void
*������YAW���޸ı߽���
*/
static void  Get_scout_direction()
{
//    if( (Chassis_process.Trip_times % SCOUT_YAW_TRUN_TIMES == 0)&&(Chassis_process.Trip_times!=0)  )
//    {
//        Gimbal_process.Scout_direction = -1;
//    }//���Ѳ��
//    else
//    {
//        Gimbal_process.Scout_direction = 1;
//    }//��ǰ���

    if( Hurt_back()|| Scan_Key_info()  )//
    {
        Gimbal_process.Scout_direction = -1;
    }//���Ѳ��
    else
    {
        Gimbal_process.Scout_direction = 1;
    }//��ǰ���
}
/**
* @brief ���ģʽ�µ���캯��
* @param void
* @return void
*	�ú�������������ֵ��.
*/
static  void Scout()
{
    static uint8_t yaw_dir = 1,pitch_dir = 1;	/*yaw���pitch��������������*/
    if(yaw_dir == 0)//�������
    {
        Gimbal_process.Yaw_taget -= SCOUT_YAW_SPEED;
        if(Gimbal_process.YAW_PPM.target <= (SCOUT_YAW_RIGHT_ANGLE) )
        {
            Gimbal_process.Yaw_taget = SCOUT_YAW_RIGHT_ANGLE;
            yaw_dir=1;
        }
    } else if(yaw_dir == 1) //�������
    {
        Gimbal_process.Yaw_taget += SCOUT_YAW_SPEED;
        if(Gimbal_process.YAW_PPM.target >= (SCOUT_YAW_LEFT_ANGLE) )
        {
            Gimbal_process.Yaw_taget = SCOUT_YAW_LEFT_ANGLE;
            yaw_dir=0;
        }
    }
    /*	�жϱ߽�  */


    /*		��������yaw�����촦���			*/

    if(pitch_dir == 0)//�������
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

    } else if(pitch_dir == 1) //�������
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
    static uint8_t yaw_dir = 1,pitch_dir = 1;	/*yaw���pitch��������������*/
    if(yaw_dir == 0)//�������
    {
        Gimbal_process.Yaw_taget -= SCOUT_YAW_SPEED;
    } else if(yaw_dir == 1) //�������
    {
        Gimbal_process.Yaw_taget += SCOUT_YAW_SPEED;
    }
    /*	�жϱ߽�  */
    if(Chassis_process.Derection_flag == 1)  //�����˶�ʱ
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
    if(Chassis_process.Derection_flag == -1)  //�����˶�ʱ
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

    /*		��������yaw�����촦���			*/

    if(pitch_dir == 0)//�������
    {
        Gimbal_process.Pitch_taget -= SCOUT_PITCH_SPEED;
    } else if(pitch_dir == 1) //�������
    {
        Gimbal_process.Pitch_taget += SCOUT_PITCH_SPEED;
    }
    /*	�жϱ߽�  */
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
    static uint8_t yaw_dir = 1,pitch_dir = 1;	/*yaw���pitch��������������*/

    Get_scout_direction();//ȷ����ǰ�����������

    if(yaw_dir == 0)//�������
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
                Gimbal_process.Yaw_taget = SCOUT_YAW_LEFT_BACK_ANGLE; //ֱ��ת
            }
        } else
        {
            if(Gimbal_process.YAW_PPM.target <= SCOUT_YAW_RIGHT_FRONT_ANGLE)
            {
                Gimbal_process.Yaw_taget = SCOUT_YAW_RIGHT_FRONT_ANGLE;
                yaw_dir = 1;
            }
        }
    } else if(yaw_dir == 1) //�������
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
                Gimbal_process.Yaw_taget = SCOUT_YAW_RIGHT_FRONT_ANGLE;  //ֱ��ת
            }
        }
    }

//    /*	�жϱ߽�  */
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
//    }//������
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

//    }//��ǰ���

    /*		��������yaw�����촦���			*/
    if(pitch_dir == 0)//�������
    { 
        Gimbal_process.Pitch_taget -= SCOUT_PITCH_SPEED;
        if(Gimbal_process.PITCH_PPM.target <= (SCOUT_PITCH_DOWN_ANGLE) )
        {
            Gimbal_process.Pitch_taget = SCOUT_PITCH_DOWN_ANGLE;
            pitch_dir=1;
        }
    } else if(pitch_dir == 1) //�������
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
//    Gimbal_process.Yaw_taget=constrain(Gimbal_process.Yaw_taget,-1650.f,1650.f);	 //�������汾YAW�������λ
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
* @brief ��̨����
* @param void
* @return void
*	������̨�������ӣ���ֹ����ѹ����ʱ��ײ��
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
    }//ֻ�Ǹ���
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
    }//ֻ�Ǹ���
}

/**
* @brief �Զ�ģʽ����
* @param void
* @return void
*	�Զ�ģʽ�µ���̨�˶�
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
* @brief �Լ�����
* @param void
* @return void
*	�Լ�ģʽ�µ���̨�˶�
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
    Gimbal_process.YAW_PPM.out=0;//yawλ�û�

    Gimbal_process.YAW_PVM.target=0;
    Gimbal_process.YAW_PVM.kp=16;//25
    Gimbal_process.YAW_PVM.ki=0.2;//0.4
    Gimbal_process.YAW_PVM.kd=0;
    Gimbal_process.YAW_PVM.integral_max=20000;
    Gimbal_process.YAW_PVM.out_max=28000;
    Gimbal_process.YAW_PVM.out=0;//yaw�ٶȻ�


    Gimbal_process.PITCH_PPM.target=0;
    Gimbal_process.PITCH_PPM.kp=10;//10
    Gimbal_process.PITCH_PPM.ki=0;
    Gimbal_process.PITCH_PPM.kd=0;
    Gimbal_process.PITCH_PPM.integral_max=8000;
    Gimbal_process.PITCH_PPM.out_max=16000;
    Gimbal_process.PITCH_PPM.out=0;//pitchλ�û�

    Gimbal_process.PITCH_PVM.target=0;
    Gimbal_process.PITCH_PVM.kp=12;//12  
    Gimbal_process.PITCH_PVM.ki=0.1;//0.3
    Gimbal_process.PITCH_PVM.kd=0;
    Gimbal_process.PITCH_PVM.integral_max=26000;
    Gimbal_process.PITCH_PVM.out_max=28000;
    Gimbal_process.PITCH_PVM.out=0;//pitch�ٶȻ�

    Gimbal_process.PITCH2_PVM.kp=12;//12  
    Gimbal_process.PITCH2_PVM.ki=0.6;//0.6
    Gimbal_process.PITCH2_PVM.kd=0;
    Gimbal_process.PITCH2_PVM.integral_max=26000;
    Gimbal_process.PITCH2_PVM.out_max=28000;
    Gimbal_process.PITCH2_PVM.out=0;//pitch�ٶȻ�
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


