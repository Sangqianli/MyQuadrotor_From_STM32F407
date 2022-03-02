/**
 * @file        monitor_task.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-November-2020
 * @brief       Monitor&Test Center
 */

/* Includes ------------------------------------------------------------------*/
#include "chassis_task.h"

#include "device.h"
#include "rp_math.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
#define SPEED_MAX  4600
#define High_Speed 3600  //3600 �� 4600 ,3000
#define Normal_Speed 2400//�Կ���3000,������2000
#define Attack_Speed 1500  //1500
#define Cover_Speed  3000
#define Escape_Speed 3000 //2500
#define First_Speed  -1000
#define Base_Speed   1000
#define More_Speed   200
#define High_Swerve_Ratio 0.6  //0.6
#define Normal_Swerve_Ratio 0.2  //�Կ���0.1
#define Remain_ATTACK 6U   //ȡ������rand%9+1 = 1~9
#define Remain_COVER  8U   //rand%5 + 1 = 1~5
#define Remain_ESCAPE 8U
#define Atrip_Num 10.f  //����ֶ���
#define Atrip_Err  200U  //��Χ
#define Atrip_Line 0U  //������λ,ʵ��Ҫ����Χ��
#define Atrip_length 50000U
#define PowerBuff_Hot    60.F
#define PowerLimit_Normal 12000U
#define HP_HURT   -10
#define HP_VERYHURT   -60
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float swerve_ratio = 0.1;
int8_t Aim_dis;
/* Exported variables --------------------------------------------------------*/
Chassis_t Chassis_process;
/* Private functions ---------------------------------------------------------*/

/**
* @brief ��ʼѲ��
* @param void
* @return void
* ���ڵ�һ���ܹ�ʱ�Թ�����ȵ�У�����ܹ��ٶȽ�����һ����3����׼�������
*/
static void Cruise_First()
{
    static uint8_t step=0;	/*��ʼ������*/
    if(Chassis_process.init_flag == false)		/*δ�������ɳ�ʼ��*/
    {
        switch(step)
        {
        case 0:			/*��ʼ����������߿�*/
        {
            if(path_sensor.info->left_touch)		/*��ߵĵ㴥����*/
            {
                Chassis_process.Speed_taget=0;
                step=1;
            }
            else	/*δʶ�� -- �����˶�*/
            {
                Chassis_process.Speed_taget = -1600;	/*�����˶�*/
            }
            break;
        }
        case 1:
        {
            if(path_sensor.info->left_touch)
            {
                Chassis_process.Speed_taget=100;	/*����΢�������ڵ��պò������㴥���ص�λ��*/
            }
            else			/*΢����ɺ���ͣס��������һ�����ڽ׶�*/
            {
                Chassis_process.Speed_taget=0;
                path_sensor.info->mileage_total=0;	/*�������� -- ��ʼ��¼*/
                step=2;
            }
            break;
        }
        case 2:
        {
            if(path_sensor.info->right_touch)	/*�Ѿ�ʶ���ҹ��*/
            {
                Chassis_process.Speed_taget=0;
                step=3;
            }
            else 		/*δʶ��*/
            {
                Chassis_process.Speed_taget=1600;			/*�����˶�*/
            }
            break;
        }
        case 3:
        {
            if(path_sensor.info->right_touch)
            {
                Chassis_process.Speed_taget=-100;		/*�Ѿ�ʶ���ҵ㴥���أ�����΢�����պ�δʶ�𵽵�״̬*/
            }
            else			/*΢�����*/
            {
                Chassis_process.Speed_taget=0;															/*ֹͣ�˶�*/
                Chassis_process.Mileage_atrip=path_sensor.info->mileage_total;	/*��¼�������*/
                step=0;																							/*��λstep*/
                Chassis_process.init_flag=true;											/*���±�־λ*/
                Chassis_process.Derection_flag=-1;//�����˶���־λ
                Chassis_process.Spot_taget = 0;//λ�û�
                Chassis_process.getchange_flag = true;
            }
            break;
        }
        }
    }
}

/**
  * @brief  Ѫ����麯��
	* @note   ÿ500ms���һ��Ѫ���仯����Ѫ������
	*					˵��װ���ܵ��˺�
  * @params  void
  * @retval bool��ֵΪ1��ʾ�ܵ��˺���ֵΪ0��ʾδ�ܵ��˺�
  */
bool HP_Check(void)
{
    static  uint16_t last_HP = 600;
    static  uint16_t cur_HP = 0;
    static  int16_t delta_HP;
    static  uint32_t HP_check_time = 1;
    static  bool result = 0;

    HP_check_time++;

    if(HP_check_time % 100 == 0)                        //ÿ200ms���һ��Ѫ���仯ֵ
    {
        cur_HP = judge_sensor.info->GameRobotStatus.remain_HP;
        delta_HP = cur_HP - last_HP;
        last_HP = cur_HP;
        if(delta_HP <= HP_HURT && delta_HP > -60)      //ÿ200ms���һ��Ѫ���仯ֵ
            result = true;
        else
            result = false;
        HP_check_time = 1;
    }
    else
        result = false;

    return result;
}

bool HP_GOLF_Check(void)
{
    static  uint16_t last_HP = 600;
    static  uint16_t cur_HP = 0;
    static  int16_t delta_HP;
    static  uint32_t HP_check_time = 1;
    static  bool result = 0;

    HP_check_time++;

    if(HP_check_time % 100 == 0)                        //ÿ200ms���һ��Ѫ���仯ֵ
    {
        cur_HP = judge_sensor.info->GameRobotStatus.remain_HP;
        delta_HP = cur_HP - last_HP;
        last_HP = cur_HP;
        if(delta_HP <= HP_VERYHURT)                        //ÿ200ms���һ��Ѫ���仯ֵ
            result = true;
        else
            result = false;
        HP_check_time = 1;
    }
    else
        result = false;

    return result;
}

/**
  * @brief  �ܹ����жϺ���
	* @note �ܵ�����3�Σ��ж�Ϊ����Σ��״̬��
	*					��6s����û���ܵ����������밲ȫ
	*					״̬��ÿ���������г��򣬵�һ����
	*					��2�ι����Ż����Σ��״̬������
	*					ÿ�ܵ�1�ι����ͻ����Σ��״̬��
  * @params ishurt���Ƿ��ܵ�����
  * @retval bool��ֵΪ1��ʾ����Σ��״̬��ֵΪ0��ʾδ���밲ȫ״̬
  */
bool IS_Danger(bool ishurt, uint8_t hurt_times)
{
    static bool hurtflag = 0;
    static bool result = 0;
    static uint32_t safe_cnt,hurt_cnt;

    if(ishurt)                //����ܵ�����
    {
        safe_cnt = 0;

        if( (judge_sensor.info->hurt_data_update)&&(judge_sensor.info->RobotHurt.hurt_type == 0) )
        {
            if(hurtflag != 1)
            {
                hurtflag = 1;
                hurt_cnt++;           //��¼�ܵ��˺�����
                if(hurt_cnt >= hurt_times)     //�ܵ�n���˺����ϣ��ж�Ϊ����Σ��
                {
                    hurt_cnt = 0;
                    result = true;
                }
            }
            judge_sensor.info->hurt_data_update = false;
        }
    }
    else
    {
        hurtflag = 0;
        safe_cnt++;
        if(safe_cnt > 6000)
            result = false;
    }
    return result;
}

bool IS_GOLF_Danger(bool ishurt, uint8_t hurt_times)
{
    static bool hurtflag = 0;
    static bool result = 0;
    static uint32_t safe_cnt,hurt_cnt;


    if(ishurt)                //����ܵ�����
    {
        safe_cnt = 0;

        if( (judge_sensor.info->hurt_data_update)&&(judge_sensor.info->RobotHurt.hurt_type == 0) )
        {
            if(hurtflag != 1)
            {
                hurtflag = 1;
                hurt_cnt++;           //��¼�ܵ��˺�����
                if(hurt_cnt >= hurt_times)     //�ܵ�n���˺����ϣ��ж�Ϊ����Σ��
                {
                    hurt_cnt = 0;
                    result = true;
                }
            }
            judge_sensor.info->hurt_data_update = false;
        }
    }
    else
    {
        hurtflag = 0;
        safe_cnt++;
        if(safe_cnt > 12000)
            result = false;
    }
    return result;
}

/**
* @brief �����ж�
* @param bool
* @return bool
* �ж������ڹ���״̬
*/
static bool Is_Attack()
{
    static bool result = false;
    static uint32_t lost_cnt;

    if( (sys.auto_mode == AUTO_MODE_ATTACK)||(leader_sensor.info->data.attack_now == 1) )
        result = true;
    else
    {
        lost_cnt ++;
        if( lost_cnt >= 1000 ) //1s
        {
            result = false;
            lost_cnt = 0;
        }
    }
    return result;
}

/**
* @brief �����ж�
* @param bool
* @return bool
* �ж��Ƿ��Ѫ����Ѫ����ȫ���ܹ�
*/
static bool Is_TimeToRun()
{
    if(judge_sensor.info->GameRobotStatus.remain_HP <= HP_Danger)
        return true;
    else
        return false;
}

/**
* @brief �����ж�
* @param bool
* @return bool
* �ж����Ŀ��λ���Ƿ񵽴�
*������������ʾ����ֶζ���λ��
*/
static bool Is_SpotArrive( int32_t spot_target)
{
    uint32_t err;
    err = abs(spot_target - path_sensor.info->mileage_total);
    if(err <= Atrip_Err)
        return true;
    else
        return false;
}
/**
* @brief �ٶ�����
* @param void
* @return void
* ��������޸ĵ����ٶȣ���������·�̵ı���
*/
static void Chassis_Speed_Set()
{
    if( Is_TimeToRun() )
    {
        Chassis_process.Speed_taget=Chassis_process.Derection_flag * High_Speed;
        swerve_ratio = High_Swerve_Ratio;
    }
    else
    {
        Chassis_process.Speed_taget=Chassis_process.Derection_flag * Normal_Speed;
        swerve_ratio = Normal_Swerve_Ratio ;
    }
}
/**
* @brief ȫ��Ѳ��
* @param void
* @return void
* ��ɹ����ʼУ׼���Ѳ����������һ���ٶ�
* -1Ϊ��1Ϊ��
*/
static void Cruise_Normal()
{
    Chassis_process.getchange_flag = true;//�������ʱ��ȡλ��

    if(Chassis_process.Derection_flag == -1)
    {
        Chassis_process.Spot_taget = 0;//����λ�û�����

//		if(path_sensor.info->mileage_total < 0)
//		{
//			Chassis_process.Derection_flag = 1;//�����ܹ�
//			Chassis_process.Spot_taget = Chassis_process.Mileage_atrip;//����λ�û�����
//		}//��ֹ����ͣ�ڹ��һ��


        if(path_sensor.info->left_touch)//�����˶�ʱ�����㴥����
        {
            Chassis_process.swerve_judge = true;//��ʼ�����ж�
            Chassis_process.swerve_flag = true;
        }

        if(Chassis_process.swerve_judge)
        {
            if(path_sensor.info->left_touch == false)//�������㴥�����ͷ�
            {
                Chassis_process.Trip_times ++;     //��¼��������
                Chassis_process.Derection_flag = 1;//�����ܹ�
                Chassis_process.Spot_taget = Chassis_process.Mileage_atrip;//����λ�û�����
                path_sensor.info->mileage_total = 0;//��������
                Chassis_process.swerve_judge = false;
                Chassis_process.swerve_flag = false;//�������
            }
        }
    }//��

    if(Chassis_process.Derection_flag == 1)
    {
        Chassis_process.Spot_taget = Chassis_process.Mileage_atrip;//����λ�û�����
//		if(path_sensor.info->mileage_total > Chassis_process.Mileage_atrip)
//		{
//			Chassis_process.Derection_flag = -1;//�����ܹ�
//			Chassis_process.Spot_taget = 0;//����λ�û�����
//		}//��ֹ����ͣ�ڹ��һ��

        if(path_sensor.info->right_touch)//�����˶�ʱ�����㴥����
        {
            Chassis_process.swerve_judge = true;//��ʼ�����ж�
            Chassis_process.swerve_flag = true;
        }
        if(Chassis_process.swerve_judge)
        {
            if(path_sensor.info->right_touch == false)//�������㴥�����ͷ�
            {
                Chassis_process.Derection_flag = -1;//�����ܹ�
                Chassis_process.Spot_taget = 0;//����λ�û�����
                Chassis_process.Mileage_atrip = path_sensor.info->mileage_total;//��¼��������
                Chassis_process.swerve_judge = false;
                Chassis_process.swerve_flag = false;//�������
            }
        }
    }//��
}

/**
* @brief ��������
* @param void
* @return void
* �����������շ�Χ�Ĵ���
*/
static void Chassis_Rebound()
{
    if(Chassis_process.swerve_judge)
    {
        Chassis_process.swerve_flag = true; //����ʱ��־λ
    }
    else//û�з���������Ѳ��
    {
        Chassis_process.Speed_taget = Chassis_process.Speed_taget;
    }
}

/**
* @brief ң�ؿ���
* @param void
* @return void
*/
static void Chassis_RCcontrol()
{
//    Chassis_process.init_flag = false;
    Chassis_process.swerve_judge = false;
    Chassis_process.swerve_flag = false;

    Chassis_process.Speed_taget = rc_sensor.info->ch2*Chassis_process.rotate_ratio;
    Chassis_process.PVM.target = Chassis_process.Speed_taget;
    Chassis_process.PVM.measure = motor[CHASSIS].info->speed;
    pid_calculate(&Chassis_process.PVM);
    NormalData_0x200[0] = Chassis_process.PVM.out;
}

/**
  * @brief 	���̵����ֹ��������汾
  * @retval void
  */
void Chassis_Stuck_Handle_1_0(void)
{
    static int16_t static_cnt = 0;
    static int16_t cold_cnt = 0;
    bool cold_judge = false;
    if( (motor[CHASSIS].info->speed < 1000)&&(Chassis_process.Fire == FIRE_RUN)&&(Chassis_process.PVM.out >6000) )//abs(Chassis_process.PVM.out)>4000
    {
        if(cold_judge == false)
        {
            static_cnt ++;
        } else
        {
            cold_cnt ++;
            static_cnt = 0;
            if(cold_cnt > 200)
            {
                cold_cnt = 0;
                cold_judge = false;
            }
        }
    } else
    {
        static_cnt = 0;
        cold_cnt = 0;
        cold_judge = false;
    }
    if( static_cnt>500 )
    {
        if(Chassis_process.Derection_flag == 1)
        {
            path_sensor.info->mileage_total = Atrip_length + 200;//У����ʱ��λ��
        }
        if(Chassis_process.Derection_flag == -1)
        {
            path_sensor.info->mileage_total = 0 - 200;//У����ʱ��λ��
        }

        Chassis_process.getchange_flag = true;

        Chassis_process.Derection_flag = - Chassis_process.Derection_flag;

        Chassis_process.swerve_flag = false;
        Chassis_process.swerve_judge = false;
        static_cnt = 0;
        cold_judge = true;
    }
}

/**
  * @brief 	���̵����ֹ�����������汾
  * @retval void
  */
void Chassis_Stuck_Handle(void)
{
    static int16_t static_cnt = 0;
    static int16_t cold_cnt = 0;
    bool cold_judge = false;
    if( (abs(path_sensor.info->mileage_dif)<20)&&(Chassis_process.Fire == FIRE_RUN) )//abs(Chassis_process.PVM.out)>4000
    {
        if(cold_judge == false)
        {
            static_cnt ++;
        } else
        {
            cold_cnt ++;
            static_cnt = 0;
            if(cold_cnt > 200)
            {
                cold_cnt = 0;
                cold_judge = false;
            }
        }
    } else
    {
        static_cnt = 0;
        cold_cnt = 0;
        cold_judge = false;
    }
    if( static_cnt>500 )
    {
        Chassis_process.getchange_flag = true;
        if(Chassis_process.Mode == CHASSIS_NORMAL)
        {
            if(path_sensor.info->mileage_total < (Chassis_process.Mileage_atrip*0.5) )
            {
                Chassis_process.Derection_flag = 1;
                Chassis_process.Spot_taget = Chassis_process.Mileage_atrip;//����λ�û�����
            }
            else
            {
                Chassis_process.Derection_flag = -1;
                Chassis_process.Spot_taget = 0;//����λ�û�����
            }
        }
        Chassis_process.swerve_flag = false;
        Chassis_process.swerve_judge = false;
        static_cnt = 0;
        cold_judge = true;
    }
}
/**
* @brief �����ܹ�
* @param void
* @return void
* ֱ�ӷ���
*/
static void Chassis_Change_Dir(void)
{
    Chassis_process.Derection_flag = -Chassis_process.Derection_flag;

}

/**
* @brief �����ٶ�
* @param void
* @return void
* �Զ������ܹ���ٶȻ�ȡ
*/
static void Chassis_Change_Speed(void)
{
    if(Chassis_process.Mode == CHASSIS_NORMAL)
    {
        Chassis_process.Speed_taget = Chassis_process.Derection_flag * Normal_Speed;
    }
    else if(Chassis_process.Mode == CHASSIS_ATTACK)
    {
        Chassis_process.Speed_taget = Chassis_process.Derection_flag *  Cover_Speed ;
    }
    else if(Chassis_process.Mode == CHASSIS_COVER)
    {
        Chassis_process.Speed_taget = Chassis_process.Derection_flag *  Escape_Speed;
    }
    else if(Chassis_process.Mode == CHASSIS_ESCAPE)
    {
        Chassis_process.Speed_taget = Chassis_process.Derection_flag *  Escape_Speed;
    }
    Chassis_process.Speed_taget = constrain(Chassis_process.Speed_taget, -3000, 3000);  //�ٶ��޷�
}

/**
* @brief �������
* @param void
* @return void
* �Զ������ܹ�ľ����ȡ,�����ϴ�Ŀ�����ٵ���
*/
static uint8_t Chassis_Change_Dis(void)
{
    static uint8_t remain;
    uint8_t dis;

    if(Chassis_process.Mode == CHASSIS_ATTACK)
    {
        remain = Remain_ATTACK;
        dis = (rand() % remain) + 1;//����1��9 �������
    } else if(Chassis_process.Mode == CHASSIS_COVER)
    {
        remain = Remain_COVER;
        dis = (rand() % remain) + 2;//����3��9 �������
    } else if(Chassis_process.Mode == CHASSIS_ESCAPE)
    {
        remain = Remain_ESCAPE;
        dis = (rand() % remain) + 2;//����3��9 �������
    }

    return dis;
}

/**
* @brief �Զ��ܹ����
* @param void
* @return void
* ��������ʱֱ�������Ϊ0
*/
static void Chassis_AUTOcontrol()
{
    if(Chassis_process.init_flag)
    {
        Chassis_Speed_Set(); //�趨�ٶ�
        Chassis_Stuck_Handle();
    }
    Chassis_process.PVM.target = Chassis_process.Speed_taget;
    Chassis_process.PVM.measure = motor[CHASSIS].info->speed;
    pid_calculate(&Chassis_process.PVM);
    if(Chassis_process.swerve_flag)
    {
        Chassis_process.PVM.target = 0;
        Chassis_process.PVM.out = 0;  //����ʱ���Ϊ0
    }
    NormalData_0x200[0] = (int16_t)Chassis_process.PVM.out;
}

/**
* @brief �Զ��ܹ�
* @param void
* @return void
*/
static void Chassis_AUTO()
{
//    if(Chassis_process.init_flag)
//    {
//        Cruise_Normal();
//        Chassis_Rebound();
//    }
//    else
//    {
//        Cruise_First();
//    }
//    Chassis_AUTOcontrol();
    Chassis_process.init_flag = true;
    Chassis_RCcontrol();

}

/**
* @brief ��̨��������Ϣ
* @param void
* @return void
*�ṩ��ⷢ�Ͱ������ж��Ƿ��������ģʽ
*����true��ʾ���ܣ�����flase��ʾ������
*/
static bool Run_Key_info()
{
    static	bool result = false;
    if(judge_sensor.info->AerialData.cmd & escape)
        result	= true;
    else
        result	= false;
    return	result;
}

/**
* @brief ��̨�ֹ�·��Ϣ
* @param void
* @return void
*�ṩ��ⷢ�Ͱ������ж��Ƿ�ͣ�ڹ�·�ˣ�ֻ����ǰ��վ���ʱ����Ч
*����true��ʾ��·�ˣ�����flase��ʾ������
*/
static bool Highway_Key_info()
{
    static	bool result = false;
    if(judge_sensor.info->AerialData.cmd & check_road)
        result	= true;
    else
        result	= false;
    return	result;
}

/**
* @brief ��̨�ֽӹ���Ϣ
* @param void
* @return void
*�ṩ��ⷢ�Ͱ���ȷ���Ƿ�����̨�ַ��ͽӹ�
*����true��ʾ�ӹܣ�����flase��ʾ������
*/
static bool Aerial_Control_info()
{
    static	bool result = false;
    static uint32_t wait_cnt;
    if(judge_sensor.info->AerialData.cmd & control_aerial)
        result	= true;
    else
        result	= false;

//    if(judge_sensor.info->communication_data_update)
//    {
//        wait_cnt = 0;
        judge_sensor.info->communication_data_update = false;
//    }
//    else
//    {
//        wait_cnt ++;  
//        if(wait_cnt >= 1000)
//        {
//            judge_sensor.info->AerialData.cmd = 0;
//            judge_sensor.info->AerialData.control_dir = 0;
//            wait_cnt = 0;
//        }
//    }
    return	result;
}

/**
* @brief ����״̬��ȡ
* @param void
* @return void
* ��������л�ģʽ
*/
static void Chassis_GetMode()
{
    if( IS_GOLF_Danger(HP_GOLF_Check(), 1) )//||(vision_sensor.info->RxPacket.RxData.identify_hero == 1)
    {
        Chassis_process.Safe = CHASSIS_DANGER;
    } else if( IS_Danger(HP_Check(), 1) )
    {
        Chassis_process.Safe = CHASSIS_HURT;
    } else
    {
        Chassis_process.Safe = CHASSIS_SAFE;
    }

    if( Is_Attack() == false )
    {
        if( Chassis_process.Safe == CHASSIS_SAFE )
        {
            Chassis_process.Mode = CHASSIS_NORMAL;
        }
        else if( Chassis_process.Safe == CHASSIS_HURT )
        {
            Chassis_process.Mode = CHASSIS_COVER;
        }
        else if( Chassis_process.Safe == CHASSIS_DANGER|| Run_Key_info() )
        {
            Chassis_process.Mode = CHASSIS_ESCAPE;
        }
    } else if(  Is_Attack() == true )
    {
        if( Chassis_process.Safe == CHASSIS_SAFE )
        {
            Chassis_process.Mode = CHASSIS_ATTACK;
        }
        else if( Chassis_process.Safe == CHASSIS_HURT )
        {
            Chassis_process.Mode = CHASSIS_COVER;
        }
        else if( (Chassis_process.Safe == CHASSIS_DANGER)|| Run_Key_info() )
        {
            Chassis_process.Mode = CHASSIS_ESCAPE;
        }
    }
}

/**
* @brief ��������
* @param void
* @return void
*/
static void Show_Time()
{
    static bool left_err_judge = false; //�������˶������󿪹�
    static bool right_err_judge = false;//�������˶������ҿ���

    Chassis_process.swerve_judge = false;
    Chassis_process.swerve_flag = false;//���Ѳ��ģʽʱ�ķ�������


    if(Chassis_process.getchange_flag)
    {
        Chassis_Change_Dir(); //����
        Aim_dis = Chassis_Change_Dis() * Chassis_process.Derection_flag ;//��ȡ�����������
        Chassis_process.Spot_taget = path_sensor.info->mileage_total + (int32_t)(Chassis_process.Mileage_atrip * (Aim_dis/Atrip_Num) );

        Chassis_process.Spot_taget = constrain(Chassis_process.Spot_taget, Atrip_Line,Chassis_process.Mileage_atrip - Atrip_Line );   //���Ը����������ͬ����λ

        Chassis_process.getchange_flag = false;
    }

    if( (Chassis_process.Spot_taget - path_sensor.info->mileage_total) > 0 )
    {
        Chassis_process.Derection_flag = 1;
    } else if( (path_sensor.info->mileage_total - Chassis_process.Spot_taget) >= 0 )
    {
        Chassis_process.Derection_flag = -1;
    }

    if( Is_SpotArrive(Chassis_process.Spot_taget) )
    {
        Chassis_process.getchange_flag = true;  //�����ٽ����ȡ�������
    }
    //ɲ��ס��Ҳû�£���ʱ�����жϿ��أ�ͬʱУ׼����������
    if(Chassis_process.Derection_flag == -1)//�����˶�������
    {
        if(path_sensor.info->right_touch)//�����ҿ���
        {
            Chassis_process.Mileage_atrip = path_sensor.info->mileage_total;//��¼��������
        }
//            right_err_judge = true;
//        }
//        if(right_err_judge)
//        {
//            if(path_sensor.info->right_touch == false)//�������㴥�����ͷ�
//            {
//                Chassis_process.Mileage_atrip = path_sensor.info->mileage_total;//��¼��������
//                right_err_judge = false;
//            }
//        }
        if(path_sensor.info->left_touch)//�����˶�ʱ�����󿪹�
        {
            path_sensor.info->mileage_total = 0;//��������
        }

//            Chassis_process.swerve_judge = true;//��ʼ�����ж�
//            Chassis_process.swerve_flag = true;
//        }
//        if(Chassis_process.swerve_judge)
//        {
//            if(path_sensor.info->left_touch == false)//�������㴥�����ͷ�
//            {
////                Chassis_process.getchange_flag = true; //���»�ȡ����
//                path_sensor.info->mileage_total = 0;//��������
//                Chassis_process.swerve_judge = false;
//                Chassis_process.swerve_flag = false;//�������
//            }
//        }
    }//��

    if(Chassis_process.Derection_flag == 1)//�����˶�������
    {
        if(path_sensor.info->left_touch)//�����󿪹�
        {
            path_sensor.info->mileage_total = 0;//��������

        }

//            left_err_judge = true;
//        }
//        if(left_err_judge)
//        {
//            if(path_sensor.info->left_touch == false)//�������㴥�����ͷ�
//            {
//                path_sensor.info->mileage_total = 0;//��������
//                left_err_judge = false;
//            }
//        }

        if(path_sensor.info->right_touch)//�����ҿ���
        {
            Chassis_process.Mileage_atrip = path_sensor.info->mileage_total;//��¼��������
        }

//            Chassis_process.swerve_judge = true;//��ʼ�����ж�
//            Chassis_process.swerve_flag = true;
//        }
//        if(Chassis_process.swerve_judge)
//        {
//            if(path_sensor.info->right_touch == false)//�������㴥�����ͷ�
//            {
////                Chassis_process.getchange_flag = true;//���»�ȡ����
//                Chassis_process.Mileage_atrip = path_sensor.info->mileage_total;//��¼��������
//                Chassis_process.swerve_judge = false;
//                Chassis_process.swerve_flag = false;//�������
//            }
//        }
    }//��
}

/**
* @brief ������
* @param void
* @return void
* ǰ��վ���ʱ��ֹ�ڿ�������Ҳ࣬�����Ӵ�����
*/
static void Static_shoot()
{
    Chassis_process.Mode = CHASSIS_NORMAL;//һ��Ϊ����״̬

    if( Highway_Key_info() )
        Chassis_process.Spot_taget = 2000;
    else
        Chassis_process.Spot_taget = (Chassis_process.Mileage_atrip - 2000);
    if(Is_SpotArrive(Chassis_process.Spot_taget) )
    {
        Chassis_process.static_want =true;
    } else
    {
        if( (Chassis_process.Spot_taget - path_sensor.info->mileage_total) > 0 )
        {
            Chassis_process.Derection_flag = 1;
        } else if( (path_sensor.info->mileage_total - Chassis_process.Spot_taget) > 0 )
        {
            Chassis_process.Derection_flag = -1;
        }
        Chassis_process.static_want = false;
    }
}

/**
* @brief �����˶�ģʽ��ȡ
* @param void
* @return void
* ����ǰ��վ״̬�����Ƿ�ֹ���������ܹ�
*/
static void Outpost_get()
{
    static uint16_t  tum_cnt;

    if(judge_sensor.info->EventData.outpost == 1) //ǰ��վ���ʱ��bit10 == 1
    {
        Chassis_process.Fire = FIRE_ALL;
    } else {
        /***************ң�������ֿ���ģ��ǰ��վ����Ҫ����ѵ�����㣬����ʱȥ��*********/
        if( abs(rc_sensor.info->thumbwheel) >=630 )
        {
            tum_cnt++;
            if(tum_cnt > 100)
            {
                if(rc_sensor.info->thumbwheel >= 630)
                {
                    Chassis_process.Fire = FIRE_ALL;
                } else if(rc_sensor.info->thumbwheel <= -630)
                {
                    Chassis_process.Fire = FIRE_RUN;
                }
                tum_cnt = 0;
            }
        }
        /******************************************************/
//        Chassis_process.Fire = FIRE_RUN;//����ʱ�ָ�
    }
}
/**
* @brief �Զ��ܹ����2.0
* @param void
* @return void
* �������
*/
static void Chassis_AUTOcontrol_2_0()
{
    if(Chassis_process.init_flag)
    {
        if(Chassis_process.Fire == FIRE_RUN)
        {
            Chassis_Change_Speed();
            Chassis_Stuck_Handle_1_0();
        } else if(Chassis_process.Fire == FIRE_ALL)
        {
            Chassis_process.Speed_taget = 0;
        }
    }
    Chassis_process.PVM.target = Chassis_process.Speed_taget;
    Chassis_process.PVM.measure = motor[CHASSIS].info->speed;
    pid_calculate(&Chassis_process.PVM);
    if(Chassis_process.swerve_flag)
    {
        Chassis_process.PVM.out = 0;  //����ʱ���Ϊ0
    }
    NormalData_0x200[0] = (int16_t)Chassis_process.PVM.out;
}

/**
* @brief �Զ��ܹ����3.0
* @param void
* @return void
* ȫ��˫��PID���������ϣ����������Զ��������...
*/
static void Chassis_AUTOcontrol_3_0()
{
    if(Chassis_process.init_flag)
    {
        if(Chassis_process.Mode == CHASSIS_NORMAL)
        {
            Chassis_process.PPM.kp = 0.4;
            Chassis_process.PPM.out_max = 4000;
        }
        else if(Chassis_process.Mode == CHASSIS_ATTACK)
        {
            Chassis_process.PPM.kp = 0.3;
            Chassis_process.PPM.out_max = 3000;
        }
        else if(Chassis_process.Mode == CHASSIS_COVER)
        {
            Chassis_process.PPM.kp = 0.4;
            Chassis_process.PPM.out_max = SPEED_MAX;
        }
        else if(Chassis_process.Mode == CHASSIS_ESCAPE)
        {
            Chassis_process.PPM.kp = 0.4;
            Chassis_process.PPM.out_max = SPEED_MAX;
        }

        Chassis_Stuck_Handle(); // ���̾�ֹ����



        Chassis_process.PPM.target = Chassis_process.Spot_taget;
        Chassis_process.PPM.measure = path_sensor.info->mileage_total;
        pid_calculate(&Chassis_process.PPM);

        if( (Chassis_process.Fire == FIRE_ALL)&&(Chassis_process.static_want == true) )
        {
            Chassis_process.Spot_taget = path_sensor.info->mileage_total;
//            Chassis_process.Speed_taget = Chassis_process.PPM.out;
            Chassis_process.Speed_taget = 0;
        } else
        {
            Chassis_process.Speed_taget = Chassis_process.PPM.out;

            if(Chassis_process.Mode == CHASSIS_NORMAL)
            {
                if(Chassis_process.Spot_taget == 0)//��������
                { 
                    Chassis_process.Speed_taget = constrain(Chassis_process.Speed_taget, -4000, -Chassis_process.PPM.integral_max);
                } else if(Chassis_process.Spot_taget == Chassis_process.Mileage_atrip)//��������
                {
                    Chassis_process.Speed_taget = constrain(Chassis_process.Speed_taget, Chassis_process.PPM.integral_max, 4000);
                }
            }
            else
            {
                if(Chassis_process.Derection_flag == 1)//����
                {
                    Chassis_process.Speed_taget = constrain(Chassis_process.Speed_taget, Chassis_process.PPM.integral_max, SPEED_MAX);
                } else if(Chassis_process.Derection_flag == -1)//����
                {
                    Chassis_process.Speed_taget = constrain(Chassis_process.Speed_taget, -SPEED_MAX, -Chassis_process.PPM.integral_max);
                }//���ٶ�ƫ�ã���ֹ�ٶ�̫��ͣ����
            }
        }
    }
    Chassis_process.PVM.target = Chassis_process.Speed_taget;
    Chassis_process.PVM.measure = motor[CHASSIS].info->speed;
    pid_calculate(&Chassis_process.PVM);
    if(Chassis_process.swerve_flag)
    {
        Chassis_process.PVM.out = 0;  //����ʱ���Ϊ0
    }
    NormalData_0x200[0] = (int16_t)Chassis_process.PVM.out;
}
/**
* @brief ��������ģʽ�Զ��ܹ�
* @param void
* @return void
*
*/
static void Chassis_AUTO_TOUCH()
{
    Outpost_get();//��ȡǰ��վ״̬���ı����ģʽ

    Chassis_process.init_flag = true;

    if(Chassis_process.Fire == FIRE_ALL)
    {
        Chassis_process.Speed_taget = 0;
    } else if(Chassis_process.Fire == FIRE_RUN)
    {
        Chassis_GetMode(); //��ȡ״̬
        if(Chassis_process.Derection_flag == -1)
        {
            if(path_sensor.info->left_touch)//�����˶�ʱ�����㴥����
            {
                Chassis_process.swerve_judge = true;//��ʼ�����ж�
                Chassis_process.swerve_flag = true;
            }

            if(Chassis_process.swerve_judge)
            {
                if(path_sensor.info->left_touch == false)//�������㴥�����ͷ�
                {
                    Chassis_process.Trip_times ++;     //��¼��������
                    Chassis_process.Derection_flag = 1;//�����ܹ�
                    Chassis_process.Spot_taget = Chassis_process.Mileage_atrip;//����λ�û�����
                    path_sensor.info->mileage_total = 0;//��������
                    Chassis_process.swerve_judge = false;
                    Chassis_process.swerve_flag = false;//�������
                }
            }
        }//��

        if(Chassis_process.Derection_flag == 1)
        {
            if(path_sensor.info->right_touch)//�����˶�ʱ�����㴥����
            {
                Chassis_process.swerve_judge = true;//��ʼ�����ж�
                Chassis_process.swerve_flag = true;
            }
            if(Chassis_process.swerve_judge)
            {
                if(path_sensor.info->right_touch == false)//�������㴥�����ͷ�
                {
                    Chassis_process.Derection_flag = -1;//�����ܹ�
                    Chassis_process.Spot_taget = 0;//����λ�û�����
                    Chassis_process.Mileage_atrip = path_sensor.info->mileage_total;//��¼��������
                    Chassis_process.swerve_judge = false;
                    Chassis_process.swerve_flag = false;//�������
                }
            }
        }//��
    }
    Chassis_Rebound();
    Chassis_AUTOcontrol_2_0();
}

/**
* @brief ������ģʽ�ܹ�
* @param void
* @return void
*
*/
static void Chassis_AUTO_ENCODER()
{
    static float static_pot;
    static uint16_t err_cnt;

    Outpost_get();//��ȡǰ��վ״̬���ı����ģʽ

    Chassis_process.init_flag = true;

    if(Chassis_process.Fire == FIRE_ALL)
    {
        Chassis_process.Speed_taget = 0;
        static_pot = path_sensor.info->mileage_total;//��¼��ʱ��λ��
        path_sensor.info->mileage_total = Atrip_length;
    } else if(Chassis_process.Fire == FIRE_RUN)
    {
        Chassis_GetMode(); //��ȡ״̬

        if(Chassis_process.Derection_flag == -1)
        {
            if(path_sensor.info->mileage_total <= 0 )//����λ���ͷ�
            {
                Chassis_process.Derection_flag = 1;//�����ܹ�
            }
        }//��

        if(Chassis_process.Derection_flag == 1)
        {
            if(path_sensor.info->mileage_total  >= static_pot )//����λ���ͷ�
            {
                Chassis_process.Derection_flag = -1;//�����ܹ�
            }
        }//��
    }
    Chassis_AUTOcontrol_2_0();
}

/**
* @brief �Զ��ܹ�3.0
* @param void
* @return void
*ǰ��ս���ʱ�ڹ���Ҳ�ֹͣ�ƶ���ǰ��ս�����ٺ�ϸ���ȫ��Ѳ����������ģʽ���������״̬�����ܹ����
*/
static void Chassis_AUTO_3_0()
{
    Outpost_get();//��ȡǰ��վ״̬���ı����ģʽ
    if(Chassis_process.init_flag)
    {
        if(Chassis_process.Fire == FIRE_ALL)
        {
            Static_shoot();
        } else
        {
            Chassis_GetMode(); //��ȡ״̬
            if( Chassis_process.Mode == CHASSIS_NORMAL)
            {
                Cruise_Normal();
            }
            else {
                Show_Time();
            }
        }
        Chassis_Rebound();
    }
    else
    {
        Cruise_First();
    }
    Chassis_AUTOcontrol_3_0();
}

/**
* @brief �Լ����
* @param void
* @return void
*
*/
static void Chassis_INSPECTIONcontrol()
{
    Chassis_process.PPM.kp = 1;
    Chassis_process.PPM.out_max = 3000;

    Chassis_process.PPM.target = Chassis_process.Spot_taget;
    Chassis_process.PPM.measure = path_sensor.info->mileage_total;
    pid_calculate(&Chassis_process.PPM);

    if( abs(Chassis_process.Spot_taget - path_sensor.info->mileage_total) < 100 )
    {
        Chassis_process.Spot_taget = path_sensor.info->mileage_total;
//            Chassis_process.Speed_taget = Chassis_process.PPM.out;
        Chassis_process.Speed_taget = 0;
    } else
    {
        Chassis_process.Speed_taget = Chassis_process.PPM.out;

        if( (Chassis_process.Spot_taget - path_sensor.info->mileage_total) > 0 )
        {
            Chassis_process.Derection_flag = 1;
        } else if( (path_sensor.info->mileage_total - Chassis_process.Spot_taget) > 0 )
        {
            Chassis_process.Derection_flag = -1;
        }

        if(Chassis_process.Derection_flag == 1)//����
        {
            Chassis_process.Speed_taget = constrain(Chassis_process.Speed_taget, 0, 2000);
        } else if(Chassis_process.Derection_flag == -1)//����
        {
            Chassis_process.Speed_taget = constrain(Chassis_process.Speed_taget, -2000, 0);
        }//���ٶ�ƫ�ã���ֹ�ٶ�̫��ͣ����

    }

    Chassis_process.PVM.target = Chassis_process.Speed_taget;
    Chassis_process.PVM.measure = motor[CHASSIS].info->speed;
    pid_calculate(&Chassis_process.PVM);

    NormalData_0x200[0] = (int16_t)Chassis_process.PVM.out;
}
/**
* @brief �����Լ�
* @param void
* @return void
*�����Լ죬���ڼ��΢�����غͱ������Ƿ�������������������
*/
static void Chassis_INSPECTION()
{
    static bool right_judge,left_judge;

    if(path_sensor.info->left_touch)//�����󿪹�
    {
        left_judge = true;
    }
    if(path_sensor.info->right_touch)//�����ҿ���
    {
        right_judge = true;
    }

    if(left_judge == true)
    {
        if(path_sensor.info->left_touch == false)//�󿪹��ɿ�
        {
            Chassis_process.Spot_taget = path_sensor.info->mileage_total + 4000;
            left_judge = false;
        }
    } else if(right_judge == true)
    {
        if(path_sensor.info->right_touch == false)//�ҿ����ɿ�
        {
            Chassis_process.Spot_taget = path_sensor.info->mileage_total - 4000;
            right_judge = false;
        }
    }
    Chassis_INSPECTIONcontrol();
}
/**
* @brief ���̵Ĺ��ʻ�ģʽ����
* @param void
* @return void
* @berif  K_limit = (Jʣ��/J���)��  Out_final = K_limit^2*Out_PID
*/
float K_limit = 1;
static void Chassis_Power_Control()
{
    static int16_t Cold_time = 0;
    if(judge_sensor.info->power_heat_update) //�������ݸ�������ʱ���й��ʿ��ƣ�����Ƶ��50hz
    {
        if(judge_sensor.info->PowerHeatData.chassis_power_buffer < PowerBuff_Hot)
        {
            Cold_time = 0;
            Chassis_process.overbuff_flag = true;
            K_limit =(float)(judge_sensor.info->PowerHeatData.chassis_power_buffer / PowerBuff_Hot);
            Chassis_process.PVM.out_max = PowerLimit_Normal * K_limit * K_limit;
        }
        else
        {
            Cold_time++;
            if(Cold_time >= 100)
            {
                Cold_time = 0;
                Chassis_process.overbuff_flag = false;
                Chassis_process.PVM.out_max = PowerLimit_Normal;
                K_limit = 1;
            }
        }
        judge_sensor.info->power_heat_update = false;
    }
//	else  //û��������
//	{
//		Chassis_process.PVM.out_max = PowerLimit_Normal;
//		//K_limit = 1;
//	}
}

/**
* @brief �Զ�ģʽ��ȡ
* @param void
* @return void
*���ھ���ʹ�������Զ�ģʽ����ң��������
*/
static void Chassis_AutoWayGet()
{
    if(rc_sensor.info->s1 == 3)
        Chassis_process.Way = WAY_NORMAL;
    else if(rc_sensor.info->s1 == 1)
        Chassis_process.Way = WAY_TOUCH;
    else if(rc_sensor.info->s1 == 2)
        Chassis_process.Way = WAY_ENCODER;
}

/**
* @brief �Զ�ģʽ��ȡ
* @param void
* @return void
*���ھ���ʹ�������Զ�ģʽ����ң��������
*/
static void Aerial_Control()
{
    static int8_t record_dir;

    Chassis_process.swerve_judge = false;
    Chassis_process.swerve_flag = false;//���Ѳ��ģʽʱ�ķ�������
	
	Chassis_process.static_want = false;//�����ֹ��־λ

    if(judge_sensor.info->AerialData.control_dir != record_dir)
    {
        if(judge_sensor.info->AerialData.control_dir == -1)
            Chassis_process.Speed_taget = -2000;
        else if(judge_sensor.info->AerialData.control_dir == 1)
            Chassis_process.Speed_taget = 2000;
    }

    if(judge_sensor.info->AerialData.control_dir == -1)
    {
        Chassis_process.Speed_taget -= 5;
    } else if(judge_sensor.info->AerialData.control_dir == 1)
    {
        Chassis_process.Speed_taget += 5;
    }

    record_dir = judge_sensor.info->AerialData.control_dir;

    Chassis_process.Speed_taget = constrain(Chassis_process.Speed_taget, -SPEED_MAX, SPEED_MAX);

    Chassis_process.PVM.target = Chassis_process.Speed_taget;
    Chassis_process.PVM.measure = motor[CHASSIS].info->speed;
    pid_calculate(&Chassis_process.PVM);
    NormalData_0x200[0] = Chassis_process.PVM.out;
}

/**
* @brief �����Զ�ģʽ�ļ���
* @param void
* @return void
*�������Զ�����̨�ֲٿص�����
*/
static void AUTO_ALL()
{
    if( Aerial_Control_info() &&  Chassis_process.init_flag )
    {
        Aerial_Control();
    } else
    {
//        Chassis_AutoWayGet();
//        if( Chassis_process.Way == WAY_NORMAL )
//            Chassis_AUTO_3_0();	//����ܹ�
//        else if( Chassis_process.Way == WAY_TOUCH )
//            Chassis_AUTO_TOUCH(); //�������ܹ�
//        else if( Chassis_process.Way == WAY_ENCODER )
//            Chassis_AUTO_ENCODER();//���������ܹ�

                Chassis_AUTO();//ң��
    }
}
/* Exported functions --------------------------------------------------------*/

/**
* @brief ���̲�����ʼ��
* @param void
* @return void
* @berif  ֱ��һ��
*/
void Chassis_Init()
{
    Chassis_process.PPM.kp = 0.4;
    Chassis_process.PPM.ki = 0;//0.001
    Chassis_process.PPM.kd = 0;
    Chassis_process.PPM.integral_max = 1600;
    Chassis_process.PPM.out_max = 5000;

    Chassis_process.PVM.kp = 12;//10
    Chassis_process.PVM.ki = 0.05;//0.02
    Chassis_process.PVM.kd = 0;
    Chassis_process.PVM.integral_max = 8000;
    Chassis_process.PVM.out_max = 12000;
    Chassis_process.init_flag = false;
    Chassis_process.Derection_flag = -1;//��ʼ����
    Chassis_process.rotate_ratio = 8;
    Chassis_process.Trip_times = 1;

    Chassis_process.Fire = FIRE_ALL;;
}
/**
* @brief ��������
* @param void
* @return void
* @berif  2ms
*/
void StartChassisTask(void const * argument)
{
    for(;;)
    {
        if(sys.state == SYS_STATE_NORMAL)
        {
            if(sys.remote_mode == RC)
            {
                Chassis_RCcontrol();
            }
            else if(sys.remote_mode == AUTO)
            {
                AUTO_ALL();
            } else if(sys.remote_mode == INSPECTION)
            {
                Chassis_INSPECTION();
            }
            Chassis_Power_Control();
        } else
        {
            Chassis_process.init_flag = false;
        }
        osDelay(2);
    }
}


