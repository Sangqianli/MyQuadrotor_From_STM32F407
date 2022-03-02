/**
 * @file        monitor_task.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-November-2020
 * @brief       Monitor&Test Center
 */

/* Includes ------------------------------------------------------------------*/
#include "vision_task.h"
#include "device.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
#define ACTIVE_MAX_CNT  1
#define LOST_MAX_CNT    4	/*����ʶ��Ͷ�ʧ�ж�����ֵ*/
#define CONVER_SCALE_YAW    22.463f//20.86
#define CONVER_SCALE_PITCH  22.26f//22.9
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float vision_mach_yaw,vision_mach_pitch,vision_dis_meter;//�Ӿ�����ת��

extKalman_t kalman_visionYaw,kalman_targetYaw,kalman_visionPitch,kalman_targetPitch,kalman_visionDistance,kalman_targetDistance;
extKalman_t kalman_accel,kalman_speedYaw;
float visionYaw_R=0,targetYaw_R=400,visionPitch_R=0,targetPitch_R=100,visionDis_R=1,targetDis_R=100;//1,1000,1,1000
float predictAccel_R=10,speedYaw_R=100;//1 , 200

uint8_t Vision_SentData[60];//���͸��Ӿ�������

float YawTarget_now,PitchTarget_now;//ʵ���Ӿ������Ƕ�
float update_cloud_yaw = 0,update_cloud_pitch=0;	/*��¼�Ӿ���������ʱ����̨���ݣ����´ν�����*/
float lastupdate_cloud_yaw = 0,lastupdate_cloud_pitch=0; /*ǰ��֡������*/
/* Exported variables --------------------------------------------------------*/
Vision_process_t Vision_process;
/* Private functions ---------------------------------------------------------*/
static void Sent_to_Vision_Version2_1()
{
    static uint8_t Sent_cnt=0;//���ͼ��
    static uint32_t now_time;
    static uint8_t  colour;
    uint8_t *yw,*pw;//����Ƕ�ָ��
    static float Yaw_raw,Pitch_raw;
    Yaw_raw=((Gimbal_process.YAW_PPM.measure)/8191)*360;
    Pitch_raw=((Gimbal_process.PITCH_PPM.measure)/8191)*360;//����Ƕ�ת��
//    uint8_t *time;
//    now_time=xTaskGetTickCount();
//    time=(uint8_t*)&now_time;
    yw = (uint8_t*)&Yaw_raw;
    pw = (uint8_t*)&Pitch_raw;


    if(judge_sensor.info->GameRobotStatus.robot_id == 7)//��ɫid
        colour = 1;
    else if(judge_sensor.info->GameRobotStatus.robot_id == 107)//��ɫid
        colour = 0;

    Append_CRC8_Check_Sum(Vision_SentData, 3);
    Append_CRC16_Check_Sum(Vision_SentData,22);

    Vision_SentData[0] = 0xA5;
    Vision_SentData[1] = 0;
    /*С�˷��ͣ����ֽ��Ǹ�λ*/
    Vision_SentData[3] = colour;//��ɫʶ��1����ɫ��0�Ǻ�ɫ
    Vision_SentData[4] = 0;//1�����ݣ�0�ط�����

    /*С�˷��ͣ����ֽ��Ǹ�λ*/
    Vision_SentData[5]= *yw;
    Vision_SentData[6]=*(yw+1);
    Vision_SentData[7]=*(yw+2);
    Vision_SentData[8]=*(yw+3);//Yaw��Ƕ�����

    Vision_SentData[9]= *pw;
    Vision_SentData[10]=*(pw+1);
    Vision_SentData[11]=*(pw+2);
    Vision_SentData[12]=*(pw+3);//Pitch��Ƕ�����

    Sent_cnt++;
    if(Sent_cnt>=100)
    {
        UART1_SendData(Vision_SentData,23);
        Sent_cnt=0;
    }
}

static void Offset_Angle_Get()
{
    if(Vision_process.data_kal.DistanceGet_KF > 1.25f && Vision_process.data_kal.DistanceGet_KF <= 3.25f)
        Vision_process.offset_pitch = 1.6;
    else if(Vision_process.data_kal.DistanceGet_KF > 3.25f && Vision_process.data_kal.DistanceGet_KF <= 4.25f)
        Vision_process.offset_pitch = 1.83;
    else if(Vision_process.data_kal.DistanceGet_KF > 4.25f && Vision_process.data_kal.DistanceGet_KF <= 5.25f)
        Vision_process.offset_pitch = 2.4f;
    else if(Vision_process.data_kal.DistanceGet_KF > 5.25f && Vision_process.data_kal.DistanceGet_KF <= 6.f)
        Vision_process.offset_pitch = 2.6;
    else
        Vision_process.offset_pitch = 0;

    Vision_process.offset_yaw = 0.2;//0.8
}

static void Offset_Angle_Get_2_1()
{
    if(Vision_process.data_kal.DistanceGet_KF > 0.5f && Vision_process.data_kal.DistanceGet_KF <= 3.5f)
        Vision_process.offset_pitch = 0.7954f * Vision_process.data_kal.DistanceGet_KF - 0.1428f + 1.f;
    else if(Vision_process.data_kal.DistanceGet_KF > 3.5f &&  Vision_process.data_kal.DistanceGet_KF <= 7.f)// <= 6.25f
        Vision_process.offset_pitch = 0.1827f * Vision_process.data_kal.DistanceGet_KF + 2.0095f + 1.f;//���һ�����ֶ�����1.3
    else
        Vision_process.offset_pitch = 0;

    Vision_process.offset_yaw = 0.9f;//-0.6

//	    Vision_process.offset_yaw = 0;
//	  Vision_process.offset_pitch = 0;
}

static void Vision_Normal()
{
    static uint16_t active_cnt=0,lost_cnt=0;/*�������/��ʧ����--����ʶ���δʶ���໥�л��Ĺ���*/
    static int16_t  Record_Auto_Mode = AUTO_MODE_SCOUT;
    static int32_t pitch_temp;//�����ж�Ŀ��Ƕ��Ƿ񳬳�pitch��λ�������򲻽�������

    Offset_Angle_Get_2_1();//���ݾ����ȡ������


    if(vision_sensor.info->State.rx_data_update)//���ݸ���
    {
        pitch_temp = lastupdate_cloud_pitch + (vision_sensor.info->RxPacket.RxData.pitch_angle + Vision_process.offset_pitch) * CONVER_SCALE_PITCH;	//�����pitch��е�Ƕ�
        if(Record_Auto_Mode != sys.auto_mode)
        {
            sys.switch_state.AUTO_MODE_SWITCH = true;
        }
        Record_Auto_Mode = sys.auto_mode;

        if( (vision_sensor.info->RxPacket.RxData.identify_target == 1 ) && ( (vision_dis_meter>0.3f) && (vision_dis_meter < ANTI_DISTANDCE) )  &&( Fire_Key_info() == false)  ) // &&( (pitch_temp <= PITCH_UP_LINE)&&(pitch_temp >= PITCH_DOWN_LINE) )
        { 
            active_cnt++; 	/*��Ծ����*/
            if( active_cnt >= ACTIVE_MAX_CNT ) /*�ﵽ��ֵ���϶�Ϊʶ��*/
            {
                sys.auto_mode = AUTO_MODE_ATTACK;
                active_cnt = 0;
                lost_cnt = 0;
                /*����ȷ�Ͻ������ж�*/
            }
        }
        else
        {
            lost_cnt++;  
            if(lost_cnt >= LOST_MAX_CNT) /*�ﵽ��ֵ���϶�Ϊ��ʧ*/
            {
                sys.auto_mode = AUTO_MODE_SCOUT;
                active_cnt = 0;
                lost_cnt = 0;
                /*���ģʽ���л�*/
                Clear_Queue(&Vision_process.speed_queue);
                Clear_Queue(&Vision_process.accel_queue);
                Clear_Queue(&Vision_process.dis_queue);
                /*���������Ϣ����ֹ��һ�������ܵ�Ӱ��*/
                Vision_process.data_kal.DistanceGet_KF = 0;//��ȷ���¾���
                Vision_process.feedforwaurd_angle = 0;
                Vision_process.predict_angle = 0;//��0Ԥ���
                sys.predict_state.PREDICT_OPEN = false;			/*�ر�Ԥ��*/
            }
        }

        if(vision_sensor.info->RxPacket.RxData.anti_gyro == 1)
        {
//            Vision_process.gyro_anti = true;
            Vision_process.gyro_anti = false;
        }
        else
        {
            Vision_process.gyro_anti = false;
//            Vision_process.gyro_judge = true;  //�´��ٽ��뷴����ʱ���ж�
        }

        if(Vision_process.gyro_anti == false) //û�����ݵ�ʱ����г������
        {
            vision_mach_yaw = (vision_sensor.info->RxPacket.RxData.yaw_angle + Vision_process.offset_yaw) * CONVER_SCALE_YAW;
            vision_mach_pitch = (vision_sensor.info->RxPacket.RxData.pitch_angle + Vision_process.offset_pitch) * CONVER_SCALE_PITCH;		//���ϲ����ǣ�ת���ɻ�е�Ƕ�
            vision_dis_meter =  vision_sensor.info->RxPacket.RxData.distance/1000.f;

//            vision_mach_yaw  = 	DeathZoom(vision_mach_yaw,0,0.1);
//            vision_mach_pitch=  DeathZoom(vision_mach_pitch,0,0.1);
        }
        Vision_process.data_kal.YawGet_KF = KalmanFilter(&kalman_visionYaw,vision_mach_yaw); 	/*���Ӿ��Ƕ��������������˲�*/
        Vision_process.data_kal.PitchGet_KF = KalmanFilter(&kalman_visionPitch,vision_mach_pitch);
        if( (vision_dis_meter>0.3f) && (vision_dis_meter<10.f) )
            Vision_process.data_kal.DistanceGet_KF =KalmanFilter(&kalman_visionDistance,vision_dis_meter);

        YawTarget_now=lastupdate_cloud_yaw+Vision_process.data_kal.YawGet_KF;
        PitchTarget_now=lastupdate_cloud_pitch+Vision_process.data_kal.PitchGet_KF;

//		YawTarget_now=update_cloud_yaw+Vision_process.data_kal.YawGet_KF;
//        PitchTarget_now=update_cloud_pitch+Vision_process.data_kal.PitchGet_KF;

        lastupdate_cloud_yaw = update_cloud_yaw;
        lastupdate_cloud_pitch = update_cloud_pitch;      /*��¼ǰ2֡������*/
        update_cloud_yaw = Gimbal_process.YAW_PPM.measure;/*�Ӿ����ݸ���ʱ����̨�Ƕ�*/
        update_cloud_pitch = Gimbal_process.PITCH_PPM.measure;

        /*�Ӿ���������.......................................*/
        Vision_process.speed_get = Get_Diff(3,&Vision_process.speed_queue,YawTarget_now);//20
        Vision_process.speed_get = 30 * (Vision_process.speed_get/vision_sensor.info->State.rx_time_fps); //ÿ����
        Vision_process.speed_get = KalmanFilter(&kalman_speedYaw,Vision_process.speed_get);
//      Vision_process.speed_get = DeathZoom(Vision_process.speed_get,0,1);
        Vision_process.speed_get = constrain(Vision_process.speed_get, -40, 40);

        Vision_process.accel_get = Get_Diff(3,&Vision_process.accel_queue,Vision_process.speed_get);	 /*�°��ȡ���ٶ�10*/
        Vision_process.accel_get = 20 * (Vision_process.accel_get/vision_sensor.info->State.rx_time_fps);//ÿ����
        Vision_process.accel_get = KalmanFilter(&kalman_accel,Vision_process.accel_get);
//      Vision_process.accel_get = DeathZoom(Vision_process.accel_get,0,0.1);		/*�������� - �˳�0�㸽��������*/
        Vision_process.accel_get = constrain(Vision_process.accel_get, -30, 30);

        Vision_process.distend_get =  Get_Diff(5,&Vision_process.dis_queue,Vision_process.data_kal.DistanceGet_KF);
        /*......................................................*/
        vision_sensor.info->State.rx_data_update = false;
    }
}

static void Vision_Pridict()
{
    static float acc_use = 1.f;
    static float predic_use = 2.f;
    float dir_factor;
    if( (Vision_process.speed_get * Vision_process.accel_get)>=0 )
    {
        dir_factor= 1.f;//1
    }
    else
    {
        dir_factor= 2.f;//1.5
    }

    Vision_process.feedforwaurd_angle = acc_use * Vision_process.accel_get; 	/*����ǰ����*/

    Vision_process.predict_angle = predic_use * (1.f*Vision_process.speed_get*Vision_process.data_kal.DistanceGet_KF+1.f*dir_factor*Vision_process.feedforwaurd_angle*Vision_process.data_kal.DistanceGet_KF) ;//�ٶ�1.1�����ٶ�3
    Vision_process.predict_angle = constrain(Vision_process.predict_angle, -120, 120);
}
static void AntiNormal()
{
    /*ֱ�Ӹ������.................................*/
//	    Vision_process.data_kal.YawTarget_KF=KalmanFilter(&kalman_targetYaw,YawTarget_now);
//	    Vision_process.data_kal.PitchTarget_KF=KalmanFilter(&kalman_targetPitch,PitchTarget_now);
    /*..................................................*/

    Vision_process.data_kal.YawTarget_KF=YawTarget_now+Vision_process.predict_angle;
    Vision_process.data_kal.YawTarget_KF=KalmanFilter(&kalman_targetYaw,Vision_process.data_kal.YawTarget_KF);
//    Vision_process.data_kal.PitchTarget_KF=PitchTarget_now+10*Vision_process.distend_get;                               //�������pitch��Ԥ���ϵ��
//    Vision_process.data_kal.PitchTarget_KF=KalmanFilter(&kalman_targetPitch,Vision_process.data_kal.PitchTarget_KF);
    Vision_process.data_kal.PitchTarget_KF=KalmanFilter(&kalman_targetPitch,PitchTarget_now);//����Ԥ��
}
static void AntiGyro()
{
//    static float anti_yaw, anti_pitch, record_yaw, record_pitch;
//    static bool judge_flag = true;
//    anti_yaw = (vision_sensor.info->RxPacket.RxData.yaw_angle + Vision_process.offset_yaw) * CONVER_SCALE_YAW;
//    anti_pitch = (vision_sensor.info->RxPacket.RxData.pitch_angle + Vision_process.offset_pitch) * CONVER_SCALE_PITCH;

//    if(judge_flag)
//    {
//        Vision_process.data_kal.YawTarget_KF = anti_yaw + update_cloud_yaw;
//        Vision_process.data_kal.PitchTarget_KF = anti_pitch + update_cloud_pitch;
//        Clear_Queue(&Vision_process.speed_queue);
//        Clear_Queue(&Vision_process.accel_queue);
//        Clear_Queue(&Vision_process.dis_queue);
//        judge_flag = false;
//    }
//    if( (abs(record_yaw-anti_yaw)<0.1f) && (abs(record_pitch-anti_pitch)<0.1f) )
//    {
//        judge_flag = false;
//    } else
//    {
//        judge_flag = true;
//    }

//    record_yaw = anti_yaw;
//    record_pitch = anti_pitch;
    /*ֱ�Ӹ������.................................*/
    Vision_process.data_kal.YawTarget_KF=KalmanFilter(&kalman_targetYaw,YawTarget_now);
    Vision_process.data_kal.PitchTarget_KF=KalmanFilter(&kalman_targetPitch,PitchTarget_now);
    /*..................................................*/
    Vision_process.data_kal.YawTarget_KF=KalmanFilter(&kalman_targetYaw,Vision_process.data_kal.YawTarget_KF);
    Vision_process.data_kal.PitchTarget_KF=KalmanFilter(&kalman_targetPitch,PitchTarget_now);//����Ԥ��
}

static void Anti_Target()
{
    if(Vision_process.gyro_anti)
        AntiGyro();
    else
        AntiNormal();
}

/* Exported functions --------------------------------------------------------*/
void Vision_Init()
{
    KalmanCreate(&kalman_visionYaw,1,visionYaw_R);
    KalmanCreate(&kalman_targetYaw,1,targetYaw_R);
    KalmanCreate(&kalman_visionPitch,1,visionPitch_R);
    KalmanCreate(&kalman_targetPitch,1,targetPitch_R);
    KalmanCreate(&kalman_visionDistance,1,visionDis_R);
    KalmanCreate(&kalman_targetDistance,1,targetDis_R);
    KalmanCreate(&kalman_accel,1,predictAccel_R);
    KalmanCreate(&kalman_speedYaw,1,speedYaw_R);

    Vision_process.speed_queue.queueLength = 60;
    Vision_process.accel_queue.queueLength = 60;
    Vision_process.dis_queue.queueLength = 60;

}

void StartVisionTask(void const * argument)
{
    for(;;)
    {
        if( (sys.state == SYS_STATE_NORMAL) && (sys.switch_state.ALL_READY) )
        {
            Vision_Normal();
            if(sys.predict_state.PREDICT_OPEN)
            {
                Vision_Pridict();
            }
            Anti_Target();
            Sent_to_Vision_Version2_1();
        }
        osDelay(2);
    }
}
