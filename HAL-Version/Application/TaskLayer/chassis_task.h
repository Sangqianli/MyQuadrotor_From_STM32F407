#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "system_task.h"
/* Exported macro ------------------------------------------------------------*/
#define HP_Danger   300
/* Exported types ------------------------------------------------------------*/
typedef enum {
    FIRE_ALL,
    FIRE_RUN,
} chassis_fire_t; //������������ģʽ

typedef enum {
    CHASSIS_SAFE,
    CHASSIS_HURT,
    CHASSIS_DANGER,
} chassis_safe_mode_t; //�����Ƿ�ȫ��״̬��־λ

typedef enum {
    CHASSIS_NORMAL,	// ���ģʽ
    CHASSIS_ATTACK, //���ģʽ
    CHASSIS_COVER,  //�ڻ�ģʽ
    CHASSIS_ESCAPE, //����ģʽ
} chassis_mode_t;

typedef enum {
    WAY_NORMAL,	// ����ģʽ
    WAY_TOUCH,   //����ģʽ
    WAY_ENCODER,   //������ģʽ
} chassis_way_t;

typedef struct Chassis {
    pid_ctrl_t	 PPM;
    pid_ctrl_t	 PVM;
    float        Speed_taget;
    int32_t		 Mileage_atrip;//�������
    uint8_t		 init_flag;
    int8_t       Derection_flag;//1Ϊ��-1Ϊ��
    int16_t      Trip_times;   //������������
    uint8_t      rotate_ratio;//ң��������
    bool         swerve_judge;//�����Ƿ���ɵ��ж�
    bool         swerve_flag;//�������̱�־λ
    bool         overbuff_flag;//������������־λ
    bool         getchange_flag;//��ȡ���Ŀ��λ�ñ�־λ
    bool         static_want;//����Ŀ�꾲ֹ
    int32_t      Spot_taget;//���Ŀ��λ��
    chassis_mode_t  Mode ; //����ģʽ
    chassis_safe_mode_t Safe;
    chassis_fire_t Fire;
    chassis_way_t Way;
} Chassis_t;
/* Exported functions --------------------------------------------------------*/
extern Chassis_t Chassis_process;
void   Chassis_Init(void);
void   StartChassisTask(void const * argument);
#endif
