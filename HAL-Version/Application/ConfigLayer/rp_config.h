#ifndef __RP_CONFIG_H
#define __RP_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "arm_math.h"
#include "stdlib.h"
/* Exported macro ------------------------------------------------------------*/
/* ʱ��� */

#define		TIME_STAMP_250MS	250
#define 	TIME_STAMP_500MS	500
#define    	TIME_STAMP_400MS    400
#define		TIME_STAMP_5000MS	5000

/* Exported types ------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
 *	@brief	��������
 *	@class	driver
 */
typedef enum drv_type{
	DRV_CAN1,
	DRV_CAN2,
	DRV_PWM_FRIC_L,
	DRV_PWM_FRIC_R,
	DRV_PWM_SERVO,
	DRV_IIC,
	DRV_ENCODER_AND_IOIN,
	DRV_UART1,
	DRV_UART2,
	DRV_UART3,
	DRV_UART4,
	DRV_UART5,
} drv_type_t;

/**
 *	@brief	iic����
 *	@class	driver
 */
typedef struct drv_iic {
	enum drv_type 	type;	
} drv_iic_t;

/**
 *	@brief	can����
 *	@class	driver
 */
typedef struct drv_can {
	enum drv_type 	type;
	uint32_t		can_id;
	uint32_t		std_id;
	uint8_t			drv_id;
	void			(*tx_data)(struct drv_can *self, int16_t txData);
} drv_can_t;

/**
 *	@brief	pwm����
 *	@class	driver
 */
typedef struct drv_pwm {
	enum drv_type	type;
	void			(*output)(struct drv_pwm *self, int16_t pwm);
} drv_pwm_t;

/**
 *	@brief	uart����
 *	@class	driver
 */
typedef struct drv_uart {
	enum drv_type	type;
	void			(*tx_byte)(struct drv_uart *self, uint8_t *byte,uint16_t size);
} drv_uart_t;
/**
 *	@brief	TIM+IO����(��ʵû����)
 *	@class	driver
 */
typedef struct drv_path {
	enum drv_type 	type;	
} drv_path_t;

/* �豸�� --------------------------------------------------------------------*/
/**
 *	@brief	�豸id�б�
 *	@class	device
 */
typedef enum {
	DEV_ID_RC = 0,
	DEV_ID_IMU = 1,
	DEV_ID_CHASSIS = 2,
	DEV_ID_DIAL = 3,
	DEV_ID_GIMBAL_PITCH = 4,
	DEV_ID_GIMBAL_YAW = 5,
	DEV_ID_ENCODER_AND_TOUCH = 6,
	DEV_ID_VISION = 7,
	DEV_ID_JUDJE = 8,
	DEV_ID_CNT = 9,
} dev_id_t;

/**
 *	@brief	����豸����
 *	@class	device
 */
typedef enum {
	CHASSIS,
	DIAL,
	GIMBAL_PITCH,
	GIMBAL_YAW,
	MOTOR_CNT,
} motor_cnt_t;

/**
 *	@brief	�豸����״̬
 *	@class	device
 */
typedef enum {
	DEV_ONLINE,
	DEV_OFFLINE,
} dev_work_state_t;

/**
 *	@brief	�������
 *	@class	device
 */
typedef enum {
	NONE_ERR,		// ����(�޴���)
	DEV_ID_ERR,		// �豸ID����
	DEV_INIT_ERR,	// �豸��ʼ������
	DEV_DATA_ERR,	// �豸���ݴ���
} dev_errno_t;

/**
 *	@brief	�豸�ṹ�嶨��ģ��
 *	@class	device
 */
typedef struct device {
	void				*info;		// �Զ�������豸��Ϣ�ṹ��
	void				*driver;	// �Զ�������豸�����ṹ��
	void				(*init)(struct device *self);	// �豸��ʼ������
	void				(*update)(struct device *self, uint8_t *rxBuf);	// �豸���ݸ��º���
	void				(*check)(struct device *self);	// �豸���ݼ�麯��
	void				(*heart_beat)(struct device *self);	// �豸������
	dev_work_state_t	work_state;	// �豸����״̬
	dev_errno_t			errno;		// ���Զ�������豸�������
	dev_id_t			id;			// �豸id
} device_t;

/* Ӧ�ò� --------------------------------------------------------------------*/
/**
 *	@brief	pid������
 *	@class	controller
 */
typedef struct pid_ctrl {
	float	target;
	float	measure;
	float 	err;
	float 	last_err;
	float	kp;
	float 	ki;
	float 	kd;
	float 	pout;
	float 	iout;
	float 	dout;
	float 	out;
	float	integral;
	float 	integral_max;
	float 	out_max;
} pid_ctrl_t;

typedef struct pid2_ctrl {
	float 	err;
	float 	last_err;
	float	kp;
	float 	ki;
	float 	kd;
	float 	pout;
	float 	iout;
	float 	dout;
	float 	out;
	float	integral;
	float 	integral_max;
	float 	out_max;
} pid2_ctrl_t;
/* Remote Mode Enum */
typedef enum {
	ATTITUDE = 0,  //��̬ģʽ���ֶ�����
	STEADY = 1,  //����ģʽ�����߶��㣬�вٿ�
	AUTO = 2,  //�Զ�ģʽ�����Զ���ͣ
	REMOTE_MODE_CNT = 3,
} remote_mode_t;

typedef enum {
	SYS_STATE_NORMAL,	// ϵͳ����
	SYS_STATE_RCLOST,	// ң��ʧ��
	SYS_STATE_RCERR,	// ң�س���
	SYS_STATE_WRONG,	// ����ϵͳ����
} sys_state_t;

typedef enum{
	RCDATA_NOMAL =0,
	RC_OFFLINE =1,
	RCDATA_ERR=2,
	RC_WRONG=3,
}rc_event_t;
typedef struct {
	
	bool  SYS_RESET;
	bool  REMOTE_SWITCH;
	bool  AUTO_MODE_SWITCH;
	bool  RESET_CAL;
	bool  ALL_READY;
}sys_event_t;

typedef struct {
	sys_state_t			state;			// ϵͳ״̬
	sys_event_t         event;   //��Ӧ�¼�
	remote_mode_t		remote_mode;	// ���Ʒ�ʽ
	
	void (*sys_action)(void); //ϵͳ������
} system_t;

typedef struct{
	sys_state_t cur_state;//��ǰ״̬
	sys_state_t next_state;//��һ��״̬
	system_t *sys_table;//״̬��
	int size;	//�������
}fsm_t;//״̬��

//extern flag_t	flag;
extern system_t sys;

/* Exported functions --------------------------------------------------------*/
#define RP_SET_BIT(x,n)    (x | 1U<<(n-1))
#define RP_CLEAR_BIT(x,n)    (x & ~(1U<<(n-1)))
#define RP_SET_BITS(x,n,m)    (x | ~(~0U<<(m-n+1))<<(n-1)) 
#define RP_GET_BIT(x,n,m)    (x & ~(~0U<<(m-n+1))<<(n-1)) >>(n-1)
#define GETBITS_N_M(UA, BIT_M, BIT_N)   ( (UA & ~ (~ ((typeof (UA))0U) << (BIT_N - BIT_M + 1)) << BIT_M) >> BIT_M) //ȡ��M��Nλ


//����Ϊ��ຯ��
void WFI_SET(void);		//ִ��WFIָ��
void INTX_DISABLE(void);//�ر������ж�
void INTX_ENABLE(void);	//���������ж�
void MSR_MSP(uint32_t addr);	//���ö�ջ��ַ 

#endif
