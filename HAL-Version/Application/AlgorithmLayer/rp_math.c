/**
 * @file        rp_math.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        11-September-2020
 * @brief       RP Algorithm.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "rp_math.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
* @brief б�º���
* @param void
* @return void
*/
int16_t RampInt(int16_t final, int16_t now, int16_t ramp)
{
	int32_t buffer = 0;
	
	buffer = final - now;
	if (buffer > 0)
	{
		if (buffer > ramp)
			now += ramp;
		else
			now += buffer;
	}
	else
	{
		if (buffer < -ramp)
			now += -ramp;
		else
			now += buffer;
	}
	return now;
}

float RampFloat(float step,float target,float current)
{
    float tmp;
    if(abs(current - target)>abs(step))        //step�Ǿ���ֵ
    {
        if(current < target)
            tmp = current + step;
        else
            tmp = current - step;
    }
    else
    {
        tmp = target;
    }
    return tmp;
}
/**
* @brief ��������
* @param void
* @return void
*/
float DeathZoom(float input, float center, float death)
{
	if(abs(input - center) < death)
		return center;
	return input;
}

/**
* @brief ��ȡĿ��Ĳ��
* @param void
* @return void
*	�Զ��е��߼�
*/
float Get_Diff(uint8_t queue_len, QueueObj *Data,float add_data)
{
    if(queue_len>=Data->queueLength)
        queue_len=Data->queueLength;
    //��ֹ���
    Data->queueTotal-=Data->queue[Data->nowLength];
    Data->queueTotal+=add_data;

    Data->queue[Data->nowLength]=add_data;
	
    Data->nowLength++;

    if(Data->full_flag==0)//��ʼ����δ��
    {
        Data->aver_num=Data->queueTotal/Data->nowLength;
    }else if(Data->full_flag == 1)
	{
	    Data->aver_num=(Data->queueTotal)/queue_len;			
	}
    if(Data->nowLength>=queue_len)
    {
        Data->nowLength=0;
        Data->full_flag=1;
    }

    Data->Diff=add_data - Data->aver_num;
    return Data->Diff;
}
/**
* @brief ��ն���
* @param void
* @return void
*	�Զ��е��߼�
*/
void Clear_Queue(QueueObj* queue)
{
    for(uint16_t i=0; i<queue->queueLength; i++)
    {
        queue->queue[i]=0;
    }
    queue->nowLength = 0;
    queue->queueTotal = 0;
    queue->aver_num=0;
    queue->Diff=0;
    queue->full_flag=0;
}
