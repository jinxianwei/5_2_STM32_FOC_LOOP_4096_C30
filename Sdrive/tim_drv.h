/*
***********************************************************************
 * �ļ�����tim_drv.c
 * ��  �ߣ�???
 * ��  �ڣ�2024/06/20
 * ˵  ������ʱ������
***********************************************************************
*/

#ifndef __TIM_DRV_H__
#define __TIM_DRV_H__

#include "main.h"

typedef struct
{
    int32_t 	EncoderCnt;       //����������
    int32_t 	EncoLastCnt;
    uint16_t 	SpeedClock;
}ENCOSPEED;
extern ENCOSPEED EnCoSpeed;
extern volatile uint32_t EncoderSpiErrorCount;


void MotorAngleGet(void);
void MotorSpeedGet(void);

#endif 
