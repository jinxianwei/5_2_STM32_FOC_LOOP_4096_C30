/*
***********************************************************************
 * 文件名：tim_drv.c
 * 作  者：???
 * 日  期：2024/06/20
 * 说  明：定时器任务
***********************************************************************
*/

#ifndef __TIM_DRV_H__
#define __TIM_DRV_H__

#include "main.h"

typedef struct
{
    volatile int32_t 	EncoderCnt;       //编码器计数
    int32_t 	EncoLastCnt;
    uint16_t 	SpeedClock;
}ENCOSPEED;
extern ENCOSPEED EnCoSpeed;
extern volatile uint32_t EncoderSpiErrorCount;


void MotorAngleGet(void);
void MotorSpeedGet(void);
uint8_t MotorEncoderCalibrateElectricalZero(void);

#endif 
