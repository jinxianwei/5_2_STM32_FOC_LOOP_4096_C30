/*
***********************************************************************
* �ļ�����task.c
* ��  �ߣ�???
* ��  �ڣ�2024/05/16
* ˵  ������������
***********************************************************************
*/
#include "task.h"

typedef struct
{
	uint8_t run;			   // ���ȱ�־��1�����ȣ�0������
	uint16_t timCount;		   // ʱ��Ƭ����ֵ
	uint16_t timRload;		   // ʱ��Ƭ����ֵ
	void (*pTaskFuncCb)(void); // ����ָ���������������ҵ����ģ�麯����ַ
} TaskComps_t;

static TaskComps_t g_taskComps[] =
	{
#if VR_OR_PC
		{0, 1, 1, VrTask}, // ��λ����������
#else
		{0, 10, 10, RecevToVofa}, // ������������
#endif
		{0, 5, 5, HmiTask},		 // ������������
		//{0, 10, 10, SendToVofa}, // ������������
		{0, 1, 1, FaultCheck},	 // ���ϼ������

		/* ����ҵ����ģ�� */
};

#define TASK_NUM_MAX (sizeof(g_taskComps) / sizeof(g_taskComps[0]))

/**
***********************************************************
* @brief ������Ⱥ���
* @param
* @return
***********************************************************
*/
static void TaskHandler(void)
{
	for (uint8_t i = 0; i < TASK_NUM_MAX; i++)
	{
		if (g_taskComps[i].run) // �ж�ʱ��Ƭ��־
		{
			g_taskComps[i].run = 0; // ��־����
			if (g_taskComps[i].pTaskFuncCb == NULL)
			{
				continue;
			}
			g_taskComps[i].pTaskFuncCb(); // ִ�е���ҵ����ģ��
		}
	}
}

/**
***********************************************************
* @brief �ڶ�ʱ���жϷ������б���ӵ��ã�����ʱ��Ƭ��ǣ�
		 ��Ҫ��ʱ��1ms����1���ж�
* @param
* @return
***********************************************************
*/
static void TaskScheduleCb(void)
{
	for (uint8_t i = 0; i < TASK_NUM_MAX; i++)
	{
		if (g_taskComps[i].timCount)
		{
			g_taskComps[i].timCount--;
			if (g_taskComps[i].timCount == 0)
			{
				g_taskComps[i].run = 1;
				g_taskComps[i].timCount = g_taskComps[i].timRload;
			}
		}
	}
}

/**
***********************************************************
* @brief ������ʼ��
* @param
* @return
***********************************************************
*/
static void DrvInit()
{
	// DelayInit();
	KeyDrvInit();
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); // receive interrrupt������ʱ�俪���ж�
	__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);	 // �����ж��ѿ����ͽ����ж�
	HAL_TIM_Base_Start_IT(&htim1);				 // ������ʱ��1�ж�
}

/**
***********************************************************
* @brief Ӧ�ó�ʼ��
* @param
* @return
***********************************************************
*/
static void AppInit()
{
	TaskScheduleCbReg(TaskScheduleCb);
}

/**
***********************************************************
* @brief ��ʼ��
* @param
* @return
***********************************************************
*/
void task_init(void)
{
	DrvInit();
	AppInit();

	MotorInit(); // ���������ʼ��

	HAL_Delay(100);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC_Value, 5);		  // ����ADC-DMAת��
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2); // ����������ʹ�ܺ���

	AdcSampleOffset(); // ���У׼

	Init_Over = 1;

	Motor.CtrlMode = TO_ZERO;
	if (!PostionToZeroDouble())
		Error_Handler();
	Motor.CtrlMode = STOP;
}

/**
***********************************************************
* @brief ������ѭ��
* @param
* @return
***********************************************************
*/
void task_loop(void)
{
	TaskHandler(); // �������
    SendToVofa(); // ������������
}
