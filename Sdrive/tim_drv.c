/*
***********************************************************************
* �ļ�����tim_drv.c
* ��  �ߣ�������
* ��  �ڣ�2024/06/20
* ˵  ������ʱ������
***********************************************************************
*/

#include "task.h"

#define TLE5012_COUNTS_PER_REVOLUTION 32768U
#define TLE5012_SPI_TIMEOUT_CYCLES (SystemCoreClock / 40000U)

extern SPI_HandleTypeDef hspi1;

ENCOSPEED EnCoSpeed;
volatile uint32_t EncoderSpiErrorCount;

static uint8_t TLE5012_WaitForFlag(uint32_t flag, uint8_t set, uint32_t start)
{
	while (((SPI1->SR & flag) != 0U) != (set != 0U))
	{
		if ((uint32_t)(DWT->CYCCNT - start) >= TLE5012_SPI_TIMEOUT_CYCLES)
			return 0U;
	}
	return 1U;
}

static uint8_t TLE5012_ReadRawAngle(uint16_t *rawAngle)
{
	const uint8_t command[2] = {0x80U, 0x21U};
	uint8_t response[2];
	uint32_t start = DWT->CYCCNT;
	uint8_t ok = 0U;

	if (hspi1.State != HAL_SPI_STATE_READY)
		goto failed;

	HAL_GPIO_WritePin(ENCODER_CS_GPIO_Port, ENCODER_CS_Pin, GPIO_PIN_RESET);

	__HAL_SPI_DISABLE(&hspi1);
	SET_BIT(SPI1->CR1, SPI_CR1_BIDIOE);
	__HAL_SPI_ENABLE(&hspi1);

	for (uint32_t i = 0U; i < sizeof(command); i++)
	{
		if (!TLE5012_WaitForFlag(SPI_FLAG_TXE, 1U, start))
			goto failed;
		*((__IO uint8_t *)&SPI1->DR) = command[i];
	}

	if (!TLE5012_WaitForFlag(SPI_FLAG_TXE, 1U, start) ||
		!TLE5012_WaitForFlag(SPI_FLAG_BSY, 0U, start))
		goto failed;

	__HAL_SPI_DISABLE(&hspi1);
	CLEAR_BIT(SPI1->CR1, SPI_CR1_BIDIOE);
	__HAL_SPI_ENABLE(&hspi1);

	for (uint32_t i = 0U; i < sizeof(response); i++)
	{
		if (!TLE5012_WaitForFlag(SPI_FLAG_RXNE, 1U, start))
			goto failed;
		response[i] = *((__IO uint8_t *)&SPI1->DR);
	}

	ok = 1U;

failed:
	HAL_GPIO_WritePin(ENCODER_CS_GPIO_Port, ENCODER_CS_Pin, GPIO_PIN_SET);
	__HAL_SPI_DISABLE(&hspi1);
	SET_BIT(SPI1->CR1, SPI_CR1_BIDIOE);
	__HAL_SPI_ENABLE(&hspi1);

	if (ok == 0U)
	{
		__HAL_SPI_CLEAR_OVRFLAG(&hspi1);
		EncoderSpiErrorCount++;
		return 0U;
	}

	*rawAngle = (uint16_t)((((uint16_t)response[0] << 8) | response[1]) & 0x7FFFU);
	return 1U;
}

void MotorSpeedGet(void)
{
	static float SpeedMrpmFil = 0.0f;
	static uint8_t SpeedInitialized = 0U;

	if (SpeedInitialized == 0U)
	{
		EnCoSpeed.EncoLastCnt = EnCoSpeed.EncoderCnt;
		SpeedMrpmFil = 0.0f;
		Motor.speed_M_rpm = 0.0f;
		Motor.speed_E_rpm = 0.0f;
		SpeedInitialized = 1U;
		return;
	}

	int32_t Diff = EnCoSpeed.EncoderCnt - EnCoSpeed.EncoLastCnt;

	if (Diff > (int32_t)(TLE5012_COUNTS_PER_REVOLUTION / 2U))
		Diff -= (int32_t)TLE5012_COUNTS_PER_REVOLUTION;
	else if (Diff < -(int32_t)(TLE5012_COUNTS_PER_REVOLUTION / 2U))
		Diff += (int32_t)TLE5012_COUNTS_PER_REVOLUTION;

	float SpeedMrpmTemp = (float)Diff * ((float)ENCODER_FRQ / (float)TLE5012_COUNTS_PER_REVOLUTION);

	SpeedMrpmFil = (LPF_U_A * SpeedMrpmTemp) + (LPF_U_B * SpeedMrpmFil);

	Motor.speed_M_rpm = SpeedMrpmFil;

	Motor.speed_E_rpm = Motor.speed_M_rpm * Motor.P; // 锟斤拷锟斤拷俣锟? = 锟斤拷械锟斤拷锟劫讹拷 * 锟斤拷锟斤拷锟斤拷
	EnCoSpeed.EncoLastCnt = EnCoSpeed.EncoderCnt;
}

void MotorAngleGet(void)
{
	uint16_t rawAngle;

	if (!TLE5012_ReadRawAngle(&rawAngle))
		return;

	EnCoSpeed.EncoderCnt = (int32_t)rawAngle;
	// Motor.E_theta = (float)EnCoSpeed.EncoderCnt * PIX2 /
	// 				(float)TLE5012_COUNTS_PER_REVOLUTION * Motor.P;
	Motor.E_theta = (float)EnCoSpeed.EncoderCnt * PIX2 /
					(float)TLE5012_COUNTS_PER_REVOLUTION;

	while (Motor.E_theta >= PIX2)
		Motor.E_theta -= PIX2;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // ��ʱ�������жϻص�����(htim:��ʱ�����)
{
	// update �¼����������
	if (htim == (&htim1)) // FOC����		20KHz
	{
		GetAdc(); // ��ȡ����
		AxisDq(); // ������任 // TODO: ���������MotorAngleGet(),�����Ƕ�

		EnCoSpeed.SpeedClock++;
		if (EnCoSpeed.SpeedClock == SPEED_CNT) // ����һ�ε��ת�٣��͵�����Ƶ���γɲ���
		{
			EnCoSpeed.SpeedClock %= SPEED_CNT;
			MotorSpeedGet(); // �ٶȻ�ȡ
		}

		MotorModeSel(); // ����ģʽѡ��
	}
}
