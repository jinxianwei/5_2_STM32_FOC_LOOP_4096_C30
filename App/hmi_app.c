/*
***********************************************************************
* �ļ�����hmi_app.c
* ��  �ߣ�������
* ��  �ڣ�2024/10/07
* ˵  �����˻���������
***********************************************************************
*/

#include "hmi_app.h"
#include "led_drv.h"
#include "delay.h"
#include "string.h"
#include "motor_ctrl.h"
#include "key_drv.h"

FloatToChar fc;
USART_DATA Usart;

const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f}; // JustFloat�涨{0x00, 0x00, 0x80, 0x7f}Ϊһ�����ݵĽ���

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) // ����Ǵ���1
	{
		Usart.RxLine++;											 // ÿ���յ�һ�����ݣ�����ص����ݳ��ȼ�1
		Usart.UsartRxBuf[Usart.RxLine - 1] = Usart.aRxBuffer[0]; // ��ÿ�ν��յ������ݱ��浽��������
		if (Usart.aRxBuffer[0] == 0xfa)							 // ���ս�����־λ��������ݿ����Զ��壬����ʵ������
		{
			Usart.UsartRxSta = 1; // ������ɱ�־λ��һ
		}
		Usart.aRxBuffer[0] = 0; // ��մ��ڽ��ջ���
	}
}

/*JustFloat*/
// ����һ�������壬������ĸ����Ͳ������뵽�����unsigned char������
void float_turn_u8(float f, uint8_t *c)
{
	uint8_t x;
	FloatLongType data;
	data.fdata = f;

	for (x = 0; x < 4; x++)
	{
		c[x] = (uint8_t)(data.ldata >> (x * 8));
	}
}

/*

*JustFloat:
*fdata:��������ָ��
*fdata_num:�������ݸ���
*Usart_choose:���͵Ĵ��ڵ�ַ

ǰ����for�� float����ת��λ��λunsigned char���Ͳ����ͳ�ȥ��������forѭ����β������ȥ
*/

static void JustFloatSend(float *fdata, uint16_t fdata_num, USART_TypeDef *Usart_choose)
{
	uint16_t x;
	uint8_t y;
	for (x = 0; x < fdata_num; x++)
	{
		float_turn_u8(fdata[x], Usart.Cdata);
		for (y = 0; y < 4; y++)
		{
			Usart_choose->DR = Usart.Cdata[y];
			while ((Usart_choose->SR & 0X40) == 0)
				;
		}
	}

	for (y = 0; y < 4; y++)
	{
		Usart_choose->DR = tail[y];
		while ((Usart_choose->SR & 0X40) == 0)
			;
	}
}

/*
 * ���ݴ�����Ϣ����PID����
 */
static void VofaPiAdjust(uint8_t *DataBuff, uint8_t on)
{
	static uint8_t reg = 0;
	memcpy(&fc.c_data, DataBuff, 4); // ��ѭIEEE 754Э�飬������ǰ��λ���蹲����

	reg = DataBuff[6]; // ����

	switch (reg)
	{
	case 0x01:
		pi_spd.Kp = fc.f_data;
		break;
	case 0x02:
		pi_spd.Ki = fc.f_data;
		break;
	case 0x03:
		pi_iq.Kp = fc.f_data;
		break;
	case 0x04:
		pi_iq.Ki = fc.f_data;
		break;
	case 0x05:
		Para.CurRef = fc.f_data;
		break;
	case 0x06:
		Para.PosRef = fc.f_data;
		break;
	case 0x07:
		Para.SpeedRef = fc.f_data;
		break;
	case 0x08:
	{
		switch ((uint8_t)fc.f_data)
		{
		case STOP:
		{
			MotorStop();
			Motor.CtrlMode = STOP;
			break;
		}
		case SPEED_LOOP:
		{
			MotorRun();					 // ����PWM
			Motor.CtrlMode = SPEED_LOOP; // �иУ��ٶ�&�����ջ�
			break;
		}
		case CUR_LOOP:
		{
			MotorRun();				   // ����PWM
			Motor.CtrlMode = CUR_LOOP; // �иУ������ջ�
			break;
		}
		case POS_LOOP:
		{
			MotorRun();				   // ����PWM
			Motor.CtrlMode = POS_LOOP; // �иУ�λ�ñջ�
			break;
		}
		case IF_CTRL:
		{
			MotorRun();				  // ����PWM
			Motor.CtrlMode = IF_CTRL; // IF����
			break;
		}

		case HFI_CTRL:
		{
			PllHfiPara.Theta = 0;
			MotorRun();				   // ����PWM
			Motor.CtrlMode = HFI_CTRL; // HFIģʽ
			break;
		}

		case HFI_SMO:
		{
			MotorRun();				  // ����PWM
			Motor.CtrlMode = HFI_SMO; // ����HFI�����ٻ�Ĥ
			break;
		}

		case HFI_FLUX:
		{
			MotorRun();				   // ����PWM
			Motor.CtrlMode = HFI_FLUX; // ����HFI�����ٴ���
			break;
		}

		case IF_SMO:
		{
			IFObserverStartInit();
			MotorRun();
			Motor.CtrlMode = IF_SMO; // IF���������ٻ�Ĥ
			break;
		}

		case IF_FLUX:
		{
			IFObserverStartInit();
			MotorRun();
			Motor.CtrlMode = IF_FLUX; // IF���������ٴ���
			break;
		}
		case TO_ZERO:
		{
			Motor.CtrlMode = TO_ZERO;
			Motor.CtrlMode = PostionToZeroDouble() ? STOP : M_ERR;
			break;
		}
		default:
			break;
		}
	}
	break;

	default:
		break;
	}
}

void RecevToVofa(void)
{
	if (Usart.UsartRxSta)
	{
		if (Usart.UsartRxBuf[5] == 0xfb) // У��λ�������յ��������Ƿ���ȷ
		{
			VofaPiAdjust(Usart.UsartRxBuf, 1); // ���ݽ����Ͳ�����ֵ����
		}
		memset(Usart.UsartRxBuf, 0, 8); // ��ջ�������
		Usart.RxLine = 0;				// ��ս��ճ���
		Usart.UsartRxSta = 0;
	}
}

// ���ݷ�������λ���۲�
void SendToVofa(void)
{
	Usart.VofaData[0] = Motor.speed_M_rpm;	  // ��Ƕ�
	Usart.VofaData[1] = Para.SpeedRef;		  // Ŀ���ٶ�
	Usart.VofaData[2] = pi_spd.Fbk / Motor.P; // ʵ���ٶ�
	Usart.VofaData[3] = Motor.E_theta;		  // �������Ƕ�
	Usart.VofaData[4] = flux.theta_e;		  // �����۲�Ƕ�
	Usart.VofaData[5] = PllHfiPara.Theta;	  // HFI�Ƕ�
	Usart.VofaData[6] = PllSmoPara.Angle;	  // ��Ĥ�۲�Ƕ�
	Usart.VofaData[7] = Para.IfTheta;		  // IFǿ�ϽǶ�
	Usart.VofaData[8] = AdcFilPara.CurU;	  // U�����
	Usart.VofaData[9] = AdcFilPara.CurV;	  // V�����
	Usart.VofaData[10] = AdcFilPara.CurW;	  // W�����
	Usart.VofaData[11] = TIM3->CNT;			  // ʵ��λ��
	Usart.VofaData[12] = Pos.P_Ref;			  // Ŀ��λ��
	JustFloatSend(Usart.VofaData, 13, USART1);
}

/**
***********************************************************
* @brief LEDӦ�ó���
* @param
* @return
***********************************************************
*/
#define LED_B_INDEX 0
#define LED_G_INDEX 1
#define LED_R_INDEX 2
void LedTask(uint8_t r, uint8_t g, uint8_t b)
{
	/* �� */
	if (r)
		TurnOnLed(LED_R_INDEX);
	else
		TurnOffLed(LED_R_INDEX);

	/* �� */
	if (g)
		TurnOnLed(LED_G_INDEX);
	else
		TurnOffLed(LED_G_INDEX);

	/* �� */
	if (b)
		TurnOnLed(LED_B_INDEX);
	else
		TurnOffLed(LED_B_INDEX);
}

/**
***********************************************************
* @brief ��λ��Ӧ�ó���
* @param
* @return
***********************************************************
*/
void VrTask(void)
{
	static float AdcVr = 0;

	switch (Motor.CtrlMode)
	{
	case STOP:
	{
		Para.SpeedRef = 0;
		Para.CurRef = 0;
		break;
	}
	case SPEED_LOOP:
	{
		AdcVr = (float)AdcPara.Vr;
		Para.SpeedRef = (int16_t)AdcVr;
		Para.SpeedRef = Limit_Sat(Para.SpeedRef, SPD_REF_MAX, 100.0f);
		break;
	}
	case CUR_LOOP:
	{
		AdcVr = (float)(AdcPara.Vr / 2000.0f);
		Para.CurRef = (float)AdcVr;
		Para.CurRef = Limit_Sat(Para.CurRef, 1.0f, 0.05f);
		break;
	}
	case POS_LOOP:
	{
		Para.PosRef = (int32_t)AdcPara.Vr;
		break;
	}
	case IF_CTRL:
	{
		AdcVr = (float)(AdcPara.Vr / 1000.0f);
		Para.CurRef = (float)AdcVr;
		Para.CurRef = Limit_Sat(Para.CurRef, 4.0f, 0.05f);
		break;
	}

	case HFI_CTRL:
	{
		AdcVr = (float)(AdcPara.Vr / 10.0f);
		Para.SpeedRef = (int16_t)AdcVr;
		Para.SpeedRef = Limit_Sat(Para.SpeedRef, HFI_SPD_REF_MAX, 10.0f);
		break;
	}

	case HFI_SMO:
	case HFI_FLUX:
	case IF_SMO:
	case IF_FLUX:
	{
		AdcVr = (float)AdcPara.Vr;
		Para.SpeedRef = (int16_t)AdcVr;
		Para.SpeedRef = Limit_Sat(Para.SpeedRef, SPD_REF_MAX, 500.0f);
		break;
	}
	case TO_ZERO:
	{
		break;
	}
	default:
		break;
	}
}

/**
***********************************************************
* @brief ����Ӧ�ó���
* @param
* @return
***********************************************************
*/
void HmiTask(void)
{
	uint8_t keyVal;

	keyVal = GetKeyVal();

	switch (keyVal)
	{
	case KEY4_SHORT_PRESS:
	{
	}
	break;

	case KEY4_DOUBLE_PRESS:
	{
	}
	break;

	case KEY4_LONG_PRESS:
	{
		HAL_GPIO_WritePin(PWR_GPIO_Port, PWR_Pin, GPIO_PIN_RESET); // �����ػ�
	}
	break;
	case KEY1_SHORT_PRESS:
	{
		MotorRun();				  // ����PWM
		Motor.CtrlMode = HFI_SMO; // ����HFI�����ٻ�Ĥ
	}
	break;

	case KEY1_DOUBLE_PRESS:
	{
		MotorRun();				 // ����PWM
		Motor.CtrlMode = IF_SMO; // ����IF����SMO
	}
	break;

	case KEY1_LONG_PRESS:
	{
		MotorRun();				   // ����PWM
		Motor.CtrlMode = HFI_FLUX; // ����HFI�����ٴ���
	}
	break;

	case KEY2_SHORT_PRESS:
	{
		MotorRun();					 // ����PWM
		Motor.CtrlMode = SPEED_LOOP; // �иУ��ٶ�&�����ջ�
	}
	break;

	case KEY2_DOUBLE_PRESS:
	{
		MotorRun();				   // ����PWM
		Motor.CtrlMode = CUR_LOOP; // �иУ������ջ�
	}
	break;

	case KEY2_LONG_PRESS:
	{
		MotorRun();				   // ����PWM
		Motor.CtrlMode = POS_LOOP; // �иУ�λ�ñջ�
	}
	break;

	case KEY3_SHORT_PRESS:
	{
		MotorStop();
		Motor.CtrlMode = STOP;
	}
	break;

	case KEY3_DOUBLE_PRESS:
	{
		MotorRun();				  // ����PWM
		Motor.CtrlMode = IF_FLUX; // ����IF������FLUX
	}
	break;

	case KEY3_LONG_PRESS:
	{
		Motor.CtrlMode = TO_ZERO;
		Motor.CtrlMode = PostionToZeroDouble() ? STOP : M_ERR;
	}
	break;

	default:
		break;
	}

	if (Motor.CtrlMode == POS_LOOP)
	{
		pi_spd.Kp = Para.SpdSenKp;
		pi_spd.Ki = Para.SpdSenKi;
	}
	else
	{
		pi_spd.Kp = Para.SpdLessKp;
		pi_spd.Ki = Para.SpdLessKi;
	}
}
