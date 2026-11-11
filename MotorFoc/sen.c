#include "motor_ctrl.h"

// �������
void PositionToZero(void)
{
    if (!PostionToZeroDouble())
        Motor.CtrlMode = M_ERR;
}

// ���ζ�λ����
uint8_t PostionToZeroDouble(void)
{
    TIM1->CCER |= 0x5555;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    HAL_Delay(100);

    Svpwm.Ualpha = 0.5f; // �����ѹ
    Svpwm.Ubeta = 0.0f;
    SvpwmCal((M_SVPWM)&Svpwm); // ��Alpha��Beta��ѹ�������SVPWM��ռ�ձ�
    HAL_Delay(1000);

    if (!MotorEncoderCalibrateElectricalZero())
    {
        Fault.bit.OffsetFlag = 1;
        MotorStop();
        return 0U;
    }

    Fault.bit.OffsetFlag = 0;
    MotorStop();
    return 1U;
}

// �и�,�����ջ�
void CurCloseloop(float IqRef, float Theta)
{
    pi_id.Ref = 0.0f; // id=0��d������ջ������ش�
    pi_id.Fbk = Park.Ds;
    PiController((M_PI_Control)&pi_id); // Id��ջ�

    pi_iq.Ref = IqRef;
    pi_iq.Fbk = Park.Qs;
    PiController((M_PI_Control)&pi_iq); // Iq��ջ�

    Ipark.Theta = Theta;
    Ipark.Ds = pi_id.Out;
    Ipark.Qs = pi_iq.Out;
    IPARK_Cale((M_IPARK)&Ipark); // IPARK��������
}

// UseRamp: 1-ʹ���ݶȼ��㣨���ڴ��ٶ�ģʽ��, 0-ֱ�Ӹ���������λ��ģʽ��
void SpeedCloseloop(int16_t SpeedRef, uint8_t UseRamp)
{
    float final_spd_ref = 0.0f;

    if (UseRamp)
    {
		SpdRefGXieLv.XieLv_Grad = SPD_REF_OD;	 // �����ٶȵ��ݶ�ֵ
		SpdRefGXieLv.Grad_Timer = SPD_REF_TIMER; // �����ٶȵ��ݶ�ʱ��
        SpdRefGXieLv.XieLv_X = SpeedRef;
        SpdRefGXieLv.Timer_Count++;
        if (my_abs(pi_spd.Ref - pi_spd.Fbk) >= 1000)
            SpdRefGXieLv.Timer_Count--;

        if (SpdRefGXieLv.Timer_Count > SpdRefGXieLv.Grad_Timer)
        {
            SpdRefGXieLv.Timer_Count = 0;
            GradXieLv((p_GXieLv)&SpdRefGXieLv);
        }
        final_spd_ref = Limit_Sat(SpdRefGXieLv.XieLv_Y, SPD_REF_MAX, -SPD_REF_MAX);
    }
    else
    {
        final_spd_ref = (float)SpeedRef;
        final_spd_ref = Limit_Sat(final_spd_ref, 100, -100);//λ�û��ٶ�����
    }

    // �ٶȻ�����Ƶ�ʿ���
    if (++Para.SpeedCalTime >= SPEED_CNT)
    {
        Para.SpeedCalTime = 0;

        pi_spd.Ref = final_spd_ref * Motor.P;
        pi_spd.Fbk = Motor.speed_E_rpm;

        PiController((M_PI_Control)&pi_spd); // �ٶ�PI����

        pi_spd.OutF = pi_spd.OutF * LPF_I_RUN_B + pi_spd.Out * LPF_I_RUN_A;
    }

    CurCloseloop(pi_spd.OutF, Motor.E_theta);
}

// �иУ�λ�û�
#define ENC_MIN 1
#define ENC_MAX 3999
#define ENC_RANGE (ENC_MAX - ENC_MIN + 1) // 3999
#define POS_MAX_SPEED 3000                // λ�û���������ת������ (RPM)
#define POS_I_LIMIT 500                   // λ�û������޷�����ֹ����

void PosCloseloop(void)
{
    static uint32_t PosLoopDivCnt = 0;
    float SpeedRef = 0.0f;

    Pos.P_Fdb = TIM3->CNT;

    if (++PosLoopDivCnt >= 4)
    {
        PosLoopDivCnt = 0;

        // Ŀ��λ��ӳ�� (Para.PosRef: 0..4095 -> mapped: 3999..1)
        uint32_t t = ((uint32_t)Para.PosRef * (ENC_RANGE - 1)) >> 12;
        uint16_t mapped = (uint16_t)(ENC_MAX - t);
        Pos.P_Ref = (int32_t)mapped;

        int32_t err = (int32_t)mapped - (int32_t)Pos.P_Fdb;
        int32_t half = ENC_RANGE / 2;
        if (err > half)
            err -= ENC_RANGE;
        else if (err < -half)
            err += ENC_RANGE;

        // ������
        float up = Pos.Kp * (float)err;

        // ������
        /*
        Pos.P_Integral += Pos.Ki * (float)err;
        Pos.P_Integral = Limit_Sat(Pos.P_Integral, POS_I_LIMIT, -POS_I_LIMIT); // ������
        SpeedRef = up + Pos.P_Integral;
        */

        SpeedRef = up; // Ŀǰ��ʹ�� P ����

        // ����޷�����ֹλ��ƫ������¸������ٶȲο�ֵ��������
        Pos.P_Out = (int16_t)Limit_Sat(SpeedRef, POS_MAX_SPEED, -POS_MAX_SPEED);
    }

    SpeedCloseloop(Pos.P_Out, 0);
}
