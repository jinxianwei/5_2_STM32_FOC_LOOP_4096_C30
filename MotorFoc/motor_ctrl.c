#include "motor_ctrl.h"

// FOC 坐标变换与电机运行状态对象
CLARKE /******************/ Clarke = CLARKE_DEFAULTS;
PARK /********************/ Park = PARK_DEFAULTS;
IPARK /*******************/ Ipark = IPARK_DEFAULTS;
motor_p /*****************/ Motor = MOTOR_DEFAULTS;

// 频率、速度、电流给定的斜率规划对象
GXieLv /******************/ IfFreqGXieLv = GXieLv_DEFAULTS;
GXieLv /******************/ SpdRefGXieLv = GXieLv_DEFAULTS;
GXieLv /******************/ IdqRefGXieLv = GXieLv_DEFAULTS;

// 无感观测器、运行参数与位置控制对象
Sensorless_Control /******/ SensorLess = Sensorless_DEFAULTS;
IPARK /*******************/ IparkSmoPv = IPARK_DEFAULTS;
PARA /********************/ Para;
POS_DRV /*****************/ Pos;

// FLUX_DRV                 flux = FLUX_DEFAULTS;

bool Init_Over = 0;
bool HFI_Init_Angle_flag = 0;

// 按设定梯度把当前输出 XieLv_Y 逐步逼近目标 XieLv_X
void GradXieLv(p_GXieLv pv) // 斜率梯度计算 ，按照一定梯度值加加减减
{
	if (pv->XieLv_X < (pv->XieLv_Y - pv->XieLv_Grad)) // XieLv_Grad 减梯度值时候
	{
		pv->XieLv_Y = pv->XieLv_Y - pv->XieLv_Grad;
	}
	else if (pv->XieLv_X > (pv->XieLv_Y + pv->XieLv_Grad)) // 加梯度值时候
	{
		pv->XieLv_Y = pv->XieLv_Y + pv->XieLv_Grad;
	}
	else
	{
		pv->XieLv_Y = pv->XieLv_X;
	}
}

// 停止电机
void MotorStop(void)
{
	// 关闭三相互补 PWM 输出，防止停机后继续驱动功率管
	TIM1->CCER &= 0xAAAA; // 不使能PWM输出通道OC1/OC1N/OC2/OC2N/OC3/OC3N

	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;

	VariableClear(); // 变量清零
}

// 启动电机
void MotorRun(void)
{
	// Fault_Detection();				//启动前先进行故障检测
	// 使能三相上下桥 PWM 输出，后续占空比由 SVPWM 更新
	TIM1->CCER |= 0x5555;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

// 强制拖动
void QtMotor(void)
{
	Para.IfFrq++;
	if (Para.IfFrq == QT_FRQ) // 强拖频率
	{
		Para.IfFrq = 0;
		Ipark.Alpha = ualpha_tab[Para.IfAngle];
		Ipark.Beta = ubeta_tab[Para.IfAngle];
		Para.IfAngle++;
		if (Para.IfAngle > 359)
			Para.IfAngle = 0;
	}
}

// 观测器滞回区间状态判断
static void ObsHysteresisCheck(void)
{
	// Observer: 1 表示低速 HFI，3 表示高速 SMO/Flux；w1/w2 形成滞回区间
	SensorLess.Observer_last = SensorLess.Observer;
	float abs_ref = my_abs(SensorLess.Speed_Ref);

	if (abs_ref <= SensorLess.w2)
	{
		if ((SensorLess.Observer_last == 3) && (abs_ref > SensorLess.w1))
			SensorLess.Observer = 3; // 保持高速
		else
			SensorLess.Observer = 1; // 低速 HFI
	}
	else if (abs_ref <= SensorLess.w1)
	{
		SensorLess.Observer = 1;
	}
	else // > w2
	{
		SensorLess.Observer = 3;
	}
}

void Pll_Sync(p_PLL pv, float target_theta, float target_omega)
{
	// 切换观测器前同步 PLL 状态，避免角度/速度突跳
	pv->Theta = target_theta;
	pv->Omega = target_omega;
	pv->Omega_F = target_omega;
	pv->Theta_Err = 0;
	pv->Theta_Err_last = 0;
	// 也要对限幅进行处理
	if (pv->Theta >= PIX2)
		pv->Theta -= PIX2;
	else if (pv->Theta < 0)
		pv->Theta += PIX2;

	pv->Angle = pv->Theta;
}

static float GetIfDirection(void)
{
	// IF 开环角度方向跟随机械速度给定方向
	return (Para.SpeedRef < 0) ? -1.0f : 1.0f;
}

static float Wrap_0_2pi(float theta)
{
	// 将电角度限制到 0~2PI，调用处每次步进小于一圈
	if (theta >= PIX2)
		theta -= PIX2;
	else if (theta < 0.0f)
		theta += PIX2;
	return theta;
}

static int8_t SpeedSign(float speed)
{
	// 小速度视为零，避免方向判断在零速附近抖动
	if (speed > 1.0f)
		return 1;
	if (speed < -1.0f)
		return -1;
	return 0;
}

static bool SpeedDirConflict(float ref, float fbk)
{
	// 目标速度与反馈速度反向时，认为观测器暂不适合继续闭环加速
	int8_t ref_sign = SpeedSign(ref);
	int8_t fbk_sign = SpeedSign(fbk);

	return (ref_sign != 0) && (fbk_sign != 0) && (ref_sign != fbk_sign);
}

static float GetTargetElecSpeedRpm(void)
{
	// 机械速度给定限幅后换算为电角速度 rpm
	return Limit_Sat((float)Para.SpeedRef, SPD_REF_MAX, -SPD_REF_MAX) * Motor.P;
}

static void ResetSpeedPiState(void)
{
	// 观测器切换时清速度环积分，减少切换瞬间的电流冲击
	pi_spd.err = 0.0f;
	pi_spd.err_l = 0.0f;
	pi_spd.Out = 0.0f;
	pi_spd.OutF = 0.0f;
	pi_spd.integrator = 0.0f;
}

static float GetObserverSpeedRef(void)
{
	// 高速观测器闭环给定；方向冲突时先给 0，等待反馈速度回到合理方向
	float target_speed = GetTargetElecSpeedRpm();

	if (SpeedDirConflict(target_speed, SensorLess.Speed_Fbk))
		return 0.0f;
	return target_speed;
}

static bool IfReadyToSwitchHigh(float target_speed, float if_speed)
{
	// IF 频率和目标速度都进入高速区，且方向一致，才允许切到高速观测器
	return (my_abs(target_speed) >= SensorLess.w2) &&
		   (my_abs(if_speed) >= SensorLess.w2) &&
		   (SpeedSign(target_speed) == SpeedSign(if_speed));
}

static bool NeedSwitchBackToIf(float target_speed)
{
	// 目标回到低速区或反馈方向异常时，回切到 IF 开环托管
	return (my_abs(target_speed) < SensorLess.w2) ||
		   (SpeedDirConflict(target_speed, SensorLess.Speed_Fbk) &&
			(my_abs(SensorLess.Speed_Fbk) <= SensorLess.w2));
}

static void SyncIfFromObserver(float theta, float elec_speed_rpm)
{
	// 从高速观测器回切 IF 时，沿用当前角度和速度，保证开环角度连续
	float if_freq = Limit_Sat(elec_speed_rpm / 60.0f, MOTOR_FRE_MAX, -MOTOR_FRE_MAX);

	Para.IfTheta = Wrap_0_2pi(theta);
	Para.FocAngle = Para.IfTheta;

	IfFreqGXieLv.XieLv_Y = if_freq;
	IfFreqGXieLv.XieLv_X = if_freq;
	IfFreqGXieLv.Timer_Count = 0;

	SensorLess.Observer_last = SensorLess.Observer;
	SensorLess.Observer = 1;
	Para.UpTime = 0;
	Para.DownTime = 1;
	ResetSpeedPiState();
}

static float IFThetaRampCale(void)
{
	// IF 启动角度斜坡：频率按梯度升高，电角度按 PWM 周期积分
	float IF_Freq_EX = 0.0f;

	IfFreqGXieLv.XieLv_Grad = IF_F_GRAD_0D1HZ;
	IfFreqGXieLv.Grad_Timer = IF_F_GRAD_TIMER;
	IfFreqGXieLv.XieLv_X = IF_FRE_MAX;

	IfFreqGXieLv.Timer_Count++;
	if (IfFreqGXieLv.Timer_Count > IfFreqGXieLv.Grad_Timer)
	{
		IfFreqGXieLv.Timer_Count = 0;
		GradXieLv((p_GXieLv)&IfFreqGXieLv);
	}

	IF_Freq_EX = Limit_Sat(IfFreqGXieLv.XieLv_Y, MOTOR_FRE_MAX, MOTOR_FRE_MIN);
	Para.IfTheta += GetIfDirection() * (float)(PIX2 * IF_Freq_EX) / PWM_FRQ;
	if (Para.IfTheta >= PIX2)
		Para.IfTheta -= PIX2;
	else if (Para.IfTheta < 0)
		Para.IfTheta += PIX2;

	Para.FocAngle = Para.IfTheta;
	return IF_Freq_EX;
}

static float IFObserverThetaRampCale(void)
{
	// IF+观测器模式角度斜坡：频率目标跟随速度给定，可正反转
	float target_freq = GetTargetElecSpeedRpm() / 60.0f;

	target_freq = Limit_Sat(target_freq, MOTOR_FRE_MAX, -MOTOR_FRE_MAX);

	IfFreqGXieLv.XieLv_Grad = IF_F_GRAD_0D1HZ;
	IfFreqGXieLv.Grad_Timer = IF_F_GRAD_TIMER;
	IfFreqGXieLv.XieLv_X = target_freq;

	IfFreqGXieLv.Timer_Count++;
	if (IfFreqGXieLv.Timer_Count > IfFreqGXieLv.Grad_Timer)
	{
		IfFreqGXieLv.Timer_Count = 0;
		GradXieLv((p_GXieLv)&IfFreqGXieLv);
	}

	Para.IfTheta += (float)(PIX2 * IfFreqGXieLv.XieLv_Y) / PWM_FRQ;
	Para.IfTheta = Wrap_0_2pi(Para.IfTheta);

	Para.FocAngle = Para.IfTheta;
	return IfFreqGXieLv.XieLv_Y;
}

void IFObserverStartInit(void)
{
	// IF+观测器启动前复位状态，并先进入 IF 低速托管
	VariableClear();
	Para.CurRef = Para.IfCurrent;
	SensorLess.Observer = 1;
	SensorLess.Observer_last = 1;
}

static void Process_IF_SMO(void)
{
	// IF 负责低速启动，达到切换条件后交给 SMO；必要时可回切 IF
	float if_omega = IfFreqGXieLv.XieLv_Y * PIX2;
	float if_speed = if_omega / PIX2 * 60.0f;
	float target_speed = GetTargetElecSpeedRpm();

	SmoThetaCale(Clarke.Alpha, Clarke.Beta, Svpwm.Ualpha, Svpwm.Ubeta);

	if (SensorLess.Observer == 1)
	{
		// 低速 IF 阶段：SMO 的 PLL 跟随 IF 角度，提前完成角度同步
		Pll_Sync((p_PLL)&PllSmoPara, Para.FocAngle, if_omega);
		SensorLess.theta = Para.FocAngle;
		SensorLess.Omega = if_omega;
		SensorLess.Speed_Fbk = if_speed;
		SensorLess.Speed_Ref = target_speed;

		if (IfReadyToSwitchHigh(target_speed, if_speed))
		{
			SensorLess.Observer_last = 1;
			SensorLess.Observer = 3;
			Para.UpTime = 1;
			Para.DownTime = 0;
			ResetSpeedPiState();
		}
	}
	else
	{
		// 高速 SMO 阶段：使用 SMO 角度和速度反馈，低速或方向异常时回切 IF
		SensorLess.Observer_last = 3;
		SensorLess.theta = PllSmoPara.Angle;
		SensorLess.Omega = PllSmoPara.Omega_F;
		SensorLess.Speed_Fbk = SensorLess.Omega / PIX2 * 60.0f;

		if (NeedSwitchBackToIf(target_speed))
			SyncIfFromObserver(PllSmoPara.Angle, SensorLess.Speed_Fbk);
	}
}

static void Process_IF_FLUX(void)
{
	// IF 负责低速启动，达到切换条件后交给磁链观测器；必要时可回切 IF
	float if_omega = IfFreqGXieLv.XieLv_Y * PIX2;
	float if_speed = if_omega / PIX2 * 60.0f;
	float target_speed = GetTargetElecSpeedRpm();

	if (SensorLess.Observer == 1)
	{
		// 低速 IF 阶段：磁链观测器同步 IF 角度和速度
		FluxSync(&flux, Para.FocAngle, if_omega);
		SensorLess.theta = Para.FocAngle;
		SensorLess.Omega = if_omega;
		SensorLess.Speed_Fbk = if_speed;
		SensorLess.Speed_Ref = target_speed;

		if (IfReadyToSwitchHigh(target_speed, if_speed))
		{
			SensorLess.Observer_last = 1;
			SensorLess.Observer = 3;
			Para.UpTime = 1;
			Para.DownTime = 0;
			ResetSpeedPiState();
		}
	}
	else
	{
		// 高速 Flux 阶段：使用磁链观测角度和速度反馈，低速或方向异常时回切 IF
		SensorLess.Observer_last = 3;
		SensorLess.theta = flux.theta_e;
		SensorLess.Omega = flux.w_e_rpm * PIX2 / 60.0f;
		SensorLess.Speed_Fbk = flux.w_e_rpm;

		if (NeedSwitchBackToIf(target_speed))
			SyncIfFromObserver(flux.theta_e, SensorLess.Speed_Fbk);
	}
}
// 处理 HFI 与 SMO 的切换与计算
static void Process_HFI_SMO(void)
{
	ObsHysteresisCheck(); // 1. 更新状态

	// 2. 处理切换过程 (UpTime/DownTime)
	// 比较对象: PllHfiPara.Theta vs PllSmoPara.Angle
	if (((SensorLess.Observer == 3) && (SensorLess.Observer_last == 1)) || (Para.UpTime != 0))
	{
		Para.UpTime++;
		if (Para.UpTime == 2)
		{
			float err = my_abs(PllHfiPara.Theta - PllSmoPara.Angle);
			if (err > PI)
				Para.HfiSmoErr = my_abs(err - PIX2);
			else
				Para.HfiSmoErr = err;

			if (Para.HfiSmoErr >= DEG_10)
				Para.UpTime--;
		}
		else if (Para.UpTime == 400)
			Para.UpTime = 0;
	}
	else if (((SensorLess.Observer == 1) && (SensorLess.Observer_last == 3)) || (Para.DownTime != 0))
	{
		Para.DownTime++;
		if (Para.DownTime == 3)
		{
			float err = my_abs(PllHfiPara.Theta - PllSmoPara.Angle);
			if (err > PI)
				Para.HfiSmoErr = my_abs(err - PIX2);
			else
				Para.HfiSmoErr = err;

			if (Para.HfiSmoErr >= DEG_10)
				Para.DownTime--;
		}
		else if (Para.DownTime == 400)
			Para.DownTime = 0;
	}

	// 3. 执行观测器计算
	switch (SensorLess.Observer)
	{
	case 1:						   // HFI 模式（低速）
		HfiAngleCale((p_HFI)&HFI); // HFI 计算

		// 在 HFI 运行时，强制 SMO 的 PLL 跟踪 HFI 的结果
		// 这样当速度升上来准备切到 SMO 时，SMO 的角度已经和 HFI 完全一致了
		Pll_Sync((p_PLL)&PllSmoPara, PllHfiPara.Theta, PllHfiPara.Omega);

		// 执行 SMO 的计算（仅为了保持 SMO 内部变量更新），但不用它的结果
		SmoThetaCale(HFI.ialpha_f, HFI.ibeta_f, IparkSmoPv.Alpha, IparkSmoPv.Beta);

		SensorLess.theta = PllHfiPara.Theta;
		SensorLess.Speed_Fbk = PllHfiPara.Omega_F / PIX2 * 60;
		break;

	case 3: // SMO 模式（高速）
		// 正常执行 SMO 计算
		SmoThetaCale(Clarke.Alpha, Clarke.Beta, Svpwm.Ualpha, Svpwm.Ubeta);

		// 反过来，SMO 运行时，也要让 HFI 的 PLL 跟着 SMO 走
		// 这样在减速反转切回 HFI 时，HFI 也能无缝接手
		Pll_Sync((p_PLL)&PllHfiPara, PllSmoPara.Angle, PllSmoPara.Omega);

		SensorLess.theta = PllSmoPara.Angle;
		SensorLess.Speed_Fbk = PllSmoPara.Omega_F / PIX2 * 60;
		break;
	}
}

// 处理 HFI 与 FLUX 的切换与计算
static void Process_HFI_FLUX(void)
{
	ObsHysteresisCheck(); // 1. 更新状态
	// 2. 处理切换过程
	// 比较对象: PllHfiPara.Theta vs flux.theta_e
	if (((SensorLess.Observer == 3) && (SensorLess.Observer_last == 1)) || (Para.UpTime != 0))
	{
		Para.UpTime++;
		if (Para.UpTime == 2)
		{
			float err = my_abs(PllHfiPara.Theta - flux.theta_e);
			if (err > PI)
				Para.HfiSmoErr = my_abs(err - PIX2);
			else
				Para.HfiSmoErr = err;

			if (Para.HfiSmoErr >= DEG_10)
				Para.UpTime--;
		}
		else if (Para.UpTime == 400)
			Para.UpTime = 0;
	}
	else if (((SensorLess.Observer == 1) && (SensorLess.Observer_last == 3)) || (Para.DownTime != 0))
	{
		Para.DownTime++;
		if (Para.DownTime == 3)
		{
			float err = my_abs(PllHfiPara.Theta - flux.theta_e);
			if (err > PI)
				Para.HfiSmoErr = my_abs(err - PIX2);
			else
				Para.HfiSmoErr = err;

			if (Para.HfiSmoErr >= DEG_10)
				Para.DownTime--;
		}
		else if (Para.DownTime == 400)
			Para.DownTime = 0;
	}

	// 3. 执行观测器计算
	switch (SensorLess.Observer)
	{
	case 1: // HFI Mode，低速区由高频注入提供角度
		if (Para.DownTime == 1)
		{
			// 刚从 Flux 回切时先沿用 Flux 角度，等待 HFI 状态恢复
			Para.ObsSwFlag = 1;
			SensorLess.theta = flux.theta_e;
			SensorLess.Omega = flux.w_e_rpm * PIX2 / 60.0f;
			SensorLess.Speed_Fbk = flux.w_e_rpm;
		}
		else if (Para.DownTime >= 2)
		{
			// 回切过渡期继续计算 HFI，但角度仍保持 Flux 侧连续
			HfiAngleCale((p_HFI)&HFI);
			SensorLess.theta = flux.theta_e;
			SensorLess.Omega = flux.w_e_rpm * PIX2 / 60.0f;
			SensorLess.Speed_Fbk = flux.w_e_rpm;
		}
		else
		{
			// 稳定低速区：使用 HFI 角度与速度反馈
			Para.ObsSwFlag = 0;
			HfiAngleCale((p_HFI)&HFI);
			SensorLess.theta = PllHfiPara.Theta;
			if (Para.SpeedCalTime == SPD_CAL_PERIOD)
			{
				SensorLess.Omega = PllHfiPara.Omega_F;
				SensorLess.Speed_Fbk = SensorLess.Omega / PIX2 * 60;
			}
		}
		break;
	case 3: // Flux Mode，高速区由磁链观测器提供角度
		if (Para.UpTime == 1)
		{
			// 刚进入高速区时先保持 HFI 角度，避免切换突变
			Para.ObsSwFlag = 1;
			HfiAngleCale((p_HFI)&HFI);
			SensorLess.theta = PllHfiPara.Theta;
		}
		else if (Para.UpTime == 2)
		{
			// 切换过渡期继续运行 HFI，给 Flux 接管留出同步时间
			Para.CalTime++;
			HfiAngleCale((p_HFI)&HFI);
			SensorLess.theta = PllHfiPara.Theta;
			SensorLess.Omega = PllHfiPara.Omega_F;
			SensorLess.Speed_Fbk = SensorLess.Omega / PIX2 * 60;
		}
		else
		{
			// 稳定高速区：使用 Flux 角度与速度反馈
			Para.ObsSwFlag = 0;
			SensorLess.theta = flux.theta_e;
			if (Para.SpeedCalTime == SPD_CAL_PERIOD)
			{
				SensorLess.Omega = flux.w_e_rpm * PIX2 / 60.0f;
				SensorLess.Speed_Fbk = flux.w_e_rpm;
			}
		}
		break;
	}
}

// 三相电流到dq轴电流变化
void AxisDq(void)
{
	// FOC 主采样链路：三相电流 -> Clarke -> 角度选择 -> Park
	Para.SpeedCalTime++;

	// 1. 采样三相电流并做 Clarke 变换
	Clarke.Iu = AdcPara.CurU;
	Clarke.Iv = AdcPara.CurV;
	Clarke.Iw = AdcPara.CurW;
	ClarkeCale((M_CLARKE)&Clarke);

	MotorAngleGet(); // 获取编码器角度

	// 2. 运行磁链观测器，供 Flux 控制模式或切换过程使用
	FluxCale(&flux, Clarke.Alpha, Clarke.Beta, Ipark.Alpha, Ipark.Beta);

	// 3. 根据控制模式更新无感角度或切换状态
	if (Motor.CtrlMode == HFI_CTRL)
	{
		HfiAngleCale((p_HFI)&HFI);
	}
	else if (Motor.CtrlMode == HFI_SMO)
	{
		Process_HFI_SMO(); // HFI+SMO 处理
	}
	else if (Motor.CtrlMode == HFI_FLUX)
	{
		Process_HFI_FLUX(); // HFI+Flux 处理
	}
	else if (Motor.CtrlMode == IF_SMO)
	{
		Process_IF_SMO();
	}
	else if (Motor.CtrlMode == IF_FLUX)
	{
		Process_IF_FLUX();
	}

	// 4. 选择 Park 变换使用的电角度来源
	if ((QT_IF_MODE == 3) && ((Motor.CtrlMode == IF_CTRL) || ((Motor.CtrlMode == IF_SMO || Motor.CtrlMode == IF_FLUX) && (SensorLess.Observer == 1))))
	{
		Park.Theta = Para.FocAngle;
	}
	else if (Motor.CtrlMode == HFI_CTRL)
	{
		Park.Theta = PllHfiPara.Theta; // 高频注入角度
	}
	else if (Para.SmoSwFlag)
	{
		Park.Theta = PllSmoPara.Angle; // 滑膜观测角度
	}
	else if (Motor.CtrlMode == HFI_SMO || Motor.CtrlMode == HFI_FLUX || Motor.CtrlMode == IF_SMO || Motor.CtrlMode == IF_FLUX)
	{
		Park.Theta = SensorLess.theta; // 观测角度
	}
	else
	{
		Park.Theta = Motor.E_theta; // 编码器角度
	}

	// 5. Park 变换，得到 d/q 轴电流反馈
	Park.Alpha = Clarke.Alpha;
	Park.Beta = Clarke.Beta;
	ParkCale((M_PARK)&Park);
}

float kRpm2Dtheta = (2.0f * PI * 10.0f) / (60.0f * (20.0f * 1000.0f));

uint8_t FluxFlag = 0;
// 电机速度开环，电流闭环启动
// IF启动速度=IF_FRE_MAX/P*60 rad/min
void IFStartControl(void)
{
	// IF 开环产生角度，电流环按给定电流闭环输出电压
	IFThetaRampCale();
	CurCloseloop(Para.CurRef, Para.FocAngle);
}

// SVPWM计算
void FocSvpwm(void)
{
	// 电流环输出的 d/q 电压经 IPark 后送入 SVPWM 更新三相占空比
	Svpwm.Ualpha = Ipark.Alpha; // IPARK的数据传入SVPWM
	Svpwm.Ubeta = Ipark.Beta;
	SvpwmCal((M_SVPWM)&Svpwm); // 将Alpha和Beta电压带入计算SVPWM的占空比
}

// HFI + Flux Observer 全速无感控制
void LessHfiFluxControl(void)
{
	// 控制策略：低速 HFI，速度升高后切换到 Flux 观测器
	float ISpeed_Ref_EX = 0.0f;

	// 1. 速度斜坡规划
	SpdRefGXieLv.XieLv_Grad = SPD_REF_OD;
	SpdRefGXieLv.Grad_Timer = SPD_REF_TIMER;
	SpdRefGXieLv.XieLv_X = Para.SpeedRef;

	if (HFI_Init_Angle_flag)
	{
		if (Para.ObsSwFlag != 1) // 切换过程中暂停加减速
			SpdRefGXieLv.Timer_Count++;

		// 如果误差过大，减慢斜坡
		if ((my_abs(SensorLess.Speed_Ref - SensorLess.Speed_Fbk) >= 1000) && (SensorLess.Observer == 1))
			SpdRefGXieLv.Timer_Count--;

		if (SpdRefGXieLv.Timer_Count > SpdRefGXieLv.Grad_Timer)
		{
			SpdRefGXieLv.Timer_Count = 0;
			GradXieLv((p_GXieLv)&SpdRefGXieLv);
		}
		ISpeed_Ref_EX = Limit_Sat(SpdRefGXieLv.XieLv_Y, SPD_REF_MAX, -SPD_REF_MAX);
		SensorLess.Speed_Ref = ISpeed_Ref_EX * Motor.P;
	}

	// 2. 观测器执行逻辑
	switch (SensorLess.Observer)
	{
	case 1: // 低速模式：必须注入 HFI
	{
		if (HFI_Init_Angle_flag)
		{
			if (Para.DownTime == 1)
			{
				// 从 Flux 切回 HFI，重置 HFI 相关积分器
				HFI.Id_last = 0;
				HFI.Iq_last = 0;
				// 这里可能需要重置 PLL，视具体实现而定

				ExtractIdqF();				   // 提取高频分量
				HfiObserver(Park.Ds, Park.Qs); // HFI 核心计算

				HFI.Ialpha_last = 0;
				HFI.Ibeta_last = 0;
				HFI.ialpha_h = 0;
				HFI.ibeta_h = 0;
			}
			else
			{
				ExtractIdqF();
				HfiObserver(HFI.Idf, HFI.Iqf);
			}
		}
		else
			HfiInitTheta(); // 初始对齐
		break;
	}
	case 3: // 高速模式
	{
		if (Para.UpTime == 1)
		{
			// 刚进入高速，还可以跑一跑 HFI 保证平滑
			ExtractIdqF();
			HfiObserver(HFI.Idf, HFI.Iqf);
		}
		else if (Para.UpTime == 2)
		{
			// 切换中，继续跑
			ExtractIdqF();
			HfiObserver(HFI.Idf, HFI.Iqf);
		}
		else
		{
			FluxObserver(Park.Ds, Park.Qs);
		}
		break;
	}
	default:
		break;
	}

	Para.SpeedCalTime %= SPD_CAL_PERIOD;
}

// 全速度无感控制
void LessHfiSmoControl(void)
{
	// 控制策略：低速 HFI，速度升高后切换到 SMO 观测器
	float ISpeed_Ref_EX = 0.0f;

	SpdRefGXieLv.XieLv_Grad = SPD_REF_OD;	 // 定义速度的梯度值
	SpdRefGXieLv.Grad_Timer = SPD_REF_TIMER; // 定义速度的梯度时间
	SpdRefGXieLv.XieLv_X = Para.SpeedRef;	 // SensorLess.Speed_Max;	//定义梯度上限

	if (HFI_Init_Angle_flag) // 完成初始角度识别后开始，速度开始梯度增加
	{
		if (Para.ObsSwFlag != 1)
			SpdRefGXieLv.Timer_Count++;

		if ((my_abs(SensorLess.Speed_Ref - SensorLess.Speed_Fbk) >= 1000) && (SensorLess.Observer == 1))
			SpdRefGXieLv.Timer_Count--;

		if (SpdRefGXieLv.Timer_Count > SpdRefGXieLv.Grad_Timer)
		{
			SpdRefGXieLv.Timer_Count = 0;
			GradXieLv((p_GXieLv)&SpdRefGXieLv); // 梯度计算
		}
		ISpeed_Ref_EX = Limit_Sat(SpdRefGXieLv.XieLv_Y, SPD_REF_MAX, -SPD_REF_MAX); // 速度限幅
		SensorLess.Speed_Ref = ISpeed_Ref_EX * Motor.P;								// 转变为电角速度	rpm/min
	}

	switch (SensorLess.Observer) // 观测器选择
	{
	case 1:
	{
		if (HFI_Init_Angle_flag) // 初始角度检测完成后
		{
			if (Para.DownTime == 1)
			{
				HFI.Id_last = 0;
				HFI.Iq_last = 0;

				ExtractIdqF();
				HfiObserver(Park.Ds, Park.Qs); // 基于HFI的速度环控制

				HFI.Ialpha_last = 0;
				HFI.Ibeta_last = 0;
				HFI.ialpha_h = 0;
				HFI.ibeta_h = 0;
			}
			else
			{
				ExtractIdqF();
				HfiObserver(HFI.Idf, HFI.Iqf);
			}
		}
		else
			HfiInitTheta();
		break;
	}
	case 3:
	{
		if (Para.UpTime == 1)
		{
			ExtractIdqF();
			HfiObserver(HFI.Idf, HFI.Iqf);
		}
		else if (Para.UpTime == 2) //
		{
			ExtractIdqF();				   // 提取低频电流分量
			SmoObserver(HFI.Idf, HFI.Iqf); // SMO速度闭环
		}
		else
		{
			SmoObserver(Park.Ds, Park.Qs);
		}
		break;
	}
	default:
		break;
	}

	Para.SpeedCalTime %= SPD_CAL_PERIOD;
}

static void SixStepApplyVector(uint8_t vector, float duty)
{
	static const uint32_t high_enable[3] = {
		TIM_CCER_CC1E, TIM_CCER_CC2E, TIM_CCER_CC3E};
	static const uint32_t low_enable[3] = {
		TIM_CCER_CC1NE, TIM_CCER_CC2NE, TIM_CCER_CC3NE};
	volatile uint32_t *compare[3] = {
		&TIM1->CCR1, &TIM1->CCR2, &TIM1->CCR3};
	uint32_t output_mask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
						   TIM_CCER_CC2E | TIM_CCER_CC2NE |
						   TIM_CCER_CC3E | TIM_CCER_CC3NE;
	uint32_t active_compare;
	uint32_t enabled_outputs = 0U;

	duty = Limit_Sat(duty, SIX_STEP_DUTY_MAX, 0.0f);
	if (duty <= 0.0f)
	{
		TIM1->CCER &= ~output_mask;
		return;
	}

	active_compare = (uint32_t)(duty * (float)TIM1_PERIOD);

	for (uint8_t phase = 0U; phase < 3U; phase++)
	{
		if ((vector & (1U << phase)) != 0U)
		{
			*compare[phase] = 0U;
			enabled_outputs |= high_enable[phase];
		}
		else
		{
			*compare[phase] = active_compare;
			enabled_outputs |= high_enable[phase] | low_enable[phase];
		}
	}
	TIM1->CCER = (TIM1->CCER & ~output_mask) | enabled_outputs;
}

#if SIX_STEP_METHOD == 1
static void SixStepApplyFloatingSector(uint8_t sector, float duty)
{
	static const uint8_t high_phase[6] = {0U, 0U, 1U, 1U, 2U, 2U};
	static const uint8_t low_phase[6] = {1U, 2U, 2U, 0U, 0U, 1U};
	static const uint32_t high_enable[3] = {
		TIM_CCER_CC1E, TIM_CCER_CC2E, TIM_CCER_CC3E};
	static const uint32_t low_enable[3] = {
		TIM_CCER_CC1NE, TIM_CCER_CC2NE, TIM_CCER_CC3NE};
	volatile uint32_t *compare[3] = {
		&TIM1->CCR1, &TIM1->CCR2, &TIM1->CCR3};
	uint32_t output_mask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
						   TIM_CCER_CC2E | TIM_CCER_CC2NE |
						   TIM_CCER_CC3E | TIM_CCER_CC3NE;
	uint32_t active_compare;
	uint32_t enabled_outputs = 0U;

	duty = Limit_Sat(duty, SIX_STEP_DUTY_MAX, 0.0f);
	if ((duty <= 0.0f) || (sector >= 6U))
	{
		TIM1->CCER &= ~output_mask;
		return;
	}

	active_compare = (uint32_t)(duty * (float)TIM1_PERIOD);

	for (uint8_t phase = 0U; phase < 3U; phase++)
	{
		if (phase == high_phase[sector])
		{
			/* PWM2 with CCR=0 holds the high-side switch on. */
			*compare[phase] = 0U;
			enabled_outputs |= high_enable[phase];
		}
		else if (phase == low_phase[sector])
		{
			/* The complementary pair PWM-drives the low phase. */
			*compare[phase] = active_compare;
			enabled_outputs |= high_enable[phase] | low_enable[phase];
		}
		else
		{
			/* Disable both switches so this phase is electrically floating. */
			*compare[phase] = 0U;
		}
	}

	TIM1->CCER = (TIM1->CCER & ~output_mask) | enabled_outputs;
}
#endif

static void SixStepCommutation(float electrical_angle, float speed_ref, float duty)
{
	/* Bit 0/1/2 represent U/V/W; these are the six active inverter vectors. */
	static const uint8_t active_vector[6] = {
		0x1U, 0x3U, 0x2U, 0x6U, 0x4U, 0x5U};
	uint8_t direction;
	uint8_t sector;
	float commutation_angle;
	float sector_offset;

	if ((speed_ref == 0.0f) || (duty <= 0.0f))
	{
		SixStepApplyVector(0U, 0.0f);
		return;
	}

	direction = (speed_ref > 0.0f) ? 0U : 1U;
	/* Center the selected active vector at the configured electrical advance. */
	sector_offset = (direction == 0U)
						? (SIX_STEP_ADVANCE_ANGLE_DEG + 30.0f)
						: (30.0f - SIX_STEP_ADVANCE_ANGLE_DEG);
	commutation_angle = electrical_angle + sector_offset * PIX2 / 360.0f;
	while (commutation_angle >= PIX2)
		commutation_angle -= PIX2;
	while (commutation_angle < 0.0f)
		commutation_angle += PIX2;

	sector = (uint8_t)(commutation_angle / (PIX2 / 6.0f));
	if (sector > 5U)
		sector = 5U;
	SixStepApplyVector(active_vector[sector], duty);
}

#if SIX_STEP_METHOD == 1
static void SixStepCommutationFloating(float electrical_angle, float speed_ref, float duty)
{
	uint8_t direction;
	uint8_t sector;
	float commutation_angle;
	float sector_offset;

	if ((speed_ref == 0.0f) || (duty <= 0.0f))
	{
		SixStepApplyFloatingSector(0U, 0.0f);
		return;
	}

	direction = (speed_ref > 0.0f) ? 0U : 1U;
	sector_offset = (direction == 0U)
						? (SIX_STEP_ADVANCE_ANGLE_DEG + 30.0f)
						: (30.0f - SIX_STEP_ADVANCE_ANGLE_DEG);
	commutation_angle = electrical_angle + sector_offset * PIX2 / 360.0f;
	while (commutation_angle >= PIX2)
		commutation_angle -= PIX2;
	while (commutation_angle < 0.0f)
		commutation_angle += PIX2;

	sector = (uint8_t)(commutation_angle / (PIX2 / 6.0f));
	if (sector > 5U)
		sector = 5U;
	SixStepApplyFloatingSector(sector, duty);
}
#endif

// 控制模式选择
void MotorModeSel(void)
{
	static uint8_t openloop_active = 0U;
	static uint32_t openloop_align_ticks = 0U;
	static uint32_t openloop_align_ticks_required = 0U;
	static float openloop_angle = 0.0f;
	static float openloop_duty = 0.0f;
	static uint8_t sixstep_speed_active = 0U;
	static uint8_t sixstep_speed_calc_ticks = 0U;
	static float sixstep_speed_integrator = 0.0f;
	static float sixstep_speed_duty = 0.0f;
	static uint8_t sixstep_current_active = 0U;
	static float sixstep_current_integrator = 0.0f;

	if (Motor.CtrlMode != SIX_STEP_OPENLOOP)
	{
		openloop_active = 0U;
		openloop_align_ticks = 0U;
		openloop_align_ticks_required = 0U;
		openloop_duty = 0.0f;
	}
	if (Motor.CtrlMode != SIX_STEP_SPEED_LOOP)
	{
		sixstep_speed_active = 0U;
		sixstep_speed_calc_ticks = 0U;
		sixstep_speed_integrator = 0.0f;
		sixstep_speed_duty = 0.0f;
	}
	if (Motor.CtrlMode != SIX_STEP_CURRENT_LOOP)
	{
		sixstep_current_active = 0U;
		sixstep_current_integrator = 0.0f;
	}

	// 控制入口：根据 CtrlMode 执行对应闭环，再统一进入 SVPWM 输出
	switch (Motor.CtrlMode)
	{
	case SPEED_LOOP:
	{
		SpeedCloseloop(Para.SpeedRef, 1);
		FocSvpwm(); // SVPWM计算
		break;
	}

	case SIX_STEP_SPEED_LOOP:
	{
		// 获取三相电流
		float phase_current = AdcPara.CurU;
		float current_v = AdcPara.CurV;
		float current_w = AdcPara.CurW;
		float speed_ref;
		float speed_fbk;
		float speed_error;
		float duty;

		if (phase_current < 0.0f)
			phase_current = -phase_current;
		if (current_v < 0.0f)
			current_v = -current_v;
		if (current_w < 0.0f)
			current_w = -current_w;
		// 确定三相电流最大值，做过流检测
		if (current_v > phase_current)
			phase_current = current_v;
		if (current_w > phase_current)
			phase_current = current_w;

		if (sixstep_speed_active == 0U)
		{
			sixstep_speed_active = 1U;
			sixstep_speed_calc_ticks = 0U;
			sixstep_speed_integrator = 0.0f;
			sixstep_speed_duty = 0.0f;
			SpdRefGXieLv.XieLv_Y = 0.0f;
			SpdRefGXieLv.XieLv_Grad = SPD_REF_OD;
			SpdRefGXieLv.Grad_Timer = SPD_REF_TIMER;
			SpdRefGXieLv.Timer_Count = 0U;

			/* The six-step controller uses its own duty PI, not the FOC current PI. */
			pi_spd.Ref = 0.0f;
			pi_spd.Fbk = 0.0f;
			pi_spd.err = 0.0f;
			pi_spd.err_l = 0.0f;
			pi_spd.Out = 0.0f;
			pi_spd.OutF = 0.0f;
			pi_spd.integrator = 0.0f;
		}
		// 目标速度限幅
		SpdRefGXieLv.XieLv_X =
			Limit_Sat((float)Para.SpeedRef, SPD_REF_MAX, -SPD_REF_MAX);
		SpdRefGXieLv.Timer_Count++;// 计数超过Grad_Timer更新一次斜坡
		if (SpdRefGXieLv.Timer_Count > SpdRefGXieLv.Grad_Timer)
		{
			SpdRefGXieLv.Timer_Count = 0U;
			GradXieLv(&SpdRefGXieLv);
		}
		speed_ref = SpdRefGXieLv.XieLv_Y;
		// 速度环每累计SPD_CAL_PERIOD更新一次(20khz/SPD_CAL_PERIOD=1khz)
		if (++sixstep_speed_calc_ticks >= SPD_CAL_PERIOD)
		{
			float direction = (speed_ref < 0.0f) ? -1.0f : 1.0f;
			float candidate_integrator;
			float unsaturated_duty;
			const float speed_loop_period = TS * (float)SPD_CAL_PERIOD;

			sixstep_speed_calc_ticks = 0U;
			speed_fbk = Motor.speed_M_rpm * direction;
			speed_error = my_abs(speed_ref) - speed_fbk;

			/* Keep the existing speed telemetry and fault checks in electrical RPM. */
			pi_spd.Ref = speed_ref * (float)Motor.P;
			pi_spd.Fbk = Motor.speed_E_rpm;
			pi_spd.err = pi_spd.Ref - pi_spd.Fbk;

			if (my_abs(speed_ref) < 0.5f)
			{
				sixstep_speed_integrator = 0.0f;
				sixstep_speed_duty = 0.0f;
			}
			else
			{
				if (phase_current < OVER_CURRENT)
				{
					candidate_integrator = sixstep_speed_integrator +
										  Para.SpdSenKi * speed_error * speed_loop_period;
					unsaturated_duty = Para.SpdSenKp * speed_error + candidate_integrator;

					/* Prevent further windup when duty or current is limited. */
					if (!((unsaturated_duty >= SIX_STEP_DUTY_MAX && speed_error > 0.0f) ||
						  (unsaturated_duty <= 0.0f && speed_error < 0.0f)))
					{
						sixstep_speed_integrator = candidate_integrator;
					}
				}

				sixstep_speed_duty = Limit_Sat(
					Para.SpdSenKp * speed_error + sixstep_speed_integrator,
					SIX_STEP_DUTY_MAX, 0.0f);
			}
		}

		duty = sixstep_speed_duty;
		if (phase_current >= OVER_CURRENT)
			duty *= (0.9f * OVER_CURRENT) / phase_current;

		/* The calibrated closed-loop phase was observed to produce reverse torque. */
#if SIX_STEP_METHOD == 0
		SixStepCommutation(
			Motor.E_theta + SIX_STEP_CLOSEDLOOP_PHASE_OFFSET_DEG * PIX2 / 360.0f,
			speed_ref, duty);
#elif SIX_STEP_METHOD == 1
		SixStepCommutationFloating(
			Motor.E_theta + SIX_STEP_CLOSEDLOOP_PHASE_OFFSET_DEG * PIX2 / 360.0f,
			speed_ref, duty);
#else
#error SIX_STEP_METHOD must be 0 or 1
#endif
		break;
	}

	case SIX_STEP_CURRENT_LOOP:
	{
		float current_ref = Limit_Sat(
			Para.CurRef, SIX_STEP_CURRENT_REF_MAX, -SIX_STEP_CURRENT_REF_MAX);
		float current_feedback = 0.0f;
		float current_peak = 0.0f;
		float current_error;
		float speed_ref = current_ref;
		float duty = 0.0f;
		float vbus = AdcPara.Vbus;
		float phase_current;
		float current_u = AdcFilPara.CurU;
		float current_v = AdcFilPara.CurV;
		float current_w = AdcFilPara.CurW;
		float raw_current_u = AdcPara.CurU;
		float raw_current_v = AdcPara.CurV;
		float raw_current_w = AdcPara.CurW;

		if (current_u < 0.0f)
			current_u = -current_u;
		if (current_v < 0.0f)
			current_v = -current_v;
		if (current_w < 0.0f)
			current_w = -current_w;
		/* 三相三线制下 |Iu|+|Iv|+|Iw| = 2*导通相电流，抗浮空相噪声优于取最大值 */
		current_feedback = (current_u + current_v + current_w) * 0.5f;

		if (raw_current_u < 0.0f)
			raw_current_u = -raw_current_u;
		if (raw_current_v < 0.0f)
			raw_current_v = -raw_current_v;
		if (raw_current_w < 0.0f)
			raw_current_w = -raw_current_w;
		current_peak = raw_current_u;
		if (raw_current_v > current_peak)
			current_peak = raw_current_v;
		if (raw_current_w > current_peak)
			current_peak = raw_current_w;

		if (sixstep_current_active == 0U)
		{
			sixstep_current_active = 1U;
			sixstep_current_integrator = 0.0f;
		}

		phase_current = my_abs(current_ref);
		current_error = phase_current - current_feedback;
		if ((phase_current >= 0.05f) &&
			(vbus > 1.0f) &&
			(current_peak < OVER_CURRENT) &&
			(my_abs(Motor.speed_M_rpm) < SIX_STEP_CURRENT_MAX_SPEED_RPM))
		{
			float max_voltage = vbus * SIX_STEP_DUTY_MAX;
			float candidate_integrator = Limit_Sat(
				sixstep_current_integrator +
					SIX_STEP_CURRENT_KI * current_error * TS,
				max_voltage, 0.0f);
			float unsaturated_voltage =
				SIX_STEP_CURRENT_KP * current_error + candidate_integrator;

			if (!((unsaturated_voltage >= max_voltage) && (current_error > 0.0f)))
				sixstep_current_integrator = candidate_integrator;

			duty = Limit_Sat(
					   SIX_STEP_CURRENT_KP * current_error +
						   sixstep_current_integrator,
					   max_voltage, 0.0f) /
				   vbus;
		}
		else
		{
			/* 不在过流/超速时硬清零积分器，缓慢泄放避免占空比崩溃形成极限环 */
			sixstep_current_integrator *= 0.9f;
		}

		/* Signed current reference selects commutation direction; magnitude is the current target. */
#if SIX_STEP_METHOD == 0
		SixStepCommutation(
			Motor.E_theta + SIX_STEP_CLOSEDLOOP_PHASE_OFFSET_DEG * PIX2 / 360.0f,
			speed_ref, duty);
#elif SIX_STEP_METHOD == 1
		SixStepCommutationFloating(
			Motor.E_theta + SIX_STEP_CLOSEDLOOP_PHASE_OFFSET_DEG * PIX2 / 360.0f,
			speed_ref, duty);
#else
#error SIX_STEP_METHOD must be 0 or 1
#endif
		break;
	}

	case SIX_STEP_OPENLOOP:
	{
		float duty_target = Limit_Sat(SIX_STEP_OPENLOOP_DUTY, SIX_STEP_DUTY_MAX, 0.0f);
		float duty;
		// 获取三相电流值，用于后续过流限制
		float phase_current = AdcPara.CurU;
		float current_v = AdcPara.CurV;
		float current_w = AdcPara.CurW;
		float speed_ref = SIX_STEP_OPENLOOP_SPEED_RPM;
		
		// 三相电流取绝对值
		if (phase_current < 0.0f)
			phase_current = -phase_current;
		if (current_v < 0.0f)
			current_v = -current_v;
		if (current_w < 0.0f)
			current_w = -current_w;
		// 三相电流取最大值，用于判断过流
		if (current_v > phase_current)
			phase_current = current_v;
		if (current_w > phase_current)
			phase_current = current_w;

		if (openloop_active == 0U)
		{
			openloop_active = 1U;
			openloop_align_ticks = 0U;
			openloop_align_ticks_required =
				(uint32_t)((float)SIX_STEP_OPENLOOP_ALIGN_TIME_MS / (TS * 1000.0f));
			openloop_angle = 0.0f;
			openloop_duty = 0.0f;
		}

		/* Back off above the limit; recover below 80% to avoid duty chatter. */
		// 当三相电流超过过流阈值后，降低占空比起到限流作用
		if (phase_current >= OVER_CURRENT)
		{
			openloop_duty *= (0.9f * OVER_CURRENT) / phase_current;
		}
		// 电流正常时，缓慢增加占空比，直到达到目标占空比
		else if ((phase_current < 0.8f * OVER_CURRENT) && (openloop_duty < duty_target))
		{
			openloop_duty += 0.001f; // 每次占空比增加 0.1%，直到达到目标占空比
			if (openloop_duty > duty_target)
				openloop_duty = duty_target;
		}
		duty = openloop_duty;

		if (openloop_align_ticks < openloop_align_ticks_required)
		{
#if SIX_STEP_METHOD == 0
			SixStepApplyVector(0x1U, duty); // 0x1U，二进制001，对应控制时的U相激励
#elif SIX_STEP_METHOD == 1
			SixStepApplyFloatingSector(0U, duty);
#else
#error SIX_STEP_METHOD must be 0 or 1
#endif
			openloop_align_ticks++;
			break;
		}

		if (openloop_align_ticks == openloop_align_ticks_required)
			openloop_align_ticks++;

#if SIX_STEP_METHOD == 0
		SixStepCommutation(openloop_angle, speed_ref, duty);
#elif SIX_STEP_METHOD == 1
		SixStepCommutationFloating(openloop_angle, speed_ref, duty);
#endif
		// 模拟电角度更新
		openloop_angle += speed_ref * (float)Motor.P * PIX2 * TS / 60.0f;
		while (openloop_angle >= PIX2)
			openloop_angle -= PIX2;
		while (openloop_angle < 0.0f)
			openloop_angle += PIX2;
		break;
	}

	case CUR_LOOP:
	{
		CurCloseloop(Para.CurRef, Motor.E_theta);
		FocSvpwm(); // SVPWM计算
		break;
	}

	case POS_LOOP:
	{
		PosCloseloop();
		FocSvpwm(); // SVPWM计算
		break;
	}

	case IF_CTRL:
	{
		switch (QT_IF_MODE)
		{
		case 1: // 强拖至角度
		{
			QtMotor();	// 电机强拖控制
			FocSvpwm(); // SVPWM计算
			if (Para.IfAngle == QT_ANGLE)
			{
				MotorStop(); // 停机
				Para.IfAngle = 0;
				Para.IfFrq = 0;
			}
			break;
		}
		case 2: // 强拖旋转
		{
			QtMotor();	// 电机强拖控制
			FocSvpwm(); // SVPWM计算
			break;
		}
		case 3: // IF启动
		{
			IFStartControl(); //	I/F流频比控制
			FocSvpwm();
			break;
		}
		default:
			break;
		}
		break;
	}

	case HFI_CTRL:
	{
		if (HFI_Init_Angle_flag) // 初始角度检测完毕后
		{
			HfiSpdCloseLoop(); // HFI速度闭环模式
		}
		else
		{
			HfiInitTheta(); // HFI初始角度检测
		}

		FocSvpwm(); // SVPWM计算
		break;
	}

	case HFI_SMO: // 低速HFI，高速SMO的全速域无感控制
	{
		LessHfiSmoControl();
		FocSvpwm(); // SVPWM计算
		break;
	}

	case HFI_FLUX: // 低速HFI，高速FLUX的全速域无感控制
	{
		LessHfiFluxControl();
		FocSvpwm();
		break;
	}

	case IF_SMO:
	{
		if (SensorLess.Observer == 1)
		{
			// 低速阶段使用 IF 开环角度和固定电流启动
			IFObserverThetaRampCale();
			CurCloseloop(1.0f, Para.FocAngle);
		}
		else
		{
			// 高速阶段使用 SMO 速度闭环
			SensorLess.Speed_Ref = GetObserverSpeedRef();
			SmoObserver(Park.Ds, Park.Qs);
		}
		FocSvpwm();
		Para.SpeedCalTime %= SPD_CAL_PERIOD;
		break;
	}

	case IF_FLUX:
	{
		if (SensorLess.Observer == 1)
		{
			// 低速阶段使用 IF 开环角度和固定电流启动
			IFObserverThetaRampCale();
			CurCloseloop(1.0f, Para.FocAngle);
		}
		else
		{
			// 高速阶段使用 Flux 速度闭环
			SensorLess.Speed_Ref = GetObserverSpeedRef();
			FluxObserver(Park.Ds, Park.Qs);
		}
		FocSvpwm();
		Para.SpeedCalTime %= SPD_CAL_PERIOD;
		break;
	}
	case TO_ZERO:
	{
		break;
	}

	default:
	{
		MotorStop(); // 停机
		break;
	}
	}
}

// 变量清零
void VariableClear(void)
{
	// 停机或重新启动前清除运行状态，避免旧积分和旧角度影响下次启动
	FluxFlag = 0;
	Para.FluxSpeedCnt = 0;
	Para.RpmCmd = 0;
	Para.FocAngle = 0;

	Motor.speed_M_rpm = 0; // 电机机械角速度清零
	Motor.speed_E_rpm = 0; // 电机电角速度清零

	// Motor.CtrlMode = 0;	   // 电机控制模式清零

	Para.IfFrq = 0; // 强拖频率计数清零

	// 速度、电流、位置 PI 控制器状态清零
	pi_spd.Ref = 0;
	pi_spd.Fbk = 0;
	pi_spd.err = 0;
	pi_spd.err_l = 0;
	pi_spd.Out = 0;
	pi_spd.OutF = 0;
	pi_spd.integrator = 0;

	pi_id.Ref = 0;
	pi_id.Fbk = 0;
	pi_id.err = 0;
	pi_id.Out = 0;
	pi_id.OutF = 0;
	pi_id.err_l = 0;
	pi_id.integrator = 0;

	pi_iq.Ref = 0;
	pi_iq.Fbk = 0;
	pi_iq.err = 0;
	pi_iq.Out = 0;
	pi_iq.OutF = 0;
	pi_iq.err_l = 0;
	pi_iq.integrator = 0;

	pi_pos.Ref = 0;
	pi_pos.Fbk = 0;
	pi_pos.err = 0;
	pi_pos.Out = 0;
	pi_pos.OutF = 0;
	pi_pos.err_l = 0;
	pi_pos.integrator = 0;

	// IF、速度、电流给定斜率规划状态清零
	IfFreqGXieLv.Timer_Count = 0;
	Para.IfTheta = 0;
	IfFreqGXieLv.XieLv_Y = 0;

	SpdRefGXieLv.Timer_Count = 0;
	SpdRefGXieLv.XieLv_Y = 0;

	IdqRefGXieLv.Timer_Count = 0;
	IdqRefGXieLv.XieLv_Y = 0;

	Ipark.Alpha = 0;
	Ipark.Beta = 0;
	Ipark.Ds = 0;
	Ipark.Qs = 0;

	// 逆 Park 与 SVPWM 输出缓存清零
	Svpwm.Ualpha = 0;
	Svpwm.Ubeta = 0;

	// SMO/HFI PLL 状态清零
	PllSmoPara.Theta = 0;
	PllSmoPara.Angle = 0;
	PllSmoPara.Theta_Err = 0;
	PllSmoPara.Theta_Err_last = 0;
	PllSmoPara.Omega = 0;
	PllSmoPara.Omega_F = 0;
	PllSmoPara.Omega2 = 0;
	PllSmoPara.Omega2_F = 0;
	PllSmoPara.Theta_last = 0;

	PllHfiPara.Theta = 0;
	PllHfiPara.Theta_Err = 0;
	PllHfiPara.Theta_Err_last = 0;
	PllHfiPara.Omega = 0;
	PllHfiPara.Omega_F = 0;
	PllHfiPara.Omega2 = 0;
	PllHfiPara.Omega2_F = 0;
	PllHfiPara.Theta_last = 0;
	PllHfiPara.Angle = 0;

	// HFI 与 SMO 内部采样/估算状态清零
	HFI_Init_Angle_flag = 0;
	HFI.Ialpha_last = 0;
	HFI.Ibeta_last = 0;
	HFI.Id_last = 0;
	HFI.Iq_last = 0;
	HFI.Uin = 0;
	AngleSmoPara.Est_Ialpha = 0;
	AngleSmoPara.Est_Ibeta = 0;
	AngleSmoPara.Ialpha_Err = 0;
	AngleSmoPara.Ibeta_Err = 0;
	AngleSmoPara.Zalpha = 0;
	AngleSmoPara.Zbeta = 0;
	AngleSmoPara.Ealpha = 0;
	AngleSmoPara.Ebeta = 0;
	HfiUinFlag = 0;

	// 无感观测器状态回到低速默认状态
	SensorLess.Observer = 1;
	SensorLess.Observer_last = 1;
	SensorLess.theta = 0;
	SensorLess.Omega = 0;
	SensorLess.Speed_Fbk = 0;
	SensorLess.Speed_Ref = 0;

	FluxReset(&flux);

	// 观测器切换计数与标志清零
	Para.SpeedCalTime = 0;
	Para.UpTime = 0;
	Para.DownTime = 0;
	Para.CalTime = 0;
	Para.ObsSwFlag = 0;
	Para.SmoSwFlag = 0;
}
