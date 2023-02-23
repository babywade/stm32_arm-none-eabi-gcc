#include "foc_park.h"

void motor_init() {
    // PWM初始化
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    // PWMN初始化
    HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,0);
    // 通道4触发ADC采样
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,1000);//初始占空比应该多少？
    // 开启ADC注入转换
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    // 使能ABC编码器
    HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start_IT(&htim2);
    // 初始化PID控制器
    PID_Init(&PID_Torque_InitStructure,&PID_Flux_InitStructure,&PID_Speed_InitStructure);
    State = START; // 状态机
}

// 电流环处理函数
void FOC_Model() {
    Stat_Curr_a_b = SVPWM_3ShuntGetPhaseCurrentValues(); // 读取两相的电流值
    Stat_Curr_alfa_beta = Clarke(Stat_Curr_a_b);
    Stat_Curr_q_d = Park(Stat_Curr_alfa_beta, ENC_Get_Electrical_Angle()); // 输入电角度，Ialpha和Ibeta，经过Park变换得到Iq，Id
    Stat_Volt_q_d.qV_Component1 = PID_Regulator(hTorque_Reference, Stat_Curr_q_d.qI_Component1, &PID_Torque_InitStructure);
    Stat_Volt_q_d.qV_Component2 = PID_Regulator(hFlux_Reference, Stat_Curr_q_d.qI_Component2, &PID_Flux_InitStructure);
    RevPark_Circle_Limitation(); // 归一化

    // 开环调试
    Stat_Volt_q_d.qV_Component1 = 0;
    Stat_Volt_q_d.qV_Component2 = 3000;
    cnt += 500;
    if (cnt > S16_MAX) {
        cnt = S16_MIN;
    }
    Vector_Components = Trig_Functions(cnt);
    Stat_Volt_alfa_beta = Rev_Park(Stat_Volt_q_d); // 反Park变换
    SVPWM_3ShuntCalDutyCycles(Stat_Volt_alfa_beta); // svpwm实现函数，实际的电流输出控制
}

// SVPWM
void SVPWM_3ShuntCalDutyCycles(Volt_Components Stat_Volt_Input) {
    s32 wX, wY, wZ, wUAlpha, wUBeta;
    u16 hTimePhA = 0;
    u16 hTimePhB = 0;
    u16 hTimePhC = 0;
    u16 hTimePhD = 0;
    u16 hDeltaDuty;

    wUAlpha = Stat_Volt_Input.qV_Component1 * T_SQRT3;
    wUBeta = -(Stat_Volt_Input.qV_Component2 * T);

    wX = wUBeta;
    wY = (wUBeta + wUAlpha) / 2;
    wZ = (wUBeta - wUAlpha) / 2;

    // Sector calculation from wX, WY, wZ
    
}