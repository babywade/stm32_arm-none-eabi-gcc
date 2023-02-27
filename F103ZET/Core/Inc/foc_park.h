#ifndef __FOC_PARK_H
#define __FOC_PARK_H

#include "foc.h"

// 数学变换部分
#define divSQRT_3 (s16)0x49E6 // 1/sqrt(3)的Q15格式,1/sqrt(3)*2^15=18918=0x49E6
#define SIN_MASK 0x0300
#define U0_90 0x0200
#define U90_180 0x0300
#define U180_270 0x0000
#define U270_360 0x0100
#define SQRT_3 1.732051
#define T (PWM_PERIOD * 4)
#define T_SQRT3 (u16)(T * SQRT_3)

// SVPWM部分
#define SECTOR_1 (u32)1
#define SECTOR_2 (u32)2
#define SECTOR_3 (u32)3
#define SECTOR_4 (u32)4
#define SECTOR_5 (u32)5
#define SECTOR_6 (u32)6
#define PWM2_MODE 0
#define PWM1_MODE 1
#define TW_AFTER ((u16)(((DEADTIME_NS+MAX_TNTR_NS)*168uL)/1000ul))
#define TW_BEFORE (((u16)(((((u16)(SAMPLING_TIME_NS)))*168uL)/1000ul))+1)
#define TNOISE_NS 1550 // 2.55usec
#define TRISE_NS 1550 // 2.55usec
#define SAMPLING_TIME_NS 700 // 700ns
#define SAMPLING_TIME (u16)(((u16)(SAMPLING_TIME_NS)*168uL)/1000uL)
#define TNOISE (u16)((((u16)(TNOISE_NS))*168uL)/1000uL)
#define TRISE (u16)((((u16)(TRISE_NS))*168uL)/1000uL)
#define TDEAD (u16)((DEADTIME*168uL)/1000uL)

#if (TNOISE_NS > TRISE_NS)
  #define MAX_TNTR_NS TNOISE_NS
#else 
  #define MAX_TNTR_NS TRISE_NS
#endif

// 函数声明
// 数学变换
Curr_Components Clarke(Curr_Components Curr_Input);
Trig_Components Trig_Functions(s16 hAngle);
Curr_Components Park(Curr_Components Curr_Input, s16 Theta);
Volt_Components Rev_Park(Volt_Components Volt_Input);
// SVPWM
void SVPWM_3ShuntCalDutyCycles(Volt_Components Stat_Volt_Input);
// 3电阻采样电流值
Curr_Components SVPWM_3ShuntGetPhaseCurrentValues();
// FOC核心
void FOC_Model(void);
// 系统初始化
void motor_init(void);
// PID初始化
void PID_Init (PID_Struct_t *PID_Torque, PID_Struct_t *PID_Flux, PID_Struct_t *PID_Speed);
// PID调节器
s16 PID_Regulator(s16 hReference, s16 hPresentFeedback, PID_Struct_t *PID_Struct);

// 变量定义部分
Trig_Components Vector_Components;
u8 bSector;
u8 PWM4Direction = PWM2_MODE;
s16 cnt = S16_MIN; // 开环调试变量

// FOC相关
Curr_Components Stat_Curr_a_b;
Curr_Components Stat_Curr_alfa_beta;
Curr_Components Stat_Curr_q_d;
Curr_Components Stat_Curr_q_d_ref_ref; // 电流环给定，用于电流环dq给定和前馈电流控制给定
Volt_Components Stat_Volt_q_d;
Volt_Components Stat_Volt_alfa_beta;
PID_Struct_t PID_Torque_InitStructure; // 转矩
PID_Struct_t PID_Flux_InitStructure;   // 磁通
PID_Struct_t PID_Speed_InitStructure;  // 速度

#endif