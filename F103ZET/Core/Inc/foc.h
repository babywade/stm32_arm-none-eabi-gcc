#ifndef __FOC_H
#define __FOC_H

#define CKTIM 168000000 // 定时器时钟频率168M Hz
#define PWM_PRSC
#define PWM_FREQ 15000 // PWM频率15K Hz
#define PWM_PERIOD CKTIM/(2*PWM_FREQ*(PWM_PRSC+1))
#define REP_RATE 1 //电流环刷新频率为(REP_RATE+1)/(2*PWM_FREQ)
#define DEADTIME_NS 1000 //死区时间ns 这个ns不是纳秒的意思，那是什么？
#define DEADTIME CKTIM/1000000/2*DEADTIME_NS/1000
#define POLE_PAIR_NUM 4 //极对数
#define ENCODER_PPR 1250 //编码器线数
#define ALIGNMENT_ANGLE 300
#define COUNTER_RESET (ALIGNMENT_ANGLE*4*ENCODER_PPR/360-1)/POLE_PAIR_NUM
#define ICx_FILTER 8

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef __IO uint32_t vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t vu8;

#define U8_MAX ((u8)255)
#define S8_MAX ((s8)127)
#define S8_MIN ((s8)-128)
#define U16_MAX ((u16)65535u)
#define S16_MAX ((s16)32767)
#define S16_MIN ((s16)-32768)
#define U32_MAX ((u32)4294967295uL)
#define S32_MAX ((s32)2147483647)
#define S32_MIN ((s32)-2147483647)

// 电流值结构体
typedef struct {
    s16 qI_Component1;
    s16 qI_Component2;
} Curr_Component;

// 电压值结构体
typedef struct {
    s16 qV_Component1;
    s16 qV_Component2;
} Void_Component;

// 角度结构体
typedef struct {
    s16 hCos;
    s16 hSin;
} Trig_Components;

// PID结构体



// PWM测试
void pwm_test();


#endif