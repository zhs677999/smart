#ifndef _mycommon_h
#define _mycommon_h

#include "zf_common_headfile.h"
#include "math.h"

// PIT 通道选择（修改时注意同步 isr.c 中的通道）
#define PIT_CH                      (PIT_CH0)
#define LED1                        (B9)
#define BEEP                        (B11)

#define CONTROL_PERIOD_MS           (5)   // 主控制周期（pit中断周期），用于计时逻辑

// 占空比限制
#define MAX_PWM_DUTY                (50)

// 电机引脚定义
#define MOTOR1_DIR                  (C9)
#define MOTOR1_PWM                  (PWM2_MODULE1_CHA_C8)

#define MOTOR2_DIR                  (C7)
#define MOTOR2_PWM                  (PWM2_MODULE0_CHA_C6)

#define MOTOR3_DIR                  (D2)
#define MOTOR3_PWM                  (PWM2_MODULE3_CHB_D3)

#define MOTOR4_DIR                  (C10)
#define MOTOR4_PWM                  (PWM2_MODULE2_CHB_C11)

// 编码器引脚定义
#define ENCODER_1                   (QTIMER1_ENCODER1)
#define ENCODER_1_A                 (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_1_B                 (QTIMER1_ENCODER1_CH2_C1)

#define ENCODER_2                   (QTIMER1_ENCODER2)
#define ENCODER_2_A                 (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_B                 (QTIMER1_ENCODER2_CH2_C24)

// 线传感器 ADC 通道
#define ADC_CHANNEL_NUMBER          (4)

#define ADC_CHANNEL1                (ADC1_CH12_B23)
#define ADC_CHANNEL2                (ADC1_CH10_B21)
#define ADC_CHANNEL3                (ADC1_CH4_B15)
#define ADC_CHANNEL4                (ADC1_CH3_B14)

// 舵机 PWM 定义
#define SERVO_MOTOR1_PWM            (PWM4_MODULE2_CHA_C30)                         // 舵机输出
#define SERVO_MOTOR2_PWM            (PWM1_MODULE3_CHA_D0)                          // 舵机输出
#define SERVO_MOTOR3_PWM            (PWM1_MODULE3_CHB_D1)                          // 舵机输出

#define SERVO_MOTOR_FREQ            (50)                                           // 舵机频率，范围 50-300

#define SERVO_MOTOR_L_MAX           (80)                                           // 左转极限
#define SERVO_MOTOR_R_MAX           (100)                                          // 右转极限
#define SERVO_MOTOR_M               (90)                                           // 中值

#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

#if (SERVO_MOTOR_FREQ<50 || SERVO_MOTOR_FREQ>300)
    #error "SERVO_MOTOR_FREQ ERROR!"
#endif

// 环岛检测
#define ROUNDABOUT_THRESHOLD        (800)
#define ROUNDABOUT_DEBOUNCE         (25)
#define ROUNDABOUT_HOLD_TIME        (150)
#define ROUNDABOUT_COOLDOWN         (300)

// 终点检测
#define FINISH_THRESHOLD            (0.85f)  // 归一化后的阈值
#define FINISH_DEBOUNCE             (40)     // 基于控制周期

// 归一化与滤波配置
#define ADC_FULL_SCALE              (4095.0f)
#define FILTER_ALPHA                (0.35f)  // 一阶低通滤波系数

// 速度分段控制
#define STRAIGHT_DEAD_ZONE_DEG      (3.0f)   // 直道死区，舵机偏差小于该角度时视为直道
#define STRAIGHT_SPEED_DUTY         (35)     // 直道目标 duty
#define CURVE_SPEED_DUTY            (26)     // 弯道目标 duty
#define ROUNDABOUT_SPEED_DUTY       (22)     // 环岛、终点等复杂场景的限速
#define DUTY_MAX_LIMIT              (50)
#define DUTY_MIN_LIMIT              (10)

// 电机 PID 参数（用于速度环）
#define MOTOR_PID_KP                (0.12f)
#define MOTOR_PID_KI                (0.02f)
#define MOTOR_PID_KD                (0.01f)
#define MOTOR_PID_OUTPUT_LIMIT      ((float)PWM_DUTY_MAX)
#define MOTOR_PID_INTEGRAL_LIMIT    (150.0f)

// 速度闭环期望计数（根据编码器分辨率可调）
#define TARGET_COUNT_STRAIGHT       (180.0f)
#define TARGET_COUNT_CURVE          (120.0f)
#define TARGET_COUNT_ROUNDABOUT     (90.0f)

// 对外暴露的检测状态
extern float filtered_adc[ADC_CHANNEL_NUMBER];
extern float normalized_adc[ADC_CHANNEL_NUMBER];
extern float normalized_error;
extern uint8_t finish_detected;
extern uint8_t roundabout_detected;
extern uint16_t roundabout_timer;
extern uint16_t roundabout_cooldown;

// 数据处理接口
void process_sensor_data(void);
float get_target_count_from_state(void);

#endif
