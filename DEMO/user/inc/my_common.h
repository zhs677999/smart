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

#define SERVO_MOTOR_L_MAX           (76)                                           // 左转极限
#define SERVO_MOTOR_R_MAX           (104)                                          // 右转极限
#define SERVO_MOTOR_M               (90)                                           // 中值

#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

#if (SERVO_MOTOR_FREQ<50 || SERVO_MOTOR_FREQ>300)
    #error "SERVO_MOTOR_FREQ ERROR!"
#endif

// 环岛检测
#define ROUNDABOUT_THRESHOLD        (800)
#define ROUNDABOUT_DEBOUNCE         (30)
#define ROUNDABOUT_HOLD_TIME        (200)
#define ROUNDABOUT_COOLDOWN         (300)
#define ROUNDABOUT_INERTIA_TIME     (0)     // 默认关闭惰性变向，改为锁死入环/出环
#define ROUNDABOUT_ENCODER_SPEED_MIN (40)   // 编码器速度下限：与电感双重验证环岛
#define ROUNDABOUT_ENCODER_TRAVEL_MIN (320) // 两次环岛判定之间至少行进的编码器计数
#define ROUNDABOUT_LOCK_ANGLE       (20.0f) // 入环时锁定打死的角度
#define ROUNDABOUT_LOCK_TIME        (160)   // 入环锁死持续时间（控制周期计）
#define ROUNDABOUT_EXIT_ANGLE       (16.0f) // 出环时保持的开环角度
#define ROUNDABOUT_EXIT_TIME        (120)   // 出环开环持续时间
// 环岛绕行逻辑
#define ROUNDABOUT_LAP_MIN_TIME     (500)   // 需要至少绕行一段时间后再寻找出口
#define ROUNDABOUT_MAX_LAP_TIME     (1600)  // 超时保护，防止一直卡在环岛状态
#define ROUNDABOUT_EXIT_CONFIRM     (18)    // 检测到出口模式的防抖计数
#define ROUNDABOUT_EXIT_GAP_SOFT    (0.38f) // 绕行一圈后的出口放宽：允许更暗的中右
#define ROUNDABOUT_EXIT_LEFT_MIN    (0.50f) // 仍需保持外圈亮度，避免误将急弯识别为出口
#define ROUNDABOUT_EXIT_RIGHT_MIN   (0.40f)
#define ROUNDABOUT_EXIT_OUTER_DIFF  (0.20f)
#define ROUNDABOUT_EXIT_MID_DIFF    (0.14f)
// 出口原始值辅助通道：依据实测（1488/1641/1103/1099 → 3602/3594/3595/2709 → 3585/3617/2334/1855 → 2307/107/803/3571）
#define ROUNDABOUT_EXIT_RAW_PLATEAU_L   (3000) // 接近切点时四路普遍升高
#define ROUNDABOUT_EXIT_RAW_PLATEAU_LM  (3000)
#define ROUNDABOUT_EXIT_RAW_PLATEAU_RM  (2000)
#define ROUNDABOUT_EXIT_RAW_PLATEAU_R   (1600)
#define ROUNDABOUT_EXIT_RAW_CUT_L       (1800) // 切点瞬间左外/右外仍高亮
#define ROUNDABOUT_EXIT_RAW_CUT_R       (2500)
#define ROUNDABOUT_EXIT_RAW_CUT_LM      (400)  // 左中、右中迅速变暗
#define ROUNDABOUT_EXIT_RAW_CUT_RM      (1200)
// 环岛判定：结合原始值特征，避免与急弯/十字路混淆
#define ROUNDABOUT_RAW_GAP_STRONG   (180)   // 中右传感器极低，进环岛标志
#define ROUNDABOUT_RAW_GAP_SOFT     (500)   // 中右明显变暗
#define ROUNDABOUT_OUTER_L_HIGH     (2100)  // 外侧左需高亮
#define ROUNDABOUT_OUTER_R_HIGH     (1800)  // 外侧右需高亮（排除急弯时右侧变暗）
#define ROUNDABOUT_MID_HIGH         (1200)  // 中左保持亮度，表示尚未偏出赛道
#define ROUNDABOUT_LEFT_SPIKE       (3200)  // 到切点时左侧会冲高
#define ROUNDABOUT_RIGHT_SUPPORT    (2400)  // 切点时右外侧仍保持高亮

// 终点检测
#define FINISH_THRESHOLD            (0.85f)  // 归一化后的阈值
#define FINISH_DEBOUNCE             (40)     // 基于控制周期
#define FINISH_HALL_PIN             (D4)     // 霍尔终点检测引脚
#define FINISH_HALL_DEBOUNCE        (2)      // 霍尔信号变沿计数阈值（结合亮线判定）

// 归一化与滤波配置
#define ADC_FULL_SCALE              (4095.0f)
#define FILTER_ALPHA                (0.20f)  // 一阶低通滤波系数

// 失线检测（原始值全低则停车）
#define OFF_TRACK_THRESHOLD_RAW      (100)
#define OFF_TRACK_DEBOUNCE           (5)

// 速度分段控制
#define STRAIGHT_DEAD_ZONE_DEG      (2.0f)   // 直道死区，舵机偏差小于该角度时视为直道
#define STRAIGHT_SPEED_DUTY         (40)     // 直道目标 duty
#define CURVE_SPEED_DUTY            (28)     // 弯道目标 duty，降低过弯速度防止打滑
#define ROUNDABOUT_SPEED_DUTY       (26)     // 环岛、终点等复杂场景的限速
#define DUTY_MAX_LIMIT              (50)
#define DUTY_MIN_LIMIT              (10)

// 电机 PID 参数（用于速度环）
#define MOTOR_PID_KP                (0.22f)
#define MOTOR_PID_KI                (0.06f)
#define MOTOR_PID_KD                (0.03f)
#define MOTOR_PID_OUTPUT_LIMIT      ((float)PWM_DUTY_MAX)
#define MOTOR_PID_INTEGRAL_LIMIT    (150.0f)

// 速度闭环期望计数（根据编码器分辨率可调）
#define TARGET_COUNT_STRAIGHT       (180.0f)
#define TARGET_COUNT_CURVE          (105.0f)
#define TARGET_COUNT_ROUNDABOUT     (90.0f)

// 环岛状态机（便于打标记调试进/出环）
typedef enum
{
    ROUND_STATE_IDLE = 0,
    ROUND_STATE_ENTRY_LOCK,
    ROUND_STATE_LAP,
    ROUND_STATE_EXIT_SEARCH,
} roundabout_state_t;

// 对外暴露的检测状态
extern float filtered_adc[ADC_CHANNEL_NUMBER];
extern float normalized_adc[ADC_CHANNEL_NUMBER];
extern float normalized_error;
extern uint16_t raw_adc[ADC_CHANNEL_NUMBER];
extern uint8_t finish_detected;
extern uint8_t off_track_detected;
extern uint8_t roundabout_detected;
extern uint16_t roundabout_timer;
extern uint16_t roundabout_cooldown;
extern uint16_t roundabout_lap_timer;
extern roundabout_state_t roundabout_state;
extern uint8_t roundabout_entry_mark;
extern uint8_t roundabout_exit_mark;

// 数据处理接口
void process_sensor_data(void);
float get_target_count_from_state(void);

#endif
