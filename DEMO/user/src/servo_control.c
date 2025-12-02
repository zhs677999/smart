#include "zf_common_headfile.h"
#include "my_common.h"
#include <math.h>

// 当前舵机目标角度，默认保持在中值
float servo_motor_angle = SERVO_MOTOR_M;

// -----------------------------------------------------------
// PD 调节参数
// -----------------------------------------------------------
// 基于归一化误差重新整定，整体增大舵机响应速度
float kp = 180.0f;
float kd = 70.5f;
// 锐角弯动态增益
static const float sharp_turn_error = 0.42f;
static const float sharp_turn_gain  = 7.8f;
static const float quick_turn_feedforward = 500.0f;
// 连续反向急弯的额外前馈，帮助舵机快速“打回来”
static const float sign_flip_threshold = 1.58f;
static const float sign_flip_boost     = 100.0f;
// 环岛进入时的舵机直行保持比例（靠计时器逐渐退出）
static const float roundabout_straight_ratio_high = 0.35f;
static const float roundabout_straight_ratio_low  = 0.05f;
// 环岛阶段允许的最大偏转角，先小后大，避免提前偏航
static const float roundabout_entry_offset_min = 20.0f;
static const float roundabout_entry_offset_max = 38.0f;
// -----------------------------------------------------------

extern float normalized_error;      // 归一化后的左右差值
extern float normalized_adc[ADC_CHANNEL_NUMBER];
float last_adc_error = 0;           // 上一次误差（用于 D 项与换向检测）
extern uint8_t roundabout_detected;
extern uint16_t roundabout_timer;

void set_servo_pwm()
{
    float kp_local = 180.0f;
    float kd_local = 70.5f;

    // 动态 PID：误差较大时提高响应
    if (fabsf(normalized_error) > 0.2f) {
        kp_local = 300.0f;  // 大误差时更激进
        kd_local = 20.0f;
    }

    kp = kp_local;
    kd = kd_local;

    // 基于整体左右强度差的急弯特征，先行放大误差
    float left_sum  = normalized_adc[0] + normalized_adc[1];
    float right_sum = normalized_adc[2] + normalized_adc[3];
    float lateral_balance = left_sum - right_sum;
    float strong_turn_weight = fminf(fabsf(lateral_balance) / 0.55f, 1.5f); // 2000/1930 vs 539/1141 对应约 0.55 差值

    // 非线性误差放大，对小误差敏感，对大误差饱和
    float error_gain = 2.0f + 0.6f * strong_turn_weight;
    if (fabsf(normalized_error) > 0.1f) {
        error_gain = 1.5f + 0.8f * strong_turn_weight;  // 大误差时增益更大
    }

    // 锐角弯时进一步提升比例系数
    float adaptive_kp = kp_local;
    if ((fabsf(normalized_error) > sharp_turn_error) || (fabsf(lateral_balance) > 0.35f)) {
        adaptive_kp *= sharp_turn_gain;
    }

    float enhanced_error = normalized_error * error_gain;
    float error_delta = enhanced_error - last_adc_error;

    float p_out = adaptive_kp * enhanced_error;
    float d_out = kd_local * error_delta;

    // 快速转向前馈：误差大的时候直接给舵机额外角度，减少响应延迟
    float quick_out = 0.0f;
    if ((fabsf(normalized_error) > sharp_turn_error) || (fabsf(lateral_balance) > 0.35f)) {
        float lateral_feedforward = quick_turn_feedforward * 0.85f * lateral_balance;
        quick_out = quick_turn_feedforward * normalized_error + lateral_feedforward;
    }

    // 连续左右急弯或换向时，额外给出“反打”前馈，提前让舵机回中换向
    if ((enhanced_error * last_adc_error < 0.0f) && (fabsf(enhanced_error) > sign_flip_threshold)) {
        quick_out += sign_flip_boost * enhanced_error;
    }

    float commanded_angle = SERVO_MOTOR_M - (p_out + d_out + quick_out);

    // 环岛进入：先保持直行，然后按计时器渐进恢复正常控制，同时限制最大偏转角，防止过早偏航
    if (roundabout_detected && roundabout_timer > 0) {
        float straight_ratio = roundabout_straight_ratio_low;
        if (roundabout_timer > (ROUNDABOUT_HOLD_TIME * 2 / 3)) {
            straight_ratio = roundabout_straight_ratio_high;
        }
        else if (roundabout_timer > (ROUNDABOUT_HOLD_TIME / 3)) {
            straight_ratio = (roundabout_straight_ratio_high + roundabout_straight_ratio_low) * 0.25f;
        }

        float entry_offset_limit = roundabout_entry_offset_min + (roundabout_entry_offset_max - roundabout_entry_offset_min) * (1.0f - straight_ratio);
        float deviation_from_center = commanded_angle - SERVO_MOTOR_M;
        if (fabsf(deviation_from_center) > entry_offset_limit) {
            deviation_from_center = copysignf(entry_offset_limit, deviation_from_center);
        }

        float softened_angle = SERVO_MOTOR_M + deviation_from_center;
        commanded_angle = SERVO_MOTOR_M * straight_ratio + softened_angle * (1.0f - straight_ratio);
    }

    servo_motor_angle = commanded_angle;

    // 3. 存储误差供下一周期使用
    last_adc_error = enhanced_error;

    // 4. 限幅 (防止舵机超行程)
    if(servo_motor_angle > SERVO_MOTOR_R_MAX) servo_motor_angle = SERVO_MOTOR_R_MAX;
    if(servo_motor_angle < SERVO_MOTOR_L_MAX) servo_motor_angle = SERVO_MOTOR_L_MAX;

    // 5. 输出 PWM
    pwm_set_duty(SERVO_MOTOR1_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
    pwm_set_duty(SERVO_MOTOR2_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
    pwm_set_duty(SERVO_MOTOR3_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
}