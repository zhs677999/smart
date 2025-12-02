#include "zf_common_headfile.h"
#include "my_common.h"
#include <math.h>

#include "pid.h"

// duty 以占空比百分比为单位
// 采用分段 + PID 组合调速
int8 duty = STRAIGHT_SPEED_DUTY;

static pid_controller_t speed_pid;
static uint8_t pid_initialized = 0;

extern float servo_motor_angle;
extern float normalized_error;
extern uint8_t finish_detected;
extern uint8_t off_track_detected;
extern uint8_t roundabout_detected;
extern int16 encoder_data_1;
extern int16 encoder_data_2;

void set_speed_pwm()
{
    if(!pid_initialized)
    {
        pid_init(&speed_pid, MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, MOTOR_PID_OUTPUT_LIMIT, MOTOR_PID_INTEGRAL_LIMIT);
        pid_initialized = 1;
    }

    // 根据舵机偏差设置分段速度，加入直道死区
    float steering_offset = servo_motor_angle - SERVO_MOTOR_M;
    float angle_delta = fabsf(steering_offset);
    int8 target_duty = duty;

    if(finish_detected || off_track_detected)
    {
        target_duty = 0; // 终点停下
    }
    else if(roundabout_detected)
    {
        target_duty = ROUNDABOUT_SPEED_DUTY;
    }
    else if(angle_delta <= STRAIGHT_DEAD_ZONE_DEG)
    {
        target_duty = STRAIGHT_SPEED_DUTY;
    }
    else
    {
        target_duty = CURVE_SPEED_DUTY;
    }

    // duty 允许外部动态调整，但要限幅
    if(target_duty > DUTY_MAX_LIMIT) target_duty = DUTY_MAX_LIMIT;
    if(target_duty < DUTY_MIN_LIMIT) target_duty = DUTY_MIN_LIMIT;
    duty = target_duty;

    // 读取编码器速度，取后轮平均值
    float speed_measure = (fabsf((float)encoder_data_1) + fabsf((float)encoder_data_2)) / 2.0f;

    // 目标速度采用编码器计数，匹配当前状态
    float target_count = get_target_count_from_state();

    float pid_out = pid_update(&speed_pid, target_count, speed_measure);

    // 将 PID 输出叠加在分段 duty 上
    float duty_with_pid = (float)target_duty + pid_out / (PWM_DUTY_MAX / 100.0f);

    if(duty_with_pid > DUTY_MAX_LIMIT) duty_with_pid = DUTY_MAX_LIMIT;
    if(duty_with_pid < 0) duty_with_pid = 0;

    float base_pwm_f = duty_with_pid * (PWM_DUTY_MAX / 100.0f);

    if(base_pwm_f > PWM_DUTY_MAX) base_pwm_f = PWM_DUTY_MAX;
    if(base_pwm_f < 0) base_pwm_f = 0;

    // 急转弯差速：根据舵机偏转和传感器误差加大内外轮速度差，帮助更快拐弯
    float steer_ratio = fminf(angle_delta / 10.0f, 1.0f);
    float error_ratio = fminf(fabsf(normalized_error) * 2.0f, 1.2f);
    float blended_turn = fminf(0.65f * steer_ratio + 0.35f * error_ratio, 1.2f);
    float diff_strength = fminf(blended_turn * blended_turn, 1.0f); // 高偏转时急剧放大
    const float outer_boost = 0.10f;    // 略微加速外轮，仍保留内轮减速以防过冲
    const float inner_cut   = 0.50f;   // 仅适度降速内轮

    float left_pwm_f  = base_pwm_f;
    float right_pwm_f = base_pwm_f;

    if(base_pwm_f > 0)
    {
        if(steering_offset >= 0) // 右转：左轮加速，右轮减速
        {
            left_pwm_f  = base_pwm_f * (1.0f + outer_boost * diff_strength);
            right_pwm_f = base_pwm_f * (1.0f - inner_cut * diff_strength);
        }
        else // 左转
        {
            left_pwm_f  = base_pwm_f * (1.0f - inner_cut * diff_strength);
            right_pwm_f = base_pwm_f * (1.0f + outer_boost * diff_strength);
        }
    }

    float max_pwm_f = (float)PWM_DUTY_MAX;
    if(left_pwm_f > max_pwm_f) left_pwm_f = max_pwm_f;
    if(right_pwm_f > max_pwm_f) right_pwm_f = max_pwm_f;
    if(left_pwm_f < 0) left_pwm_f = 0;
    if(right_pwm_f < 0) right_pwm_f = 0;

    int32_t left_pwm = (int32_t)(left_pwm_f);
    int32_t right_pwm = (int32_t)(right_pwm_f);

    // --- 电机驱动 ---
    gpio_set_level(MOTOR1_DIR, GPIO_HIGH);
    pwm_set_duty(MOTOR1_PWM, left_pwm);

    gpio_set_level(MOTOR2_DIR, GPIO_HIGH);
    pwm_set_duty(MOTOR2_PWM, right_pwm);

    // --- 后轮 ---
    gpio_set_level(MOTOR3_DIR, GPIO_HIGH);
    pwm_set_duty(MOTOR3_PWM, left_pwm);

    gpio_set_level(MOTOR4_DIR, GPIO_HIGH);
    pwm_set_duty(MOTOR4_PWM, right_pwm);
}