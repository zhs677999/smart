#include "zf_common_headfile.h"
#include "my_common.h"

// ????
float servo_motor_angle = SERVO_MOTOR_M; 

// -----------------------------------------------------------
// PD 调节参数
// -----------------------------------------------------------
// 基于归一化误差重新整定，整体增大舵机响应速度
float kp = 15.0f;
float kd = 5.0f;
// 锐角弯动态增益
static const float sharp_turn_error = 0.35f;
static const float sharp_turn_gain  = 1.6f;
static const float quick_turn_feedforward = 18.0f;
// 环岛进入时的舵机直行保持比例（靠计时器逐渐退出）
static const float roundabout_straight_ratio_high = 0.85f;
static const float roundabout_straight_ratio_low  = 0.35f;
// -----------------------------------------------------------

extern float normalized_error;      // 归一化后的左右差值
float last_adc_error = 0;           // 上一次误差（用于 D 项）
extern uint8_t roundabout_detected;
extern uint16_t roundabout_timer;

void set_servo_pwm()
{
/*动态pid
	if (fabsf(normalized_error) > 0.3f) {
    kp = 18.0f;  // 大误差时更激进
    kd = 0.0f;   //
}  */
	
	
	
    // 非线性误差放大，对小误差敏感，对大误差饱和
    float error_gain = 1.0f;
    if (fabsf(normalized_error) > 0.2f) {
        error_gain = 1.5f;  // 大误差时增益更大
    }

    // 锐角弯时进一步提升比例系数
    float adaptive_kp = kp;
    if (fabsf(normalized_error) > sharp_turn_error) {
        adaptive_kp *= sharp_turn_gain;
    }

    float enhanced_error = normalized_error * error_gain;
    float error_delta = enhanced_error - last_adc_error;

    float p_out = adaptive_kp * enhanced_error;
    float d_out = kd * error_delta;

    // 快速转向前馈：误差大的时候直接给舵机额外角度，减少响应延迟
    float quick_out = 0.0f;
    if (fabsf(normalized_error) > sharp_turn_error) {
        quick_out = quick_turn_feedforward * normalized_error;
    }

    float commanded_angle = SERVO_MOTOR_M - (p_out + d_out + quick_out);

    // 环岛进入：先保持直行，然后按计时器渐进恢复正常控制
    if (roundabout_detected && roundabout_timer > 0) {
        float straight_ratio = roundabout_straight_ratio_low;
        if (roundabout_timer > (ROUNDABOUT_HOLD_TIME * 2 / 3)) {
            straight_ratio = roundabout_straight_ratio_high;
        }
        else if (roundabout_timer > (ROUNDABOUT_HOLD_TIME / 3)) {
            straight_ratio = (roundabout_straight_ratio_high + roundabout_straight_ratio_low) * 0.5f;
        }

        commanded_angle = SERVO_MOTOR_M * straight_ratio + commanded_angle * (1.0f - straight_ratio);
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