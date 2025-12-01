#include "zf_common_headfile.h"
#include "my_common.h"

// ????
float servo_motor_angle = SERVO_MOTOR_M; 

// -----------------------------------------------------------
// PD 调节参数
// -----------------------------------------------------------
// 基于归一化误差重新整定，整体增大舵机响应速度
float kp = 15.0f;
float kd = 0.0f;
// -----------------------------------------------------------

extern float normalized_error;      // 归一化后的左右差值
float last_adc_error = 0;           // 上一次误差（用于 D 项）

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
    
    float enhanced_error = normalized_error * error_gain;
    
    float p_out = kp * enhanced_error;
    float d_out = kd * (enhanced_error - last_adc_error);
    
    servo_motor_angle = SERVO_MOTOR_M - (p_out + d_out);
    
    
    // 3. ?????????
    last_adc_error = normalized_error;

    // 4. ?? (????????)
    if(servo_motor_angle > SERVO_MOTOR_R_MAX) servo_motor_angle = SERVO_MOTOR_R_MAX;
    if(servo_motor_angle < SERVO_MOTOR_L_MAX) servo_motor_angle = SERVO_MOTOR_L_MAX;

    // 5. ?? PWM
    pwm_set_duty(SERVO_MOTOR1_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
    pwm_set_duty(SERVO_MOTOR2_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
    pwm_set_duty(SERVO_MOTOR3_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
}