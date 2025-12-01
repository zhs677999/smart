#include "pid.h"

// ???PID??
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_out, float max_int) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_output = max_out;
    pid->max_integral = max_int;
    pid->error = 0;
    pid->last_error = 0;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->output = 0;
}

// ??? PID ?? (??????????)
// ???? PWM ? ?? (???)
float PID_Calc_Incremental(PID_TypeDef *pid, float target, float feedback) {
    pid->error = target - feedback;
    
    float p_out = pid->kp * (pid->error - pid->last_error);
    float i_out = pid->ki * pid->error;
    float d_out = pid->kd * (pid->error - 2 * pid->last_error + pid->prev_error);
    
    float delta_output = p_out + i_out + d_out;
    
    pid->output += delta_output;
    
    // ??
    if (pid->output > pid->max_output) pid->output = pid->max_output;
    if (pid->output < -pid->max_output) pid->output = -pid->max_output; // ???????,?????0
    
    pid->prev_error = pid->last_error;
    pid->last_error = pid->error;
    
    return pid->output;
}

// ??? PID/PD ?? (????????)
float PID_Calc_Positional(PID_TypeDef *pid, float target, float feedback) {
    pid->error = target - feedback;
    
    // ????/?? (??)
    pid->integral += pid->error;
    if(pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    if(pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;

    float p_out = pid->kp * pid->error;
    float i_out = pid->ki * pid->integral;
    float d_out = pid->kd * (pid->error - pid->last_error);
    
    pid->output = p_out + i_out + d_out;
    
    // ????
    if (pid->output > pid->max_output) pid->output = pid->max_output;
    if (pid->output < -pid->max_output) pid->output = -pid->max_output;

    pid->last_error = pid->error;
    
    return pid->output;
}