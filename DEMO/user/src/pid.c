#include "pid.h"

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float output_limit, float integral_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_limit = output_limit;
    pid->integral_limit = integral_limit;
    pid_reset(pid);
}

void pid_reset(pid_controller_t *pid)
{
    pid->integral = 0;
    pid->last_error = 0;
}

float pid_update(pid_controller_t *pid, float target, float measurement)
{
    float error = target - measurement;
    pid->integral += error;

    // 限制积分，避免积分饱和
    if(pid->integral > pid->integral_limit)
    {
        pid->integral = pid->integral_limit;
    }
    else if(pid->integral < -pid->integral_limit)
    {
        pid->integral = -pid->integral_limit;
    }

    float derivative = error - pid->last_error;
    pid->last_error = error;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    if(output > pid->output_limit)
    {
        output = pid->output_limit;
    }
    else if(output < -pid->output_limit)
    {
        output = -pid->output_limit;
    }

    return output;
}
