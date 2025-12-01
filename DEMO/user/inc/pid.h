#ifndef _PID_H_
#define _PID_H_

#include "zf_common_headfile.h"

// 统一的PID控制器描述，用于电机速度控制
// 使用浮点参数以便快速调参

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;
    float last_error;
    float output_limit;
    float integral_limit;
} pid_controller_t;

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float output_limit, float integral_limit);
void pid_reset(pid_controller_t *pid);
float pid_update(pid_controller_t *pid, float target, float measurement);

#endif
