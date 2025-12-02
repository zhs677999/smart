#include "zf_common_headfile.h"
#include "my_common.h"
#include <math.h>

// 当前舵机目标角度，默认保持在中值
float servo_motor_angle = SERVO_MOTOR_M;

// -----------------------------------------------------------
// PD 调节参数（按状态机集中配置，方便调参）
// -----------------------------------------------------------
float kp = 160.0f;
float kd = 40.0f;

typedef enum
{
    STEER_STRAIGHT = 0,
    STEER_EASY_LEFT,
    STEER_EASY_RIGHT,
    STEER_HARD_LEFT,
    STEER_HARD_RIGHT,
    STEER_ROUND_HOLD,
    STEER_ROUND_CRUISE,
    STEER_ROUND_EXIT
} steering_state_t;

typedef struct
{
    float kp;
    float kd;
    float ff_gain;      // 单位：deg/归一化误差
    float ff_cap_deg;   // 单位：deg
    float d_scale;
    float settle_blend; // 直道或惰性阶段的输出粘滞系数（0~1，越大越粘）
} steering_profile_t;

// 统一的调参表，配合状态机一眼能看到各工况参数
static const steering_profile_t steering_profiles[] =
{
    [STEER_STRAIGHT]     = {160.0f, 20.0f,   0.0f,  0.0f, 0.25f, 0.84f},
    [STEER_EASY_LEFT]    = {180.0f, 34.0f, 160.0f, 16.0f, 1.00f, 0.10f},
    [STEER_EASY_RIGHT]   = {180.0f, 34.0f, 160.0f, 16.0f, 1.00f, 0.10f},
    [STEER_HARD_LEFT]    = {230.0f, 42.0f, 220.0f, 24.0f, 1.10f, 0.00f},
    [STEER_HARD_RIGHT]   = {230.0f, 42.0f, 220.0f, 24.0f, 1.10f, 0.00f},
    [STEER_ROUND_HOLD]   = {110.0f,  0.0f,   0.0f,  0.0f, 0.00f, 0.92f},
    [STEER_ROUND_CRUISE] = {170.0f, 24.0f, 150.0f, 18.0f, 0.65f, 0.35f},
    [STEER_ROUND_EXIT]   = {190.0f, 30.0f, 200.0f, 24.0f, 0.90f, 0.25f},
};

// 状态判定阈值
static const float straight_error_band     = 0.08f;
static const float straight_delta_band     = 0.10f;
static const float sharp_turn_error        = 0.42f;
static const float round_exit_trigger_time = 0.75f; // 达到最小绕行时间后算出口阶段

// 反向急弯/掉头软化
static const float hairpin_sustain_error    = 0.52f;
static const uint16_t hairpin_sustain_ticks = 16;
static const uint16_t hairpin_settle_ticks  = 12;
static const float hairpin_damping          = 0.50f;
static const float hairpin_quick_scale      = 0.42f;

// 环岛进入时的直行比例与偏转限制
static const float roundabout_straight_ratio_high = 0.50f;
static const float roundabout_straight_ratio_low  = 0.18f;
static const float roundabout_entry_offset_min     = 10.0f;
static const float roundabout_entry_offset_max     = 34.0f;

// 环岛检测后的一段惰性变向
static const float roundabout_inertia_blend_soft   = 0.82f;
static const float roundabout_inertia_blend_strong = 0.93f;
static const float roundabout_inertia_gain_scale   = 0.70f;
// -----------------------------------------------------------

extern float normalized_error;      // 归一化后的左右差值
float last_adc_error = 0;           // 上一次误差（用于 D 项与换向检测）
extern uint8_t roundabout_detected;
extern uint16_t roundabout_timer;
extern uint16_t roundabout_lap_timer;
static uint16_t sustained_turn_ticks = 0;   // 同向大幅转弯持续时间
static int8_t sustained_turn_sign = 0;       // 最近一次持续转弯的方向
static uint16_t hairpin_settle_timer = 0;    // 掉头弯换向后的软化计时
static uint16_t roundabout_inertia_timer = 0; // 环岛检测后的惰性变向计时
static uint8_t last_roundabout_state = 0;

static steering_state_t classify_state(float error, float delta)
{
    if(roundabout_detected)
    {
        if(roundabout_timer > 0)
        {
            return STEER_ROUND_HOLD;
        }

        if(roundabout_lap_timer >= (uint16_t)(ROUNDABOUT_LAP_MIN_TIME * round_exit_trigger_time))
        {
            return STEER_ROUND_EXIT;
        }

        return STEER_ROUND_CRUISE;
    }

    if(fabsf(error) < straight_error_band && fabsf(delta) < straight_delta_band)
    {
        return STEER_STRAIGHT;
    }

    if(fabsf(error) > sharp_turn_error)
    {
        return error > 0 ? STEER_HARD_LEFT : STEER_HARD_RIGHT;
    }

    return error > 0 ? STEER_EASY_LEFT : STEER_EASY_RIGHT;
}

static float apply_entry_hold(float commanded_angle)
{
    float straight_ratio = roundabout_straight_ratio_low;
    if (roundabout_timer > (ROUNDABOUT_HOLD_TIME * 3 / 5)) {
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
    return SERVO_MOTOR_M * straight_ratio + softened_angle * (1.0f - straight_ratio);
}

void set_servo_pwm()
{
    // 环岛识别后的惰性计时
    if(roundabout_detected && !last_roundabout_state)
    {
        roundabout_inertia_timer = ROUNDABOUT_INERTIA_TIME;
    }
    else if(!roundabout_detected)
    {
        roundabout_inertia_timer = 0;
    }
    last_roundabout_state = roundabout_detected;

    if(roundabout_inertia_timer > 0)
    {
        roundabout_inertia_timer--;
    }

    // 掉头弯检测计时
    if(hairpin_settle_timer > 0)
    {
        hairpin_settle_timer--;
    }

    float error = normalized_error;
    float error_delta = error - last_adc_error;

    // 统计同向持续计时
    int8_t current_sign = (error > 0.0f) - (error < 0.0f);
    int8_t prev_sign = sustained_turn_sign;
    uint16_t prev_ticks = sustained_turn_ticks;

    if((fabsf(error) > hairpin_sustain_error) && (current_sign != 0))
    {
        if(current_sign == sustained_turn_sign)
        {
            if(sustained_turn_ticks < 0xFFFF)
            {
                sustained_turn_ticks++;
            }
        }
        else
        {
            sustained_turn_sign = current_sign;
            sustained_turn_ticks = 1;
        }
    }
    else if(current_sign == sustained_turn_sign)
    {
        if(sustained_turn_ticks > 0)
        {
            sustained_turn_ticks--;
        }
    }
    else
    {
        sustained_turn_ticks = 0;
        sustained_turn_sign = current_sign;
    }

    // 长时间同向后突然换向，触发软化计时
    if((current_sign != 0) && (prev_sign != 0) && (current_sign != prev_sign) &&
       (prev_ticks >= hairpin_sustain_ticks) && (fabsf(last_adc_error) > hairpin_sustain_error))
    {
        hairpin_settle_timer = hairpin_settle_ticks;
        sustained_turn_ticks = 1;
        sustained_turn_sign = current_sign;
    }

    steering_state_t state = classify_state(error, error_delta);
    steering_profile_t profile = steering_profiles[state];
    kp = profile.kp;
    kd = profile.kd;

    // 环岛惰性阶段整体降增益
    if(roundabout_detected && roundabout_inertia_timer > 0)
    {
        profile.kp *= roundabout_inertia_gain_scale;
        profile.kd *= roundabout_inertia_gain_scale;
        profile.ff_gain *= roundabout_inertia_gain_scale;
    }

    float p_out = profile.kp * error;
    float d_out = profile.kd * error_delta * profile.d_scale;

    // 掉头软化：抑制 D 项与前馈
    if(hairpin_settle_timer > 0)
    {
        d_out *= hairpin_damping;
    }

    // 状态机版前馈：弯道/环岛阶段按表给出前馈并限幅
    float quick_out = profile.ff_gain * error;
    float quick_cap = profile.ff_cap_deg;

    // 环岛出口阶段允许稍大前馈
    if(state == STEER_ROUND_EXIT)
    {
        quick_cap += 2.0f;
    }

    if(hairpin_settle_timer > 0)
    {
        quick_out *= hairpin_quick_scale;
    }

    if(quick_cap > 0.0f)
    {
        if (quick_out > quick_cap) quick_out = quick_cap;
        if (quick_out < -quick_cap) quick_out = -quick_cap;
    }
    else
    {
        quick_out = 0.0f;
    }

    float commanded_angle = SERVO_MOTOR_M - (p_out + d_out + quick_out);

    // 直道或惰性阶段的平滑输出
    if(profile.settle_blend > 0.0f)
    {
        commanded_angle = servo_motor_angle * profile.settle_blend + commanded_angle * (1.0f - profile.settle_blend);
    }

    // 环岛进入保持直行
    if(state == STEER_ROUND_HOLD)
    {
        commanded_angle = apply_entry_hold(commanded_angle);
    }

    // 环岛识别后的惰性变向：跨中值时更粘滞
    if(roundabout_detected && roundabout_inertia_timer > 0)
    {
        float inertia_blend = roundabout_inertia_blend_soft;
        if((servo_motor_angle - SERVO_MOTOR_M) * (commanded_angle - SERVO_MOTOR_M) < 0.0f)
        {
            inertia_blend = roundabout_inertia_blend_strong;
        }
        commanded_angle = servo_motor_angle * inertia_blend + commanded_angle * (1.0f - inertia_blend);
    }

    servo_motor_angle = commanded_angle;
    last_adc_error = error;

    // 限幅与输出
    if(servo_motor_angle > SERVO_MOTOR_R_MAX) servo_motor_angle = SERVO_MOTOR_R_MAX;
    if(servo_motor_angle < SERVO_MOTOR_L_MAX) servo_motor_angle = SERVO_MOTOR_L_MAX;

    pwm_set_duty(SERVO_MOTOR1_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
    pwm_set_duty(SERVO_MOTOR2_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
    pwm_set_duty(SERVO_MOTOR3_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
}
