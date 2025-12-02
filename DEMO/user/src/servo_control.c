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
float kd = 60.0f;
// 锐角弯动态增益
static const float sharp_turn_error = 0.42f;
static const float sharp_turn_gain  = 5.4f;
static const float quick_turn_feedforward = 260.0f;
// 连续反向急弯的额外前馈，帮助舵机快速“打回来”
static const float sign_flip_threshold = 0.95f;
static const float sign_flip_boost     = 140.0f;
// 掉头弯：长时间大幅同向转弯后，如果瞬间换向，先暂缓反打，避免按“S 弯”处理
static const float hairpin_sustain_error    = 0.55f;   // 认定为大幅同向转弯的误差门限
static const uint16_t hairpin_sustain_ticks = 18;      // 连续多久算“已进入掉头弯”（约 90ms）
static const uint16_t hairpin_settle_ticks  = 14;      // 翻向后软化期（约 70ms）
static const float hairpin_damping          = 0.55f;   // 软化期降低 D 项幅度
static const float hairpin_quick_scale      = 0.35f;   // 软化期削弱快速前馈
// 限制快速前馈的量级，避免在连续急弯时过度回正
static const float quick_out_limit_deg = 22.0f;
static const float s_bend_extra_quick  = 6.0f;    // 反向急转弯时允许更大的快速前馈，防止出弯
// 环岛进入时的舵机直行保持比例（靠计时器逐渐退出）
static const float roundabout_straight_ratio_high = 0.45f;
static const float roundabout_straight_ratio_low  = 0.15f;
// 环岛阶段允许的最大偏转角，先小后大，避免提前偏航
static const float roundabout_entry_offset_min = 10.0f;
static const float roundabout_entry_offset_max = 38.0f;
// 环岛检测后的一段惰性变向，防止刚识别时因外侧抖动又拐回
static const float roundabout_inertia_blend_soft   = 0.80f;
static const float roundabout_inertia_blend_strong = 0.92f;
static const float roundabout_inertia_kp_scale     = 0.72f;  // 惰性阶段降低比例响应，避免超调
static const float roundabout_inertia_quick_scale  = 0.60f;  // 惰性阶段削弱快速前馈
// -----------------------------------------------------------

extern float normalized_error;      // 归一化后的左右差值
extern float normalized_adc[ADC_CHANNEL_NUMBER];
float last_adc_error = 0;           // 上一次误差（用于 D 项与换向检测）
extern uint8_t roundabout_detected;
extern uint16_t roundabout_timer;
extern uint16_t roundabout_lap_timer;
static uint16_t sustained_turn_ticks = 0;   // 同向大幅转弯持续时间
static int8_t sustained_turn_sign = 0;       // 最近一次持续转弯的方向
static uint16_t hairpin_settle_timer = 0;    // 掉头弯换向后的软化计时
static uint16_t roundabout_inertia_timer = 0; // 环岛检测后的惰性变向计时
static uint8_t last_roundabout_state = 0;

void set_servo_pwm()
{
    float kp_local = 180.0f;
    float kd_local = 60.0f;

    // 环岛识别后开启一段惰性变向计时，防止刚进入时因外圈抖动又快速回头
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

    // 直道稳定器：在误差很小且变化不剧烈时，减小 D 项与前馈，避免来回抖动
    const float straight_error_band = 0.08f;
    const float straight_delta_band = 0.10f;
    const float straight_settle_ratio = 0.82f; // 越大越贴合上一次输出

    // 动态 PID：误差较大时提高响应
    if (fabsf(normalized_error) > 0.2f) {
        kp_local = 230.0f;  // 大误差时更激进
        kd_local = 18.0f;
    }

    kp = kp_local;
    kd = kd_local;

    // 基于整体左右强度差的急弯特征，先行放大误差
    float left_sum  = normalized_adc[0] + normalized_adc[1];
    float right_sum = normalized_adc[2] + normalized_adc[3];
    float lateral_balance = left_sum - right_sum;
    float strong_turn_weight = fminf(fabsf(lateral_balance) / 0.65f, 1.4f); // 2000/1930 vs 539/1141 对应约 0.65 差值

    // 非线性误差放大，对小误差敏感，对大误差饱和
    float error_gain = 1.9f + 0.55f * strong_turn_weight;
    if (fabsf(normalized_error) > 0.1f) {
        error_gain = 1.35f + 0.75f * strong_turn_weight;  // 大误差时增益更大
    }

    // 锐角弯时进一步提升比例系数
    float adaptive_kp = kp_local;
    if ((fabsf(normalized_error) > sharp_turn_error) || (fabsf(lateral_balance) > 0.38f)) {
        adaptive_kp *= sharp_turn_gain;
    }

    if(roundabout_detected && roundabout_inertia_timer > 0)
    {
        adaptive_kp *= roundabout_inertia_kp_scale;
    }

    float enhanced_error = normalized_error * error_gain;
    float error_delta = enhanced_error - last_adc_error;

    // 统计是否出现“掉头弯”：同向大误差持续后突然换向
    if(hairpin_settle_timer > 0)
    {
        hairpin_settle_timer--;
    }

    int8_t current_sign = (enhanced_error > 0.0f) - (enhanced_error < 0.0f);
    if((fabsf(enhanced_error) > hairpin_sustain_error) && (current_sign != 0))
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
        // 保持方向但幅度变小：轻微衰减计数，避免偶尔抖动清零
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

    if((current_sign != 0) && (sustained_turn_sign != 0) && (current_sign != sustained_turn_sign) &&
       (sustained_turn_ticks >= hairpin_sustain_ticks) && (fabsf(last_adc_error) > hairpin_sustain_error))
    {
        // 处于掉头弯：刚换向时先进入软化期，防止误判为连续反打
        hairpin_settle_timer = hairpin_settle_ticks;
        sustained_turn_ticks = 1;
        sustained_turn_sign = current_sign;
    }

    // 直道抑制 D 项，避免轻微噪声触发反复回正
    if (fabsf(normalized_error) < straight_error_band && fabsf(error_delta) < straight_delta_band) {
        kd_local *= 0.35f;
    }

    float p_out = adaptive_kp * enhanced_error;
    float d_out = kd_local * error_delta * (hairpin_settle_timer ? hairpin_damping : 1.0f);

    // 快速转向前馈：误差大的时候直接给舵机额外角度，减少响应延迟
    float quick_out = 0.0f;
    if ((fabsf(normalized_error) > sharp_turn_error) || (fabsf(lateral_balance) > 0.35f)) {
        float lateral_feedforward = quick_turn_feedforward * 0.82f * lateral_balance;
        quick_out = quick_turn_feedforward * normalized_error + lateral_feedforward;
    }

    // 反向连续急弯：放宽快速前馈限制，让第二个急弯能更快贴近赛道
    float quick_out_cap = quick_out_limit_deg;
    if ((enhanced_error * last_adc_error < 0.0f) && (fabsf(enhanced_error) > sign_flip_threshold) && (hairpin_settle_timer == 0)) {
        quick_out_cap += s_bend_extra_quick;
    }

    // 连续左右急弯或换向时，额外给出“反打”前馈，提前让舵机回中换向
    if ((enhanced_error * last_adc_error < 0.0f) && (fabsf(enhanced_error) > sign_flip_threshold) && (hairpin_settle_timer == 0)) {
        // 二次急弯时回正更温和，避免过度反打
        quick_out += sign_flip_boost * enhanced_error;
    }

    // 直道时屏蔽快速前馈，防止微小误差也触发大幅反打
    if (fabsf(normalized_error) < straight_error_band && fabsf(error_delta) < straight_delta_band) {
        quick_out = 0.0f;
    }

    // 环岛惰性阶段：抑制前馈幅度，防止刚识别时反向超调
    if(roundabout_detected && roundabout_inertia_timer > 0)
    {
        quick_out *= roundabout_inertia_quick_scale;
        quick_out_cap = fminf(quick_out_cap, quick_out_limit_deg - 2.0f);
    }

    // 掉头弯换向软化：暂时削弱前馈，防止直接把舵机打到反方向极限
    if(hairpin_settle_timer > 0)
    {
        quick_out *= hairpin_quick_scale;
    }

    // 环岛绕行接近一圈后，允许更大的前馈幅度，以便迅速“拉”出出口
    if (roundabout_detected && (roundabout_lap_timer >= ROUNDABOUT_LAP_MIN_TIME)) {
        quick_out_cap += 4.0f;
    }

    // 限制快速前馈幅度，防止占满舵机行程导致第二个急弯出不去
    if (quick_out > quick_out_cap) quick_out = quick_out_cap;
    if (quick_out < -quick_out_cap) quick_out = -quick_out_cap;

    float commanded_angle = SERVO_MOTOR_M - (p_out + d_out + quick_out);

    // 直道稳态：输出向上一周期粘滞，防止在小误差附近左右抖动
    if (fabsf(normalized_error) < straight_error_band && fabsf(error_delta) < straight_delta_band) {
        commanded_angle = servo_motor_angle * straight_settle_ratio + commanded_angle * (1.0f - straight_settle_ratio);
    }

    // 环岛进入：先保持直行，然后按计时器渐进恢复正常控制，同时限制最大偏转角，防止过早偏航
    if (roundabout_detected && roundabout_timer > 0) {
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
        commanded_angle = SERVO_MOTOR_M * straight_ratio + softened_angle * (1.0f - straight_ratio);
    }

    // 环岛识别后的惰性变向：短时间内对舵机命令做平滑，抑制突然反打导致的“跑出环岛”
    if(roundabout_detected && roundabout_inertia_timer > 0)
    {
        float inertia_blend = roundabout_inertia_blend_soft;
        if((servo_motor_angle - SERVO_MOTOR_M) * (commanded_angle - SERVO_MOTOR_M) < 0.0f)
        {
            inertia_blend = roundabout_inertia_blend_strong; // 跨过中值要更粘滞
        }
        commanded_angle = servo_motor_angle * inertia_blend + commanded_angle * (1.0f - inertia_blend);
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