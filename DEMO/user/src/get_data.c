#include "zf_common_headfile.h"
#include "my_common.h"

// ------------------ 数据读取与处理 ------------------

// 编码器计数
int16 encoder_data_1 = 0;
int16 encoder_data_2 = 0;

// ADC 原始缓冲区
uint8 channel_index = 0;
adc_channel_enum channel_list[ADC_CHANNEL_NUMBER] =
{
    ADC_CHANNEL1, ADC_CHANNEL2, ADC_CHANNEL3, ADC_CHANNEL4,
};
uint16_t raw_adc[ADC_CHANNEL_NUMBER];

// 滤波与归一化后的数据
float filtered_adc[ADC_CHANNEL_NUMBER] = {0};
float normalized_adc[ADC_CHANNEL_NUMBER] = {0};
float normalized_error = 0; // 归一化差值，左大于右为正

// 检测状态
uint8_t finish_detected = 0;
uint8_t off_track_detected = 0;
uint8_t roundabout_detected = 0;
uint16_t roundabout_timer = 0;
uint16_t roundabout_cooldown = 0;
uint16_t roundabout_lap_timer = 0;
roundabout_state_t roundabout_state = ROUND_STATE_IDLE;
uint8_t roundabout_entry_mark = 0;
uint8_t roundabout_exit_mark = 0;

// 内部计时与防抖
static uint16_t finish_counter = 0;
static uint16_t off_track_counter = 0;
static uint16_t roundabout_counter = 0;
static uint16_t roundabout_exit_counter = 0;
static uint32_t travel_since_roundabout = 0;

// -------------------------------------------------------------

// 编码器读取
void get_encoder()
{
    encoder_data_1 = encoder_get_count(ENCODER_1);
    encoder_clear_count(ENCODER_1);

    encoder_data_2 = encoder_get_count(ENCODER_2);
    encoder_clear_count(ENCODER_2);
}

// ADC 读取
static void get_adc()
{
    for(channel_index = 0; channel_index < ADC_CHANNEL_NUMBER; channel_index ++)
    {
        raw_adc[channel_index] = adc_convert(channel_list[channel_index]);
    }
}

// 一阶低通滤波，提供可检测的滤波输出
static void filter_adc(void)
{
    for(channel_index = 0; channel_index < ADC_CHANNEL_NUMBER; channel_index ++)
    {
        float raw = (float)raw_adc[channel_index];
        filtered_adc[channel_index] = FILTER_ALPHA * filtered_adc[channel_index] + (1.0f - FILTER_ALPHA) * raw;
    }
}

// 归一化到 0~1
static void normalize_adc(void)
{
    for(channel_index = 0; channel_index < ADC_CHANNEL_NUMBER; channel_index ++)
    {
        normalized_adc[channel_index] = filtered_adc[channel_index] / ADC_FULL_SCALE;
        if(normalized_adc[channel_index] > 1.0f) normalized_adc[channel_index] = 1.0f;
        if(normalized_adc[channel_index] < 0.0f) normalized_adc[channel_index] = 0.0f;
    }

    normalized_error = normalized_adc[0] - normalized_adc[3];
}

// 失线检测：四路原始值同时低于阈值则判定为超出跑道
static void off_track_detect(void)
{
    uint8_t all_low = 1;
    for(channel_index = 0; channel_index < ADC_CHANNEL_NUMBER; channel_index ++)
    {
        if(raw_adc[channel_index] >= OFF_TRACK_THRESHOLD_RAW)
        {
            all_low = 0;
            break;
        }
    }

    if(all_low)
    {
        if(off_track_counter < 0xFFFF)
        {
            off_track_counter++;
        }
    }
    else
    {
        off_track_counter = 0;
    }

    if(off_track_counter >= OFF_TRACK_DEBOUNCE)
    {
        off_track_detected = 1;
    }
}

// 终点检测：四路同时高亮并持续若干周期
static void finish_line_detect(void)
{
    static uint8_t hall_inited = 0;
    static uint8_t hall_last_level = 0;
    static uint16_t hall_change_counter = 0;

    uint8_t hall_level = gpio_get_level(FINISH_HALL_PIN);
    if(!hall_inited)
    {
        hall_last_level = hall_level;
        hall_inited = 1;
    }

    if(hall_level != hall_last_level)
    {
        hall_last_level = hall_level;
        if(hall_change_counter < 0xFFFF)
        {
            hall_change_counter++;
        }
    }

    uint8_t all_high = 1;
    for(channel_index = 0; channel_index < ADC_CHANNEL_NUMBER; channel_index ++)
    {
        if(normalized_adc[channel_index] < FINISH_THRESHOLD)
        {
            all_high = 0;
            break;
        }
    }

    if(all_high)
    {
        if(finish_counter < 0xFFFF)
        {
            finish_counter++;
        }
    }
    else
    {
        finish_counter = 0;
    }

    if(finish_counter >= FINISH_DEBOUNCE)
    {
        finish_detected = 1;
    }

    if(hall_change_counter >= FINISH_HALL_DEBOUNCE)
    {
        finish_detected = 1;
    }
}

static void update_roundabout_alert(void)
{
    if(!roundabout_detected)
    {
        gpio_set_level(LED1, GPIO_HIGH);
    }
    else
    {
        gpio_set_level(LED1, GPIO_LOW);
    }
}

// 环岛检测：匹配接近环岛的原始值特征，避免与急弯/十字路混淆；进入后绕一圈再退出
static void roundabout_detect(void)
{
    // 单次事件标记：供调试查看进/出环瞬间
    roundabout_entry_mark = 0;
    roundabout_exit_mark = 0;

    float encoder_speed = (fabsf((float)encoder_data_1) + fabsf((float)encoder_data_2)) * 0.5f;
    travel_since_roundabout += (uint32_t)(fabsf((float)encoder_data_1) + fabsf((float)encoder_data_2));

    uint16_t left_outer   = raw_adc[0];
    uint16_t left_middle  = raw_adc[1];
    uint16_t right_middle = raw_adc[2];
    uint16_t right_outer  = raw_adc[3];

    if(roundabout_cooldown > 0)
    {
        roundabout_cooldown--;
    }

    // 模式 1：接近环岛时，中右迅速变暗且两侧保持亮度（2290/1435/77/2024，3602/3237/79/2421）
    uint8_t approach_pattern = (right_middle <= ROUNDABOUT_RAW_GAP_SOFT) &&
                               (left_outer  >= ROUNDABOUT_OUTER_L_HIGH) &&
                               (right_outer >= ROUNDABOUT_OUTER_R_HIGH) &&
                               (left_middle >= ROUNDABOUT_MID_HIGH);

    // 模式 2：到切点时，中左短暂变暗但左右外侧冲高（3420/443/111/3146，3585/1957/403/2986）
    uint8_t tangent_pattern = (right_middle <= ROUNDABOUT_RAW_GAP_STRONG) &&
                              (left_middle  <= ROUNDABOUT_MID_HIGH) &&
                              (left_outer   >= ROUNDABOUT_LEFT_SPIKE) &&
                              (right_outer  >= ROUNDABOUT_RIGHT_SUPPORT);

    // 自适应模式：整体亮度偏低时，要求左右外侧至少保持 55%/45% 亮度，且中右明显暗于中左
    float left_outer_norm   = normalized_adc[0];
    float left_middle_norm  = normalized_adc[1];
    float right_middle_norm = normalized_adc[2];
    float right_outer_norm  = normalized_adc[3];

    uint8_t adaptive_gap = (right_middle_norm < 0.32f) &&
                           (left_outer_norm  > 0.55f) &&
                           (right_outer_norm > 0.45f) &&
                           (left_middle_norm > 0.42f) &&
                           ((left_middle_norm - right_middle_norm) > 0.22f) &&
                           ((left_outer_norm  - right_middle_norm) > 0.28f);

    // 出口放宽：绕行达到最小时长后才开启宽松通道，亮度偏低或距离稍远也能退出计数
    uint8_t soft_exit_pattern = (roundabout_state == ROUND_STATE_EXIT_SEARCH) &&
                                (right_middle_norm < ROUNDABOUT_EXIT_GAP_SOFT) &&
                                (left_outer_norm  > ROUNDABOUT_EXIT_LEFT_MIN) &&
                                (right_outer_norm > ROUNDABOUT_EXIT_RIGHT_MIN) &&
                                ((left_outer_norm - right_middle_norm) > ROUNDABOUT_EXIT_OUTER_DIFF) &&
                                ((left_middle_norm - right_middle_norm) > ROUNDABOUT_EXIT_MID_DIFF);

    // 基于实测原始值的出口辅助通道：靠近切点时四路拉高，切点瞬间左中/右中掉落而右外保持高亮
    uint8_t raw_exit_plateau = (roundabout_state == ROUND_STATE_EXIT_SEARCH) &&
                               (left_outer   >= ROUNDABOUT_EXIT_RAW_PLATEAU_L) &&
                               (left_middle  >= ROUNDABOUT_EXIT_RAW_PLATEAU_LM) &&
                               (right_middle >= ROUNDABOUT_EXIT_RAW_PLATEAU_RM) &&
                               (right_outer  >= ROUNDABOUT_EXIT_RAW_PLATEAU_R);

    uint8_t raw_exit_cut = (roundabout_state == ROUND_STATE_EXIT_SEARCH) &&
                           (left_outer   >= ROUNDABOUT_EXIT_RAW_CUT_L) &&
                           (right_outer  >= ROUNDABOUT_EXIT_RAW_CUT_R) &&
                           (left_middle  <= ROUNDABOUT_EXIT_RAW_CUT_LM) &&
                           (right_middle <= ROUNDABOUT_EXIT_RAW_CUT_RM);

    uint8_t encoder_ready = (encoder_speed >= ROUNDABOUT_ENCODER_SPEED_MIN) &&
                            (travel_since_roundabout >= ROUNDABOUT_ENCODER_TRAVEL_MIN);

    if(roundabout_state == ROUND_STATE_IDLE)
    {
        if(approach_pattern || tangent_pattern || adaptive_gap)
        {
            if(roundabout_counter < 0xFFFF)
            {
                roundabout_counter++;
            }
        }
        else
        {
            roundabout_counter = 0;
        }

        if(roundabout_counter >= ROUNDABOUT_DEBOUNCE && roundabout_cooldown == 0 && encoder_ready)
        {
            roundabout_detected = 1;
            roundabout_timer = ROUNDABOUT_HOLD_TIME;
            roundabout_cooldown = ROUNDABOUT_COOLDOWN;
            roundabout_lap_timer = 0;
            roundabout_exit_counter = 0;
            travel_since_roundabout = 0;
            roundabout_state = ROUND_STATE_ENTRY_LOCK;
            roundabout_entry_mark = 1;
        }
    }
    else
    {
        // 已进入环岛：计时并寻找绕行一圈后的出口特征
        if(roundabout_timer > 0)
        {
            roundabout_timer--;
        }

        // 入环锁死阶段结束后进入绕行计时状态
        if(roundabout_state == ROUND_STATE_ENTRY_LOCK && roundabout_timer == 0)
        {
            roundabout_state = ROUND_STATE_LAP;
        }

        if(roundabout_lap_timer < 0xFFFF)
        {
            roundabout_lap_timer++;
        }

        // 达到最小绕行时间后，切换到出口搜索阶段，允许开启放宽通道
        if(roundabout_state == ROUND_STATE_LAP && roundabout_lap_timer >= ROUNDABOUT_LAP_MIN_TIME)
        {
            roundabout_state = ROUND_STATE_EXIT_SEARCH;
        }

        uint8_t base_exit_pattern = approach_pattern || tangent_pattern || adaptive_gap;
        uint8_t exit_pattern = (roundabout_state == ROUND_STATE_EXIT_SEARCH) &&
                                (base_exit_pattern || soft_exit_pattern || raw_exit_plateau || raw_exit_cut); // 绕行一圈后出口会再次出现这些亮暗组合

        if(exit_pattern)
        {
            if(roundabout_exit_counter < 0xFFFF)
            {
                roundabout_exit_counter++;
            }
        }
        else
        {
            roundabout_exit_counter = 0;
        }

        if(roundabout_exit_counter >= ROUNDABOUT_EXIT_CONFIRM || roundabout_lap_timer >= ROUNDABOUT_MAX_LAP_TIME)
        {
            roundabout_detected = 0;
            roundabout_timer = 0;
            roundabout_cooldown = ROUNDABOUT_COOLDOWN; // 出环岛后暂时不再触发
            roundabout_counter = 0;
            roundabout_exit_counter = 0;
            roundabout_lap_timer = 0;
            travel_since_roundabout = 0;
            roundabout_state = ROUND_STATE_IDLE;
            roundabout_exit_mark = 1;
        }
    }

    update_roundabout_alert();
}

// 数据处理主流程
void process_sensor_data(void)
{
    get_encoder();
    get_adc();
    off_track_detect();
    filter_adc();
    normalize_adc();
    finish_line_detect();
    roundabout_detect();
}

// 对外统一接口，兼容旧调用
void get_data()
{
    process_sensor_data();
}

// 根据状态选择目标编码器计数，供速度环使用
float get_target_count_from_state(void)
{
    if(finish_detected || roundabout_detected)
    {
        return TARGET_COUNT_ROUNDABOUT;
    }

    if(fabsf(normalized_error) <= (STRAIGHT_DEAD_ZONE_DEG / 90.0f))
    {
        return TARGET_COUNT_STRAIGHT;
    }

    return TARGET_COUNT_CURVE;
}