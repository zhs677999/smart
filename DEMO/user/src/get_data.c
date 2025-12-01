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
uint16_t adc_buffer[ADC_CHANNEL_NUMBER];

// 滤波与归一化后的数据
float filtered_adc[ADC_CHANNEL_NUMBER] = {0};
float normalized_adc[ADC_CHANNEL_NUMBER] = {0};
float normalized_error = 0; // 归一化差值，左大于右为正

// 检测状态
uint8_t finish_detected = 0;
uint8_t roundabout_detected = 0;
uint16_t roundabout_timer = 0;
uint16_t roundabout_cooldown = 0;

// 内部计时与防抖
static uint16_t finish_counter = 0;
static uint16_t roundabout_counter = 0;

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
        adc_buffer[channel_index] = adc_convert(channel_list[channel_index]);
    }
}

// 一阶低通滤波，提供可检测的滤波输出
static void filter_adc(void)
{
    for(channel_index = 0; channel_index < ADC_CHANNEL_NUMBER; channel_index ++)
    {
        float raw = (float)adc_buffer[channel_index];
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

// 终点检测：四路同时高亮并持续若干周期
static void finish_line_detect(void)
{
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
}

static void update_roundabout_alert(void)
{
    if(!roundabout_detected)
    {
        gpio_set_level(LED1, GPIO_HIGH);
        gpio_set_level(BEEP, GPIO_LOW);
    }
    else
    {
        gpio_set_level(LED1, GPIO_LOW);
        gpio_set_level(BEEP, GPIO_HIGH);
    }
}

// 环岛检测：两侧传感器高亮并维持一定时间，带冷却
static void roundabout_detect(void)
{
    float left_norm = normalized_adc[0];
    float right_norm = normalized_adc[3];
    float threshold_norm = (float)ROUNDABOUT_THRESHOLD / ADC_FULL_SCALE;

    if(roundabout_cooldown > 0)
    {
        roundabout_cooldown--;
    }

    if((left_norm > threshold_norm) && (right_norm > threshold_norm))
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

    if(roundabout_counter >= ROUNDABOUT_DEBOUNCE && roundabout_detected == 0 && roundabout_cooldown == 0)
    {
        roundabout_detected = 1;
        roundabout_timer = ROUNDABOUT_HOLD_TIME;
        roundabout_cooldown = ROUNDABOUT_COOLDOWN;
    }

    if(roundabout_detected)
    {
        if(roundabout_timer > 0)
        {
            roundabout_timer--;
        }
        else
        {
            roundabout_detected = 0;
        }
    }

    update_roundabout_alert();
}

// 数据处理主流程
void process_sensor_data(void)
{
    get_encoder();
    get_adc();
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