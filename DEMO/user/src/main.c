//2025.11.25 ѧϰ

#include "zf_common_headfile.h"
#include "my_common.h"

extern int8 duty;         // 当前 duty

void Init_All(void);

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    system_delay_ms(300);
    Init_All();

    // 控制周期调整为可配置，默认 5ms
    pit_ms_init(PIT_CH, CONTROL_PERIOD_MS);

    while(1)
    {
        // 主循环可用于扩展其它非实时任务
    }
}

// 初始化硬件资源
void Init_All(void)
{
    // DRV8701
    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR1_PWM, 17000, 0);

    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR2_PWM, 17000, 0);

    gpio_init(MOTOR3_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR3_PWM, 17000, 0);

    gpio_init(MOTOR4_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR4_PWM, 17000, 0);

    // 编码器
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);

    // ADC
    adc_init(ADC_CHANNEL1, ADC_12BIT);
    adc_init(ADC_CHANNEL2, ADC_12BIT);
    adc_init(ADC_CHANNEL3, ADC_12BIT);
    adc_init(ADC_CHANNEL4, ADC_12BIT);

    // 舵机 PWM
    pwm_init(SERVO_MOTOR1_PWM, SERVO_MOTOR_FREQ, 0);
    pwm_init(SERVO_MOTOR2_PWM, SERVO_MOTOR_FREQ, 0);
    pwm_init(SERVO_MOTOR3_PWM, SERVO_MOTOR_FREQ, 0);

    // 指示灯与蜂鸣器
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(BEEP, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_set_level(LED1, GPIO_LOW);
    gpio_set_level(BEEP, GPIO_HIGH);
}

extern void get_data();
extern void set_servo_pwm();
extern void set_speed_pwm();

// PIT 中断服务函数，每个控制周期调用
void pit_handler (void)
{
    get_data();
    set_servo_pwm();
    set_speed_pwm();
}
