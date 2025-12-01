# 智能车算法流程说明

本文档详细记录当前用户层代码的核心算法流程，便于调试与后续优化。

## 总体控制周期
- PIT 定时中断：周期 `CONTROL_PERIOD_MS`（默认 5 ms）。
- 中断入口 `pit_handler` 依次执行：`get_data()` → `set_servo_pwm()` → `set_speed_pwm()`。

## 传感与状态处理（`DEMO/user/src/get_data.c`）
1. **编码器读取**：`get_encoder` 采样后清零，得到本周期脉冲计数。
2. **ADC 采样与滤波**：
   - `get_adc` 轮询四路循迹传感器。
   - `filter_adc` 使用一阶低通（系数 `FILTER_ALPHA`）抑制抖动。
3. **归一化**：`normalize_adc` 以 `ADC_FULL_SCALE` 归一化到 0~1，并计算左右差值 `normalized_error = left - right`。
4. **终点检测**：`finish_line_detect` 判断四路同时高亮且持续 `FINISH_DEBOUNCE` 个周期后拉高 `finish_detected`。
5. **环岛检测**：`roundabout_detect` 检查两侧是否高于 `ROUNDABOUT_THRESHOLD`，计数超过 `ROUNDABOUT_DEBOUNCE` 且冷却结束后，
   - 置位 `roundabout_detected`；
   - 启动 `roundabout_timer = ROUNDABOUT_HOLD_TIME`；
   - 开启冷却 `roundabout_cooldown` 防止重复触发；
   - 根据状态点亮/熄灭 `LED1` 作为提示。
6. **速度目标选择**：`get_target_count_from_state` 根据当前状态切换三档编码器目标：直道、弯道、环岛/终点限速。

## 舵机控制（`DEMO/user/src/servo_control.c`）
1. **误差预处理**：
   - 对误差使用分段增益，`|error| > 0.2` 时放大到 `1.5×`，提升小角度灵敏度。
   - 锐角弯（`|error| > 0.32`）时动态提升 `kp` 到 `sharp_turn_gain` 倍。
2. **PD+前馈输出**：
   - `p_out = kp * enhanced_error`；`d_out = kd * (error_delta)`（`kd` 现为 6.5，响应更快）。
   - 大误差时添加 `quick_turn_feedforward` 前馈；
   - 若本周期与上周期误差符号相反且幅值超过 `sign_flip_threshold`，叠加 `sign_flip_boost` 作为“反打”前馈，专门加速连续左右急弯的回中与换向。
3. **环岛进入辅助**：
   - 检测到环岛且计时未归零时，计算 `straight_ratio`（先大后小）让转向从直行平滑过渡回正常；
   - 同时按计时动态限制最大偏转角（`10°→28°`），避免过早偏航，保障车辆先进入环岛再恢复寻线。
4. **限幅与输出**：
   - 舵机角度限制在 `SERVO_MOTOR_L_MAX` ~ `SERVO_MOTOR_R_MAX`；
   - 通过三路 PWM 同步输出到舵机。

## 速度控制（`DEMO/user/src/speed_control.c`）
1. **分段目标占空比**：
   - 终点：立即置 0；
   - 环岛：`ROUNDABOUT_SPEED_DUTY`；
   - 直道死区内（`|舵机偏差| <= STRAIGHT_DEAD_ZONE_DEG`）：`STRAIGHT_SPEED_DUTY`；
   - 其余情况视为弯道：`CURVE_SPEED_DUTY`。
2. **闭环速度 PID**：
   - 首次调用初始化 `speed_pid`，目标为 `get_target_count_from_state()` 给出的编码器计数；
   - PID 输出换算为占空比增量并与分段占空比叠加；
   - 最终 PWM 限制在 0~`PWM_DUTY_MAX`，同时对 duty 做上下限防护。

## 硬件与辅助
- `main.c` 中保留 LED 初始化，蜂鸣器 `BEEP` 相关代码已按要求注释，不再驱动。
- 所有定时、阈值与占空比上限均集中在 `DEMO/user/inc/my_common.h` 中，可按赛道与硬件微调。

