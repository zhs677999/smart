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
   - 选用差值（difference）而非比值（ratio），避免分母接近 0 时放大噪声，也便于在左右光强接近饱和或过暗时保持线性响应；
   - 如果赛道或传感器布局导致左右幅值差异巨大、易饱和，可考虑改为 `(left - right) / max(left + right, eps)` 的归一化比值，但需要针对空白底板做额外滤波与阈值调整。
4. **终点检测**：`finish_line_detect` 判断四路同时高亮且持续 `FINISH_DEBOUNCE` 个周期后拉高 `finish_detected`。
5. **环岛检测**：`roundabout_detect` 采用“电感特征 + 编码器速度/里程”双重验证并内置状态机：
   - ADC 检出环岛特征后，只有当编码器平均速度 ≥ `ROUNDABOUT_ENCODER_SPEED_MIN` 且距离上次环岛判定累积行程 ≥ `ROUNDABOUT_ENCODER_TRAVEL_MIN` 才会生效；
   - 触发后置位 `roundabout_detected`，状态切到 `ROUND_STATE_ENTRY_LOCK`，启动 `roundabout_timer = ROUNDABOUT_HOLD_TIME`，开启冷却 `roundabout_cooldown` 并重置里程计数；
   - 状态依次经历“入环锁死 → 绕行计时 → 出口搜索”，并在进入/退出瞬间打单次标记 `roundabout_entry_mark` / `roundabout_exit_mark` 便于示波或日志标注；
   - 根据状态点亮/熄灭 `LED1` 作为提示；
   - 绕行一圈寻找出口时，除原有特征外还开启“出口放宽”通道（`ROUNDABOUT_EXIT_*`），在亮度偏低或距离稍远时也能计数退出，避免卡在环岛内。
6. **速度目标选择**：`get_target_count_from_state` 根据当前状态切换三档编码器目标：直道、弯道、环岛/终点限速。

## 舵机控制（`DEMO/user/src/servo_control.c`）
1. **状态机统一调参**：
   - 依据误差幅值、变化率与环岛计时划分 7 个状态（直道、缓左/缓右、急左/急右、环岛进入/绕行/出口）。
   - 每个状态对应一行参数表，集中设置 `kp/kd`、前馈幅度上限与粘滞系数，便于快速对比和微调。
2. **掉头/反向急弯软化**：
   - 长时间同向高误差后突然换向会触发软化计时，D 项与前馈都会衰减，避免把连续发夹弯当成“来回反打”。
3. **环岛辅助**：
   - 触发环岛即锁定打死方向（`ROUNDABOUT_LOCK_ANGLE`，持续 `ROUNDABOUT_LOCK_TIME`），开环拉车入环；
   - 识别出口后继续按同向开环角度（`ROUNDABOUT_EXIT_ANGLE`，持续 `ROUNDABOUT_EXIT_TIME`）拉出出口；
   - 惰性变向默认关闭（`ROUNDABOUT_INERTIA_TIME=0`），如需叠加可调大该值，锁死阶段会直接跳过惰性平滑；
   - 绕行达到最小时长后自动切换到出口状态，前馈限幅略放宽，帮助迅速拉出出口。
4. **输出与限幅**：
   - 直道/惰性阶段按粘滞系数与上一周期平滑过渡；
   - 最终角度仍限制在 `SERVO_MOTOR_L_MAX` ~ `SERVO_MOTOR_R_MAX`，并通过三路 PWM 同步输出。

## 环岛易漏检的调试思路
若车辆进入环岛时速度过快，外圈未跟上导致检测不到，可以按以下两种方案择一（或结合）调试：
1. **延长环岛保持/绕行计时**（更宽松的时间窗口容错）：
   - 在 `DEMO/user/inc/my_common.h` 中增加 `ROUNDABOUT_HOLD_TIME`（进入后保持直行的时间）、`ROUNDABOUT_LAP_MIN_TIME`（至少绕行多久再找出口）或 `ROUNDABOUT_EXIT_CONFIRM`（出口模式防抖计数）。
   - 逐步把计时参数各自提高 10%~20%，实车测试到能稳定进入/识别出口为止。
2. **降低环岛段速度**（让传感器有充足时间采样特征）：
   - 在同一文件减小 `ROUNDABOUT_SPEED_DUTY` 或 `TARGET_COUNT_ROUNDABOUT`，必要时也可以略降 `CURVE_SPEED_DUTY`，保证进环岛前就提前减速。
   - 每次下调 2~4 个 duty 或 5% 的编码器目标，测试环岛是否能稳定触发，避免降速过度影响直道成绩。

## 速度控制（`DEMO/user/src/speed_control.c`）
1. **分段目标占空比**：
   - 终点：立即置 0；
   - 环岛：`ROUNDABOUT_SPEED_DUTY`；
   - 直道死区内（`|舵机偏差| <= STRAIGHT_DEAD_ZONE_DEG`）：`STRAIGHT_SPEED_DUTY`；
   - 其余情况视为弯道：`CURVE_SPEED_DUTY`。
2. **闭环速度 PID**：
   - 首次调用初始化 `speed_pid`，目标为 `get_target_count_from_state()` 给出的编码器计数；
   - PID 输出换算为占空比增量并与分段占空比叠加；
   - 最终 PWM 限制在 0~`PWM_DUTY_MAX`，同时对 duty 做上下限防护；差速阶段只轻微削减内轮，不再额外提升外轮速度，避免拐弯时整体加速过冲。

## 硬件与辅助
- `main.c` 中保留 LED 初始化，蜂鸣器 `BEEP` 相关代码已按要求注释，不再驱动。
- 所有定时、阈值与占空比上限均集中在 `DEMO/user/inc/my_common.h` 中，可按赛道与硬件微调。
