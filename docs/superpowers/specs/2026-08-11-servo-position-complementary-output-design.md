# ServoPosition 互补输出与脉宽修复设计

## 目标

- 让 `ServoPosition` 显式区分普通 PWM 输出和互补 PWM 输出。
- 让 TIM8 上的三个机械臂舵机使用 0.5 ms、1.5 ms、2.5 ms 作为最小、中位、最大脉宽。
- 保持现有归一化位置接口和插值行为不变。

## 接口设计

新增 `ServoPositionOutput` 枚举，包含普通输出和互补输出两种取值。`ServoPosition` 保存输出类型，`ServoPosition_Init()` 增加必填的 `output` 参数。

初始化和反初始化根据输出类型分派：

- 普通输出使用 `HAL_TIM_PWM_Start()` 和 `HAL_TIM_PWM_Stop()`。
- 互补输出使用 `HAL_TIMEx_PWMN_Start()` 和 `HAL_TIMEx_PWMN_Stop()`。

非法输出类型直接返回 `HAL_ERROR`。现有位置设置仍通过同一 CCR 寄存器完成，不需要区分输出类型。

## 调用方配置

TIM8_CH1 对应的 PA7 实际为 `TIM8_CH1N`，因此 `arm_servo[0]` 使用互补输出。TIM8_CH2 和 TIM8_CH3 使用普通输出。

TIM8 计数频率为 2.75 MHz，因此三个脉宽对应的比较值为：

| 位置 | 脉宽 | 比较值 |
| --- | ---: | ---: |
| 最小 | 0.5 ms | 1375 |
| 中位 | 1.5 ms | 4125 |
| 最大 | 2.5 ms | 6875 |

三个机械臂舵机统一使用这组范围。

## 测试

新增主机侧 `ServoPosition` 测试和所需 HAL 桩，覆盖：

- 普通输出初始化调用普通 PWM 启动函数。
- 互补输出初始化调用互补 PWM 启动函数。
- 两种输出反初始化时调用与启动类型匹配的停止函数。
- 非法输出类型初始化失败。
- 0.5/1.5/2.5 ms 比较值在 `-1/0/1` 位置得到正确结果。

最后运行专项测试、现有主机测试和 Debug 固件构建。
