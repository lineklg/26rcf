# 编码器与电机模块移植设计

## 目标

将 MSPM0G3507 工程中的 `encoder.c/.h` 和 `motor.c/.h` 移植到当前
STM32H723 HAL 工程，同时保留原文件、主要公开接口、调用顺序和核心算法。
新增文件使用无 BOM UTF-8 编码，公开类型、宏和函数使用中文 Doxygen 注释。

本次只移植通用模块，不创建具体编码器或电机对象，不修改 `User_Init()`、
CubeMX 配置、GPIO 中断入口、定时器配置及现有 `DRV8870` 模块。

## 文件与依赖

- 新增 `User/encoder.h` 和 `User/encoder.c`。
- 新增 `User/motor.h` 和 `User/motor.c`。
- `CMakeLists.txt` 已递归收集 `User/*.c`，无需手工登记新增源文件。
- 编码器实现使用 STM32 HAL 的 `HAL_GPIO_ReadPin()` 和 `HAL_GetTick()`。
- 电机运动学模块继续通过调用方传入的微秒时基回调获取时间，不直接依赖某个
  STM32 定时器。
- `Motor.device` 保持为不透明指针，`motor.h` 不直接包含 `DRV8870.h` 或其他
  电机驱动头文件。

## 编码器接口兼容

保留以下主要类型和接口名称：

- `GPIO_Pin`：仍表示一个“GPIO 端口 + GPIO 引脚掩码”。端口在公开头文件中
  以不透明指针保存，实现文件中再转换为 `GPIO_TypeDef *`。
- `GPIO_Pin_IID`：为兼容原有 `Encoder_Create_UsePin()` 调用而保留；STM32
  实现不使用 MSPM0 的 IID 值。
- `GPIO_ISR_Item`、`Encoder`、`Encoder_USI`。
- `Encoder_Callback()`、`Encoder_Create_GPIO_ISR_Item()`、`Encoder_Create()`、
  `Encoder_Create_UsePin()`、`Encoder_GetChange()`、
  `Encoder_SetPrescaler()`、`Encoder_USI_Create()` 和
  `Encoder_USI_Update()`。

新增 `Encoder_GPIO_EXTI_Callback(uint16_t gpio_pin)` 作为 STM32 HAL 的转发
入口。应用若采用中断模式，应在自己的 `HAL_GPIO_EXTI_Callback()` 中把
`gpio_pin` 转交给该函数。模块不会自行定义 HAL 的弱回调，避免覆盖应用已有
中断处理。

模块最多管理 `ENCODER_MAX_COUNT` 个中断式编码器，并在内部静态保存每个
编码器的两个中断引脚项，不使用动态内存。创建失败时不写入无效对象或越过
静态表边界。

## 编码器行为

中断式编码器保留原来的双相电平判向原理：任一相发生边沿后读取另一相，按
相位关系对计数器加一或减一。轮询式 `Encoder_USI_Update()` 保留四状态转移表、
最近方向和跨两状态补偿逻辑。

轮询更新时间戳由每个 `Encoder_USI` 实例独立保存，避免多个实例共享原代码中
的全局时间戳而导致同一轮只有第一个实例更新。`ENCODER_USI_UPDATE_PERIOD`
仍以毫秒表示，默认值保持为 3。

`Encoder_GetChange()` 保留预分频余数累计方式：返回自上次读取以来的有符号
增量除以 `prescaler + 1`，未整除部分留待下次读取。创建时会完整初始化当前
计数、上次读取计数和预分频状态。

## 电机接口与计算

保留 `SetSpeedCallback`、`GetTimeCallback`、`Motor`、`Motor_Group` 以及：

- `Motor_Init()`；
- `Motor_CalcSpeed()`；
- `Motor_CalcSpeed_Smooth()`；
- `Motor_RecordCurrentPulse()`。

`Motor_CalcSpeed()` 继续根据编码器增量、每圈计数 `k`、轮周长 `l` 和微秒
时间差计算线速度，同时累计路程并计算加速度。`reverse` 继续统一反转速度和
路程符号。`Motor_CalcSpeed_Smooth()` 继续返回最近三次速度的中值。

每次速度计算只读取一次时基回调，保证本次计算的时间戳一致。若对象、编码器、
时基回调无效，或 `k == 0`、时间差为零，则函数返回 `0.0f`，且不执行除法。
`Motor_RecordCurrentPulse()` 仅接受 0 到 5 的标定下标，越界输入不写数组。

## 调用边界

模块不会替应用选择中断式或轮询式编码器：

- 中断式：应用配置 GPIO 双边沿 EXTI、调用 `Encoder_Create_UsePin()`，并转发
  HAL EXTI 回调。
- 轮询式：应用把 GPIO 配置为输入、调用 `Encoder_USI_Create()`，并按需要周期
  调用 `Encoder_USI_Update()`。

当前工程中 `PG6` 与 `PF6` 共用 EXTI6 线，因此是否能同时启用对应通道的
GPIO 中断属于后续硬件绑定阶段的问题，不由通用模块修改 CubeMX 配置解决。

## 测试与验收

新增主机端测试和最小 HAL 桩，覆盖：

- 正向、反向和跨两状态的轮询正交解码；
- 多个轮询实例拥有独立更新时间；
- 编码器增量、预分频和余数保留；
- 中断引脚分派和编码器数量上限；
- 电机速度、反向、路程、加速度和三点中值；
- 零时间差、零每圈计数、空依赖和标定下标越界。

验收时执行主机测试、STM32 Debug 构建、严格 UTF-8 检查，并重新计算四个
MSPM0 原文件的 SHA-256，确认源文件未被改动。
