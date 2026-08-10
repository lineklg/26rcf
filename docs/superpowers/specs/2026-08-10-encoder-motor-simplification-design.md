# 编码器与电机模块精简设计

## 目标

精简 `User/encoder.h/.c` 和 `User/motor.h/.c`，删除当前 STM32 工程不需要的
MSPM0 底层兼容层，同时保持中断式编码器、轮询式编码器、电机速度计算和安全
边界行为不变。四个文件继续使用无 BOM UTF-8 和简洁的中文 Doxygen 注释。

## 编码器接口

删除以下未使用的公开类型和函数：

- `GPIO_ISR_Item`、`GPIO_Pin_IID`、`GPIO_Interrupt_Callback`；
- `Encoder_Callback()`；
- `Encoder_Create_GPIO_ISR_Item()`；
- 接收中断项指针的 `Encoder_Create()`。

`Encoder` 改为直接保存 `GPIO_Pin pin_A` 和 `GPIO_Pin pin_B`。保留以下接口：

```c
void Encoder_Create_UsePin(Encoder *encoder, GPIO_Pin pin_A, GPIO_Pin pin_B);
int32_t Encoder_GetChange(Encoder *encoder);
void Encoder_SetPrescaler(Encoder *encoder, uint16_t prescaler);
void Encoder_USI_Create(Encoder_USI *encoder, GPIO_Pin pin_A, GPIO_Pin pin_B);
void Encoder_USI_Update(Encoder_USI *encoder);
void Encoder_GPIO_EXTI_Callback(uint16_t gpio_pin);
```

`Encoder_Create_UsePin()` 初始化对象并把对象指针登记到最多四项的静态表。
`Encoder_GPIO_EXTI_Callback()` 直接遍历该表：匹配 A 相时读取 A/B 电平并按 A 相
公式计数，匹配 B 相时按 B 相公式计数。模块仍不定义 HAL 弱回调，也不配置
CubeMX 或具体引脚。

轮询式编码器继续使用原四状态转移表、每实例更新时间戳、最近方向和跨两状态
补偿。`Encoder_GetChange()` 的预分频及余数保留方式不变。

## 电机模块

`Motor` 字段、`Motor_Group`、两个回调类型及以下函数签名保持不变：

```c
Motor Motor_Init(...);
float Motor_CalcSpeed(Motor *motor);
float Motor_CalcSpeed_Smooth(Motor *motor);
void Motor_RecordCurrentPulse(Motor *motor, uint16_t index, float pulse);
```

实现继续保留速度历史、三点中值、反向、累计路程、加速度和六项标定表。通过
合并局部变量、简化初始化和缩短注释减少代码，但不删除空依赖、`k == 0`、
零时间差及标定下标越界保护。

## 注释与编码

头文件为每个公开类型、宏和函数保留中文 Doxygen；字段使用简短行尾 Doxygen，
不重复解释能从名称直接看出的实现细节。源文件只在状态转移或判向等不直观位置
保留必要注释。四个模块文件继续使用严格无 BOM UTF-8。

## 测试

先修改主机测试以使用三参数 `Encoder_Create_UsePin()` 并观察旧实现编译失败，
再重构生产代码。测试继续覆盖：

- 正反向轮询、跨两状态补偿和多实例独立时间戳；
- 中断分派、预分频和四实例容量上限；
- 电机速度、反向、路程、加速度、中值和异常输入。

完成后重新运行严格主机 GCC 测试、原状态机回归测试、STM32 Debug 构建和
UTF-8 检查。外部 MSPM0 源文件仍保持只读且哈希不变。
