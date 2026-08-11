# 四轮 PID VOFA 串口上报设计

## 目标

在四轮速度环的 `PID_Task()` 中，通过现有
`VOFA_JustFloat_UART_Send()` 函数将每个轮子的目标速度、反馈速度和 PID
输出发送到 VOFA+。串口固定使用 115200 波特率的 `huart1`，PID 控制周期仍为
20 ms。

## 方案

每次 `PID_Task()` 完成四个轮子的测速、PID 计算和 PWM 写入后，统一发送一帧
12 通道 JustFloat 数据。相比逐轮发送，此方案只进行一次阻塞式 UART 调用，
四轮数据也属于同一个 PID 采样周期。

每帧包含 12 个 `float` 和 4 字节 JustFloat 帧尾，共 52 字节。50 Hz 上报时
数据量约为 2.6 KB/s，低于 115200 波特率 UART 的有效吞吐能力。

## 通道顺序

VOFA+ 通道索引固定为：

1. A 轮目标速度
2. A 轮反馈速度
3. A 轮 PID 输出
4. B 轮目标速度
5. B 轮反馈速度
6. B 轮 PID 输出
7. C 轮目标速度
8. C 轮反馈速度
9. C 轮 PID 输出
10. D 轮目标速度
11. D 轮反馈速度
12. D 轮 PID 输出

速度单位为 m/s，PID 输出为传给 `DRV8870_SetDutyPercent()` 的归一化占空比，
范围为 `-1.0f` 到 `1.0f`。

## 数据流

`PID_Task()` 创建 12 元素局部浮点数组。处理每个轮子时，将该轮目标速度、
`Motor_CalcSpeed_Smooth()` 的反馈值和 `PID_Calc()` 的输出按上述顺序写入数组，
随后照常将输出写入 DRV8870。

四个轮子全部处理完成后调用：

```c
VOFA_JustFloat_UART_Send(&huart1, vofa_data, 12U);
```

速度环停止后 PID 周期任务处于休眠状态，因此不会继续发送 VOFA 数据。

## 异常与实时性

VOFA 函数为阻塞式发送，单帧理论串行时间约 4.5 ms。该开销位于 20 ms PID
任务末尾，不改变本周期已经完成的测速、计算和 PWM 输出顺序。

串口发送返回值不改变 PID 历史、目标值或驱动输出。`PID_Task()` 使用显式
`(void)` 忽略返回值；本次不增加重试、错误计数、DMA 或发送降频。

若某个 PID 未成功创建，该轮沿用当前任务的跳过行为。VOFA 帧中该轮反馈值和
输出值写为零，目标速度仍按通道顺序发送，避免未初始化数据进入串口帧。

## 文件范围

- 修改 `Core/User/wheel_pid.c`：收集并发送 12 通道数据。
- 修改 `tests/wheel_pid_test.c`：捕获 VOFA 调用并验证句柄、次数、通道数和数据。
- 修改 `tests/stubs/main.h`：增加主机测试所需的最小 UART 类型。
- 不修改 `User/vofa.c`、`User/vofa.h`、`Core/User/behavior.c` 或
  `Core/User/behavior.h`。

现有 `Core/User/user.c` 的已暂存和未暂存并行修改不属于本任务，不改变、不暂存、
不提交。

## 测试与验收

主机测试使用真实 `PID_Task()`、PID 和任务调度代码，仅以桩函数替代 UART/VOFA
边界。测试覆盖：

- 启动速度环后，每个 PID 周期只调用一次 VOFA 发送函数。
- 发送句柄为 `&huart1`，通道数为 12。
- 12 通道严格按 A、B、C、D 的目标、反馈、输出顺序排列。
- 正向、负向和各轮不同反馈均保持现有 PID 计算结果。
- 速度环停止后不再产生 VOFA 帧。
- VOFA 返回错误时，本周期四轮 PWM 输出仍已完成。

实现完成后重新运行轮速 PID 主机测试、全部现有主机测试和 STM32 ARM Debug
构建。所有测试和构建必须返回零，新增和修改文件必须保持 UTF-8 编码。
