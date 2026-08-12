# 四轮独立目标速度设计

## 目标

在现有四轮 PID 速度环中增加单轮和四轮目标速度设置接口，使 A、B、C、D 四个轮子可以使用不同的目标线速度。新接口设置目标后自动启动 PID 速度环，现有直行、转向和停止接口保持兼容。

## 方案

在 `Core/User/wheel_pid.h` 和 `Core/User/wheel_pid.c` 中增加以下接口：

```c
void WheelPID_SetSpeed(uint8_t index, float speed);
void WheelPID_SetSpeeds(
    float speed_a,
    float speed_b,
    float speed_c,
    float speed_d
);
```

轮子索引 `0`、`1`、`2`、`3` 分别对应 A、B、C、D 轮，速度单位为 `m/s`。新增和修改的 C 源码使用 UTF-8 编码以及中文 Doxygen 注释。

## 行为与数据流

`WheelPID_SetSpeed(index, speed)` 只更新指定轮子的 `wheel_target[index]`，其余三个轮子的目标速度保持不变。有效目标写入后调用现有内部启动函数；如果速度环已经运行，只更新目标而不清除四个 PID 的历史状态。

`WheelPID_SetSpeeds(speed_a, speed_b, speed_c, speed_d)` 一次更新 A、B、C、D 四个目标速度，然后调用现有内部启动函数。四个参数直接作为各轮目标值使用，不应用 `WheelPID_Forward()` 当前针对部分轮子的比例修正，以保证独立设置接口的输入语义明确。

速度环处于停止状态时，任一有效的新接口调用都会清除现有 PID 历史并自动启动周期任务。后续仍通过 `PID_Task()` 分别读取每个轮子的编码器反馈、执行对应 PID 计算并输出到对应驱动。

现有接口行为保持不变：

- `WheelPID_Forward(speed)` 继续设置直行目标及其现有修正。
- `WheelPID_Turn(speed)` 继续设置原地转向目标。
- `WheelPID_Stop()` 继续停止任务、清零四轮目标并清除 PID 历史。

## 异常与边界

当 `WheelPID_SetSpeed()` 收到大于或等于 `WHEEL_PID_COUNT` 的索引时，函数直接返回，不修改任何目标，也不启动速度环。目标速度范围不在本次改动中新增限制，PID 和驱动层继续负责现有输出限幅。

## 测试与验收

新增主机侧 `tests/wheel_pid_test.c`，使用真实 PID 与任务调度代码，并通过硬件边界桩函数观察四路控制输出。测试至少覆盖：

- 四轮设置接口将四个不同目标分别传给对应 PID。
- 单轮设置接口只修改指定轮目标，其他轮目标保持不变。
- 新接口在停止状态下自动启动速度环。
- 速度环运行时更新目标不清除已有 PID 历史。
- 无效轮子索引不修改目标且不启动速度环。
- 现有直行、转向和停止行为不回归。

实现完成后运行新增主机测试、现有主机回归测试和 STM32 Debug 构建。验收标准为测试全部通过、交叉编译成功且没有新增编译警告。
