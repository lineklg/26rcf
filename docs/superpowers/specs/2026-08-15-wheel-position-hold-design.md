# 单轴位置保持设计

## 目标

在四轮差速底盘转向期间，通过雷达位置反馈保持运行时指定的世界坐标 X 轴或 Y 轴位置。控制器允许调用方显式设置坐标轴和目标位置，并与现有轮速环、角度保持环同时工作。

该功能以减小单轴位置误差为目标。受差速底盘非完整约束影响，当车体前进方向与目标轴垂直时，底盘瞬时没有该轴向的修正能力，因此不承诺全过程零误差，而是保证控制输出有界并在具备运动学控制能力时收敛。

## 接口

在 `wheel_pid.h` 中新增位置轴枚举、位置保持使能量和目标设置函数：

```c
typedef enum {
    WHEEL_PID_POSITION_AXIS_X = 0,
    WHEEL_PID_POSITION_AXIS_Y = 1
} WheelPID_PositionAxis;

extern uint8_t enable_fix_pos;

void WheelPID_SetTargetPosition(
    WheelPID_PositionAxis axis,
    float target_position
);
```

`WheelPID_SetTargetPosition()` 显式保存目标轴和目标坐标。无效轴或非有限目标值不会修改当前有效配置。

`enable_fix_pos` 与现有 `enable_fix_angle` 保持相同的使用方式：只有值严格等于 `1U` 时才调用位置 `PID_Calc()` 并叠加位置修正。包括 `2U` 在内的其他值都视为禁用。`WheelPID_Stop()` 将其清零。

## 控制过程

初始化时只创建一个 X/Y 共用的 `radar_pos_error_pid`，初始参数为 `kp = 1.0f`、`ki = 0.0f`、`kd = 0.0f`，PID 输出限幅为 `0.1f`。同一时刻只保持当前选定的一个坐标轴，因此不为两个轴分别分配 PID。

每次 `PID_Task()` 运行时，位置环按以下过程计算：

1. 检查位置保持是否启用，以及目标轴、雷达坐标和雷达角度是否为有效有限值。
2. 使用 `PID_Calc(radar_pos_error_pid, target_position, radar_get_axis[axis])` 得到世界坐标目标轴上的位置修正量。
3. 将修正量投影到底盘前进方向。X 轴使用 `cosf(radar_get_angle)`，Y 轴使用 `sinf(radar_get_angle)`。
4. 将投影后的修正量同号叠加到 A、B、C、D 四个轮子的最终驱动输出。

四轮输出关系为：

```text
drive[i] = speed_pid[i]
         + speed_feed_forward[i]
         + angle_correction[i]
         + position_correction
```

角度修正继续对 A/C 与 B/D 使用相反符号，位置修正对四轮使用相同符号。因此转动和平移修正可以同时叠加。

采用乘法投影而不是除法补偿，可以避免车体朝向接近目标轴垂直方向时产生奇异值或放大雷达噪声。垂直时修正自然降为零，旋转到重新具备该轴控制能力后继续收敛。

## 状态管理

使用 `enable_fix_pos_previous` 记录上个控制周期的原始使能值。只要 `enable_fix_pos` 的值发生任何变化，包括 `0U -> 1U`、`1U -> 0U` 或 `1U -> 2U`，就在本周期计算前清除 `radar_pos_error_pid` 的历史并更新记录值。目标轴发生变化时，同样清除该 PID 的历史，防止上一轴的历史状态影响新轴控制。

设置新的同轴目标不主动停止速度环，也不修改四轮基础目标速度。位置保持禁用时不叠加任何位置修正。停车时关闭位置保持并清除轮速 PID 历史，保持现有停车语义。

## 异常处理

- 无效坐标轴或非有限目标值：忽略本次设置，保留上一次有效配置。
- 当前选定轴的雷达位置非有限值：本周期位置修正为零。
- 雷达角度非有限值：本周期位置修正为零。
- `radar_pos_error_pid` 创建失败：位置修正保持为零，其他轮速和角度控制继续运行。
- `enable_fix_pos` 不是 `1U`：位置修正保持为零。

## 测试

扩展主机端 `wheel_pid_test.c`，以真实 PID 实现验证：

- 禁用位置保持时不改变四轮输出。
- X 轴位置误差按 `cosf(yaw)` 投影并同号叠加到四轮。
- Y 轴位置误差按 `sinf(yaw)` 投影并同号叠加到四轮。
- 航向与目标轴垂直时位置修正为零。
- 位置 PID 输出遵守 `0.1f` 限幅。
- 无效轴或非有限目标不覆盖上一次有效配置。
- 非有限雷达位置或角度不产生位置修正。
- 使能变化和目标轴切换会清除对应 PID 历史。
- `WheelPID_Stop()` 禁用位置保持。
- 现有直行、转向、角度保持和停车测试继续通过。
