# 按当前雷达角度直行设计

## 目标

新增一个与 `Wheel_Forward_WithRadar_AxisX` 类似的行为函数，使底盘在调用瞬间的雷达角度方向上直行，并在沿该方向移动指定距离后自动停车。方向由当前 `radar_get_angle` 决定，不增加角度参数。

## 接口

```c
void Wheel_Forward_WithRadar_CurrentAngle(float speed, float route_m);
```

`route_m` 表示沿调用瞬间朝向的有符号位移；正值沿当前朝向前进，负值沿反方向移动。`speed` 保持现有直行函数语义，实际轮速由调用者决定，停止方向判断以 `route_m` 为准。

## 实现

调用时保存雷达角度 `angle0` 以及位置 `(x0, y0)`，并将 `angle0` 设置为车轮角度保持目标。行驶过程中计算：

```text
progress = (radar_x - x0) * cos(angle0)
         + (radar_y - y0) * sin(angle0)
```

目标为 `route_m`。当目标误差小于 `WHEEL_TARGET_AXIS_ERROR`，或运动已经越过目标时，停止任务触发 `Wheel_Stop`。任意输入、坐标或角度非有限，速度接近零，距离过小，或停止任务不存在时，立即停车。

开始运动后启用 `enable_fix_angle`，确保底盘保持调用瞬间朝向；取消或完成运动时沿用现有角度控制清理逻辑。

## 测试

- 角度为 0 时，X 方向位移达到目标后停车。
- 角度为 `pi / 2` 时，Y 方向位移达到目标后停车。
- 角度为非轴向值（如 `pi / 4`）时，仅沿投影方向达到目标后停车，纯横向位移不会误触发。
- 负距离、无效参数、零速度和越过目标时均能安全停车。
