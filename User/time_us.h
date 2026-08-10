#ifndef TIME_US_H
#define TIME_US_H

#include <stdint.h>

/**
 * @brief 初始化 DWT 微秒时基。
 *
 * 必须在系统时钟配置完成后调用一次。
 */
void TimeUs_Init(void);

/**
 * @brief 获取初始化后经过的微秒时间。
 *
 * 本函数应由同一执行上下文周期调用，调用间隔必须短于 DWT 周期计数器的
 * 单次回绕时间。
 *
 * @return 单调递增的微秒时间戳。
 */
uint64_t TimeUs_Get(void);

#endif /* TIME_US_H */
