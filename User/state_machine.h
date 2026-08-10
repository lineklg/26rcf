#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include <stdint.h>

/**
 * @file state_machine.h
 * @brief 简单有限状态机的公开接口。
 *
 * 本模块使用整数 ID 表示状态，不分配动态内存，也不依赖具体单片机或操作系统。需要更改StateMachine_GetTick(void);
 */

/** @brief 表示状态机当前没有活动状态。 */
#define STATE_MACHINE_NO_STATE UINT16_MAX

/**
 * @brief 状态进入和退出事件的回调函数类型。
 *
 * @param[in] state_id 发生事件的状态 ID。
 * @param[in] enter_or_exit 事件类型，取值为 @ref STATE_ENTER 或 @ref STATE_EXIT。
 */
typedef void (*StateChangeCallback)(uint16_t state_id, uint8_t enter_or_exit);

/**
 * @brief 状态事件类型。
 */
typedef enum
{
    STATE_EXIT = 0, /**< 离开状态。 */
    STATE_ENTER = 1 /**< 进入状态。 */
} StateEvent;

/**
 * @brief 状态机。
 */
typedef struct
{
    uint16_t current_state_id;       /**< 当前状态 ID；无活动状态时为 @ref STATE_MACHINE_NO_STATE。 */
    uint32_t last_state_change_tick; /**< 初始化或上次实际切换状态时的 HAL tick。 */
    StateChangeCallback callback;    /**< 状态事件回调；允许为空。 */
} StateMachine;

/**
 * @brief 获取当前 tick。
 *
 * @return @ref GetTick() 的当前返回值。
 */
uint32_t StateMachine_GetTick(void);

/**
 * @brief 初始化状态机。
 *
 * 当 @p initial_state_id 不是 @ref STATE_MACHINE_NO_STATE 且 @p callback 不为空时，
 * 初始化会立即触发一次 @ref STATE_ENTER 事件。传入空的 @p machine 时不执行任何操作。
 *
 * @param[out] machine 要初始化的状态机。
 * @param[in] initial_state_id 初始状态 ID；无初始状态时传入 @ref STATE_MACHINE_NO_STATE。
 * @param[in] callback 状态事件回调；允许为空。
 */
void StateMachine_Init(
    StateMachine *machine,
    uint16_t initial_state_id,
    StateChangeCallback callback
);

/**
 * @brief 切换状态机的当前状态。
 *
 * 切换时先触发旧状态的 @ref STATE_EXIT 事件，再更新当前状态 ID，最后触发新状态的
 * @ref STATE_ENTER 事件。切换到当前状态自身时不会重复触发事件。将
 * @p next_state_id 设为 @ref STATE_MACHINE_NO_STATE 可以退出当前状态。
 *
 * @param[in,out] machine 目标状态机。
 * @param[in] next_state_id 要进入的状态 ID；退出当前状态时传入 @ref STATE_MACHINE_NO_STATE。
 * @return 成功返回 1；@p machine 为空时返回 0。
 */
uint8_t StateMachine_Change(StateMachine *machine, uint16_t next_state_id);

/**
 * @brief 获取状态机的当前状态 ID。
 *
 * @param[in] machine 要查询的状态机。
 * @return 当前状态 ID；@p machine 为空或当前无状态时返回 @ref STATE_MACHINE_NO_STATE。
 */
uint16_t StateMachine_GetCurrent(const StateMachine *machine);

/**
 * @brief 获取状态机初始化或上次实际切换状态时的 HAL tick。
 *
 * @param[in] machine 要查询的状态机。
 * @return 上次状态变化时记录的 tick；@p machine 为空时返回 0。
 */
uint32_t StateMachine_GetLastChangeTick(const StateMachine *machine);

#endif /* STATE_MANAGER_H */
