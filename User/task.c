#include "task.h"

static Task_Manager task_manager;

/**
 * @brief 完成普通任务或条件任务执行后的收尾处理。
 *
 * 将任务状态设置为休眠，使任务在本次执行完成后不再参与调度，
 * 直到外部再次调用 Task_Awake() 唤醒。
 *
 * @param[in,out] this 指向需要结束处理的任务对象，不可为 NULL。
 * @return 无。
 */
static void Task_EndProcess_Common(Task *this)
{
    Task_Sleep(this);
}

/**
 * @brief 完成周期任务执行后的收尾处理。
 *
 * 当周期参数有效时，在原计划运行时间上累加一个周期，计算下一次
 * 运行时间；周期无效时将任务转为休眠状态。
 *
 * @param[in,out] this 指向需要更新的周期任务对象，不可为 NULL。
 * @return 无。
 * @note 下一次运行时间基于原 run_tick 累加，可减少任务执行耗时引起的周期漂移。
 * @warning 任务周期必须大于 0 且不超过 TASK_PERIOD_MAX。
 */
static void Task_EndProcess_Period(Task *this)
{
    if (this->data.period != 0 && this->data.period <= TASK_PERIOD_MAX)
    {   
        this->run_tick += this->data.period;
    }
    else
    {
        Task_EndProcess_Common(this);
    }
}

/**********************************/

/**
 * @brief 创建一个任务并加入任务队列。
 *
 * 初始化任务的回调函数、类型和结束处理方式。新创建的任务默认为
 * 休眠状态，必须设置运行时间并调用 Task_Awake() 后才会参与调度。
 *
 * @param[out] dest 用于接收创建出的任务对象指针，不可为 NULL；
 *                  队列已满时写入 NULL。
 * @param[in] func 任务执行回调函数；允许为 NULL，但空回调任务不会被执行。
 * @param[in] type 任务类型，可选 TASK_COMMON、TASK_PERIOD 或 TASK_CONDITION。
 * @return 创建成功时返回任务在队列中的索引；队列已满时返回 UINT16_MAX。
 * @note 调用本函数前必须先调用 Task_Init()。
 */
uint16_t Task_Create(Task **dest, FuncCallback func, Task_Type type)
{
    if (task_manager.count >= TASK_QUEUE_SIZE)
    {
        *dest = NULL;
        return UINT16_MAX;
    }
    Task *this = &(task_manager.queue[task_manager.count]);
    this->func = func;
    this->type = type;
    this->run_tick = UINT32_MAX;
    this->state = TASK_SLEEP;
    switch (type)
    {
    case TASK_PERIOD:
        this->end_process = Task_EndProcess_Period;
        break;
    default:
        this->end_process = Task_EndProcess_Common;
        break;
    }
    *dest = this;
    task_manager.count++;
    return task_manager.count - 1;
}

/**
 * @brief 创建任务并立即将其设置为可调度状态。
 *
 * 创建任务后设置附加数据，将计划运行时间设置为当前 tick，并唤醒任务。
 *
 * @param[out] dest 用于接收创建出的任务对象指针，不可为 NULL；
 *                  队列已满时写入 NULL。
 * @param[in] func 任务执行回调函数；允许为 NULL，但空回调任务不会被执行。
 * @param[in] type 任务类型，可选 TASK_COMMON、TASK_PERIOD 或 TASK_CONDITION。
 * @param[in] data 任务附加数据：周期任务使用 period，条件任务使用 condition，
 *                 普通任务忽略该参数。
 * @return 创建成功时返回任务在队列中的索引；队列已满时返回 UINT16_MAX。
 * @note 普通任务和周期任务将在下一次有效调度检查时具备立即执行条件，
 *       而不是先等待一个完整周期。
 */
uint16_t Task_CreateAndStart(Task **dest, FuncCallback func, Task_Type type, Task_ExtraData data)
{
    if (task_manager.count >= TASK_QUEUE_SIZE)
    {
        *dest = NULL;
        return UINT16_MAX;
    }
    uint16_t index = Task_Create(dest, func, type);
    Task_SetExtraData(*dest, data);
    Task_SetRunTick_Current(*dest);
    Task_Awake(*dest);
    return index;
}

/**
 * @brief 设置任务类型对应的附加数据。
 *
 * 周期任务设置执行周期，条件任务设置条件回调函数，普通任务不使用附加数据。
 *
 * @param[in,out] this 指向需要设置的任务对象，不可为 NULL。
 * @param[in] data 待设置的附加数据，其有效成员由任务类型决定。
 * @return 无。
 * @note 无效的周期或空条件回调将被忽略，任务原有数据保持不变。
 */
void Task_SetExtraData(Task *this, Task_ExtraData data)
{
    switch (this->type)
    {
    case TASK_PERIOD:
        if (data.period != 0 && data.period <= TASK_PERIOD_MAX)
        {   
            this->data.period = data.period;
        }
        break;
    case TASK_CONDITION:
        if (data.condition != NULL)
        {   
            this->data.condition = data.condition;
        }
        break;
    default:
        break;
    }
}

/**
 * @brief 设置任务的绝对计划运行时间。
 *
 * @param[in,out] this 指向需要设置的任务对象，不可为 NULL。
 * @param[in] run_tick 任务计划运行的绝对 tick 值。
 * @return 无。
 * @note 本函数只修改运行时间，不会自动唤醒任务。
 * @warning 计划时间与当前时间的间隔不应超过 TASK_PERIOD_MAX，
 *          以保证有符号时间差比较有效。
 */
void Task_SetRunTick(Task *this, uint32_t run_tick)
{
    this->run_tick = run_tick;
}

/**
 * @brief 将任务的计划运行时间设置为当前 tick。
 *
 * @param[in,out] this 指向需要设置的任务对象，不可为 NULL。
 * @return 无。
 * @note 本函数只修改运行时间，不会自动唤醒任务。
 */
void Task_SetRunTick_Current(Task *this)
{
    Task_SetRunTick(this, task_manager.get_tick());
}

/**
 * @brief 设置任务从当前时刻开始延迟指定时间后运行。
 *
 * @param[in,out] this 指向需要设置的任务对象，不可为 NULL。
 * @param[in] delay 相对于当前 tick 的延迟量，单位由 get_tick 回调决定。
 * @return 无。
 * @note 本函数只修改运行时间，不会自动唤醒任务。
 * @warning delay 不应超过 TASK_PERIOD_MAX。
 */
void Task_SetRunTick_Delay(Task *this, uint32_t delay)
{
    Task_SetRunTick(this, task_manager.get_tick() + delay);
}

/**
 * @brief 唤醒任务，使其参与后续调度检查。
 *
 * @param[in,out] this 指向需要唤醒的任务对象，不可为 NULL。
 * @return 无。
 * @note 本函数不会修改任务的计划运行时间或附加数据。
 */
void Task_Awake(Task *this)
{
    this->state = TASK_AWAKE;
}

/**
 * @brief 使任务进入休眠状态并停止参与调度。
 *
 * @param[in,out] this 指向需要休眠的任务对象，不可为 NULL。
 * @return 无。
 * @note 休眠不会删除任务，也不会清除任务的运行时间和附加数据。
 */
void Task_Sleep(Task *this)
{
    this->state = TASK_SLEEP;
}

/**
 * @brief 初始化任务管理器。
 *
 * 设置系统 tick 获取回调，清空当前任务计数，并初始化上一次调度时间。
 *
 * @param[in] get_tick 获取当前系统 tick 的回调函数，不可为 NULL。
 * @return 无。
 * @note 必须在创建任务和调用 Task_Update() 前执行。
 * @warning 再次调用本函数会清空任务计数，原有任务将不再被调度。
 */
void Task_Init(GetTickCallback get_tick)
{
    task_manager.get_tick = get_tick;
    task_manager.count = 0;
    task_manager.last_tick = get_tick() - 1;
}

/**
 * @brief 根据任务索引获取任务对象指针。
 *
 * @param[out] dest 用于接收任务对象指针，不可为 NULL；索引无效时写入 NULL。
 * @param[in] index 创建任务时返回的队列索引。
 * @return 无。
 */
void Task_Get(Task **dest, uint16_t index)
{
    if (index < task_manager.count)
    {
        *dest = &(task_manager.queue[index]);
    }
    else
    {
        *dest = NULL;
    }
}

/**
 * @brief 轮询并执行满足条件的任务。
 *
 * 每当系统 tick 发生变化时遍历任务队列。仅处理处于唤醒状态且执行
 * 回调不为空的任务。对于普通任务和周期任务，在到达计划运行时间后
 * 执行回调；对于条件任务，在条件回调返回非零时执行任务回调。
 * 任务执行完成后，根据任务类型进行相应的结束处理。
 *
 * @return 无。
 * @note 本函数应在主循环中持续调用，同一个 tick 内最多执行一次队列扫描。
 * @note 执行回调为 NULL 的任务会被直接跳过，并保持原有任务状态。
 * @note 普通任务和条件任务执行一次后自动休眠；周期任务会更新下次运行时间。
 * @warning 所有回调均在调用 Task_Update() 的上下文中同步执行，应避免阻塞、
 *          长时间延时或在回调中再次调用 Task_Update()。
 */
void Task_Update()
{
    uint32_t tick = task_manager.get_tick();
    if (tick != task_manager.last_tick)
    {
        Task *task;
        for (uint16_t i = 0; i < task_manager.count; i++)
        {
            task = &(task_manager.queue[i]);
            if (task->state == TASK_AWAKE && task->func != NULL)
            {
                switch (task->type)
                {
                case TASK_CONDITION:
                    if (task->data.condition != NULL)
                    {
                        if (task->data.condition())
                        {
                            task->end_process(task);
                            task->func();
                        }
                    }
                    break;
                default:
                    if ((int32_t)(tick - task->run_tick) >= 0)
                    {
                        task->end_process(task);
                        task->func();
                    }
                    break;
                }
            }
        }
    }
    task_manager.last_tick = tick;
}

