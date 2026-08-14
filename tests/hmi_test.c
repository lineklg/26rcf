#include "hmi.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

#define UART_CALL_CAPACITY 4U
#define UART_DATA_CAPACITY 128U

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t data[UART_DATA_CAPACITY];
    uint16_t size;
    uint32_t timeout;
} UartCall;

static UART_HandleTypeDef test_uart;
static UartCall uart_calls[UART_CALL_CAPACITY];
static uint32_t uart_call_count;
static HAL_StatusTypeDef next_uart_status;

/**
 * @brief 捕获阻塞式 UART 发送调用。
 * @param[in,out] huart UART 句柄。
 * @param[in] data 待发送数据。
 * @param[in] size 数据长度。
 * @param[in] timeout 超时时间。
 * @return 当前测试设置的 HAL 状态。
 */
HAL_StatusTypeDef HAL_UART_Transmit(
    UART_HandleTypeDef *huart,
    const uint8_t *data,
    uint16_t size,
    uint32_t timeout
)
{
    UartCall *call;

    assert(uart_call_count < UART_CALL_CAPACITY);
    assert(size <= UART_DATA_CAPACITY);
    call = &uart_calls[uart_call_count++];
    call->huart = huart;
    call->size = size;
    call->timeout = timeout;
    memcpy(call->data, data, size);
    return next_uart_status;
}

/**
 * @brief 重置 UART 发送测试夹具。
 * @return 无。
 */
static void Reset_Fixture(void)
{
    memset(uart_calls, 0, sizeof(uart_calls));
    uart_call_count = 0U;
    next_uart_status = HAL_OK;
}

/**
 * @brief 验证文本控件命令及帧尾格式。
 * @return 无。
 */
static void Test_ModifyTxt_Sends_Formatted_Command_And_Tail(void)
{
    static const char expected[] = "main.t0.txt=\"A: aigavh123\"";
    static const uint8_t expected_tail[] = {0xFFU, 0xFFU, 0xFFU};

    Reset_Fixture();
    HMI_UART_Send_ModifyTxt(&test_uart, "main.t0", "A: aigavh123");

    assert(uart_call_count == 2U);
    assert(uart_calls[0].huart == &test_uart);
    assert(uart_calls[0].size == sizeof(expected) - 1U);
    assert(memcmp(uart_calls[0].data, expected, sizeof(expected) - 1U) == 0);
    assert(uart_calls[0].timeout == HAL_MAX_DELAY);
    assert(uart_calls[1].size == sizeof(expected_tail));
    assert(memcmp(uart_calls[1].data, expected_tail, sizeof(expected_tail)) == 0);
}

/**
 * @brief 验证基础接口按明确长度发送二进制数据。
 * @return 无。
 */
static void Test_Send_Uses_Explicit_Binary_Size(void)
{
    static const uint8_t data[] = {0x41U, 0x00U, 0x42U};

    Reset_Fixture();
    HMI_UART_Send(&test_uart, data, sizeof(data));

    assert(uart_call_count == 2U);
    assert(uart_calls[0].size == sizeof(data));
    assert(memcmp(uart_calls[0].data, data, sizeof(data)) == 0);
}

/**
 * @brief 验证空指针和零长度输入不会发送数据。
 * @return 无。
 */
static void Test_Invalid_Input_Does_Not_Send(void)
{
    static const uint8_t data[] = {0x41U};

    Reset_Fixture();
    HMI_UART_Send(NULL, data, sizeof(data));
    HMI_UART_Send(&test_uart, NULL, sizeof(data));
    HMI_UART_Send(&test_uart, data, 0U);
    HMI_UART_Send_ModifyTxt(NULL, "main.t0", "text");
    HMI_UART_Send_ModifyTxt(&test_uart, NULL, "text");
    HMI_UART_Send_ModifyTxt(&test_uart, "main.t0", NULL);
    assert(uart_call_count == 0U);
}

/**
 * @brief 验证恰好 127 字节的命令可发送，128 字节命令被拒绝。
 * @return 无。
 */
static void Test_ModifyTxt_Rejects_Overflow(void)
{
    char fitting_text[120];
    char overflow_text[121];

    memset(fitting_text, 'x', sizeof(fitting_text) - 1U);
    fitting_text[sizeof(fitting_text) - 1U] = '\0';
    memset(overflow_text, 'x', sizeof(overflow_text) - 1U);
    overflow_text[sizeof(overflow_text) - 1U] = '\0';

    Reset_Fixture();
    HMI_UART_Send_ModifyTxt(&test_uart, "w", fitting_text);
    assert(uart_call_count == 2U);
    assert(uart_calls[0].size == 127U);

    Reset_Fixture();
    HMI_UART_Send_ModifyTxt(&test_uart, "w", overflow_text);
    assert(uart_call_count == 0U);
}

/**
 * @brief 验证命令主体发送失败时不追加帧尾。
 * @return 无。
 */
static void Test_Send_Failure_Does_Not_Send_Tail(void)
{
    static const uint8_t data[] = {0x41U};

    Reset_Fixture();
    next_uart_status = HAL_ERROR;
    HMI_UART_Send(&test_uart, data, sizeof(data));
    assert(uart_call_count == 1U);
}

/**
 * @brief 运行 HMI 串口发送测试。
 * @return 全部断言通过时返回 0。
 */
int main(void)
{
    Test_ModifyTxt_Sends_Formatted_Command_And_Tail();
    Test_Send_Uses_Explicit_Binary_Size();
    Test_Invalid_Input_Does_Not_Send();
    Test_ModifyTxt_Rejects_Overflow();
    Test_Send_Failure_Does_Not_Send_Tail();
    return 0;
}
