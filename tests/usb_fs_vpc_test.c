#include "usb_fs_vpc.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/**
 * @brief 模拟 USB CDC 发送成功。
 * @param[in] buffer 待发送数据。
 * @param[in] length 待发送字节数。
 * @return 固定返回 USBD_OK。
 */
uint8_t CDC_Transmit_HS(uint8_t *buffer, uint16_t length)
{
    (void)buffer;
    (void)length;
    return USBD_OK;
}

/**
 * @brief 清空环形缓冲区，隔离各测试用例。
 * @return 无。
 */
static void clear_rx_buffer(void)
{
    uint8_t scratch[USB_RX_BUFFER_MAX_SIZE];

    while (USB_VPC_RxRead(scratch, sizeof(scratch)) != 0U) {
    }
}

/**
 * @brief 验证普通写入以及请求长度超过可用量时的读取。
 * @return 无。
 */
static void test_write_and_read_available_data(void)
{
    const uint8_t input[] = {1U, 2U, 3U};
    uint8_t output[5] = {0U};

    clear_rx_buffer();
    assert(USB_VPC_RxWrite(input, sizeof(input)) == sizeof(input));
    assert(USB_VPC_RxRead(output, sizeof(output)) == sizeof(input));
    assert(memcmp(output, input, sizeof(input)) == 0);
    assert(USB_VPC_RxRead(output, sizeof(output)) == 0U);
}

/**
 * @brief 验证 512 字节容量全部可用且空间不足时整包拒绝。
 * @return 无。
 */
static void test_full_buffer_rejects_entire_write(void)
{
    uint8_t input[USB_RX_BUFFER_MAX_SIZE];
    uint8_t output[USB_RX_BUFFER_MAX_SIZE];
    const uint8_t extra[] = {0xA5U};
    size_t i;

    clear_rx_buffer();
    for (i = 0U; i < sizeof(input); i++) {
        input[i] = (uint8_t)i;
    }

    assert(USB_VPC_RxWrite(input, sizeof(input)) == sizeof(input));
    assert(USB_VPC_RxWrite(extra, sizeof(extra)) == 0U);
    assert(USB_VPC_RxRead(output, sizeof(output)) == sizeof(output));
    assert(memcmp(output, input, sizeof(input)) == 0);
}

/**
 * @brief 验证读写位置跨越数组末尾后仍保持字节顺序。
 * @return 无。
 */
static void test_wraparound_preserves_order(void)
{
    uint8_t prefix[400];
    uint8_t input[200];
    uint8_t output[200];
    size_t i;

    clear_rx_buffer();
    memset(prefix, 0xCC, sizeof(prefix));
    assert(USB_VPC_RxWrite(prefix, sizeof(prefix)) == sizeof(prefix));
    assert(USB_VPC_RxRead(prefix, sizeof(prefix)) == sizeof(prefix));

    for (i = 0U; i < sizeof(input); i++) {
        input[i] = (uint8_t)(i + 17U);
    }
    assert(USB_VPC_RxWrite(input, sizeof(input)) == sizeof(input));
    assert(USB_VPC_RxRead(output, sizeof(output)) == sizeof(output));
    assert(memcmp(output, input, sizeof(input)) == 0);
}

/**
 * @brief 验证部分读取释放空间后可继续写入并保持 FIFO 顺序。
 * @return 无。
 */
static void test_interleaved_read_and_write(void)
{
    const uint8_t first[] = {10U, 11U, 12U, 13U};
    const uint8_t second[] = {20U, 21U, 22U};
    const uint8_t expected[] = {12U, 13U, 20U, 21U, 22U};
    uint8_t output[sizeof(expected)];

    clear_rx_buffer();
    assert(USB_VPC_RxWrite(first, sizeof(first)) == sizeof(first));
    assert(USB_VPC_RxRead(output, 2U) == 2U);
    assert(USB_VPC_RxWrite(second, sizeof(second)) == sizeof(second));
    assert(USB_VPC_RxRead(output, sizeof(output)) == sizeof(output));
    assert(memcmp(output, expected, sizeof(expected)) == 0);
}

/**
 * @brief 验证空指针与零长度不会改变缓冲区状态。
 * @return 无。
 */
static void test_invalid_arguments_do_not_change_state(void)
{
    const uint8_t input[] = {7U, 8U};
    uint8_t output[sizeof(input)] = {0U};

    clear_rx_buffer();
    assert(USB_VPC_RxWrite(NULL, sizeof(input)) == 0U);
    assert(USB_VPC_RxWrite(input, 0U) == 0U);
    assert(USB_VPC_RxWrite(input, sizeof(input)) == sizeof(input));
    assert(USB_VPC_RxRead(NULL, sizeof(output)) == 0U);
    assert(USB_VPC_RxRead(output, 0U) == 0U);
    assert(USB_VPC_RxRead(output, sizeof(output)) == sizeof(output));
    assert(memcmp(output, input, sizeof(input)) == 0);
}

/**
 * @brief 运行 USB VPC 接收环形缓冲区主机测试。
 * @return 全部断言通过时返回 0。
 */
int main(void)
{
    test_write_and_read_available_data();
    test_full_buffer_rejects_entire_write();
    test_wraparound_preserves_order();
    test_interleaved_read_and_write();
    test_invalid_arguments_do_not_change_state();
    return 0;
}
