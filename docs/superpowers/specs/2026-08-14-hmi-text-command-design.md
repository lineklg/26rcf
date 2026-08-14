# HMI 文本命令补全设计

## 目标

补全 `User/hmi.c` 的串口发送与文本控件修改功能。调用
`HMI_UART_Send_ModifyTxt(huart, "main.t0", "A: aigavh123")` 时，向 HMI
发送以下命令数据，并追加三个 `0xFF` 作为帧尾：

```text
main.t0.txt="A: aigavh123"
```

## 接口行为

- `HMI_UART_Send` 使用阻塞式 `HAL_UART_Transmit` 依次发送命令数据和
  `0xFF 0xFF 0xFF` 帧尾。
- 命令数据发送失败时不再发送帧尾；数据长度为零时也不进行发送。
- `HMI_UART_Send_ModifyTxt` 使用 `snprintf` 按
  `<widget>.txt="<txt>"` 格式生成命令，然后调用 `HMI_UART_Send`。
- `widget` 和 `txt` 按原字节写入，不额外转义其中的引号或控制字符；调用方
  负责传入 HMI 可接受的内容。
- 保持现有 `void` 函数签名，避免扩大调用方改动。
- UART 句柄、数据、控件名或文本为空指针时，不进行发送。

## 缓冲区与边界

- 命令缓冲区使用函数内部的 `static char data[128]`，避免占用任务栈。
- 缓冲区包含字符串终止符，因此命令数据最长为 127 字节。
- `snprintf` 返回负值或返回值大于等于缓冲区容量时，整条命令拒绝发送，
  不发送被截断的数据。
- 静态缓冲区使 `HMI_UART_Send_ModifyTxt` 不可重入；调用方不得从多个任务
  或任务与中断上下文并发调用该接口。

## 测试与验证

新增主机端测试并替换 `HAL_UART_Transmit`，验证：

1. 示例输入生成准确的命令数据，并在其后发送三个 `0xFF`。
2. `HMI_UART_Send` 按给定长度发送二进制数据，不依赖字符串终止符。
3. 空指针输入不会触发串口发送。
4. 超过静态缓冲区容量的命令不会触发串口发送。

最后运行 HMI 主机测试、工程构建和 `git diff --check`，确认行为、编译及文本
格式均符合要求。
