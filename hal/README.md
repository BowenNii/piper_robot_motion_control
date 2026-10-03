# PiPER CAN 解码器

头文件：`hal/piper_can_decoder.hpp`。CMake 库目标：`piper_hal`。
只做字节到物理量的转换，不依赖 ROS、Eigen 或 SocketCAN。

| 编号 | 函数 | 输入 | 输出 |
| --- | --- | --- | --- |
| CAN-01 | `decode_int32_be` | 8 字节数组与起始偏移 0..4 | 大端补码 int32 |
| CAN-02 | `decode_status` | 0x2A1 帧 | 状态各字节与原始故障码 |
| CAN-03 | `decode_joint_angle_pair` | 0x2A5/6/7 帧 | 两个关节角 rad，与起始关节索引 |
| CAN-04 | `decode_joint_angles` | 0x2A5、0x2A6、0x2A7 各一帧，可乱序 | J1..J6 的六关节角 rad |
| CAN-05 | `decode_end_pose_pair` | 0x2A2/3/4 帧 | 两个位置/姿态分量 m 或 rad |
| CAN-06 | `decode_end_pose` | 0x2A2、0x2A3、0x2A4 各一帧，可乱序 | XYZ(m) 和 RX/RY/RZ(rad) |

```cpp
#include "hal/piper_can_decoder.hpp"

// J1=0，J2=-1.999 度；函数输出为 rad。
const hal::CanFrame frame{
    0x2A5, {0, 0, 0, 0, 0xFF, 0xFF, 0xF8, 0x31}};
const auto result = hal::decode_joint_angle_pair(frame);
// result.first_joint == 0；result.radians[1] 约 -0.034889132。
```

`CanFrame.id` 是协议 ID，不包含 Linux CAN 的 EFF/RTR/ERR 标志。
本模块输入必须是标准数据帧且长度为 8；接收层应先拒绝扩展帧、远程帧和错误帧。
非法 ID、长度、重复 ID 或越界偏移抛出 `std::invalid_argument`。
未知报文应由接收层分流，不能直接把所有报文传给关节角解码函数。

完整角度/位姿函数只检查帧集合，不检查时间同步；后续动态接收还要增加时间戳、
完整性、新鲜度和超时检查。不能直接拼接不同时刻的帧作为同一状态。

Project 1 现在读取 `candump -L` 日志，组装六帧反馈后进行 POE 对比，
输出首组矩阵、有效样本数及误差均值/RMS/最大值。它不直接连接 CAN。
厂家末端反馈不是独立的外部测量真值。

测试覆盖真实手算样本、补码边界、乱序组装、非法输入及固定种子随机整数。
测试通过 `all_pass` 和非零退出码报告失败，在 Release 中也会执行。

```bash
cmake --preset debug
cmake --build --preset debug-build
ctest --preset debug-test --output-on-failure
./build/debug/project1
# 或指定日志文件路径（默认使用下面这份真实日志）：
./build/debug/project1 /home/nbw/piper_can_static_2026-10-01.log
```

## 日志解析与反馈组装

头文件：`hal/can_log_parser.hpp`；实现：`hal/src/can_log_parser.cpp`。

| 编号 | 函数/类 | 输入 | 输出 |
| --- | --- | --- | --- |
| LOG-01 | `parse_can_log_line` | 一行 candump -L 文本 | 成功返回 true，填入时间戳、接口和 CanFrame；失败保持输出不变 |
| LOG-02 | `PiperFeedbackAssembler::push` | 带时间戳的一帧 | 六种 ID 收齐时返回反馈样本，否则返回 std::nullopt |

日志解析接受 0～8 字节标准 CAN 数据帧。扩展帧、CAN FD、远程帧等会被拒绝。
关节角/位姿解码要求对应反馈帧恰好为 8 字节。
演示只对 `can0` 的 0x2A2..0x2A7 做比较，其他正常报文计入 ignored。
六帧配对窗口默认 2 ms；重复、超时、时间倒退或接口改变会重新开始收集。
窗口是离线配对规则，不保证硬件严格同步，也不适合未经检查直接用于动态控制。
不同设备/固件应根据实际时间跨度调整窗口，并检查数据来源及反馈生成延迟。

原始日志保持不变。多个静止且重复的样本用于验证解码/回放，不代表多个不同姿态。
暂无完整样本、文件无法打开或读取失败时演示程序以非零退出码结束。
