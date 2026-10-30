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

完整角度/位姿解码函数只检查帧集合，不检查时间同步；离线组装器另做组内接收时间窗口检查。
实时控制仍需要反馈新鲜度、看门狗和设备采样同步机制。不能把接收窗口当作采样同步保证。

Project 1 现在读取 `candump -L` 日志，组装六帧反馈后进行 POE 对比，
输出首组矩阵、有效样本数及误差均值/RMS/最大值。它不直接连接 CAN。
厂家末端反馈不是独立的外部测量真值。

测试覆盖真实手算样本、补码边界、乱序组装、非法输入及固定种子随机整数。
测试通过 `all_pass` 和非零退出码报告失败，在 Release 中也会执行。

```bash
cmake --preset debug
cmake --build --preset debug-build
ctest --preset debug-test --output-on-failure
./build/debug/fk_validation
# 或指定日志文件路径（默认使用下面这份真实日志）：
./build/debug/fk_validation /home/nbw/piper_can_static_2026-10-01.log
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

## 组装诊断（已实现）

diagnostics() 返回累计 FeedbackAssemblerDiagnostics；不改变正在组装的帧。

| 字段 | 含义 |
| --- | --- |
| received_frames / ignored_frames / invalid_frames | push调用数 / 非目标ID数 / 非法目标帧数 |
| completed_samples | 完整输出的组数 |
| discarded_groups / discarded_frames | 被重启或人工reset丢弃的未完成组数 / 旧帧数 |
| duplicate_ids | 组内同ID再次到达 |
| over_window | 当前帧距组首帧超过配置窗口 |
| time_reversals | 组内当前时间早于上一帧 |
| interface_changes | 组内接口名改变 |
| manual_resets | reset调用时有待组装帧的次数 |

多个原因可能同时触发，所以原因计数不能相加当作丢弃组数。
触发自动重启的当前帧会开始新组，不计入旧帧丢弃数量。
完整组输出后正常清空不算丢弃；EOF待处理帧另外用 pending_frame_count() 查看。

离线`project1_feedback_export`对本轮已观察到的有序反馈采用
`PiperFeedbackAssembler(0.05, true)`：等待2A2，按2A2～2A7收齐一轮。
开头半轮不与下一轮拼合；缺帧/乱序后重新等待起帧。
`unaligned_frames`记录未对齐时忽略的帧，`cycle_resets`记录中断的未完成组。
50ms仍是组装上限，2ms是质量报告线，均不证明硬件采样同步。
其他调用者默认仍使用原来的无序收齐模式，未改变真机控制逻辑。
clear_diagnostics() 只清计数，reset() 只清待组装帧并记录非空组人工丢弃。
非法目标帧计数后抛 invalid_argument，保留旧待组装帧，由调用者决定是否reset。

时间倒退诊断只覆盖正在组装的组，不是整个日志的全局时间顺序检查。
日志解析失败发生在 push 之前，所以仍由主程序的 invalid/unsupported 计数报告。
project1 在 push 前过滤其他接口/ID，主程序 ignored 与组装器 ignored_frames 含义不同。
这些规则用于离线数据检查，不提供物理急停或设备采样同步。
测试见 tests/unit/test_assembler_diagnostics.cpp。
