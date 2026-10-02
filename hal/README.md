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

目前 Project 1 用代码中保存的真实静态字节样本演示解码和 POE 对比，
尚不支持读日志文件或直接连接 CAN。厂家末端反馈不是独立的外部测量真值。

测试覆盖真实手算样本、补码边界、乱序组装、非法输入及固定种子随机整数。
测试通过 `all_pass` 和非零退出码报告失败，在 Release 中也会执行。

```bash
cmake --preset debug
cmake --build --preset debug-build
ctest --preset debug-test --output-on-failure
./build/debug/project1
```
