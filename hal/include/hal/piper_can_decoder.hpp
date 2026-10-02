#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace hal
{

// 与 Linux can_frame 解耦：用于离线测试，也可由 SocketCAN 接收层填入。
// 本模块只接收协议中的标准 CAN 数据帧，不负责发送或读取 CAN 总线。
struct CanFrame
{
    std::uint32_t id = 0;
    std::array<std::uint8_t, 8> data{};
    std::uint8_t length = 8;
};

struct PiperStatus
{
    std::uint8_t control_mode = 0;
    std::uint8_t arm_status = 0;
    std::uint8_t motion_mode = 0;
    std::uint8_t teach_status = 0;
    std::uint8_t motion_status = 0;
    std::uint8_t trajectory_point = 0;
    std::uint16_t fault_code = 0;
};

struct JointAnglePair
{
    std::size_t first_joint = 0;  // 从 0 开始：0、2、4 分别对应 J1、J3、J5。
    std::array<double, 2> radians{};
};

struct EndPosePair
{
    // 六个分量依次为 X、Y、Z、RX、RY、RZ；索引为 0、2 或 4。
    std::size_t first_component = 0;
    std::array<double, 2> values{};  // 位置分量 m，姿态分量 rad。
};

struct EndPose
{
    std::array<double, 3> position_m{};
    std::array<double, 3> rpy_rad{};
    // 构造旋转矩阵时使用 R = Rz(RZ) * Ry(RY) * Rx(RX)。
};

// 【CAN-01】4 字节大端补码 -> int32。
// u = b0*2^24 + b1*2^16 + b2*2^8 + b3；
// 当 u >= 2^31 时，带符号值 = u - 2^32，否则为 u。
// offset 为 data 内起始字节（允许 0..4），越界抛出 std::invalid_argument。
[[nodiscard]] std::int32_t decode_int32_be(
    const std::array<std::uint8_t, 8>& data, std::size_t offset);

// 【CAN-02】0x2A1 的 8 字节 -> 状态字段；Byte6/7 为大端故障码。
// 保留枚举值与故障码原值，不把“待机”或“无故障”当作“电机已使能”。
[[nodiscard]] PiperStatus decode_status(const CanFrame& frame);

// 【CAN-03】0x2A5/6/7 -> 两个关节角（rad）。
// 前 4 字节与后 4 字节分别表示两个角：rad = int32 * 0.001 * pi/180。
[[nodiscard]] JointAnglePair decode_joint_angle_pair(const CanFrame& frame);

// 【CAN-04】三帧 -> 按 J1..J6 排列的关节角（rad），支持输入帧乱序。
// 必须各有一帧 0x2A5、0x2A6、0x2A7；重复 ID 或错误长度抛出异常。
[[nodiscard]] std::array<double, 6> decode_joint_angles(
    const std::array<CanFrame, 3>& frames);

// 【CAN-05】0x2A2/3/4 -> 两个末端分量。
// 0x2A2: X/Y；0x2A3: Z/RX；0x2A4: RY/RZ。
// 位置 m = int32 * 1e-6；角度 rad = int32 * 0.001 * pi/180。
[[nodiscard]] EndPosePair decode_end_pose_pair(const CanFrame& frame);

// 【CAN-06】三帧 -> 完整位置与 RPY，支持乱序，要求三个 ID 各出现一次。
// 此处只组装数据，不证明帧同步；动态采样时接收层还需检查时间戳和新鲜度。
[[nodiscard]] EndPose decode_end_pose(const std::array<CanFrame, 3>& frames);

}  // namespace hal
