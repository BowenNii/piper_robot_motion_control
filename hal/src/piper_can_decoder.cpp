#include "hal/piper_can_decoder.hpp"

#include <stdexcept>

namespace hal
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kAngleScale = 0.001 * kPi / 180.0;
constexpr double kPositionScale = 1e-6;

void check_length(const CanFrame& frame)
{
    if (frame.length != 8)
    {
        throw std::invalid_argument("PiPER decoder: frame length must be 8");
    }
}
}  // namespace

std::int32_t decode_int32_be(
    const std::array<std::uint8_t, 8>& data, std::size_t offset)
{
    if (offset > 4)
    {
        throw std::invalid_argument("PiPER decoder: int32 offset out of range");
    }

    // 先转成无符号 32 位数再移位，避免有符号左移溢出。
    const std::uint32_t value =
        (static_cast<std::uint32_t>(data[offset]) << 24) |
        (static_cast<std::uint32_t>(data[offset + 1]) << 16) |
        (static_cast<std::uint32_t>(data[offset + 2]) << 8) |
        static_cast<std::uint32_t>(data[offset + 3]);

    // 用 int64 完成补码转换，避免超范围 unsigned -> signed 转换。
    const std::int64_t signed_value = value >= 0x80000000u
        ? static_cast<std::int64_t>(value) - 4294967296LL
        : static_cast<std::int64_t>(value);
    return static_cast<std::int32_t>(signed_value);
}

PiperStatus decode_status(const CanFrame& frame)
{
    check_length(frame);
    if (frame.id != 0x2A1)
    {
        throw std::invalid_argument("PiPER decoder: expected status ID 0x2A1");
    }
    return {frame.data[0], frame.data[1], frame.data[2], frame.data[3],
            frame.data[4], frame.data[5],
            static_cast<std::uint16_t>(
                (static_cast<std::uint16_t>(frame.data[6]) << 8) | frame.data[7])};
}

JointAnglePair decode_joint_angle_pair(const CanFrame& frame)
{
    check_length(frame);
    if (frame.id < 0x2A5 || frame.id > 0x2A7)
    {
        throw std::invalid_argument("PiPER decoder: expected joint ID 0x2A5..0x2A7");
    }
    return {static_cast<std::size_t>(frame.id - 0x2A5) * 2,
            {decode_int32_be(frame.data, 0) * kAngleScale,
             decode_int32_be(frame.data, 4) * kAngleScale}};
}

std::array<double, 6> decode_joint_angles(const std::array<CanFrame, 3>& frames)
{
    std::array<double, 6> angles{};
    std::array<bool, 3> seen{};
    for (const CanFrame& frame : frames)
    {
        const JointAnglePair pair = decode_joint_angle_pair(frame);
        const std::size_t slot = pair.first_joint / 2;
        if (seen[slot])
        {
            throw std::invalid_argument("PiPER decoder: duplicate joint frame");
        }
        seen[slot] = true;
        angles[pair.first_joint] = pair.radians[0];
        angles[pair.first_joint + 1] = pair.radians[1];
    }
    for (std::size_t i = 0; i < seen.size(); ++i)
    {
        if (!seen[i])
        {
            throw std::invalid_argument("PiPER decoder: missing joint frame");
        }
    }
    return angles;
}

EndPosePair decode_end_pose_pair(const CanFrame& frame)
{
    check_length(frame);
    if (frame.id < 0x2A2 || frame.id > 0x2A4)
    {
        throw std::invalid_argument("PiPER decoder: expected pose ID 0x2A2..0x2A4");
    }

    EndPosePair pair;
    pair.first_component = static_cast<std::size_t>(frame.id - 0x2A2) * 2;
    for (std::size_t i = 0; i < 2; ++i)
    {
        // 0x2A3 同时含位置 Z 与角度 RX，不能对整帧使用同一种缩放。
        const double scale = pair.first_component + i < 3
            ? kPositionScale : kAngleScale;
        pair.values[i] = decode_int32_be(frame.data, i * 4) * scale;
    }
    return pair;
}

EndPose decode_end_pose(const std::array<CanFrame, 3>& frames)
{
    std::array<double, 6> values{};
    std::array<bool, 3> seen{};
    for (const CanFrame& frame : frames)
    {
        const EndPosePair pair = decode_end_pose_pair(frame);
        const std::size_t slot = pair.first_component / 2;
        if (seen[slot])
        {
            throw std::invalid_argument("PiPER decoder: duplicate pose frame");
        }
        seen[slot] = true;
        values[pair.first_component] = pair.values[0];
        values[pair.first_component + 1] = pair.values[1];
    }
    for (std::size_t i = 0; i < seen.size(); ++i)
    {
        if (!seen[i])
        {
            throw std::invalid_argument("PiPER decoder: missing pose frame");
        }
    }
    return {{values[0], values[1], values[2]}, {values[3], values[4], values[5]}};
}

}  // namespace hal
