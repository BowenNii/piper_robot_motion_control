#include "hal/piper_can_decoder.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>

namespace
{
constexpr double kPi = 3.14159265358979323846;

bool expect(bool condition, const std::string& name)
{
    std::cout << (condition ? "[PASS] " : "[FAIL] ") << name << '\n';
    return condition;
}

template <std::size_t N>
bool close(const std::array<double, N>& actual, const std::array<double, N>& expected)
{
    for (std::size_t i = 0; i < N; ++i)
    {
        if (!std::isfinite(actual[i]) || std::abs(actual[i] - expected[i]) > 1e-12)
        {
            return false;
        }
    }
    return true;
}

template <typename Function>
bool rejects(Function function)
{
    try
    {
        function();
    }
    catch (const std::invalid_argument&)
    {
        return true;
    }
    catch (...)
    {
        return false;
    }
    return false;
}

// 只用于构造测试报文；signed -> unsigned 按模 2^32 转换。
std::array<std::uint8_t, 8> bytes(std::int32_t a, std::int32_t b)
{
    std::array<std::uint8_t, 8> result{};
    const std::array<std::int32_t, 2> values{a, b};
    for (std::size_t i = 0; i < 2; ++i)
    {
        const auto value = static_cast<std::uint32_t>(values[i]);
        for (std::size_t j = 0; j < 4; ++j)
        {
            result[i * 4 + j] = static_cast<std::uint8_t>(value >> (24 - j * 8));
        }
    }
    return result;
}
}  // namespace

int main()
{
    using namespace hal;
    bool all_pass = true;

    // 【CAN-01】人为构造字节顺序、负数和 int32 边界。
    const std::array<std::uint8_t, 8> endian{0x01, 0x02, 0x03, 0x04,
                                           0xFF, 0xFD, 0x74, 0x0B};
    all_pass &= expect(decode_int32_be(endian, 0) == 16909060,
                       "大端字节顺序 0x01020304");
    all_pass &= expect(decode_int32_be(endian, 4) == -166901,
                       "真实负角度补码 0xFFFD740B");
    const std::array<std::uint8_t, 8> extremes{0x80, 0, 0, 0, 0x7F, 0xFF, 0xFF, 0xFF};
    all_pass &= expect(decode_int32_be(extremes, 0) == std::numeric_limits<std::int32_t>::min()
                       && decode_int32_be(extremes, 4) == std::numeric_limits<std::int32_t>::max(),
                       "int32 最小值和最大值");
    all_pass &= expect(decode_int32_be(bytes(0, -1), 0) == 0
                       && decode_int32_be(bytes(0, -1), 4) == -1,
                       "零值与 -1");

    // 【CAN-02】状态与故障码保留原值。
    const PiperStatus idle = decode_status({0x2A1, {0, 0, 0, 0, 0, 0, 0, 0}});
    all_pass &= expect(idle.control_mode == 0 && idle.arm_status == 0
                       && idle.fault_code == 0, "真实待机状态报文");
    const PiperStatus status = decode_status({0x2A1, {1, 5, 2, 3, 1, 42, 0x12, 0x34}});
    all_pass &= expect(status.control_mode == 1 && status.arm_status == 5
                       && status.motion_mode == 2 && status.teach_status == 3
                       && status.motion_status == 1 && status.trajectory_point == 42
                       && status.fault_code == 0x1234, "状态字段与大端故障码");

    // 【CAN-03/04】真实报文；expected 来自手工解码，不由被测函数生成。
    std::array<CanFrame, 3> joints{{
        {0x2A5, {0, 0, 0, 0, 0xFF, 0xFF, 0xF8, 0x31}},
        {0x2A6, {0, 0, 0x07, 0xB3, 0xFF, 0xFF, 0xFA, 0x7C}},
        {0x2A7, {0, 0, 0x56, 0x5E, 0xFF, 0xFF, 0xF5, 0xD0}}
    }};
    std::array<double, 6> expected_q{0, -1.999, 1.971, -1.412, 22.110, -2.608};
    for (double& angle : expected_q)
    {
        angle *= kPi / 180.0;
    }
    all_pass &= expect(close(decode_joint_angles(joints), expected_q),
                       "真实六关节角，单位 rad");
    const auto joint_pair = decode_joint_angle_pair(joints[1]);
    all_pass &= expect(joint_pair.first_joint == 2
                       && close(joint_pair.radians, std::array<double, 2>{expected_q[2], expected_q[3]}),
                       "单帧 J3/J4 索引与角度");

    // 【CAN-05/06】0x2A3 的 Z 与 RX 使用不同单位比例。
    std::array<CanFrame, 3> poses{{
        {0x2A2, {0, 0, 0xC8, 0x3E, 0xFF, 0xFF, 0xFC, 0xB4}},
        {0x2A3, {0, 0x02, 0x93, 0x9C, 0xFF, 0xFD, 0x73, 0xD3}},
        {0x2A4, {0, 0x01, 0x1B, 0x0A, 0xFF, 0xFD, 0x74, 0x0B}}
    }};
    std::array<double, 3> expected_rpy{-166.957, 72.458, -166.901};
    for (double& angle : expected_rpy)
    {
        angle *= kPi / 180.0;
    }
    const auto pose = decode_end_pose(poses);
    all_pass &= expect(close(pose.position_m, std::array<double, 3>{0.051262, -0.000844, 0.168860})
                       && close(pose.rpy_rad, expected_rpy), "真实末端位置 m 与 RPY rad");
    const auto mixed = decode_end_pose_pair(poses[1]);
    all_pass &= expect(mixed.first_component == 2
                       && close(mixed.values, std::array<double, 2>{0.168860, expected_rpy[0]}),
                       "0x2A3 混合位置和角度单位");

    // 三帧的全部六种排列都应还原到相同的物理分量顺序。
    std::array<int, 3> order{0, 1, 2};
    bool order_pass = true;
    do
    {
        order_pass &= close(decode_joint_angles({joints[order[0]], joints[order[1]], joints[order[2]]}), expected_q);
        const auto reordered = decode_end_pose({poses[order[0]], poses[order[1]], poses[order[2]]});
        order_pass &= close(reordered.position_m, pose.position_m) && close(reordered.rpy_rad, expected_rpy);
    } while (std::next_permutation(order.begin(), order.end()));
    all_pass &= expect(order_pass, "关节和末端三帧的六种排列");

    // 不依赖 assert，Release 中这些判断也会执行并影响退出码。
    all_pass &= expect(rejects([&] { (void)decode_int32_be(endian, 5); })
                       && rejects([&] { (void)decode_int32_be(endian, std::numeric_limits<std::size_t>::max()); }),
                       "拒绝越界字节偏移");
    bool length_pass = true;
    for (const std::uint8_t length : {0, 7, 9, 255})
    {
        length_pass &= rejects([&] { (void)decode_status({0x2A1, {}, length}); });
        length_pass &= rejects([&] { (void)decode_joint_angle_pair({0x2A5, {}, length}); });
        length_pass &= rejects([&] { (void)decode_end_pose_pair({0x2A2, {}, length}); });
    }
    all_pass &= expect(length_pass, "拒绝不为 8 的报文长度");
    all_pass &= expect(rejects([] { (void)decode_status({0x2A5, {}}); })
                       && rejects([] { (void)decode_joint_angle_pair({0x2A4, {}}); })
                       && rejects([] { (void)decode_joint_angle_pair({0x800002A5u, {}}); })
                       && rejects([] { (void)decode_end_pose_pair({0x2A5, {}}); }),
                       "拒绝错误 ID 和带 CAN 标志的原始 ID");
    all_pass &= expect(rejects([&] { (void)decode_joint_angles({joints[0], joints[0], joints[2]}); })
                       && rejects([&] { (void)decode_end_pose({poses[0], poses[1], poses[1]}); }),
                       "拒绝重复 ID，不能将缺少的分量当成零");

    // 固定种子覆盖整个 int32 范围，测试任意正负数的字节往返。
    std::mt19937 rng(42);
    std::uniform_int_distribution<std::int32_t> distribution(
        std::numeric_limits<std::int32_t>::min(), std::numeric_limits<std::int32_t>::max());
    bool random_pass = true;
    for (int sample = 0; sample < 1000; ++sample)
    {
        const auto a = distribution(rng);
        const auto b = distribution(rng);
        const auto encoded = bytes(a, b);
        random_pass &= decode_int32_be(encoded, 0) == a && decode_int32_be(encoded, 4) == b;
    }
    all_pass &= expect(random_pass, "固定种子 1000 组 int32 补码往返");

    std::cout << (all_pass ? "All PiPER CAN decoder tests passed.\n" : "PiPER CAN decoder tests failed.\n");
    return all_pass ? 0 : 1;
}
