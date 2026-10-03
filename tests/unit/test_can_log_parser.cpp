#include "hal/can_log_parser.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
bool expect(bool condition, const std::string& name)
{
    std::cout << (condition ? "[PASS] " : "[FAIL] ") << name << '\n';
    return condition;
}

hal::TimedCanFrame make_frame(std::uint32_t id, double time, const std::string& bus = "can0")
{
    return {time, bus, {id, {}}};
}

template <typename Function>
bool rejects(Function function)
{
    try { function(); }
    catch (const std::invalid_argument&) { return true; }
    return false;
}
}  // namespace

int main()
{
    bool all_pass = true;
    hal::TimedCanFrame parsed;
    all_pass &= expect(hal::parse_can_log_line(
        "(1790861307.566883) can0 2A5#00000000FFFFF831", parsed)
        && parsed.timestamp_seconds == 1790861307.566883
        && parsed.interface_name == "can0" && parsed.frame.id == 0x2A5
        && parsed.frame.length == 8
        && parsed.frame.data == std::array<std::uint8_t, 8>{0, 0, 0, 0, 0xFF, 0xFF, 0xF8, 0x31},
        "真实日志行：时间戳、ID、字节数组");
    all_pass &= expect(hal::parse_can_log_line(
        "  (1.25)\tcan0 2a5#00000000fffff831\r", parsed)
        && hal::decode_joint_angle_pair(parsed.frame).radians[1] < 0.0,
        "空白、CRLF、小写十六进制与下游解码");

    all_pass &= expect(hal::parse_can_log_line("(1) can0 212#04010F02", parsed)
        && parsed.frame.length == 4 && parsed.frame.data[3] == 2,
        "日志中的合法 4 字节报文");
    all_pass &= expect(hal::parse_can_log_line("(1) can0 123#", parsed)
        && parsed.frame.length == 0, "合法零字节数据帧");
    const auto before = parsed;
    const std::array<std::string, 16> invalid{{
        "", "^C", "1.0 can0 2A5#0000000000000000",
        "(nan) can0 2A5#0000000000000000",
        "(-1) can0 2A5#0000000000000000",
        "(1.2.3) can0 2A5#0000000000000000",
        "(.) can0 2A5#0000000000000000",
        "(1) 2A5#0000000000000000",
        "(1) can0 2A50000000000000000",
        "(1) can0 2A5#000000000000000Z",
        "(1) can0 2A5#000",
        "(1) can0 2A5#000000000000000000",
        "(1) can0 800#0000000000000000",
        "(1) can0 000002A5#0000000000000000",
        "(1) can0 2A5##0000000000000000",
        "(1) can0 2A5#0000000000000000 extra"
    }};
    bool invalid_pass = true;
    for (const auto& line : invalid)
    {
        invalid_pass &= !hal::parse_can_log_line(line, parsed)
            && parsed.timestamp_seconds == before.timestamp_seconds
            && parsed.interface_name == before.interface_name
            && parsed.frame.id == before.frame.id && parsed.frame.data == before.frame.data;
    }
    all_pass &= expect(invalid_pass, "16 种非法/不支持格式，失败不修改输出");

    hal::PiperFeedbackAssembler assembler;
    const std::array<std::string, 6> real_lines{{
        "(1790861307.566511) can0 2A2#0000C83EFFFFFCB4",
        "(1790861307.566634) can0 2A3#0002939CFFFD73D3",
        "(1790861307.566758) can0 2A4#00011B0AFFFD740B",
        "(1790861307.566883) can0 2A5#00000000FFFFF831",
        "(1790861307.567008) can0 2A6#000007B3FFFFFA7C",
        "(1790861307.567134) can0 2A7#0000565EFFFFF5D0"
    }};
    std::optional<hal::PiperFeedbackSample> sample;
    bool complete_pass = true;
    for (std::size_t i = 0; i < real_lines.size(); ++i)
    {
        complete_pass &= hal::parse_can_log_line(real_lines[i], parsed);
        sample = assembler.push(parsed);
        complete_pass &= sample.has_value() == (i == 5);
    }
    all_pass &= expect(complete_pass && sample
        && std::abs(sample->end_pose.position_m[2] - 0.168860) < 1e-12
        && std::abs(sample->q_rad[4] - 22.110 * 3.14159265358979323846 / 180.0) < 1e-12
        && sample->span_seconds < 0.002 && assembler.pending_frame_count() == 0,
        "六帧真实反馈组装，完成后清空旧帧");

    // 新组只有一帧时不能复用前一组另外五帧。
    all_pass &= expect(!assembler.push(make_frame(0x2A2, 10.0))
        && assembler.pending_frame_count() == 1, "下一组必须六帧重新到齐");
    all_pass &= expect(!assembler.push(make_frame(0x251, 10.0))
        && assembler.pending_frame_count() == 1, "其他报文 ID 不参与组装");
    (void)assembler.push(make_frame(0x2A3, 10.0001));
    (void)assembler.push(make_frame(0x2A2, 10.0002));
    all_pass &= expect(assembler.pending_frame_count() == 1, "重复 ID 丢弃旧组并重新开始");
    (void)assembler.push(make_frame(0x2A3, 10.01));
    all_pass &= expect(assembler.pending_frame_count() == 1, "时间跨度过大：不组装旧反馈");
    (void)assembler.push(make_frame(0x2A4, 9.0));
    all_pass &= expect(assembler.pending_frame_count() == 1, "时间倒退：丢弃未完成组");
    (void)assembler.push(make_frame(0x2A5, 9.0001, "can1"));
    all_pass &= expect(assembler.pending_frame_count() == 1, "不同 CAN 接口不能混合组装");
    assembler.reset();
    all_pass &= expect(assembler.pending_frame_count() == 0, "显式清空缺帧组");

    // ID 可乱序，但时间戳依次递增，组装后分量仍按固定顺序排列。
    bool order_pass = true;
    for (int i = 0; i < 6; ++i)
    {
        sample = assembler.push(make_frame(static_cast<std::uint32_t>(0x2A7 - i), 1.0 + i * 0.0001));
        order_pass &= sample.has_value() == (i == 5);
    }
    all_pass &= expect(order_pass, "六种 ID 逆序到达仍可完成组装");

    all_pass &= expect(rejects([] { hal::PiperFeedbackAssembler bad(0.0); })
        && rejects([] { hal::PiperFeedbackAssembler bad(-0.1); })
        && rejects([] { hal::PiperFeedbackAssembler bad(std::numeric_limits<double>::quiet_NaN()); }),
        "拒绝非法时间窗口");
    auto bad_frame = make_frame(0x2A5, 1.0);
    bad_frame.frame.length = 7;
    all_pass &= expect(rejects([&] { (void)assembler.push(bad_frame); }), "拒绝非法反馈长度");

    std::cout << (all_pass ? "All CAN log parser tests passed.\n" : "CAN log parser tests failed.\n");
    return all_pass ? 0 : 1;
}
