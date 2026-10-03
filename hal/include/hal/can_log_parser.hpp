#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>

#include "hal/piper_can_decoder.hpp"

namespace hal
{

struct TimedCanFrame
{
    double timestamp_seconds = 0.0;
    std::string interface_name;
    CanFrame frame;
};

// 【LOG-01】candump -L 文本行 -> 时间戳、接口名和 CanFrame。
// 例如：(1790861307.566883) can0 2A5#00000000FFFFF831
// 支持标准 CAN 的 0..8 字节数据帧；空行/非法格式返回 false。
// 失败时不修改 result。时间单位为秒，double 精度满足此处毫秒级检查。
[[nodiscard]] bool parse_can_log_line(
    const std::string& line, TimedCanFrame& result);

struct PiperFeedbackSample
{
    double timestamp_seconds = 0.0;  // 六帧中最新的时间戳。
    double span_seconds = 0.0;       // 六帧最新时间 - 最早时间。
    std::array<double, 6> q_rad{};
    EndPose end_pose;
};

// 【LOG-02】收集 0x2A2..0x2A7，组装可用于离线 FK 对比的一组反馈。
// 每输出一组即清空，下一组必须六帧重新到齐。
// 重复 ID、超出时间窗口、时间倒退或接口变化时丢弃未完成的组，
// 再以当前帧开始新组。其他 ID 被忽略。
// max_span_seconds 是配对窗口，不是硬件同步精度保证；默认 2 ms。
class PiperFeedbackAssembler
{
public:
    explicit PiperFeedbackAssembler(double max_span_seconds = 0.002);

    [[nodiscard]] std::optional<PiperFeedbackSample> push(const TimedCanFrame& timed);
    void reset();

    [[nodiscard]] std::size_t pending_frame_count() const;

private:
    double max_span_seconds_;
    double first_timestamp_ = 0.0;
    double last_timestamp_ = 0.0;
    std::string interface_name_;
    std::array<CanFrame, 6> frames_{};
    std::array<bool, 6> seen_{};
};

}  // namespace hal
