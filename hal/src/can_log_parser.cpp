#include "hal/can_log_parser.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace hal
{
namespace
{
bool is_hex(const std::string& text)
{
    return !text.empty()
        && text.find_first_not_of("0123456789abcdefABCDEF") == std::string::npos;
}
}  // namespace

bool parse_can_log_line(const std::string& line, TimedCanFrame& result)
{
    std::istringstream input(line);
    std::string timestamp_text, interface_name, message, extra;
    if (!(input >> timestamp_text >> interface_name >> message) || (input >> extra))
    {
        return false;
    }
    if (timestamp_text.size() < 3 || timestamp_text.front() != '('
        || timestamp_text.back() != ')')
    {
        return false;
    }

    const std::string seconds_text = timestamp_text.substr(1, timestamp_text.size() - 2);
    if (seconds_text.find_first_not_of("0123456789.") != std::string::npos)
    {
        return false;
    }
    const std::size_t separator = message.find('#');
    if (separator == std::string::npos)
    {
        return false;
    }
    const std::string id_text = message.substr(0, separator);
    const std::string data_text = message.substr(separator + 1);
    if (!is_hex(id_text) || id_text.size() > 3
        || (!data_text.empty() && !is_hex(data_text))
        || data_text.size() > 16 || data_text.size() % 2 != 0)
    {
        return false;
    }

    TimedCanFrame parsed;
    parsed.interface_name = interface_name;
    try
    {
        std::size_t consumed = 0;
        parsed.timestamp_seconds = std::stod(seconds_text, &consumed);
        if (consumed != seconds_text.size() || !std::isfinite(parsed.timestamp_seconds))
        {
            return false;
        }
        const unsigned long id = std::stoul(id_text, nullptr, 16);
        if (id > 0x7FF)
        {
            return false;
        }
        parsed.frame.id = static_cast<std::uint32_t>(id);
        parsed.frame.length = static_cast<std::uint8_t>(data_text.size() / 2);
        for (std::size_t i = 0; i < parsed.frame.length; ++i)
        {
            parsed.frame.data[i] = static_cast<std::uint8_t>(
                std::stoul(data_text.substr(i * 2, 2), nullptr, 16));
        }
    }
    catch (const std::invalid_argument&)
    {
        return false;
    }
    catch (const std::out_of_range&)
    {
        return false;
    }
    result = parsed;
    return true;
}

PiperFeedbackAssembler::PiperFeedbackAssembler(double max_span_seconds)
    : max_span_seconds_(max_span_seconds)
{
    if (!std::isfinite(max_span_seconds_) || max_span_seconds_ <= 0.0)
    {
        throw std::invalid_argument("PiPER assembler: time window must be finite and positive");
    }
}

void PiperFeedbackAssembler::reset()
{
    const auto count = pending_frame_count();
    if (count != 0)
    {
        ++diagnostics_.manual_resets;
        ++diagnostics_.discarded_groups;
        diagnostics_.discarded_frames += count;
    }
    clear_pending();
}

void PiperFeedbackAssembler::clear_pending()
{
    seen_.fill(false);
    interface_name_.clear();
    first_timestamp_ = 0.0;
    last_timestamp_ = 0.0;
}

std::size_t PiperFeedbackAssembler::pending_frame_count() const
{
    return static_cast<std::size_t>(std::count(seen_.begin(), seen_.end(), true));
}

std::optional<PiperFeedbackSample> PiperFeedbackAssembler::push(const TimedCanFrame& timed)
{
    ++diagnostics_.received_frames;
    if (timed.frame.id < 0x2A2 || timed.frame.id > 0x2A7)
    {
        ++diagnostics_.ignored_frames;
        return std::nullopt;
    }
    if (timed.frame.length != 8 || timed.interface_name.empty()
        || !std::isfinite(timed.timestamp_seconds) || timed.timestamp_seconds < 0.0)
    {
        ++diagnostics_.invalid_frames;
        throw std::invalid_argument("PiPER assembler: invalid feedback frame");
    }

    const std::size_t slot = timed.frame.id - 0x2A2;
    if (pending_frame_count() != 0
        && (seen_[slot] || timed.interface_name != interface_name_
            || timed.timestamp_seconds < last_timestamp_
            || timed.timestamp_seconds - first_timestamp_ > max_span_seconds_))
    {
        diagnostics_.duplicate_ids += seen_[slot] ? 1 : 0;
        diagnostics_.interface_changes += timed.interface_name != interface_name_ ? 1 : 0;
        diagnostics_.time_reversals += timed.timestamp_seconds < last_timestamp_ ? 1 : 0;
        diagnostics_.over_window += timed.timestamp_seconds - first_timestamp_ > max_span_seconds_ ? 1 : 0;
        ++diagnostics_.discarded_groups;
        diagnostics_.discarded_frames += pending_frame_count();
        clear_pending();
    }
    if (pending_frame_count() == 0)
    {
        first_timestamp_ = timed.timestamp_seconds;
        interface_name_ = timed.interface_name;
    }
    frames_[slot] = timed.frame;
    seen_[slot] = true;
    last_timestamp_ = timed.timestamp_seconds;
    if (pending_frame_count() != 6)
    {
        return std::nullopt;
    }

    PiperFeedbackSample sample;
    sample.timestamp_seconds = last_timestamp_;
    sample.span_seconds = last_timestamp_ - first_timestamp_;
    sample.end_pose = decode_end_pose({frames_[0], frames_[1], frames_[2]});
    sample.q_rad = decode_joint_angles({frames_[3], frames_[4], frames_[5]});
    ++diagnostics_.completed_samples;
    clear_pending();
    return sample;
}

}  // namespace hal
