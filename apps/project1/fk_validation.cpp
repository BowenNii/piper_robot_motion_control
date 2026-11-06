#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "hal/can_log_parser.hpp"
#include "experiment_io/fk_csv_writer.hpp"
#include "piper_control/common/error_statistics.hpp"
#include "piper_control/validation/pose_error.hpp"

#include "piper_control/common/constants.hpp"
#include "piper_control/common/types.hpp"
#include "piper_control/kinematics/forward_kinematics.hpp"
#include "piper_control/lie_group/se3.hpp"
#include "piper_control/lie_group/so3.hpp"
#include "piper_control/robot_model/piper_model.hpp"

/*
实验1（单位形）和实验2（多位形）共用同一套离线 FK 验证逻辑。

单个位形离线复核（只写新输出）：
/home/nbw/piper_robot_motion_control/build/debug/fk_validation
"/home/nbw/piper_robot_motion_control/experiments/project1/2026-10-03_Multibody_static _fk/piper_can_log/pose01.log"
"/tmp/piper_pose01_recheck.csv"

实验2：
/home/nbw/piper_robot_motion_control/build/debug/fk_validation --multi
"/home/nbw/piper_robot_motion_control/experiments/project1/2026-10-03_Multibody_static _fk/piper_can_log"
"/tmp/piper_multibody_recheck"

注意：
- 输出目录必须提前建立。
- 已有同名 CSV 时拒绝覆盖，重新验证请使用新输出路径。
- 本程序仅读取日志，不发送 CAN 指令，也不控制真机。
*/
int validate_log(const std::string& path, const std::string& csv_path)
// 函数级 try/catch：把文件或验证模块抛出的异常转换为终端提示和退出码1。
try
{
    using namespace piper_control;

    std::ifstream file(path);
    if (!file)
    {
        std::cerr << "Cannot open log: " << path << '\n';
        return 1;
    }

    // 不覆盖已有文件，避免误覆盖原始日志或之前的实验结果。
    std::error_code path_error;
    const bool csv_exists = std::filesystem::exists(csv_path, path_error);
    if (path_error || csv_exists)
    {
        std::cerr << "CSV path unavailable or already exists: " << csv_path
                  << "\nPlease choose a new output filename.\n";
        return 1;
    }
    // CSV 格式和独占创建交给工具；主文件只保留实验处理顺序。
    experiment_io::FkCsvWriter csv(csv_path);
    double first_timestamp = 0.0;
    double previous_timestamp = 0.0;

    const PiperModel model = make_piper_model();
    // 按 0x2A2 -> 0x2A7 顺序收齐，避免跨轮拼帧；跨度仍须不超过 2 ms。
    hal::PiperFeedbackAssembler assembler(0.002, true);
    std::size_t lines = 0, parsed_count = 0, invalid_count = 0;
    std::size_t ignored_count = 0, sample_count = 0;
    ErrorAccumulator position_stats, rotation_stats; // 内部使用 m、rad。
    double max_span_ms = 0.0;
    std::string line;
    std::cout << std::fixed << std::setprecision(9);

    // 第一步：逐行读文本，再解析为带时间戳的 CanFrame。
    while (std::getline(file, line))
    {
        ++lines;
        hal::TimedCanFrame timed;
        if (!hal::parse_can_log_line(line, timed))
        {
            ++invalid_count;
            assembler.reset();
            continue;
        }
        ++parsed_count;
        // 第二步：只用 can0 的末端位姿和六关节角反馈。
        if (timed.interface_name != "can0" || timed.frame.id < 0x2A2
            || timed.frame.id > 0x2A7)
        {
            ++ignored_count;
            continue;
        }
        if (timed.frame.length != 8)
        {
            ++invalid_count;
            assembler.reset();
            continue;
        }
        const auto sample = assembler.push(timed);
        if (!sample)
        {
            continue;
        }
        // 组帧器每轮会清空时间状态，因此在这里另行检查跨组时间。
        // 重复时间也拒绝，避免生成不能用于差分/时间对齐的 CSV。
        if (sample_count > 0 && sample->timestamp_seconds <= previous_timestamp)
        {
            throw std::runtime_error("Completed feedback timestamps must be strictly increasing");
        }

        // 第三步：将已经解码的关节角送入 POE 正运动学。
        VecXd q(6);
        for (Eigen::Index i = 0; i < q.size(); ++i)
        {
            q(i) = sample->q_rad[static_cast<std::size_t>(i)];
        }
        const Mat4 T_poe = forward_poe_space(model.S_list, q, model.M);

        // 第四步：根据 CAN 反馈的 XYZ 和 RPY 构造位姿矩阵。
        const auto& p = sample->end_pose.position_m;
        const auto& rpy = sample->end_pose.rpy_rad;
        const Mat4 T_feedback = se3_from_rt(
            rot_z(rpy[2]) * rot_y(rpy[1]) * rot_x(rpy[0]), Vec3(p[0], p[1], p[2]));

        // 第五步：通用位姿误差计算，然后保存一组结构化结果。
        // t=0 对应首组最后一帧；跨度只代表接收配对，不保证同步采样。
        const PoseError error = pose_error(T_poe, T_feedback);
        const double position_mm = error.position_m * 1000.0;
        const double rotation_deg = error.rotation_rad * 180.0 / kPi;
        if (sample_count == 0)
            first_timestamp = sample->timestamp_seconds;
        experiment_io::FkValidationResult result;
        result.t_s = sample->timestamp_seconds - first_timestamp;
        result.timestamp_s = sample->timestamp_seconds;
        result.frame_span_s = sample->span_seconds;
        result.q_rad = q;
        result.T_feedback = T_feedback;
        result.feedback_rpy_rad = rpy;
        result.T_poe = T_poe;
        result.error = error;
        csv.write(result);
        previous_timestamp = sample->timestamp_seconds;
        // 只打印首组矩阵，避免数百组重复输出影响阅读。
        if (sample_count == 0)
        {
            std::cout << "First sample timestamp (s): " << sample->timestamp_seconds
                      << "\nDecoded q (rad): " << q.transpose()
                      << "\n\nT_POE:\n" << T_poe
                      << "\n\nT_feedback:\nhal::PiperFeedbackAssembler assembler(0.002, true);" << T_feedback
                      << "\n\nPosition error: " << position_mm << " mm"
                      << "\nRotation error: " << rotation_deg << " deg\n\n";
        }
        ++sample_count;
        position_stats.add(error.position_m);
        rotation_stats.add(error.rotation_rad);
        max_span_ms = std::max(max_span_ms, sample->span_seconds * 1000.0);
    }
    if (file.bad())
    {
        std::cerr << "Error reading log: " << path << '\n';
        return 1;
    }
    std::cout << "Log: " << path
              << "\nLines: " << lines << ", parsed: " << parsed_count
              << ", invalid/unsupported: " << invalid_count
              << ", ignored: " << ignored_count
              << "\nValid feedback samples: " << sample_count
              << "\nPending frames at EOF: " << assembler.pending_frame_count() << '\n';
    if (sample_count == 0)
    {
        std::cerr << "No complete feedback samples within the 2 ms window.\n";
        return 1;
    }
    csv.close(); // 显式检查延迟写入/关闭错误后，才报告成功。
    const auto p_stats = position_stats.statistics();
    const auto r_stats = rotation_stats.statistics();
    const auto& diagnostic = assembler.diagnostics();
    std::cout << "Assembler discarded groups / frames: "
              << diagnostic.discarded_groups << " / " << diagnostic.discarded_frames
              << "\nRestart reasons duplicate / window / reversal / interface / manual: "
              << diagnostic.duplicate_ids << " / " << diagnostic.over_window
              << " / " << diagnostic.time_reversals << " / " << diagnostic.interface_changes
              << " / " << diagnostic.manual_resets
              << "\nOrdered cycle resets / unaligned frames: "
              << diagnostic.cycle_resets << " / " << diagnostic.unaligned_frames << '\n';
    std::cout << "CSV saved: " << csv_path << " (" << sample_count << " rows)\n";
    std::cout << "Position error mean / RMS / max (mm): "
              << p_stats.mean * 1000.0 << " / " << p_stats.rms * 1000.0
              << " / " << p_stats.max * 1000.0
              << "\nRotation error mean / RMS / max (deg): "
              << r_stats.mean * 180.0 / kPi << " / " << r_stats.rms * 180.0 / kPi
              << " / " << r_stats.max * 180.0 / kPi
              << "\nMax six-frame time span (ms): " << max_span_ms << '\n';
    return 0;
}

catch (const std::exception& error)
{
    std::cerr << "FK validation failed: " << error.what()
              << "\nOutput may be incomplete: " << csv_path << '\n';
    return 1;
}

int main(int argc, char* argv[])
{
    // 实验2：一次处理十个位形。输出目录必须事先建立；推荐新建目录，
    // 例如 data_csv_recheck，以保留原实验 CSV，不改变原始归档结果。
    if (argc >= 2 && std::string(argv[1]) == "--multi")
    {
        if (argc != 4)
        {
            std::cerr << "Usage: fk_validation --multi <log-directory> <csv-directory>\n";
            return 1;
        }
        const std::filesystem::path log_dir(argv[2]);
        const std::filesystem::path csv_dir(argv[3]);
        std::error_code directory_error;
        if (!std::filesystem::is_directory(csv_dir, directory_error) || directory_error)
        {
            std::cerr << "CSV directory must already exist: " << csv_dir << '\n';
            return 1;
        }
        std::array<std::string, 10> logs;
        std::array<std::string, 10> outputs;
        // 先检查全部输入和输出，再开始计算，避免已有 CSV 被覆盖。
        for (std::size_t i = 0; i < logs.size(); ++i)
        {
            const std::string name = "pose" + std::string(i < 9 ? "0" : "")
                + std::to_string(i + 1);
            logs[i] = (log_dir / (name + ".log")).string();
            outputs[i] = (csv_dir / (name + ".csv")).string();
            std::ifstream input(logs[i]);
            std::error_code output_error;
            const bool exists = std::filesystem::exists(outputs[i], output_error);
            if (!input || exists || output_error)
            {
                std::cerr << "Missing/unreadable log or unavailable/existing CSV: "
                          << logs[i] << " -> " << outputs[i] << '\n';
                return 1;
            }
        }
        for (std::size_t i = 0; i < logs.size(); ++i)
        {
            std::cout << "\n=== Experiment 2: pose " << i + 1 << "/10 ===\n";
            if (validate_log(logs[i], outputs[i]) != 0)
            {
                std::cerr << "Batch stopped; earlier CSV files may already be saved.\n";
                return 1;
            }
        }
        std::cout << "\nValidated all 10 static poses.\n";
        return 0;
    }

    // 实验1：单日志模式；实验2也可以逐个位形调用此模式。
    // 默认输出为原日志路径加 .csv，可用第二个参数指定其他输出路径。
    if (argc > 3)
    {
        std::cerr << "Usage: fk_validation [candump-log-path] [output-csv-path]\n"
                  << "       fk_validation --multi <log-directory> <csv-directory>\n";
        return 1;
    }
    const std::string path = argc >= 2 ? argv[1]
        : "/home/nbw/piper_can_static_2026-10-01.log";
    const std::string csv_path = argc == 3 ? argv[2] : path + ".csv";
    return validate_log(path, csv_path);
}
