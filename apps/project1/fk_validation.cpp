#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <string>

#include "hal/can_log_parser.hpp"

#include "piper_control/common/constants.hpp"
#include "piper_control/common/types.hpp"
#include "piper_control/kinematics/forward_kinematics.hpp"
#include "piper_control/lie_group/se3.hpp"
#include "piper_control/lie_group/so3.hpp"
#include "piper_control/robot_model/piper_model.hpp"

// 实验1（单位形）和实验2（多位形）共用同一套离线 FK 验证逻辑。
// 本程序仅读取日志，不发送 CAN 指令，也不控制真机。
int validate_log(const std::string& path, const std::string& csv_path)
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
    std::ofstream csv(csv_path);
    if (!csv)
    {
        std::cerr << "Cannot create CSV: " << csv_path << '\n';
        return 1;
    }
    // 固定小数点格式，单位统一为秒、弧度、米，方便 MATLAB readtable。
    csv.imbue(std::locale::classic());
    csv << std::setprecision(17);
    csv << "t_s,timestamp_s,frame_span_s"
        << ",q1_rad,q2_rad,q3_rad,q4_rad,q5_rad,q6_rad"
        << ",feedback_x_m,feedback_y_m,feedback_z_m"
        << ",feedback_roll_rad,feedback_pitch_rad,feedback_yaw_rad"
        << ",poe_x_m,poe_y_m,poe_z_m";
    for (int row = 1; row <= 3; ++row)
        for (int col = 1; col <= 3; ++col)
            csv << ",poe_r" << row << col;
    csv << ",position_error_m,rotation_error_rad\n";
    double first_timestamp = 0.0;

    const PiperModel model = make_piper_model();
    // 六种反馈 ID 必须重新收齐，且时间跨度不超过 2 ms。
    hal::PiperFeedbackAssembler assembler(0.002);
    std::size_t lines = 0, parsed_count = 0, invalid_count = 0;
    std::size_t ignored_count = 0, sample_count = 0;
    double sum_position = 0.0, squared_position = 0.0, max_position = 0.0;
    double sum_rotation = 0.0, squared_rotation = 0.0, max_rotation = 0.0;
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

        // 平移误差取欧氏距离；旋转误差取相对旋转矩阵的转角。
        const double position_mm =
            (T_poe.topRightCorner<3, 1>() - T_feedback.topRightCorner<3, 1>()).norm() * 1000.0;
        const Mat3 R_error = T_poe.topLeftCorner<3, 3>().transpose()
            * T_feedback.topLeftCorner<3, 3>();
        const double rotation_deg = std::acos(std::clamp(
            (R_error.trace() - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / kPi;

        // 第五步：每组完整反馈写一行；t=0 对应第一组的最后一帧。
        // frame_span_s 记录配对时间跨度，不代表六帧硬件同步采样。
        if (sample_count == 0)
            first_timestamp = sample->timestamp_seconds;
        csv << sample->timestamp_seconds - first_timestamp
            << ',' << sample->timestamp_seconds << ',' << sample->span_seconds;
        for (Eigen::Index i = 0; i < q.size(); ++i)
            csv << ',' << q(i);
        for (double value : p)
            csv << ',' << value;
        for (double value : rpy)
            csv << ',' << value;
        for (Eigen::Index i = 0; i < 3; ++i)
            csv << ',' << T_poe(i, 3);
        for (Eigen::Index row = 0; row < 3; ++row)
            for (Eigen::Index col = 0; col < 3; ++col)
                csv << ',' << T_poe(row, col);
        csv << ',' << position_mm / 1000.0
            << ',' << rotation_deg * kPi / 180.0 << '\n';
        if (!csv)
        {
            std::cerr << "Error writing CSV (output may be incomplete): " << csv_path << '\n';
            return 1;
        }

        // 只打印首组矩阵，避免数百组重复输出影响阅读。
        if (sample_count == 0)
        {
            std::cout << "First sample timestamp (s): " << sample->timestamp_seconds
                      << "\nDecoded q (rad): " << q.transpose()
                      << "\n\nT_POE:\n" << T_poe
                      << "\n\nT_feedback:\n" << T_feedback
                      << "\n\nPosition error: " << position_mm << " mm"
                      << "\nRotation error: " << rotation_deg << " deg\n\n";
        }
        ++sample_count;
        sum_position += position_mm;
        squared_position += position_mm * position_mm;
        max_position = std::max(max_position, position_mm);
        sum_rotation += rotation_deg;
        squared_rotation += rotation_deg * rotation_deg;
        max_rotation = std::max(max_rotation, rotation_deg);
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
    csv.close();
    if (!csv)
    {
        std::cerr << "Error closing CSV: " << csv_path << '\n';
        return 1;
    }
    std::cout << "CSV saved: " << csv_path << " (" << sample_count << " rows)\n";
    const double count = static_cast<double>(sample_count);
    std::cout << "Position error mean / RMS / max (mm): "
              << sum_position / count << " / " << std::sqrt(squared_position / count)
              << " / " << max_position
              << "\nRotation error mean / RMS / max (deg): "
              << sum_rotation / count << " / " << std::sqrt(squared_rotation / count)
              << " / " << max_rotation
              << "\nMax six-frame time span (ms): " << max_span_ms << '\n';
    return 0;
}

int main(int argc, char* argv[])
{
    // 实验2：一次处理十个位形。输出目录必须事先建立；推荐新建目录，
    // 例如 data_csv_recheck，以保留原实验 CSV，不改变原始归档结果。
    if (argc >= 2 && std::string(argv[1]) == "--multi")
    {
        if (argc != 4)
        {
            std::cerr << "Usage: project1 --multi <log-directory> <csv-directory>\n";
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
        std::cerr << "Usage: project1 [candump-log-path] [output-csv-path]\n"
                  << "       project1 --multi <log-directory> <csv-directory>\n";
        return 1;
    }
    const std::string path = argc >= 2 ? argv[1]
        : "/home/nbw/piper_can_static_2026-10-01.log";
    const std::string csv_path = argc == 3 ? argv[2] : path + ".csv";
    return validate_log(path, csv_path);
}
