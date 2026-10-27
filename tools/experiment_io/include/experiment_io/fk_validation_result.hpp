#pragma once
#include "piper_control/common/types.hpp"
#include "piper_control/validation/pose_error.hpp"
#include <array>

namespace experiment_io {
// 【IO-01】单条PiPER FK验证结果。仅保存数据，不计算FK、不读取CAN。
// 时间s、长度m、角度rad。RPY按roll/pitch/yaw；原始RPY保留，不从矩阵反解。
struct FkValidationResult {
  double t_s = 0.0;
  double timestamp_s = 0.0;
  double frame_span_s = 0.0;
  piper_control::Vec6 q_rad = piper_control::Vec6::Zero();
  piper_control::Mat4 T_feedback = piper_control::Mat4::Identity();
  std::array<double, 3> feedback_rpy_rad{};
  piper_control::Mat4 T_poe = piper_control::Mat4::Identity();
  piper_control::PoseError error;
};
} // namespace experiment_io
