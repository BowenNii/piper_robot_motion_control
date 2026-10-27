#pragma once
#include "piper_control/common/types.hpp"

namespace piper_control {
// 【VAL-01】任意矩阵与容差 ->
// SE(3)结构报告。格式/非有限值返回ok=false；非法容差抛异常。
struct Se3ValidationReport {
  bool ok = false;
  bool shape_valid = false;
  bool finite = false;
  double orthogonality_error = 0.0;
  double determinant = 0.0;
  double last_row_error = 0.0;
};
[[nodiscard]] Se3ValidationReport check_se3(const MatXd &T,
                                            double tolerance = 1e-8);

struct PoseError {
  double position_m = 0.0;
  double rotation_rad = 0.0;
  Vec3 delta_p_m = Vec3::Zero(); // actual位置-reference位置
};
// 【VAL-02】同一基坐标系/末端参考点的两个4×4位姿 ->
// 距离m、主值转角rad、有符号位置差。
// 输入必须为有效SE(3)。不直接相减RPY；不表示外部测量定位精度。
[[nodiscard]] PoseError pose_error(const Mat4 &actual, const Mat4 &reference);
} // namespace piper_control
