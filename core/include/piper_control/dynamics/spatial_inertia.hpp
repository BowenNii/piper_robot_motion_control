#pragma once

#include "piper_control/common/types.hpp"

namespace piper_control
{

/*【DYN-01】
 * @brief 根据刚体质量、质心惯量和质心位置构造空间惯量矩阵。
 *
 * @param m：刚体质量。
 * @param I_body：3×3 质心坐标系下的转动惯量矩阵。
 * @param r：3×1 质心相对当前连杆坐标系的位置。
 *
 * @return：6×6 空间惯量矩阵。
 */
Mat6 spatial_inertia(
    double m,
    const Mat3& I_body,
    const Vec3& r);

}  // namespace piper_control