#pragma once

#include "piper_control/common/types.hpp"

namespace piper_control
{

/*【SO3-01】
 * @brief 根据 3×1 向量构造 3×3 反对称矩阵。
 *
 * @param v：3×1 向量。
 *
 * @return：3×3 反对称矩阵。
 */
Mat3 skew(const Vec3& v);

/*【SO3-02】
 * @brief 从 3×3 反对称矩阵提取 3×1 向量。
 *
 * @param skew_matrix：3×3 反对称矩阵。
 *
 * @return：3×1 向量。
 */
Vec3 vee(const Mat3& skew_matrix);

/*【SO3-03】
 * @brief 计算绕 X 轴的旋转矩阵。
 *
 * @param theta：旋转角度，单位 rad。
 *
 * @return：3×3 SO(3) 旋转矩阵。
 */
Mat3 rot_x(double theta);

/*【SO3-04】
 * @brief 计算绕 Y 轴的旋转矩阵。
 *
 * @param theta：旋转角度，单位 rad。
 *
 * @return：3×3 SO(3) 旋转矩阵。
 */
Mat3 rot_y(double theta);

/*【SO3-05】
 * @brief 计算绕 Z 轴的旋转矩阵。
 *
 * @param theta：旋转角度，单位 rad。
 *
 * @return：3×3 SO(3) 旋转矩阵。
 */
Mat3 rot_z(double theta);

/*【SO3-06】
 * @brief 计算 SO(3) 指数映射。
 *
 * @param rotation_vector：3×1 旋转向量 omega*theta。
 *
 * @return：3×3 SO(3) 旋转矩阵。
 */
Mat3 so3_exp(const Vec3& rotation_vector);

/*【SO3-07】
 * @brief 计算 SO(3) 对数映射。
 *
 * @param R：3×3 SO(3) 旋转矩阵。
 *
 * @return：3×1 旋转向量 omega*theta。
 */
Vec3 so3_log(const Mat3& R);

/*【SO3-08】
 * @brief 根据旋转轴和旋转角构造 SO(3) 旋转矩阵。
 *
 * @param axis：3×1 旋转轴方向。
 * @param theta：旋转角度，单位 rad。
 *
 * @return：3×3 SO(3) 旋转矩阵。
 */
Mat3 rot_axis_angle(const Vec3& axis, double theta);

}  // namespace piper_control