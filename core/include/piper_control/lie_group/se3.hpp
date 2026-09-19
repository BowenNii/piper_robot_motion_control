#pragma once

#include "piper_control/common/types.hpp"

namespace piper_control
{

/*【SE3-01】
 * @brief 将 6×1 twist 转换为 4×4 se(3) 矩阵。
 *
 * @param xi：6×1 twist，前3维为旋转部分，后3维为平移部分。
 *
 * @return：4×4 se(3) 矩阵。
 */
Mat4 twist_hat(const Twist& xi);

/*【SE3-02】
 * @brief 根据旋转矩阵和平移向量构造 SE(3) 齐次变换矩阵。
 *
 * @param R：3×3 SO(3) 旋转矩阵。
 * @param p：3×1 平移向量。
 *
 * @return：4×4 SE(3) 齐次变换矩阵。
 */
Mat4 se3_from_rt(const Mat3& R, const Vec3& p);

/*【SE3-03】
 * @brief 计算 SE(3) 齐次变换矩阵的解析逆。
 *
 * @param T：4×4 SE(3) 齐次变换矩阵。
 *
 * @return：4×4 逆齐次变换矩阵。
 */
Mat4 se3_inverse(const Mat4& T);

/*【SE3-04】
 * @brief 计算 SE(3) 变换的伴随矩阵。
 *
 * @param T：4×4 SE(3) 齐次变换矩阵。
 *
 * @return：6×6 Adjoint 矩阵。
 */
Mat6 adjoint(const Mat4& T);

/*【SE3-05】
 * @brief 计算 SE(3) 伴随矩阵的逆。
 *
 * @param T：4×4 SE(3) 齐次变换矩阵。
 *
 * @return：6×6 逆 Adjoint 矩阵。
 */
Mat6 adjoint_inverse(const Mat4& T);

/*【SE3-06】
 * @brief 计算 twist 的 little adjoint 矩阵。
 *
 * @param V：6×1 twist，前3维为旋转部分，后3维为平移部分。
 *
 * @return：6×6 little adjoint 矩阵。
 */
Mat6 little_ad(const Twist& V);

/*【SE3-07】
 * @brief 计算 SE(3) 指数映射。
 *
 * @param xi：6×1 twist coordinate。
 *
 * @return：4×4 SE(3) 齐次变换矩阵。
 */
Mat4 se3_exp(const Twist& xi);

/*【SE3-08】
 * @brief 计算 SE(3) 对数映射。
 *
 * @param T：4×4 SE(3) 齐次变换矩阵。
 *
 * @return：6×1 twist coordinate。
 */
Twist se3_log(const Mat4& T);

}  // namespace piper_control