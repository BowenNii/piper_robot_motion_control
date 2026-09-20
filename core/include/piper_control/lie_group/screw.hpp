#ifndef PIPER_CONTROL_LIE_GROUP_SCREW_HPP
#define PIPER_CONTROL_LIE_GROUP_SCREW_HPP

#include "piper_control/common/types.hpp"

namespace piper_control
{

// ============================================================
// Screw Axis
//
// 本模块用于构造机器人关节的 Screw Axis，并计算 Screw
// Axis 的指数映射。
//
// Twist 统一采用：
//
//     S = [ omega ]
//         [   v   ]
//
// 其中：
//     omega : 角速度部分
//     v     : 线速度部分
//
// 对于旋转关节：
//
//     S = [ omega       ]
//         [ -omega × q ]
//
// 其中：
//     omega : 单位旋转轴方向
//     q     : 旋转轴上一点
//
// 对于移动关节：
//
//     S = [ 0       ]
//         [ v       ]
//
// 其中 v 为单位移动方向。
//
// Screw Axis 通常在指定参考坐标系中表达。
// 本模块不强制指定具体坐标系，调用者必须保证输入的
// omega / point / direction 位于同一个坐标系中。
//
// Screw Axis 的指数映射：
//
//     T(theta) = exp([S] theta)
//
// 其中：
//     [S]    : Screw Axis 对应的 se(3) 矩阵
//     theta  : 关节变量
//     T      : SE(3) 运动变换
//
// 对旋转关节，theta 通常为弧度；
// 对移动关节，theta 通常为米。
// ============================================================


// ============================================================
// 【SCREW-01】 make_revolute_screw_axis
//
// @brief
// 根据旋转轴方向和轴上一点构造旋转关节 Screw Axis。
//
// 数学定义：
//
//     S = [ omega ]
//         [   v   ]
//
//     v = -omega × q
//
// 其中：
//     omega : 单位旋转轴方向
//     q     : 旋转轴上一点
//
// 输入：
//     omega : 旋转轴方向，可以不是单位向量
//     point : 旋转轴上一点，和 omega 位于同一坐标系
//
// 输出：
//     S     : 6×1 单位旋转关节 Screw Axis
//
// 归一化：
//
//     omega_hat = omega / ||omega||
//
// 实际构造：
//
//     v = -omega_hat × point
//
// 使用方法：
//     给定机器人某个旋转关节的旋转轴方向 omega
//     和轴上一点 point，即可构造 POE 所需的 Screw Axis。
//
// 注意：
//     omega 不能为零向量。
//     omega 与 point 必须使用相同坐标系。
//     theta 对旋转关节通常使用弧度。
// ============================================================

[[nodiscard]] Twist make_revolute_screw_axis(
    const Vec3& omega,
    const Vec3& point);


// ============================================================
// 【SCREW-02】 make_prismatic_screw_axis
//
// @brief
// 根据移动方向构造移动关节 Screw Axis。
//
// 数学定义：
//
//     S = [ 0 ]
//         [ v ]
//
// 其中：
//
//     ||v|| = 1
//
// 输入：
//     direction : 移动方向，可以不是单位向量
//
// 输出：
//     S         : 6×1 单位移动关节 Screw Axis
//
// 归一化：
//
//     v = direction / ||direction||
//
// 使用方法：
//     给定移动关节的运动方向 direction，
//     构造 POE 所需的 Screw Axis。
//
// 注意：
//     direction 不能为零向量。
//     theta 对移动关节通常使用米。
// ============================================================

[[nodiscard]] Twist make_prismatic_screw_axis(
    const Vec3& direction);


// ============================================================
// 【SCREW-03】 screw_exp
//
// @brief
// 计算 Screw Axis 的指数映射。
//
// 数学定义：
//
//     T(theta) = exp([S] theta)
//
// 其中：
//     S     : 6×1 Screw Axis
//     theta : 关节变量
//
// 等价于：
//
//     T(theta) = exp(([S]theta)^)
//
// 并通过 SE(3) 指数映射计算。
//
// 对旋转关节：
//
//     theta = 关节转角（rad）
//
// 对移动关节：
//
//     theta = 关节位移（m）
//
// 输入：
//     S     : 6×1 Screw Axis
//     theta : 关节变量
//
// 输出：
//     T     : 4×4 SE(3) 齐次变换矩阵
//
// 注意：
//     S 与 theta 必须具有一致的物理定义。
//     对于单位 Screw Axis，theta 具有直接的关节变量物理意义。
// ============================================================

[[nodiscard]] Mat4 screw_exp(
    const Twist& S,
    double theta);

} // namespace piper_control

#endif // PIPER_CONTROL_LIE_GROUP_SCREW_HPP