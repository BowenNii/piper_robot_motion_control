#pragma once

#include "piper_control/common/types.hpp"

namespace piper_control {

struct IKOptions {
  int max_iterations = 100;

  double position_tolerance = 1e-6;

  double orientation_tolerance = 1e-6;

  double lambda = 1e-3;

  double max_joint_step = 0.2;

  // D=diag(rotation_weight I3, I3)，平衡rad与m；不是物理常数。
  double rotation_weight = 0.2;
  double svd_tolerance = 1e-10;
  double sigma_threshold = 0.05; // 加权Jacobian的阈值
  double lambda_max = 0.05;
  int max_backtracks = 20;
  // 同时留空表示不限位；否则必须与q同维，初值必须在限位内。
  VecXd q_min;
  VecXd q_max;
};

enum class IKStatus { kConverged, kMaxIterations, kStalled, kNumericalFailure };

struct IKResult {
  bool success = false;

  int iterations = 0;

  double position_error = 0.0;

  double orientation_error = 0.0;

  VecXd q;
  IKStatus status = IKStatus::kMaxIterations;
};

/** 共同约定（全位姿、局部数值IK）：
 * e_b = Log(T(q)^-1 T_target)^vee，排列[旋转;平移]。
 * J_b = Ad_(T^-1) J_s，A=D J_b，b=D e_b。
 * Newton：dq=A^+ b，SVD增益1/sigma（小奇异值截断）。
 * DLS：dq=V diag(sigma/(sigma^2+lambda^2)) U^T b。
 * Adaptive：lambda=lambda_max*sqrt(1-(sigma_min/threshold)^2)，
 *           sigma_min>=threshold时lambda=0。
 * Transpose：g=A^T b，dq=eta*g，eta=||g||²/||A*g||²。
 * 所有方法再做最大关节步长限制、限位投影和误差下降回溯。
 * 终止使用实际位置差(m)和相对旋转角(rad)，不是Log平移分量。
 * 返回失败只表示本次局部搜索失败，不证明目标不可达。
 * 非有限输入、非法SE(3)、维度或配置错误抛std::invalid_argument。
 */

/*【IK-01】
 * @brief 使用 Newton-Raphson 方法求解逆运动学。
 *
 * @param S_list：6×n 空间螺旋轴矩阵。
 * @param M：home configuration 下的末端齐次变换矩阵。
 * @param target：目标末端齐次变换矩阵。
 * @param q_initial：n×1 初始关节位置。
 * @param options：IK 求解参数。
 *
 * @return：IKResult，包含求解状态、迭代次数、误差和关节结果。
 */
[[nodiscard]] IKResult solve_ik_newton(const MatXd &S_list, const Mat4 &M,
                                       const Mat4 &target,
                                       const VecXd &q_initial,
                                       const IKOptions &options = {});

/*【IK-02】
 * @brief 使用阻尼最小二乘法求解逆运动学。
 *
 * @param S_list：6×n 空间螺旋轴矩阵。
 * @param M：home configuration 下的末端齐次变换矩阵。
 * @param target：目标末端齐次变换矩阵。
 * @param q_initial：n×1 初始关节位置。
 * @param options：IK 求解参数。
 *
 * @return：IKResult，包含求解状态、迭代次数、误差和关节结果。
 */
[[nodiscard]] IKResult solve_ik_dls(const MatXd &S_list, const Mat4 &M,
                                    const Mat4 &target, const VecXd &q_initial,
                                    const IKOptions &options = {});

// 【IK-03】根据加权Jacobian最小奇异值调节阻尼。
[[nodiscard]] IKResult solve_ik_adaptive_dls(const MatXd &S_list, const Mat4 &M,
                                             const Mat4 &target,
                                             const VecXd &q_initial,
                                             const IKOptions &options = {});

// 【IK-04】Jacobian转置迭代，主要用于学习对比，通常收敛更慢。
[[nodiscard]] IKResult solve_ik_transpose(const MatXd &S_list, const Mat4 &M,
                                          const Mat4 &target,
                                          const VecXd &q_initial,
                                          const IKOptions &options = {});

} // namespace piper_control
