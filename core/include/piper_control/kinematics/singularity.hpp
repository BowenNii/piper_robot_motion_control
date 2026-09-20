#ifndef PIPER_CONTROL_KINEMATICS_SINGULARITY_HPP
#define PIPER_CONTROL_KINEMATICS_SINGULARITY_HPP

#include "piper_control/common/types.hpp"

namespace piper_control
{

// ============================================================
// Singularity Analysis
//
// 本模块用于分析机器人 Jacobian 的奇异性以及数值可逆性，
// 并提供 Moore-Penrose 伪逆、DLS 伪逆和自适应 DLS 伪逆。
//
// 对 Jacobian：
//
//     J ∈ R^(m×n)
//
// 使用奇异值分解：
//
//     J = U Σ V^T
//
// 其中奇异值满足：
//
//     σ_1 ≥ σ_2 ≥ ... ≥ σ_r ≥ 0
//
// 奇异值可以用于衡量：
//     - Jacobian 的秩
//     - 奇异程度
//     - 条件数
//     - Manipulability
//     - 伪逆的数值稳定性
//
// 本模块中的 Jacobian 默认采用机器人 Twist Jacobian，
// 但数学实现本身也适用于一般实数矩阵。
// ============================================================


// ============================================================
// 【SING-01】 singular_values
//
// @brief
// 计算 Jacobian 的全部奇异值。
//
// 数学定义：
//
//     J = U Σ V^T
//
// 返回：
//
//     σ = [σ_1, σ_2, ..., σ_min(m,n)]^T
//
// 且：
//
//     σ_1 ≥ σ_2 ≥ ... ≥ 0
//
// 输入：
//     J : m×n Jacobian
//
// 输出：
//     sigma : min(m,n)×1 奇异值向量
//
// 注意：
//     Eigen::JacobiSVD 默认返回按降序排列的奇异值。
// ============================================================

[[nodiscard]] VecXd singular_values(
    const MatXd& J);


// ============================================================
// 【SING-02】 minimum_singular_value
//
// @brief
// 计算 Jacobian 的最小奇异值。
//
// 数学定义：
//
//     σ_min = min_i σ_i
//
// 对于标准 SVD 排序：
//
//     σ_min = σ_min(m,n)
//
// 当 σ_min → 0 时，Jacobian 越接近奇异状态。
//
// 输入：
//     J : m×n Jacobian
//
// 输出：
//     sigma_min : 最小奇异值
//
// 物理意义：
//     σ_min 越小，表示某些方向上的末端运动能力越弱。
// ============================================================

[[nodiscard]] double minimum_singular_value(
    const MatXd& J);


// ============================================================
// 【SING-03】 condition_number
//
// @brief
// 计算 Jacobian 的 2-范数条件数。
//
// 数学定义：
//
//     κ(J) = σ_max / σ_min
//
// 其中：
//
//     σ_max = 最大奇异值
//     σ_min = 最小奇异值
//
// 当：
//
//     σ_min → 0
//
// 时：
//
//     κ(J) → ∞
//
// 因此条件数越大，Jacobian 越接近奇异状态。
//
// 输入：
//     J : m×n Jacobian
//
// 输出：
//     condition_number : Jacobian 条件数
//
// 注意：
//     当 σ_min 小于数值精度阈值时，返回一个有限的大值，
//     用于避免直接产生无穷大。
// ============================================================

[[nodiscard]] double condition_number(
    const MatXd& J);


// ============================================================
// 【SING-04】 manipulability
//
// @brief
// 计算 Jacobian 的 Yoshikawa manipulability measure。
//
// 对于满行秩 Jacobian：
//
//     w = sqrt(det(J J^T))
//
// 根据奇异值分解：
//
//     J = U Σ V^T
//
// 有：
//
//     w = ∏ σ_i
//
// 对于机器人常见的 6×n Jacobian，通常计算：
//
//     w = σ_1 σ_2 ... σ_6
//
// 物理意义：
//     manipulability 越大，表示机器人在各个任务空间方向上的
//     综合运动能力越强；当某个任务方向退化时，
//     manipulability 会趋近于零。
//
// 输入：
//     J : m×n Jacobian
//
// 输出：
//     w : manipulability measure
//
// 注意：
//     对于一般非满行秩或行数大于列数的矩阵，
//     该奇异值乘积应理解为基于奇异值的广义指标，
//     不一定等价于 sqrt(det(J J^T))。
// ============================================================

[[nodiscard]] double manipulability(
    const MatXd& J);


// ============================================================
// 【SING-05】 pseudoinverse_svd
//
// @brief
// 使用 SVD 计算 Moore-Penrose 伪逆。
//
// SVD：
//
//     J = U Σ V^T
//
// Moore-Penrose 伪逆：
//
//     J^+ = V Σ^+ U^T
//
// 对每个奇异值：
//
//     σ_i^+ = 1 / σ_i,    σ_i > tolerance
//
//     σ_i^+ = 0,          σ_i ≤ tolerance
//
// 输入：
//     J         : m×n Jacobian
//     tolerance : 奇异值截断阈值
//
// 输出：
//     J_pinv    : n×m Moore-Penrose 伪逆
//
// 注意：
//     tolerance 用于抑制接近零的奇异值，
//     从而避免放大数值噪声。
// ============================================================

[[nodiscard]] MatXd pseudoinverse_svd(
    const MatXd& J,
    double tolerance);


// ============================================================
// 【SING-06】 dls_pseudoinverse
//
// @brief
// 计算固定阻尼系数的 Damped Least Squares (DLS) 伪逆。
//
// 数学定义：
//
//     J_DLS^+
//     = J^T (J J^T + λ² I)^(-1)
//
// 其中：
//
//     λ ≥ 0
//
// 当 λ = 0 且 J 满行秩时：
//
//     J_DLS^+ = J^+
//
// 当 λ > 0 时，可以抑制接近奇异状态时伪逆对噪声的放大。
//
// 输入：
//     J      : m×n Jacobian
//     lambda : 阻尼系数 λ
//
// 输出：
//     J_dls  : n×m DLS 伪逆
//
// 注意：
//     λ 越大，奇异方向上的增益越小，但同时也会引入更大的
//     逆运动学/速度映射偏差。
// ============================================================

[[nodiscard]] MatXd dls_pseudoinverse(
    const MatXd& J,
    double lambda);


// ============================================================
// 【SING-07】 dls_adaptive
//
// @brief
// 根据最小奇异值自适应调整 DLS 阻尼系数。
//
// 首先计算：
//
//     σ_min = minimum_singular_value(J)
//
// 当：
//
//     σ_min ≥ σ_threshold
//
// 时：
//
//     λ = 0
//
// 当：
//
//     σ_min < σ_threshold
//
// 时：
//
//     r = σ_min / σ_threshold
//
//     λ = λ_max sqrt(1 - r²)
//
// 因此：
//
//     σ_min = σ_threshold
//         → λ = 0
//
//     σ_min → 0
//         → λ → λ_max
//
// 最终：
//
//     J_DLS^+
//     = J^T (J J^T + λ² I)^(-1)
//
// 输入：
//     J               : m×n Jacobian
//     lambda_max      : 最大阻尼系数
//     sigma_threshold : 开始施加阻尼的奇异值阈值
//
// 输出：
//     J_adaptive_dls  : n×m 自适应 DLS 伪逆
//
// 物理意义：
//     远离奇异位形时保持较小的逆映射偏差；
//     接近奇异位形时逐渐增加阻尼，提高数值稳定性。
// ============================================================

[[nodiscard]] MatXd dls_adaptive(
    const MatXd& J,
    double lambda_max,
    double sigma_threshold);

} // namespace piper_control

#endif // PIPER_CONTROL_KINEMATICS_SINGULARITY_HPP