#include "piper_control/kinematics/singularity.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

#include "piper_control/common/constants.hpp"

namespace piper_control
{

// ============================================================
// 【SING-01】 singular_values
//
// 数学定义：
//
//     J = U Σ V^T
//
// 返回奇异值：
//
//     σ_1 ≥ σ_2 ≥ ... ≥ 0
//
// Eigen::JacobiSVD 默认按照降序返回奇异值。
// ============================================================

[[nodiscard]] VecXd singular_values(const MatXd& J)
{
    assert(J.rows() > 0);
    assert(J.cols() > 0);

    Eigen::JacobiSVD<MatXd> svd(J);

    return svd.singularValues();
}


// ============================================================
// 【SING-02】 minimum_singular_value
//
// 数学定义：
//
//     σ_min = min_i σ_i
//
// 由于 Eigen 返回的奇异值已经按照降序排列，
// 因此最后一个元素就是最小奇异值。
// ============================================================

[[nodiscard]] double minimum_singular_value(const MatXd& J)
{
    const VecXd sigma = singular_values(J);

    if (sigma.size() == 0)
    {
        return 0.0;
    }

    return sigma(sigma.size() - 1);
}


// ============================================================
// 【SING-03】 condition_number
//
// 数学定义：
//
//     κ(J) = σ_max / σ_min
//
// 当 σ_min 接近零时，直接计算会产生非常大的数值。
// 因此这里使用统一的数值阈值 kEpsilon 进行判断。
// ============================================================

[[nodiscard]] double condition_number(const MatXd& J)
{
    const VecXd sigma = singular_values(J);

    if (sigma.size() == 0)
    {
        return 0.0;
    }

    const double sigma_max = sigma(0);
    const double sigma_min = sigma(sigma.size() - 1);

    // Jacobian 接近奇异状态时，条件数趋于无穷。
    // 返回有限的大值，避免产生 Inf。
    if (sigma_min <= kEpsilon)
    {
        return std::numeric_limits<double>::infinity();
    }

    return sigma_max / sigma_min;
}


// ============================================================
// 【SING-04】 manipulability
//
// 数学定义：
//
//     w = sqrt(det(J J^T))
//
// 对于满行秩 Jacobian，可由奇异值得到：
//
//     w = ∏ σ_i
//
// 这里直接计算奇异值乘积，避免显式计算 determinant。
// ============================================================

[[nodiscard]] double manipulability(const MatXd& J)
{
    assert(J.rows() > 0);
    assert(J.cols() > 0);

    const VecXd sigma = singular_values(J);

    if (sigma.size() == 0)
    {
        return 0.0;
    }

    double w = 1.0;

    for (Eigen::Index i = 0; i < sigma.size(); ++i)
    {
        w *= sigma(i);
    }

    return w;
}


// ============================================================
// 【SING-05】 pseudoinverse_svd
//
// SVD：
//
//     J = U Σ V^T
//
// Moore-Penrose 伪逆：
//
//     J^+ = V Σ^+ U^T
//
// 对奇异值进行阈值截断：
//
//     σ_i^+ = 1 / σ_i,  σ_i > tolerance
//     σ_i^+ = 0,        σ_i ≤ tolerance
//
// 通过截断接近零的奇异值，避免数值噪声被过度放大。
// ============================================================

[[nodiscard]] MatXd pseudoinverse_svd(
    const MatXd& J,
    double tolerance)
{
    assert(J.rows() > 0);
    assert(J.cols() > 0);
    assert(tolerance >= 0.0);

    Eigen::JacobiSVD<MatXd> svd(
        J,
        Eigen::ComputeFullU | Eigen::ComputeFullV);

    const VecXd& sigma = svd.singularValues();

    // Σ^+ 的尺寸为 n×m。
    MatXd sigma_inv =
        MatXd::Zero(
            svd.matrixV().cols(),
            svd.matrixU().cols());

    for (Eigen::Index i = 0; i < sigma.size(); ++i)
    {
        if (sigma(i) > tolerance)
        {
            sigma_inv(i, i) = 1.0 / sigma(i);
        }
    }

    return svd.matrixV()
         * sigma_inv
         * svd.matrixU().transpose();
}


// ============================================================
// 【SING-06】 dls_pseudoinverse
//
// 数学定义：
//
//     J_DLS^+
//     = J^T (J J^T + λ² I)^(-1)
//
// 对 λ > 0：
//
//     J J^T + λ²I
//
// 为正定矩阵，因此可以使用 LDLT 求解。
//
// λ = 0 时，该公式退化为普通左伪逆，要求 J 满行秩。
// 对于奇异或秩亏 Jacobian，不应依赖 λ = 0 的 LDLT 求解。
// ============================================================

[[nodiscard]] MatXd dls_pseudoinverse(
    const MatXd& J,
    double lambda)
{
    assert(J.rows() > 0);
    assert(J.cols() > 0);
    assert(lambda >= 0.0);

    if (lambda == 0.0)
    {
        return pseudoinverse_svd(J, kEpsilon);
    }

    const Eigen::Index m = J.rows();

    const MatXd A =
        J * J.transpose()
        + lambda * lambda * MatXd::Identity(m, m);

    // 不显式计算 A^{-1}。一定为对称方阵，所以可以用LDLT求逆
    //
    // 求解：
    //
    //     A X = I
    //
    // 得到：
    //
    //     X = A^{-1}
    //
    // 然后：
    //
    //     J_DLS^+ = J^T X
    const MatXd A_inv =
        A.ldlt().solve(MatXd::Identity(m, m));

    return J.transpose() * A_inv;
}


// ============================================================
// 【SING-07】 dls_adaptive
//
// 根据最小奇异值自适应计算阻尼系数。
//
// 当：
//
//     σ_min >= σ_threshold
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
// 最终调用固定阻尼 DLS。
// ============================================================

[[nodiscard]] MatXd dls_adaptive(
    const MatXd& J,
    double lambda_max,
    double sigma_threshold)
{
    assert(J.rows() > 0);
    assert(J.cols() > 0);
    assert(lambda_max >= 0.0);
    assert(sigma_threshold > 0.0);

    const double sigma_min =
        minimum_singular_value(J);

    double lambda = 0.0;

    if (sigma_min < sigma_threshold)
    {
        const double ratio =
            std::clamp(
                sigma_min / sigma_threshold,
                0.0,
                1.0);

        lambda =
            lambda_max
            * std::sqrt(1.0 - ratio * ratio);
    }

    return dls_pseudoinverse(J, lambda);
}

} // namespace piper_control