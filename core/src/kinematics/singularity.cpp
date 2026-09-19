#include "piper_control/kinematics/singularity.hpp"

#include <Eigen/SVD>
#include <cassert>
#include <cmath>

namespace piper_control
{

/*【SING-01】
 * @brief 计算 Jacobian 的奇异值。
 */
VecXd singular_values(const MatXd& J)
{
    assert(J.rows() > 0);
    assert(J.cols() > 0);

    Eigen::JacobiSVD<MatXd> svd(J);
    return svd.singularValues();
}

/*【SING-02】
 * @brief 计算 Jacobian 的最小奇异值。
 */
double minimum_singular_value(const MatXd& J)
{
    const VecXd sigma = singular_values(J);

    if (sigma.size() == 0)
    {
        return 0.0;
    }

    return sigma(sigma.size() - 1);
}

/*【SING-03】
 * @brief 计算 Jacobian 的条件数。
 */
double condition_number(const MatXd& J)
{
    const VecXd sigma = singular_values(J);

    if (sigma.size() == 0)
    {
        return 0.0;
    }

    const double sigma_max = sigma(0);
    const double sigma_min = sigma(sigma.size() - 1);

    if (sigma_min <= 1e-12)
    {
        return 1e12;
    }

    return sigma_max / sigma_min;
}

/*【SING-04】
 * @brief 计算 Jacobian 的可操作度。
 */
double manipulability(const MatXd& J)
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

/*【SING-05】
 * @brief 使用 SVD 计算 Jacobian 伪逆。
 */
MatXd pseudoinverse_svd(
    const MatXd& J,
    double tolerance)
{
    assert(J.rows() > 0);
    assert(J.cols() > 0);
    assert(tolerance >= 0.0);

    Eigen::JacobiSVD<MatXd> svd(
        J,
        Eigen::ComputeFullU | Eigen::ComputeFullV);

    const VecXd sigma = svd.singularValues();

    MatXd sigma_inv =
        MatXd::Zero(svd.matrixV().cols(), svd.matrixU().cols());

    for (Eigen::Index i = 0; i < sigma.size(); ++i)
    {
        if (sigma(i) > tolerance)
        {
            sigma_inv(i, i) = 1.0 / sigma(i);
        }
    }

    return svd.matrixV() * sigma_inv * svd.matrixU().transpose();
}

/*【SING-06】
 * @brief 使用阻尼最小二乘法计算 Jacobian 伪逆。
 */
MatXd dls_pseudoinverse(
    const MatXd& J,
    double lambda)
{
    assert(J.rows() > 0);
    assert(J.cols() > 0);
    assert(lambda >= 0.0);

    const Eigen::Index m = J.rows();

    const MatXd A =
        J * J.transpose()
        + lambda * lambda * MatXd::Identity(m, m);

    return J.transpose() * A.inverse();
}

}  // namespace piper_control