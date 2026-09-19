#include "piper_control/kinematics/singularity.hpp"

#include <Eigen/SVD>

#include <cassert>
#include <cmath>
#include <limits>

namespace piper_control
{

Eigen::VectorXd singular_values(
    const MatXd& J)
{
    Eigen::JacobiSVD<MatXd> svd(
        J,
        Eigen::ComputeThinU | Eigen::ComputeThinV);

    return svd.singularValues();
}

double minimum_singular_value(
    const MatXd& J)
{
    const Eigen::VectorXd sigma = singular_values(J);

    if (sigma.size() == 0)
    {
        return 0.0;
    }

    return sigma(sigma.size() - 1);
}

double condition_number(
    const MatXd& J)
{
    const Eigen::VectorXd sigma = singular_values(J);

    if (sigma.size() == 0)
    {
        return std::numeric_limits<double>::infinity();
    }

    const double sigma_max = sigma(0);
    const double sigma_min = sigma(sigma.size() - 1);

    if (sigma_min <= 1e-12)
    {
        return std::numeric_limits<double>::infinity();
    }

    return sigma_max / sigma_min;
}

double manipulability(
    const MatXd& J)
{
    const Eigen::VectorXd sigma = singular_values(J);

    if (sigma.size() == 0)
    {
        return 0.0;
    }

    double product = 1.0;

    for (Eigen::Index i = 0; i < sigma.size(); ++i)
    {
        product *= sigma(i);
    }

    return product;
}

MatXd pseudoinverse_svd(
    const MatXd& J,
    double tolerance)
{
    assert(tolerance >= 0.0);

    Eigen::JacobiSVD<MatXd> svd(
        J,
        Eigen::ComputeThinU | Eigen::ComputeThinV);

    const auto& singular = svd.singularValues();

    MatXd Sigma_inv =
        MatXd::Zero(
            svd.matrixV().cols(),
            svd.matrixU().cols());

    for (Eigen::Index i = 0; i < singular.size(); ++i)
    {
        if (singular(i) > tolerance)
        {
            Sigma_inv(i, i) = 1.0 / singular(i);
        }
    }

    return svd.matrixV() * Sigma_inv * svd.matrixU().transpose();
}

MatXd dls_pseudoinverse(
    const MatXd& J,
    double lambda)
{
    assert(lambda >= 0.0);

    const Eigen::Index m = J.rows();

    const MatXd A =
        J * J.transpose()
        + lambda * lambda * MatXd::Identity(m, m);

    const MatXd A_inv =
        A.ldlt().solve(MatXd::Identity(m, m));

    return J.transpose() * A_inv;
}

}  // namespace piper_control