#include "piper_control/kinematics/jacobian.hpp"

#include <cassert>

#include "piper_control/lie_group/se3.hpp"
#include "piper_control/lie_group/screw.hpp"

namespace piper_control
{

MatXd jacobian_space(
    const MatXd& S_list,
    const VecXd& q)
{
    assert(S_list.rows() == 6);
    assert(S_list.cols() == q.size());

    const Eigen::Index n = q.size();

    MatXd J = MatXd::Zero(6, n);

    Mat4 T = Mat4::Identity();

    for (Eigen::Index i = 0; i < n; ++i)
    {
        J.col(i) = adjoint(T) * S_list.col(i);

        T = T * screw_exp(S_list.col(i), q(i));
    }

    return J;
}

MatXd jacobian_body(
    const MatXd& B_list,
    const VecXd& q)
{
    assert(B_list.rows() == 6);
    assert(B_list.cols() == q.size());

    const Eigen::Index n = q.size();

    MatXd J = MatXd::Zero(6, n);

    Mat4 T = Mat4::Identity();

    for (Eigen::Index i = n - 1; i >= 0; --i)
    {
        J.col(i) = adjoint(T) * B_list.col(i);

        // 从右往左累积：
        // exp(-B_n q_n) ...
        T = screw_exp(B_list.col(i), -q(i)) * T;
    }

    return J;
}

Vec6 twist_from_space_jacobian(
    const MatXd& J_space,
    const VecXd& q_dot)
{
    assert(J_space.cols() == q_dot.size());

    return J_space * q_dot;
}

Vec6 twist_from_body_jacobian(
    const MatXd& J_body,
    const VecXd& q_dot)
{
    assert(J_body.cols() == q_dot.size());

    return J_body * q_dot;
}

}  // namespace piper_control