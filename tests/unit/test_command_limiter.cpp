#include <cassert>
#include <cmath>
#include <iostream>

#include "piper_control/safety/command_limiter.hpp"

namespace
{

constexpr double kTol = 1e-10;

template <typename DerivedA, typename DerivedB>
bool is_vector_close(
    const Eigen::MatrixBase<DerivedA>& A,
    const Eigen::MatrixBase<DerivedB>& B,
    double tol = kTol)
{
    if (A.rows() != B.rows() || A.cols() != B.cols())
    {
        return false;
    }

    return (A.derived() - B.derived()).array().abs().maxCoeff() < tol;
}

}  // namespace

int main()
{
    using namespace piper_control;

    //【SAFE-01】 基本关节速度限幅
    {
        VecXd q_dot(6);
        q_dot << 1.0, -2.0, 0.5, -0.3, 3.0, -4.0;

        VecXd q_dot_max(6);
        q_dot_max << 0.5, 1.0, 0.8, 0.2, 2.0, 3.0;

        const VecXd expected(6);
        VecXd expected_value(6);
        expected_value <<
            0.5,
           -1.0,
            0.5,
           -0.2,
            2.0,
           -3.0;

        const VecXd result =
            limit_joint_velocity(
                q_dot,
                q_dot_max);

        assert(is_vector_close(
            result,
            expected_value));
    }

    //【SAFE-01】 未超过限制的速度保持不变
    {
        VecXd q_dot(3);
        q_dot << 0.1, -0.2, 0.3;

        VecXd q_dot_max(3);
        q_dot_max << 1.0, 1.0, 1.0;

        const VecXd result =
            limit_joint_velocity(
                q_dot,
                q_dot_max);

        assert(is_vector_close(
            result,
            q_dot));
    }

    //【SAFE-01】 最大速度使用绝对值
    {
        VecXd q_dot(3);
        q_dot << 2.0, -2.0, 0.5;

        VecXd q_dot_max(3);
        q_dot_max << -1.0, -1.5, -0.2;

        VecXd expected(3);
        expected << 1.0, -1.5, 0.2;

        const VecXd result =
            limit_joint_velocity(
                q_dot,
                q_dot_max);

        assert(is_vector_close(
            result,
            expected));
    }

    std::cout
        << "All command limiter tests passed.\n";

    return 0;
}