#include <cassert>
#include <cmath>
#include <iostream>

#include "piper_control/kinematics/jacobian.hpp"
#include "piper_control/kinematics/forward_kinematics.hpp"
#include "piper_control/lie_group/se3.hpp"
#include "piper_control/lie_group/so3.hpp"
#include "piper_control/lie_group/screw.hpp"

namespace
{

constexpr double kTol = 1e-9;

template <typename DerivedA, typename DerivedB>
bool is_matrix_close(
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

    MatXd S_list(6, 2);

    S_list.col(0) =
        make_revolute_screw_axis(
            Vec3::UnitZ(),
            Vec3::Zero());

    S_list.col(1) =
        make_revolute_screw_axis(
            Vec3::UnitY(),
            Vec3(1.0, 0.0, 0.0));

    VecXd q(2);
    q << 0.3, -0.2;

    //【JAC-01】 q = 0 时，空间 Jacobian 应等于 S_list
    {
        const VecXd q_zero =
            VecXd::Zero(2);

        const MatXd J =
            jacobian_space(S_list, q_zero);

        assert(is_matrix_close(J, S_list));
    }

    //【JAC-01】 空间 Jacobian 与数值微分一致
    {
        const MatXd J =
            jacobian_space(S_list, q);

        const double eps = 1e-7;

        const Mat4 M = Mat4::Identity();

        const Mat4 T =
            forward_poe(S_list, q, M);

        (void)T;

        for (Eigen::Index i = 0; i < q.size(); ++i)
        {
            VecXd q_plus = q;
            VecXd q_minus = q;

            q_plus(i) += eps;
            q_minus(i) -= eps;

            const Mat4 T_plus =
                forward_poe(S_list, q_plus, M);

            const Mat4 T_minus =
                forward_poe(S_list, q_minus, M);

            const Mat4 dT =
                (T_plus - T_minus) / (2.0 * eps);

            const Mat4 V_hat =
                dT * se3_inverse(T);

            Twist V_numerical =
                Twist::Zero();

            V_numerical.head<3>() =
                vee(V_hat.block<3, 3>(0, 0));

            V_numerical.tail<3>() =
                V_hat.block<3, 1>(0, 3);

            assert(is_matrix_close(
                V_numerical,
                J.col(i),
                1e-6));
        }
    }

    //【JAC-02】 q = 0 时，本体 Jacobian 应等于 B_list
    {
        MatXd B_list = S_list;

        const VecXd q_zero =
            VecXd::Zero(2);

        const MatXd J =
            jacobian_body(B_list, q_zero);

        assert(is_matrix_close(J, B_list));
    }

    //【JAC-03】 J_space * q_dot
    {
        const MatXd J =
            jacobian_space(S_list, q);

        VecXd q_dot(2);
        q_dot << 0.5, -0.2;

        const Vec6 V =
            twist_from_space_jacobian(J, q_dot);

        assert(is_matrix_close(
            V,
            J * q_dot));
    }

    //【JAC-04】 J_body * q_dot
    {
        const MatXd J =
            jacobian_body(S_list, q);

        VecXd q_dot(2);
        q_dot << 0.5, -0.2;

        const Vec6 V =
            twist_from_body_jacobian(J, q_dot);

        assert(is_matrix_close(
            V,
            J * q_dot));
    }

    std::cout << "All Jacobian tests passed.\n";

    return 0;
}