#include <cassert>
#include <cmath>
#include <iostream>

#include "piper_control/kinematics/singularity.hpp"

namespace
{

constexpr double kTol = 1e-10;

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

bool is_close(
    double a,
    double b,
    double tol = kTol)
{
    return std::abs(a - b) < tol;
}

}  // namespace

int main()
{
    using namespace piper_control;

    const MatXd J =
        (MatXd(3, 3) <<
            1.0, 0.0, 0.0,
            0.0, 2.0, 0.0,
            0.0, 0.0, 3.0).finished();

    //【SING-01】 singular_values
    {
        const VecXd sigma =
            singular_values(J);

        const VecXd expected(3);
        VecXd expected_values(3);
        expected_values << 3.0, 2.0, 1.0;

        assert(is_matrix_close(
            sigma,
            expected_values));
    }

    //【SING-02】 minimum_singular_value
    {
        assert(is_close(
            minimum_singular_value(J),
            1.0));
    }

    //【SING-03】 condition_number
    {
        assert(is_close(
            condition_number(J),
            3.0));
    }

    //【SING-04】 manipulability
    {
        assert(is_close(
            manipulability(J),
            6.0));
    }

    //【SING-05】 pseudoinverse_svd
    {
        const MatXd J_pinv =
            pseudoinverse_svd(J);

        const MatXd expected =
            (MatXd(3, 3) <<
                1.0, 0.0, 0.0,
                0.0, 0.5, 0.0,
                0.0, 0.0, 1.0 / 3.0).finished();

        assert(is_matrix_close(
            J_pinv,
            expected));
    }

    //【SING-06】 dls_pseudoinverse
    {
        const double lambda = 0.1;

        const MatXd J_dls =
            dls_pseudoinverse(J, lambda);

        MatXd expected =
            MatXd::Zero(3, 3);

        expected(0, 0) =
            1.0 / (1.0 + lambda * lambda);

        expected(1, 1) =
            2.0 / (4.0 + lambda * lambda);

        expected(2, 2) =
            3.0 / (9.0 + lambda * lambda);

        assert(is_matrix_close(
            J_dls,
            expected));
    }

    std::cout << "All singularity tests passed.\n";

    return 0;
}