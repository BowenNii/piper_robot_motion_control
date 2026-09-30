#include <cassert>
#include <cmath>
#include <iostream>
#include <random>

#include "piper_control/kinematics/singularity.hpp"
#include "piper_control/common/constants.hpp"

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
            pseudoinverse_svd(J, kEpsilon);

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

    // 【SING-07】秩亏 Jacobian：条件数无穷，零阻尼 DLS 安全退化为 SVD 伪逆
    {
        const MatXd J_rank_deficient =
            (MatXd(3, 3) <<
                1.0, 0.0, 0.0,
                0.0, 2.0, 0.0,
                0.0, 0.0, 0.0).finished();

        assert(std::isinf(
            condition_number(J_rank_deficient)));

        assert(is_matrix_close(
            dls_pseudoinverse(
                J_rank_deficient,
                0.0),
            pseudoinverse_svd(
                J_rank_deficient,
                kEpsilon)));
    }
    
    // 【SING-08】固定随机种子：非对角满秩 Jacobian 批量测试
    {
        std::mt19937 rng(42);

        std::uniform_real_distribution<double> value_dist(
            -1.0,
            1.0);

        constexpr int kSampleCount = 100;
        constexpr double kRandomTol = 1e-9;
        constexpr double kLambda = 0.1;
        constexpr double kSigmaThreshold = 0.01;

        for (int sample = 0; sample < kSampleCount; ++sample)
        {
            MatXd J_random(3, 3);

            for (int row = 0; row < 3; ++row)
            {
                for (int col = 0; col < 3; ++col)
                {
                    J_random(row, col) =
                        value_dist(rng);
                }
            }

            // 对角线增加 4，使矩阵严格对角占优，
            // 保证生成的是远离奇异的满秩矩阵。
            J_random.diagonal().array() += 4.0;

            const VecXd sigma =
                singular_values(J_random);

            const MatXd J_pinv =
                pseudoinverse_svd(
                    J_random,
                    kEpsilon);

            const MatXd J_dls =
                dls_pseudoinverse(
                    J_random,
                    kLambda);

            const MatXd J_adaptive =
                dls_adaptive(
                    J_random,
                    kLambda,
                    kSigmaThreshold);

            // 1. 奇异值必须降序排列。
            assert(sigma(0) >= sigma(1));
            assert(sigma(1) >= sigma(2));

            // 2. minimum_singular_value 必须等于最后一个奇异值。
            assert(is_close(
                minimum_singular_value(J_random),
                sigma(2),
                kRandomTol));

            // 3. 条件数必须等于 sigma_max / sigma_min。
            assert(is_close(
                condition_number(J_random),
                sigma(0) / sigma(2),
                kRandomTol));

            // 4. 对满秩方阵：
            //
            //    manipulability = sqrt(det(J * J^T)) = |det(J)|
            assert(is_close(
                manipulability(J_random),
                std::abs(J_random.determinant()),
                kRandomTol));

            // 5. 满秩方阵的 SVD 伪逆应满足：
            //
            //    J * J^+ = I
            assert(is_matrix_close(
                J_random * J_pinv,
                MatXd::Identity(3, 3),
                kRandomTol));

            // 6. lambda > 0 的 DLS 输出必须是有限数。
            assert(J_dls.allFinite());

            // 7. 该随机矩阵远离奇异：
            //
            //    sigma_min > sigma_threshold
            //
            // adaptive DLS 应退化为零阻尼 SVD 伪逆。
            assert(sigma(2) > kSigmaThreshold);

            assert(is_matrix_close(
                J_adaptive,
                J_pinv,
                kRandomTol));
        }

        std::cout
            << "[PASS] SING-08 固定随机种子非对角 Jacobian 测试\n";
    }


    std::cout << "All singularity tests passed.\n";

    return 0;
}