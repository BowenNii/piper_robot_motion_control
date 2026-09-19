#include "piper_control/kinematics/ik_solver.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

#include "piper_control/common/constants.hpp"
#include "piper_control/kinematics/forward_kinematics.hpp"
#include "piper_control/kinematics/jacobian.hpp"
#include "piper_control/kinematics/singularity.hpp"
#include "piper_control/lie_group/se3.hpp"

namespace piper_control
{

namespace
{

IKResult solve_impl(
    const MatXd& S_list,
    const Mat4& M,
    const Mat4& target,
    const VecXd& q_initial,
    const IKOptions& options,
    bool use_dls)
{
    assert(S_list.rows() == 6);
    assert(S_list.cols() == q_initial.size());

    IKResult result;

    result.q = q_initial;

    for (int iteration = 0;
         iteration < options.max_iterations;
         ++iteration)
    {
        const Mat4 current =
            forward_poe(S_list, result.q, M);

        const Mat4 error_transform =
            target * se3_inverse(current);

        const Twist error_twist =
            se3_log(error_transform);

        const Vec3 rotation_error =
            error_twist.head<3>();

        const Vec3 position_error =
            target.block<3, 1>(0, 3)
            - current.block<3, 1>(0, 3);

        const double position_norm =
            position_error.norm();

        const double orientation_norm =
            rotation_error.norm();

        result.iterations = iteration + 1;
        result.position_error = position_norm;
        result.orientation_error = orientation_norm;

        if (position_norm < options.position_tolerance &&
            orientation_norm < options.orientation_tolerance)
        {
            result.success = true;
            return result;
        }

        const MatXd J =
            jacobian_space(S_list, result.q);

        MatXd J_pinv;

        if (use_dls)
        {
            J_pinv =
                dls_pseudoinverse(
                    J,
                    options.lambda);
        }
        else
        {
            J_pinv =
                pseudoinverse_svd(J, 1e-10);
        }

        VecXd delta_q =
            J_pinv * error_twist;

        const double step_norm =
            delta_q.norm();

        if (step_norm > options.max_joint_step &&
            step_norm > kEpsilon)
        {
            delta_q *=
                options.max_joint_step / step_norm;
        }

        result.q += delta_q;
    }

    return result;
}

}  // namespace

IKResult solve_ik_newton(
    const MatXd& S_list,
    const Mat4& M,
    const Mat4& target,
    const VecXd& q_initial,
    const IKOptions& options)
{
    return solve_impl(
        S_list,
        M,
        target,
        q_initial,
        options,
        false);
}

IKResult solve_ik_dls(
    const MatXd& S_list,
    const Mat4& M,
    const Mat4& target,
    const VecXd& q_initial,
    const IKOptions& options)
{
    return solve_impl(
        S_list,
        M,
        target,
        q_initial,
        options,
        true);
}

}  // namespace piper_control