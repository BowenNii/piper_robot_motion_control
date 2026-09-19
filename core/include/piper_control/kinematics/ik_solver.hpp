#pragma once

#include "piper_control/common/types.hpp"

namespace piper_control
{

struct IKOptions
{
    int max_iterations = 100;

    double position_tolerance = 1e-6;

    double orientation_tolerance = 1e-6;

    double lambda = 1e-3;

    double max_joint_step = 0.2;
};

struct IKResult
{
    bool success = false;

    int iterations = 0;

    double position_error = 0.0;

    double orientation_error = 0.0;

    VecXd q;
};

IKResult solve_ik_newton(
    const MatXd& S_list,
    const Mat4& M,
    const Mat4& target,
    const VecXd& q_initial,
    const IKOptions& options = {});

IKResult solve_ik_dls(
    const MatXd& S_list,
    const Mat4& M,
    const Mat4& target,
    const VecXd& q_initial,
    const IKOptions& options = {});

}  // namespace piper_control