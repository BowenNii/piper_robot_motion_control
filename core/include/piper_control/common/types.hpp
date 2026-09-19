#pragma once

#include <Eigen/Dense>

namespace piper_control
{

using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using VecXd = Eigen::VectorXd;

using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using MatXd = Eigen::MatrixXd;

using Twist = Vec6;
using Wrench = Vec6;

using JointVector = Eigen::VectorXd;
using JointMatrix = Eigen::MatrixXd;

}  // namespace piper_control